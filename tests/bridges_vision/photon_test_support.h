#pragma once

// Test support of the PhotonVision bridge:
//  * the golden fixtures that tools/gen_photon_fixtures.py generates from the REAL PhotonLib serde
//    (tests/bridges_vision/fixtures/photon/<uuid>/): their text format is documented in that tool's header;
//  * OwnedProfile, a PhotonProfile built from schema texts whose UUIDs it computes (profiles that "a new PhotonLib
//    release" could bring);
//  * RecordingNt, an INtApi in front of a FakeNtApi that remembers every set in order, so tests can check what the
//    FakeNtApi cannot show (ordering across topics, one set per topic per frame, the exact timestamp of a call).

#include <doctest/doctest.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/bridges/vision/photon_mapper.h"
#include "wpiemu/bridges/vision/photon_md5.h"
#include "wpiemu/bridges/vision/photon_profile.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::tests::photon {

using bridges::vision::PhotonValue;

// ---- fixtures -------------------------------------------------------------------------------------------------

inline std::filesystem::path fixtureRoot() {
    return std::filesystem::path(__FILE__).parent_path() / "fixtures" / "photon";
}

inline std::filesystem::path fixtureDirectory(std::string_view uuid) {
    return fixtureRoot() / std::string(uuid);
}

// The error text of a Result / Status, "" when it is ok (doctest message arguments cannot hold a ternary).
template <class R>
std::string why(const R& result) {
    return result.ok() ? std::string() : result.error().message;
}

// The lines of a text file without line ends (a CRLF checkout is read like an LF one), blank lines and comments.
inline std::vector<std::string> readFixtureLines(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty() || line.front() == '#') continue;
        lines.push_back(line);
    }
    return lines;
}

inline std::vector<std::string> splitWords(const std::string& line, char separator = ' ') {
    std::vector<std::string> words;
    std::istringstream stream(line);
    std::string word;
    while (std::getline(stream, word, separator)) words.push_back(word);
    return words;
}

struct PhotonFixture {
    std::string name;     // the file stem
    std::string message;  // PhotonPipelineResult, or Transform3d for the payload of a struct:Transform3d topic
    PhotonValue value;
    std::vector<std::uint8_t> bytes;  // what the real serde produced
};

// Reads the numbers of one fixture line; every parse error is a test failure with the offending word.
class FixtureWords {
public:
    explicit FixtureWords(std::vector<std::string> words) : words_(std::move(words)) {}

    std::string word() { return index_ < words_.size() ? words_[index_++] : std::string(); }
    void expect(std::string_view keyword) { REQUIRE(word() == keyword); }
    double real() { return parse<double>(word()); }
    float real32() { return parse<float>(word()); }
    std::int64_t integer() { return parse<std::int64_t>(word()); }
    bool atEnd() const { return index_ == words_.size(); }

    PhotonValue transform() {
        const double x = real(), y = real(), z = real(), w = real(), qx = real(), qy = real(), qz = real();
        return bridges::vision::photonTransform3d(x, y, z, w, qx, qy, qz);
    }
    PhotonValue corners() {
        PhotonValue list = PhotonValue::list();
        const std::int64_t count = integer();
        for (std::int64_t i = 0; i < count; ++i) {
            const double x = real();
            const double y = real();
            PhotonValue corner = PhotonValue::structure();
            corner.set("x", PhotonValue::real(x)).set("y", PhotonValue::real(y));
            list.push(std::move(corner));
        }
        return list;
    }

private:
    // strtod / strtof are exactly rounded, accept the subnormal and huge values of the fixtures, and (unlike
    // std::from_chars for floating point) exist in every standard library the project builds with.
    template <class T>
    static T parse(const std::string& text) {
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, float>) {
            char* end = nullptr;
            const T value = std::is_same_v<T, double> ? static_cast<T>(std::strtod(text.c_str(), &end))
                                                      : static_cast<T>(std::strtof(text.c_str(), &end));
            REQUIRE_MESSAGE((!text.empty() && end == text.c_str() + text.size()), "cannot parse '" << text << "'");
            return value;
        } else {
            T value{};
            const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
            REQUIRE_MESSAGE((result.ec == std::errc() && result.ptr == text.data() + text.size()),
                            "cannot parse '" << text << "'");
            return value;
        }
    }

    std::vector<std::string> words_;
    std::size_t index_ = 0;
};

inline std::vector<std::uint8_t> fromHex(const std::string& hex) {
    REQUIRE(hex.size() % 2 == 0);
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        unsigned value = 0;
        const auto result = std::from_chars(hex.data() + i, hex.data() + i + 2, value, 16);
        REQUIRE(result.ec == std::errc());
        bytes.push_back(static_cast<std::uint8_t>(value));
    }
    return bytes;
}

inline PhotonFixture loadFixture(const std::filesystem::path& path) {
    PhotonFixture fixture;
    fixture.name = path.stem().string();
    PhotonValue targets = PhotonValue::list();
    PhotonValue metadata = PhotonValue::structure();
    PhotonValue multi = PhotonValue::absent();
    PhotonValue transform;
    for (const std::string& line : readFixtureLines(path)) {
        FixtureWords words(splitWords(line));
        const std::string kind = words.word();
        if (kind == "message") {
            fixture.message = words.word();
        } else if (kind == "meta") {
            const std::int64_t sequence = words.integer(), capture = words.integer(), publish = words.integer(),
                               pong = words.integer();
            metadata.set("sequenceID", PhotonValue::integer(sequence))
                .set("captureTimestampMicros", PhotonValue::integer(capture))
                .set("publishTimestampMicros", PhotonValue::integer(publish))
                .set("timeSinceLastPong", PhotonValue::integer(pong));
        } else if (kind == "target") {
            const double yaw = words.real(), pitch = words.real(), area = words.real(), skew = words.real();
            const std::int64_t fiducialId = words.integer(), objDetectId = words.integer();
            const float objDetectConf = words.real32();
            const double ambiguity = words.real();
            words.expect("best");
            PhotonValue best = words.transform();
            words.expect("alt");
            PhotonValue alt = words.transform();
            words.expect("rect");
            PhotonValue rect = words.corners();
            words.expect("det");
            PhotonValue det = words.corners();
            PhotonValue target = PhotonValue::structure();
            target.set("yaw", PhotonValue::real(yaw))
                .set("pitch", PhotonValue::real(pitch))
                .set("area", PhotonValue::real(area))
                .set("skew", PhotonValue::real(skew))
                .set("fiducialId", PhotonValue::integer(fiducialId))
                .set("objDetectId", PhotonValue::integer(objDetectId))
                .set("objDetectConf", PhotonValue::real(static_cast<double>(objDetectConf)))
                .set("bestCameraToTarget", std::move(best))
                .set("altCameraToTarget", std::move(alt))
                .set("poseAmbiguity", PhotonValue::real(ambiguity))
                .set("minAreaRectCorners", std::move(rect))
                .set("detectedCorners", std::move(det));
            targets.push(std::move(target));
        } else if (kind == "multi") {
            const double bestError = words.real(), altError = words.real(), ambiguity = words.real();
            words.expect("best");
            PhotonValue best = words.transform();
            words.expect("alt");
            PhotonValue alt = words.transform();
            words.expect("ids");
            PhotonValue ids = PhotonValue::list();
            const std::int64_t count = words.integer();
            for (std::int64_t i = 0; i < count; ++i) ids.push(PhotonValue::integer(words.integer()));
            PhotonValue pnp = PhotonValue::structure();
            pnp.set("best", std::move(best))
                .set("alt", std::move(alt))
                .set("bestReprojErr", PhotonValue::real(bestError))
                .set("altReprojErr", PhotonValue::real(altError))
                .set("ambiguity", PhotonValue::real(ambiguity));
            multi = PhotonValue::structure();
            multi.set("estimatedPose", std::move(pnp)).set("fiducialIDsUsed", std::move(ids));
        } else if (kind == "value") {
            transform = words.transform();
        } else if (kind == "bytes") {
            fixture.bytes = fromHex(words.word());
        } else {
            FAIL("unknown fixture line: " << line);
        }
        CHECK(words.atEnd());
    }
    if (fixture.message == "PhotonPipelineResult") {
        fixture.value = PhotonValue::structure();
        fixture.value.set("metadata", std::move(metadata))
            .set("targets", std::move(targets))
            .set("multitagResult", std::move(multi));
    } else {
        fixture.value = std::move(transform);
    }
    return fixture;
}

inline std::vector<PhotonFixture> loadFixtures(std::string_view uuid) {
    std::vector<std::filesystem::path> paths;
    const std::filesystem::path directory = fixtureDirectory(uuid);
    if (std::filesystem::is_directory(directory)) {
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (entry.path().extension() == ".fixture") paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    std::vector<PhotonFixture> fixtures;
    for (const auto& path : paths) fixtures.push_back(loadFixture(path));
    return fixtures;
}

// ---- profiles built from texts ----------------------------------------------------------------------------------

struct MessageSpec {
    std::string name;
    std::string schema;  // "{Other}" stands for the UUID of the message called Other, which must come EARLIER
};

// The six messages of the real profile as specs: their texts with the nested UUIDs written as "{Name}", children
// before the messages that contain them.
inline std::vector<MessageSpec> realMessageSpecs() {
    const bridges::vision::PhotonProfile& real = bridges::vision::newestPhotonProfile();
    std::vector<MessageSpec> pending;
    for (const bridges::vision::PhotonMessageDef& def : real.messages)
        pending.push_back({std::string(def.name), std::string(def.schema)});
    for (MessageSpec& spec : pending) {
        for (const bridges::vision::PhotonMessageDef& def : real.messages) {
            const std::string uuid(def.uuid);
            for (std::size_t at = spec.schema.find(uuid); at != std::string::npos; at = spec.schema.find(uuid, at)) {
                spec.schema.replace(at, uuid.size(), "{" + std::string(def.name) + "}");
            }
        }
    }
    std::vector<MessageSpec> ordered;
    while (!pending.empty()) {
        bool placed = false;
        for (auto it = pending.begin(); it != pending.end() && !placed; ++it) {
            bool ready = true;
            for (std::size_t open = it->schema.find('{'); open != std::string::npos && ready;
                 open = it->schema.find('{', open + 1)) {
                const std::string name = it->schema.substr(open + 1, it->schema.find('}', open) - open - 1);
                ready = std::any_of(ordered.begin(), ordered.end(),
                                    [&](const MessageSpec& done) { return done.name == name; });
            }
            if (ready) {
                ordered.push_back(*it);
                pending.erase(it);
                placed = true;
            }
        }
        REQUIRE(placed);
    }
    return ordered;
}

// A profile whose UUIDs are the MD5 of the texts (computed here, children first), owning its strings.
class OwnedProfile {
public:
    explicit OwnedProfile(const std::vector<MessageSpec>& specs, std::string version = "v-test")
        : version_(std::move(version)) {
        for (const MessageSpec& spec : specs) {
            std::string text = spec.schema;
            for (std::size_t i = 0; i < names_.size(); ++i) {
                const std::string placeholder = "{" + names_[i] + "}";
                for (std::size_t at = text.find(placeholder); at != std::string::npos;
                     at = text.find(placeholder, at)) {
                    text.replace(at, placeholder.size(), uuids_[i]);
                }
            }
            names_.push_back(spec.name);
            uuids_.push_back(bridges::vision::md5Hex(text));
            texts_.push_back(std::move(text));
        }
        for (std::size_t i = 0; i < names_.size(); ++i) {
            defs_.push_back({names_[i], uuids_[i], texts_[i]});
            if (names_[i] == bridges::vision::kPhotonResultMessage) rootUuid_ = uuids_[i];
        }
    }
    OwnedProfile(const OwnedProfile&) = delete;
    OwnedProfile& operator=(const OwnedProfile&) = delete;

    bridges::vision::PhotonProfile profile() const { return {rootUuid_, version_, defs_}; }
    const std::string& uuid() const { return rootUuid_; }

private:
    std::string version_;
    std::string rootUuid_;
    std::vector<std::string> names_, uuids_, texts_;
    std::vector<bridges::vision::PhotonMessageDef> defs_;
};

// ---- recording NetworkTables ------------------------------------------------------------------------------------

// Forwards everything to a FakeNtApi and remembers every publish and every set in order.
class RecordingNt final : public bridges::vision::INtApi {
public:
    struct Set {
        std::string topic;
        std::int64_t timeUs = 0;
    };
    struct Publish {
        std::string topic;
        bridges::vision::NtType type = bridges::vision::NtType::Raw;
        std::string typeString;
        std::string propertiesJson;
        bool sendAll = false;
        bool keepDuplicates = false;
        double periodicS = 0.0;
    };

    explicit RecordingNt(bridges::vision::testing::FakeNtApi& inner) : inner_(inner) {}

    std::vector<Set> sets;           // every set*() call, in call order
    std::vector<Publish> publishes;  // every publish() call, in call order
    mutable int nowCalls = 0;        // how often the clock was read

    std::size_t setsOf(std::string_view topic) const {
        std::size_t count = 0;
        for (const Set& set : sets) count += set.topic == topic ? 1 : 0;
        return count;
    }
    const Publish* publishOf(std::string_view topic) const {
        for (const Publish& publish : publishes) {
            if (publish.topic == topic) return &publish;
        }
        return nullptr;
    }

    bool available() const override { return inner_.available(); }
    std::int64_t nowUs() const override {
        ++nowCalls;
        return inner_.nowUs();
    }
    bridges::vision::NtPublisher publish(std::string_view topic, bridges::vision::NtType type,
                                         const bridges::vision::NtPublishOptions& options) override {
        const bridges::vision::NtPublisher pub = inner_.publish(topic, type, options);
        publishes.push_back(Publish{std::string(topic), type, std::string(options.typeString),
                                    std::string(options.propertiesJson), options.sendAll, options.keepDuplicates,
                                    options.periodicS});
        if (pub) topicOf_[pub.id] = std::string(topic);
        return pub;
    }
    bool setBoolean(bridges::vision::NtPublisher pub, std::int64_t timeUs, bool value) override {
        record(pub, timeUs);
        return inner_.setBoolean(pub, timeUs, value);
    }
    bool setInteger(bridges::vision::NtPublisher pub, std::int64_t timeUs, std::int64_t value) override {
        record(pub, timeUs);
        return inner_.setInteger(pub, timeUs, value);
    }
    bool setDouble(bridges::vision::NtPublisher pub, std::int64_t timeUs, double value) override {
        record(pub, timeUs);
        return inner_.setDouble(pub, timeUs, value);
    }
    bool setString(bridges::vision::NtPublisher pub, std::int64_t timeUs, std::string_view value) override {
        record(pub, timeUs);
        return inner_.setString(pub, timeUs, value);
    }
    bool setRaw(bridges::vision::NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) override {
        record(pub, timeUs);
        return inner_.setRaw(pub, timeUs, value);
    }
    bool setDoubleArray(bridges::vision::NtPublisher pub, std::int64_t timeUs, std::span<const double> value) override {
        record(pub, timeUs);
        return inner_.setDoubleArray(pub, timeUs, value);
    }
    bool setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) override {
        return inner_.setTopicProperty(topic, name, jsonValue);
    }
    bridges::vision::NtSubscriber subscribe(std::string_view topic, std::size_t queueDepth) override {
        return inner_.subscribe(topic, queueDepth);
    }
    void read(bridges::vision::NtSubscriber sub, std::vector<bridges::vision::NtValue>& out) override {
        inner_.read(sub, out);
    }
    void topics(std::string_view prefix, std::vector<bridges::vision::NtTopicInfo>& out) override {
        inner_.topics(prefix, out);
    }
    bool hasForeignPublisher(std::string_view topic) override { return inner_.hasForeignPublisher(topic); }

private:
    void record(bridges::vision::NtPublisher pub, std::int64_t timeUs) {
        const auto it = topicOf_.find(pub.id);
        sets.push_back(Set{it == topicOf_.end() ? std::string("?") : it->second, timeUs});
    }

    bridges::vision::testing::FakeNtApi& inner_;
    std::map<std::uint32_t, std::string> topicOf_;
};

// Keeps what the code under test logged.
class CapturingLogger final : public sdk::ILogger {
public:
    struct Entry {
        sdk::LogLevel level;
        std::string category;
        std::string message;
    };
    std::vector<Entry> entries;

    void log(sdk::LogLevel level, std::string_view category, std::string_view message) override {
        entries.push_back(Entry{level, std::string(category), std::string(message)});
    }
    std::size_t count(sdk::LogLevel level, std::string_view contains = {}) const {
        std::size_t n = 0;
        for (const Entry& entry : entries) {
            n += entry.level == level && entry.message.find(contains) != std::string::npos ? 1 : 0;
        }
        return n;
    }
};

}  // namespace wpiemu::tests::photon
