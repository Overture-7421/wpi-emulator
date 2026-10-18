#pragma once

// In-memory INtApi for tests of the vision bridge and its publishers. It reproduces the ntcore behaviours
// that matter to them (listed in nt_api.h): stale timestamps and unchanged values are dropped, topic
// properties come only from the first publisher and must be valid JSON, a topic exists only while it has a
// publisher, a second publisher with another type string is inactive, a new subscriber gets the latest value.
// Single threaded, like the real seam.
//
// Test side:  robotPublish / robotSet*  = what the robot (or a coprocessor) does;
//             history / latest / ...    = what the code under test published.

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/bridges/vision/nt_api.h"

namespace wpiemu::bridges::vision::testing {

class FakeNtApi final : public INtApi {
public:
    // ---- test controls --------------------------------------------------------------------------
    void setAvailable(bool available) { available_ = available; }
    void setNowUs(std::int64_t timeUs) { nowUs_ = timeUs; }
    void advanceUs(std::int64_t deltaUs) { nowUs_ += deltaUs; }

    // The robot (or any other client) creates a publisher on `topic`. Call it again for a second publisher.
    void robotPublish(std::string_view topic, NtType type, std::string_view typeString = {},
                      std::string_view propertiesJson = {}) {
        NtPublishOptions options;
        options.typeString = typeString;
        options.propertiesJson = propertiesJson;
        Members members;
        if (!propertiesJson.empty()) parseJsonObject(propertiesJson, members);
        robotPubs_[std::string(topic)] = addPublisher(topic, type, options, members, false);
    }
    // The robot writes a value; the topic is published with the canonical type on the first write.
    void robotSetBoolean(std::string_view topic, bool value, std::int64_t timeUs) {
        NtValue v;
        v.type = NtType::Boolean;
        v.number = value ? 1.0 : 0.0;
        robotPut(topic, std::move(v), timeUs);
    }
    void robotSetInteger(std::string_view topic, std::int64_t value, std::int64_t timeUs) {
        NtValue v;
        v.type = NtType::Integer;
        v.number = static_cast<double>(value);
        robotPut(topic, std::move(v), timeUs);
    }
    void robotSetDouble(std::string_view topic, double value, std::int64_t timeUs) {
        NtValue v;
        v.type = NtType::Double;
        v.number = value;
        robotPut(topic, std::move(v), timeUs);
    }
    void robotSetString(std::string_view topic, std::string_view value, std::int64_t timeUs) {
        NtValue v;
        v.type = NtType::String;
        v.text = std::string(value);
        robotPut(topic, std::move(v), timeUs);
    }
    void robotSetDoubleArray(std::string_view topic, std::initializer_list<double> value, std::int64_t timeUs) {
        NtValue v;
        v.type = NtType::DoubleArray;
        v.numbers.assign(value.begin(), value.end());
        robotPut(topic, std::move(v), timeUs);
    }

    // ---- inspection: what the code under test published -----------------------------------------
    bool exists(std::string_view topic) const {
        const Topic* t = find(topic);
        return t != nullptr && t->publishers > 0;
    }
    // Accepted values, oldest first (stale and unchanged values are not in it).
    const std::vector<NtValue>& history(std::string_view topic) const {
        static const std::vector<NtValue> kNone;
        const Topic* t = find(topic);
        return t != nullptr ? t->history : kNone;
    }
    std::optional<NtValue> latest(std::string_view topic) const {
        const Topic* t = find(topic);
        if (t == nullptr || t->history.empty()) return std::nullopt;
        return t->history.back();
    }
    std::string typeString(std::string_view topic) const {
        const Topic* t = find(topic);
        return t != nullptr ? t->typeString : std::string();
    }
    // Compact JSON object, keys sorted ("{}" when none), as ntcore reports it.
    std::string propertiesJson(std::string_view topic) const {
        const Topic* t = find(topic);
        return t != nullptr ? serialize(t->props) : std::string("{}");
    }
    std::size_t publisherCount(std::string_view topic) const {
        const Topic* t = find(topic);
        return t != nullptr ? t->publishers : 0;
    }
    std::size_t subscriberCount(std::string_view topic) const {
        const Topic* t = find(topic);
        return t != nullptr ? t->subscribers.size() : 0;
    }
    std::size_t acceptedValues() const { return accepted_; }
    std::size_t droppedValues() const { return dropped_; }

    // ---- INtApi ---------------------------------------------------------------------------------
    bool available() const override { return available_; }
    std::int64_t nowUs() const override { return available_ ? nowUs_ : 0; }

    NtPublisher publish(std::string_view topic, NtType type, const NtPublishOptions& options) override {
        if (!available_) return {};
        Members members;
        if (!options.propertiesJson.empty() && !parseJsonObject(options.propertiesJson, members)) return {};
        return addPublisher(topic, type, options, members, true);
    }
    bool setBoolean(NtPublisher pub, std::int64_t timeUs, bool value) override {
        NtValue v;
        v.type = NtType::Boolean;
        v.number = value ? 1.0 : 0.0;
        return put(pub, std::move(v), timeUs);
    }
    bool setInteger(NtPublisher pub, std::int64_t timeUs, std::int64_t value) override {
        NtValue v;
        v.type = NtType::Integer;
        v.number = static_cast<double>(value);
        return put(pub, std::move(v), timeUs);
    }
    bool setDouble(NtPublisher pub, std::int64_t timeUs, double value) override {
        NtValue v;
        v.type = NtType::Double;
        v.number = value;
        return put(pub, std::move(v), timeUs);
    }
    bool setString(NtPublisher pub, std::int64_t timeUs, std::string_view value) override {
        NtValue v;
        v.type = NtType::String;
        v.text = std::string(value);
        return put(pub, std::move(v), timeUs);
    }
    bool setRaw(NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) override {
        NtValue v;
        v.type = NtType::Raw;
        v.raw.assign(value.begin(), value.end());
        return put(pub, std::move(v), timeUs);
    }
    bool setDoubleArray(NtPublisher pub, std::int64_t timeUs, std::span<const double> value) override {
        NtValue v;
        v.type = NtType::DoubleArray;
        v.numbers.assign(value.begin(), value.end());
        return put(pub, std::move(v), timeUs);
    }
    bool setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) override {
        if (!available_ || !isJsonValue(jsonValue)) return false;
        topicFor(topic).props[std::string(name)] = std::string(jsonValue);
        return true;
    }

    NtSubscriber subscribe(std::string_view topic, std::size_t queueDepth) override {
        if (!available_) return {};
        subs_.push_back(Sub{std::string(topic), queueDepth == 0 ? 1 : queueDepth, {}});
        const auto id = static_cast<std::uint32_t>(subs_.size());
        Topic& t = topicFor(topic);
        t.subscribers.push_back(id);
        if (!t.history.empty()) subs_.back().queue.push_back(t.history.back());
        return NtSubscriber{id};
    }
    void read(NtSubscriber sub, std::vector<NtValue>& out) override {
        if (!available_ || sub.id == 0 || sub.id > subs_.size()) return;
        Sub& s = subs_[sub.id - 1];
        out.insert(out.end(), s.queue.begin(), s.queue.end());
        s.queue.clear();
    }

    void topics(std::string_view prefix, std::vector<NtTopicInfo>& out) override {
        if (!available_) return;
        for (const auto& [name, t] : topics_) {
            if (t.publishers == 0 || name.compare(0, prefix.size(), prefix) != 0) continue;
            out.push_back(NtTopicInfo{name, t.typeString, serialize(t.props)});
        }
    }
    bool hasForeignPublisher(std::string_view topic) override {
        const Topic* t = find(topic);
        return available_ && t != nullptr && t->publishers > 0 && !t->ownPublished;
    }

private:
    using Members = std::vector<std::pair<std::string, std::string>>;  // key -> raw JSON value text

    struct Topic {
        NtType type = NtType::Raw;
        std::string typeString;
        std::map<std::string, std::string> props;
        std::size_t publishers = 0;
        bool ownPublished = false;
        std::vector<NtValue> history;
        std::vector<std::uint32_t> subscribers;
    };
    struct Pub {
        std::string topic;
        NtType type = NtType::Raw;
        bool keepDuplicates = false;
        bool active = true;
    };
    struct Sub {
        std::string topic;
        std::size_t depth = 1;
        std::vector<NtValue> queue;
    };

    const Topic* find(std::string_view topic) const {
        const auto it = topics_.find(topic);
        return it == topics_.end() ? nullptr : &it->second;
    }
    Topic& topicFor(std::string_view topic) {
        const auto it = topics_.find(topic);
        if (it != topics_.end()) return it->second;
        return topics_.emplace(std::string(topic), Topic{}).first->second;
    }

    NtPublisher addPublisher(std::string_view topic, NtType type, const NtPublishOptions& options, const Members& members,
                             bool ours) {
        Topic& t = topicFor(topic);
        const std::string typeString =
            options.typeString.empty() ? std::string(ntTypeString(type)) : std::string(options.typeString);
        Pub pub;
        pub.topic = std::string(topic);
        pub.type = type;
        pub.keepDuplicates = options.keepDuplicates;
        if (t.publishers == 0) {
            // Only the first publisher of a topic without publishers defines its type and properties.
            t.type = type;
            t.typeString = typeString;
            for (const auto& [key, value] : members) t.props[key] = value;
        } else {
            pub.active = t.typeString == typeString;
        }
        ++t.publishers;
        if (ours) t.ownPublished = true;
        pubs_.push_back(std::move(pub));
        return NtPublisher{static_cast<std::uint32_t>(pubs_.size())};
    }

    void robotPut(std::string_view topic, NtValue value, std::int64_t timeUs) {
        const std::string key(topic);
        if (robotPubs_.find(key) == robotPubs_.end()) robotPublish(topic, value.type);
        put(robotPubs_[key], std::move(value), timeUs);
    }

    bool put(NtPublisher handle, NtValue value, std::int64_t timeUs) {
        if (!available_ || handle.id == 0 || handle.id > pubs_.size()) return false;
        const Pub& pub = pubs_[handle.id - 1];
        if (pub.type != value.type) return false;
        if (!pub.active) {
            ++dropped_;
            return false;
        }
        Topic& t = topicFor(pub.topic);
        value.timeUs = timeUs == 0 ? nowUs_ : timeUs;
        if (!t.history.empty()) {
            const NtValue& last = t.history.back();
            if (value.timeUs < last.timeUs || (!pub.keepDuplicates && sameValue(last, value))) {
                ++dropped_;
                return true;
            }
        }
        t.history.push_back(value);
        ++accepted_;
        for (const std::uint32_t id : t.subscribers) {
            Sub& s = subs_[id - 1];
            s.queue.push_back(value);
            while (s.queue.size() > s.depth) s.queue.erase(s.queue.begin());
        }
        return true;
    }

    static bool sameValue(const NtValue& a, const NtValue& b) {
        return a.type == b.type && a.number == b.number && a.text == b.text && a.raw == b.raw && a.numbers == b.numbers;
    }

    static std::string serialize(const std::map<std::string, std::string>& props) {
        std::string out = "{";
        for (const auto& [key, value] : props) {
            if (out.size() > 1) out += ',';
            out += '"' + key + "\":" + value;
        }
        return out + "}";
    }

    // Minimal JSON reader: just enough to reject what ntcore rejects (unquoted strings, trailing text) and to
    // split a top-level object into key -> raw value text.
    struct Json {
        std::string_view s;
        std::size_t i = 0;

        void skipSpace() {
            while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
        }
        bool atEnd() {
            skipSpace();
            return i == s.size();
        }
        bool literal(std::string_view word) {
            if (s.substr(i, word.size()) != word) return false;
            i += word.size();
            return true;
        }
        bool string() {  // at the opening quote
            ++i;
            while (i < s.size()) {
                const char c = s[i++];
                if (c == '\\') {
                    if (i >= s.size()) return false;
                    ++i;
                } else if (c == '"') {
                    return true;
                }
            }
            return false;
        }
        bool number() {
            const std::size_t start = i;
            while (i < s.size() && (s[i] == '-' || s[i] == '+' || s[i] == '.' || s[i] == 'e' || s[i] == 'E' ||
                                    (s[i] >= '0' && s[i] <= '9')))
                ++i;
            return i > start;
        }
        bool array() {
            ++i;
            skipSpace();
            if (i < s.size() && s[i] == ']') {
                ++i;
                return true;
            }
            while (true) {
                if (!value()) return false;
                skipSpace();
                if (i >= s.size()) return false;
                if (s[i] == ',') {
                    ++i;
                    continue;
                }
                if (s[i] != ']') return false;
                ++i;
                return true;
            }
        }
        bool object(Members* members) {
            ++i;
            skipSpace();
            if (i < s.size() && s[i] == '}') {
                ++i;
                return true;
            }
            while (true) {
                skipSpace();
                if (i >= s.size() || s[i] != '"') return false;
                const std::size_t keyStart = i;
                if (!string()) return false;
                const std::string key(s.substr(keyStart + 1, i - keyStart - 2));
                skipSpace();
                if (i >= s.size() || s[i] != ':') return false;
                ++i;
                skipSpace();
                const std::size_t valueStart = i;
                if (!value()) return false;
                if (members != nullptr) members->emplace_back(key, std::string(s.substr(valueStart, i - valueStart)));
                skipSpace();
                if (i >= s.size()) return false;
                if (s[i] == ',') {
                    ++i;
                    continue;
                }
                if (s[i] != '}') return false;
                ++i;
                return true;
            }
        }
        bool value() {
            skipSpace();
            if (i >= s.size()) return false;
            const char c = s[i];
            if (c == '"') return string();
            if (c == '{') return object(nullptr);
            if (c == '[') return array();
            if (c == '-' || (c >= '0' && c <= '9')) return number();
            return literal("true") || literal("false") || literal("null");
        }
    };

    static bool isJsonValue(std::string_view text) {
        Json json{text};
        return json.value() && json.atEnd();
    }
    static bool parseJsonObject(std::string_view text, Members& members) {
        Json json{text};
        json.skipSpace();
        if (json.i >= text.size() || text[json.i] != '{') return false;
        return json.object(&members) && json.atEnd();
    }

    bool available_ = true;
    std::int64_t nowUs_ = 0;
    std::map<std::string, Topic, std::less<>> topics_;
    std::vector<Pub> pubs_;
    std::vector<Sub> subs_;
    std::map<std::string, NtPublisher> robotPubs_;
    std::size_t accepted_ = 0;
    std::size_t dropped_ = 0;
};

}  // namespace wpiemu::bridges::vision::testing
