#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include "../../bridges/vision/src/binding/nt_core_api.h"
#include "wpiemu/bridges/vision/nt_binding.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"

#ifndef WPIEMU_TEST_NTCORE_RUNTIME_DIR
#define WPIEMU_TEST_NTCORE_RUNTIME_DIR ""
#endif

// CONFORMANCE of INtApi: every scenario below is written ONCE and runs against two implementations,
//   * testing::FakeNtApi (what the other lanes' unit tests rely on), and
//   * NtCoreApi bound to the REAL ntcore of WPILib (loaded into this process, which therefore plays the robot
//     process; local-only, no server and no client is ever started),
// so the Fake can never drift from what ntcore really does. Where the two disagreed, the Fake was corrected.
// The real half SKIPS (with a message, never fails) when the staged ntcore is not there: no WPILib at configure
// time, or the build has no ntcore headers to pin the ABI with.

namespace {

using namespace wpiemu::bridges::vision;
using wpiemu::bridges::vision::testing::FakeNtApi;
namespace nb = wpiemu::bridges::vision::binding;

// ---- loading the real ntcore into this process --------------------------------------------------------------------

// "" once ntcore is loaded, otherwise why it could not be. Loaded at most once per process and never unloaded: from
// then on it is a module of this process, exactly like in the robot, so no test may assume it is absent.
std::string loadStagedNtcore() {
    const std::string dir = WPIEMU_TEST_NTCORE_RUNTIME_DIR;
    if (dir.empty()) {
        return "the ntcore shared libraries are not staged (no WPILib, or no ntcore-cpp / wpinet-cpp next to its HAL, at "
               "configure time: see halsim_ext/cmake/WpiEmuWpilib.cmake)";
    }
#if defined(_WIN32)
    std::string windowsDir = dir;
    for (char& c : windowsDir)
        if (c == '/') c = '\\';
    // ntcore imports wpinet and wpiutil: they come first, each from the directory of the library itself.
    for (const char* name : {"wpiutil.dll", "wpinet.dll", "ntcore.dll"}) {
        const std::string path = windowsDir + "\\" + name;
        if (LoadLibraryExA(path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH) == nullptr) {
            return "cannot load " + path + " (Windows error " + std::to_string(GetLastError()) + ")";
        }
    }
#else
#if defined(__APPLE__)
    const char* const libraries[] = {"libwpiutil.dylib", "libwpinet.dylib", "libntcore.dylib"};
#else
    const char* const libraries[] = {"libwpiutil.so", "libwpinet.so", "libntcore.so"};
#endif
    for (const char* name : libraries) {
        const std::string path = dir + "/" + name;
        if (dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL) == nullptr) {
            const char* why = dlerror();
            return "cannot load " + path + (why != nullptr ? std::string(" (") + why + ")" : std::string());
        }
    }
#endif
    return {};
}

const std::string& ntcoreLoadProblem() {
    static const std::string problem = loadStagedNtcore();
    return problem;
}

// Process-wide, so scenarios that run in one process never meet each other's topics (ntcore never forgets a topic).
std::string uniquePrefix() {
    static int counter = 0;
    return "/wpiemu_nt_conformance/" + std::to_string(++counter) + "/";
}

// ---- the two implementations behind one test interface ------------------------------------------------------------
// bridge()   the code under test: what the vision bridge is to NetworkTables
// observer() another client: subscribes and lists, through the same INtApi
// robot*()   what the robot program (or a real coprocessor) does

class FakeHarness {
public:
    INtApi& bridge() { return fake_; }
    INtApi& observer() { return fake_; }
    std::string topic(std::string_view leaf) const { return prefix_ + std::string(leaf); }
    const std::string& prefix() const { return prefix_; }

    void robotPublish(std::string_view topic, NtType type, std::string_view typeString = {},
                      std::string_view propertiesJson = {}) {
        fake_.robotPublish(topic, type, typeString, propertiesJson);
    }
    void robotSetBoolean(std::string_view topic, bool value, std::int64_t timeUs) {
        fake_.robotSetBoolean(topic, value, timeUs);
    }
    void robotSetInteger(std::string_view topic, std::int64_t value, std::int64_t timeUs) {
        fake_.robotSetInteger(topic, value, timeUs);
    }
    void robotSetDouble(std::string_view topic, double value, std::int64_t timeUs) {
        fake_.robotSetDouble(topic, value, timeUs);
    }
    void robotSetString(std::string_view topic, std::string_view value, std::int64_t timeUs) {
        fake_.robotSetString(topic, value, timeUs);
    }
    void robotSetDoubleArray(std::string_view topic, std::span<const double> value, std::int64_t timeUs) {
        fake_.robotSetDoubleArray(topic, value, timeUs);
    }

private:
    FakeNtApi fake_;
    std::string prefix_ = uniquePrefix();
};

class RealHarness {
public:
    // Null (after a MESSAGE saying why) when the real ntcore cannot be used here: the test then simply ends.
    static std::unique_ptr<RealHarness> tryCreate() {
        const std::string problem = ntcoreLoadProblem();
        if (!problem.empty()) {
            MESSAGE("SKIPPED (real ntcore): " << problem);
            return nullptr;
        }
        std::unique_ptr<RealHarness> harness(new RealHarness);
        if (!harness->bridge_->available()) {
            const std::string reason(harness->bridge_->unavailableReason());
            if (reason == nb::kBuiltWithoutHeadersReason) {
                MESSAGE("SKIPPED (real ntcore): " << reason);
                return nullptr;
            }
            FAIL("ntcore is loaded but the binding is not available: " << reason);
        }
        return harness;
    }

    INtApi& bridge() { return *bridge_; }
    INtApi& observer() { return *robot_; }
    std::string topic(std::string_view leaf) const { return prefix_ + std::string(leaf); }
    const std::string& prefix() const { return prefix_; }

    void robotPublish(std::string_view topic, NtType type, std::string_view typeString = {},
                      std::string_view propertiesJson = {}) {
        NtPublishOptions options;
        options.typeString = typeString;
        options.propertiesJson = propertiesJson;
        robotPubs_[std::string(topic)] = robot_->publish(topic, type, options);
    }
    void robotSetBoolean(std::string_view topic, bool value, std::int64_t timeUs) {
        robot_->setBoolean(robotPub(topic, NtType::Boolean), timeUs, value);
    }
    void robotSetInteger(std::string_view topic, std::int64_t value, std::int64_t timeUs) {
        robot_->setInteger(robotPub(topic, NtType::Integer), timeUs, value);
    }
    void robotSetDouble(std::string_view topic, double value, std::int64_t timeUs) {
        robot_->setDouble(robotPub(topic, NtType::Double), timeUs, value);
    }
    void robotSetString(std::string_view topic, std::string_view value, std::int64_t timeUs) {
        robot_->setString(robotPub(topic, NtType::String), timeUs, value);
    }
    void robotSetDoubleArray(std::string_view topic, std::span<const double> value, std::int64_t timeUs) {
        robot_->setDoubleArray(robotPub(topic, NtType::DoubleArray), timeUs, value);
    }

private:
    RealHarness() : bridge_(makeNtCoreApi()), robot_(makeNtCoreApi()) {}

    NtPublisher robotPub(std::string_view topic, NtType type) {
        const auto it = robotPubs_.find(std::string(topic));
        if (it != robotPubs_.end()) return it->second;
        robotPublish(topic, type);
        return robotPubs_[std::string(topic)];
    }

    // Two instances of the binding over the one default NetworkTables instance of this process: "ours" is per
    // instance, so what the robot instance publishes is foreign to the bridge instance, exactly as in the robot.
    std::unique_ptr<INtApi> bridge_;
    std::unique_ptr<INtApi> robot_;
    std::map<std::string, NtPublisher> robotPubs_;
    std::string prefix_ = uniquePrefix();
};

// One scenario = one function template, registered as two plain test cases (not TEST_CASE_TEMPLATE: the names stay
// readable and CTest-discoverable without template decoration).
#define NT_CONFORMANCE(id, title)                                                       \
    template <class Harness>                                                            \
    void scenario_##id(Harness& h);                                                     \
    TEST_CASE("nt conformance (fake): " title) {                                        \
        FakeHarness h;                                                                  \
        scenario_##id(h);                                                               \
    }                                                                                   \
    TEST_CASE("nt conformance (real ntcore): " title) {                                 \
        const std::unique_ptr<RealHarness> h = RealHarness::tryCreate();                \
        if (h) scenario_##id(*h);                                                       \
    }                                                                                   \
    template <class Harness>                                                            \
    void scenario_##id(Harness& h)

NtPublishOptions options(std::string_view typeString = {}, std::string_view propertiesJson = {},
                         bool keepDuplicates = false) {
    NtPublishOptions o;
    o.typeString = typeString;
    o.propertiesJson = propertiesJson;
    o.keepDuplicates = keepDuplicates;
    return o;
}

std::vector<NtValue> drain(INtApi& api, NtSubscriber sub) {
    std::vector<NtValue> values;
    api.read(sub, values);
    return values;
}

std::vector<NtTopicInfo> listed(INtApi& api, const std::string& prefix) {
    std::vector<NtTopicInfo> topics;
    api.topics(prefix, topics);
    return topics;
}

// The info of one topic below the harness prefix, or an empty one when it does not exist.
NtTopicInfo infoOf(INtApi& api, const std::string& topic) {
    for (NtTopicInfo& info : listed(api, topic))
        if (info.name == topic) return std::move(info);
    return {};
}

std::vector<double> numbersOf(const std::vector<NtValue>& values) {
    std::vector<double> out;
    for (const NtValue& v : values) out.push_back(v.number);
    return out;
}

// ---- scenarios: publishing -----------------------------------------------------------------------------------------

NT_CONFORMANCE(explicit_timestamps, "values reach a subscriber with their explicit timestamps") {
    const std::string topic = h.topic("t");
    const NtSubscriber sub = h.observer().subscribe(topic, 16);
    REQUIRE(sub);
    const NtPublisher pub = h.bridge().publish(topic, NtType::Double, options());
    REQUIRE(pub);
    CHECK(h.bridge().setDouble(pub, 1'000'000, 1.5));
    CHECK(h.bridge().setDouble(pub, 2'000'000, 2.5));

    const std::vector<NtValue> got = drain(h.observer(), sub);
    REQUIRE(got.size() == 2);
    CHECK(got[0].type == NtType::Double);
    CHECK(got[0].number == 1.5);
    CHECK(got[0].timeUs == 1'000'000);
    CHECK(got[1].number == 2.5);
    CHECK(got[1].timeUs == 2'000'000);
    CHECK(drain(h.observer(), sub).empty());
}

NT_CONFORMANCE(stale_timestamps, "a value older than the last one is dropped silently, an equal timestamp is kept") {
    const std::string topic = h.topic("t");
    const NtSubscriber sub = h.observer().subscribe(topic, 16);
    const NtPublisher pub = h.bridge().publish(topic, NtType::Double, options());
    REQUIRE((sub && pub));
    CHECK(h.bridge().setDouble(pub, 1000, 1.0));
    CHECK(h.bridge().setDouble(pub, 1000, 2.0));  // equal time: accepted
    CHECK(h.bridge().setDouble(pub, 900, 3.0));   // older: dropped, but the call still succeeds
    CHECK(h.bridge().setDouble(pub, 1001, 4.0));

    const std::vector<NtValue> got = drain(h.observer(), sub);
    CHECK(numbersOf(got) == std::vector<double>{1.0, 2.0, 4.0});
    REQUIRE(got.size() == 3);
    CHECK(got[2].timeUs == 1001);
}

NT_CONFORMANCE(duplicates, "an unchanged value is dropped unless the publisher keeps duplicates") {
    const std::string plainTopic = h.topic("plain");
    const std::string keepTopic = h.topic("keep");
    const NtSubscriber plainSub = h.observer().subscribe(plainTopic, 16);
    const NtSubscriber keepSub = h.observer().subscribe(keepTopic, 16);
    const NtPublisher plain = h.bridge().publish(plainTopic, NtType::Double, options());
    const NtPublisher keep = h.bridge().publish(keepTopic, NtType::Double, options({}, {}, true));
    REQUIRE((plainSub && keepSub && plain && keep));
    for (int i = 1; i <= 3; ++i) {
        CHECK(h.bridge().setDouble(plain, i * 1000, 5.0));
        CHECK(h.bridge().setDouble(keep, i * 1000, 5.0));
    }
    CHECK(drain(h.observer(), plainSub).size() == 1);
    CHECK(drain(h.observer(), keepSub).size() == 3);

    // A different value goes through either way.
    CHECK(h.bridge().setDouble(plain, 4000, 6.0));
    const std::vector<NtValue> changed = drain(h.observer(), plainSub);
    REQUIRE(changed.size() == 1);
    CHECK(changed[0].number == 6.0);
}

NT_CONFORMANCE(properties, "topic properties come from the first publisher only and are JSON") {
    const std::string topic = h.topic("raw");
    REQUIRE(h.bridge().publish(topic, NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":"abc","a":1})")));
    CHECK(infoOf(h.observer(), topic).propertiesJson == R"({"a":1,"message_uuid":"abc"})");

    // A second publisher of an existing topic cannot change its properties.
    REQUIRE(h.bridge().publish(topic, NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":"other"})")));
    CHECK(infoOf(h.observer(), topic).propertiesJson == R"({"a":1,"message_uuid":"abc"})");

    // Property values are JSON text: a string must be quoted, and text that is not JSON is refused and stores nothing.
    CHECK_FALSE(h.bridge().setTopicProperty(topic, "b", "unquoted"));
    CHECK(h.bridge().setTopicProperty(topic, "b", R"("quoted")"));
    CHECK(h.bridge().setTopicProperty(topic, "n", "5"));
    CHECK(infoOf(h.observer(), topic).propertiesJson == R"({"a":1,"b":"quoted","message_uuid":"abc","n":5})");
}

NT_CONFORMANCE(bad_properties, "malformed property JSON is refused, valid JSON that is not an object is ignored") {
    const std::string malformed = h.topic("malformed");
    CHECK_FALSE(h.bridge().publish(malformed, NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":abc})")));
    CHECK_FALSE(h.bridge().publish(malformed, NtType::Raw, options("photonstruct:X:1", R"({"a":1,})")));
    CHECK_FALSE(h.bridge().publish(malformed, NtType::Raw, options("photonstruct:X:1", "{")));
    CHECK(infoOf(h.observer(), malformed).name.empty());  // the refused publishers left no topic behind
    CHECK_FALSE(h.bridge().hasForeignPublisher(malformed));

    // JSON that is valid but not an object is accepted by ntcore, which then publishes without properties.
    const std::string notObject = h.topic("not_object");
    CHECK(h.bridge().publish(notObject, NtType::Raw, options("photonstruct:X:1", R"("abc")")));
    CHECK(infoOf(h.observer(), notObject).propertiesJson == "{}");
    CHECK(h.bridge().publish(h.topic("number"), NtType::Raw, options("photonstruct:X:1", "5")));
    CHECK(h.bridge().publish(h.topic("array"), NtType::Raw, options("photonstruct:X:1", "[1,2]")));
    CHECK(h.bridge().publish(h.topic("null"), NtType::Raw, options("photonstruct:X:1", "null")));
    CHECK(infoOf(h.observer(), h.topic("null")).propertiesJson == "{}");
}

NT_CONFORMANCE(first_publisher_replaces_properties,
               "the first publisher of a topic replaces properties set before it") {
    const std::string topic = h.topic("early");
    CHECK(h.bridge().setTopicProperty(topic, "early", "1"));  // before anybody publishes: stored on the idle topic
    REQUIRE(h.bridge().publish(topic, NtType::Raw, options("photonstruct:X:1", R"({"late":2})")));
    CHECK(infoOf(h.observer(), topic).propertiesJson == R"({"late":2})");
}

NT_CONFORMANCE(type_string_mismatch, "a second publisher with another type string is accepted but inactive") {
    const std::string topic = h.topic("rawBytes");
    h.robotPublish(topic, NtType::Raw, "photonstruct:PhotonPipelineResult:aaaa");
    const NtSubscriber sub = h.observer().subscribe(topic, 16);
    const NtPublisher ours = h.bridge().publish(topic, NtType::Raw, options("photonstruct:PhotonPipelineResult:bbbb"));
    REQUIRE(ours);
    const std::uint8_t bytes[] = {1, 2, 3};
    CHECK_FALSE(h.bridge().setRaw(ours, 10, bytes));
    CHECK(drain(h.observer(), sub).empty());
    CHECK(infoOf(h.observer(), topic).typeString == "photonstruct:PhotonPipelineResult:aaaa");
}

NT_CONFORMANCE(wrong_value_type, "a value of the wrong type for the publisher is refused") {
    const std::string topic = h.topic("i");
    const NtSubscriber sub = h.observer().subscribe(topic, 16);
    const NtPublisher pub = h.bridge().publish(topic, NtType::Integer, options());
    REQUIRE((sub && pub));
    CHECK_FALSE(h.bridge().setDouble(pub, 1, 1.0));
    CHECK_FALSE(h.bridge().setBoolean(pub, 1, true));
    CHECK_FALSE(h.bridge().setString(pub, 1, "x"));
    CHECK_FALSE(h.bridge().setInteger(NtPublisher{}, 1, 1));
    CHECK(h.bridge().setInteger(pub, 2, 7));
    const std::vector<NtValue> got = drain(h.observer(), sub);
    REQUIRE(got.size() == 1);
    CHECK(got[0].type == NtType::Integer);
    CHECK(got[0].number == 7.0);
    CHECK(infoOf(h.observer(), topic).typeString == "int");
}

NT_CONFORMANCE(empty_topic_name, "an empty topic name is refused everywhere") {
    CHECK_FALSE(h.bridge().publish("", NtType::Double, options()));
    CHECK_FALSE(h.bridge().subscribe("", kDefaultNtQueueDepth));
    CHECK_FALSE(h.bridge().setTopicProperty("", "p", "1"));
    CHECK_FALSE(h.bridge().hasForeignPublisher(""));
}

NT_CONFORMANCE(all_value_types, "every value type published by the bridge reaches a subscriber intact") {
    struct Topics {
        std::string b, i, d, s, r, a;
    } t{h.topic("b"), h.topic("i"), h.topic("d"), h.topic("s"), h.topic("r"), h.topic("a")};
    const NtSubscriber sb = h.observer().subscribe(t.b, 16), si = h.observer().subscribe(t.i, 16),
                       sd = h.observer().subscribe(t.d, 16), ss = h.observer().subscribe(t.s, 16),
                       sr = h.observer().subscribe(t.r, 16), sa = h.observer().subscribe(t.a, 16);
    const NtPublisher pb = h.bridge().publish(t.b, NtType::Boolean, options());
    const NtPublisher pi = h.bridge().publish(t.i, NtType::Integer, options());
    const NtPublisher pd = h.bridge().publish(t.d, NtType::Double, options());
    const NtPublisher ps = h.bridge().publish(t.s, NtType::String, options());
    const NtPublisher pr = h.bridge().publish(t.r, NtType::Raw, options("struct:Transform3d"));
    const NtPublisher pa = h.bridge().publish(t.a, NtType::DoubleArray, options());
    REQUIRE((sb && si && sd && ss && sr && sa && pb && pi && pd && ps && pr && pa));

    const std::string withNul("a\0b", 3);
    const std::uint8_t bytes[] = {0x00, 0xFF, 0x7F, 0x00};
    const double numbers[] = {1.5, -2.5, 1e300};
    CHECK(h.bridge().setBoolean(pb, 100, true));
    CHECK(h.bridge().setBoolean(pb, 200, false));
    CHECK(h.bridge().setInteger(pi, 100, -9007199254740991LL));
    CHECK(h.bridge().setDouble(pd, 100, -0.0));
    CHECK(h.bridge().setDouble(pd, 200, 3.141592653589793));
    CHECK(h.bridge().setString(ps, 100, withNul));
    // UTF-8 bytes, spelled out so that the encoding of this source file cannot matter.
    CHECK(h.bridge().setString(ps, 200, "h\xC3\xA9llo"));
    CHECK(h.bridge().setString(ps, 300, ""));
    CHECK(h.bridge().setRaw(pr, 100, bytes));
    CHECK(h.bridge().setRaw(pr, 200, {}));
    CHECK(h.bridge().setDoubleArray(pa, 100, numbers));
    CHECK(h.bridge().setDoubleArray(pa, 200, {}));

    const std::vector<NtValue> booleans = drain(h.observer(), sb);
    REQUIRE(booleans.size() == 2);
    CHECK(booleans[0].type == NtType::Boolean);
    CHECK(booleans[0].number == 1.0);
    CHECK(booleans[1].number == 0.0);
    const std::vector<NtValue> integers = drain(h.observer(), si);
    REQUIRE(integers.size() == 1);
    CHECK(integers[0].type == NtType::Integer);
    CHECK(integers[0].number == -9007199254740991.0);
    const std::vector<NtValue> doubles = drain(h.observer(), sd);
    REQUIRE(doubles.size() == 2);
    CHECK(std::signbit(doubles[0].number));
    CHECK(doubles[1].number == 3.141592653589793);
    const std::vector<NtValue> strings = drain(h.observer(), ss);
    REQUIRE(strings.size() == 3);
    CHECK(strings[0].type == NtType::String);
    CHECK(strings[0].text == withNul);
    CHECK(strings[1].text == "h\xC3\xA9llo");
    CHECK(strings[2].text.empty());
    const std::vector<NtValue> raws = drain(h.observer(), sr);
    REQUIRE(raws.size() == 2);
    CHECK(raws[0].type == NtType::Raw);
    CHECK(raws[0].raw == std::vector<std::uint8_t>(std::begin(bytes), std::end(bytes)));
    CHECK(raws[1].raw.empty());
    const std::vector<NtValue> arrays = drain(h.observer(), sa);
    REQUIRE(arrays.size() == 2);
    CHECK(arrays[0].type == NtType::DoubleArray);
    CHECK(arrays[0].numbers == std::vector<double>(std::begin(numbers), std::end(numbers)));
    CHECK(arrays[1].numbers.empty());

    // The canonical type strings are what the robot's typed subscribers match on.
    CHECK(infoOf(h.observer(), t.b).typeString == "boolean");
    CHECK(infoOf(h.observer(), t.i).typeString == "int");
    CHECK(infoOf(h.observer(), t.d).typeString == "double");
    CHECK(infoOf(h.observer(), t.s).typeString == "string");
    CHECK(infoOf(h.observer(), t.r).typeString == "struct:Transform3d");
    CHECK(infoOf(h.observer(), t.a).typeString == "double[]");
}

// ---- scenarios: reading what the robot writes ----------------------------------------------------------------------

NT_CONFORMANCE(subscribe_initial_value, "a subscriber gets the latest value first, an empty topic gives nothing") {
    const std::string topic = h.topic("pipeline");
    h.robotSetDouble(topic, 1.0, 100);
    const NtSubscriber late = h.observer().subscribe(topic, 8);
    const std::vector<NtValue> first = drain(h.observer(), late);
    REQUIRE(first.size() == 1);
    CHECK(first[0].number == 1.0);
    CHECK(first[0].timeUs == 100);
    CHECK(drain(h.observer(), late).empty());

    // A topic that exists (somebody publishes it) but has no value yet: only ntcore's empty placeholder, which is
    // skipped.
    const std::string idle = h.topic("idle");
    h.robotPublish(idle, NtType::Double);
    const NtSubscriber none = h.observer().subscribe(idle, 8);
    CHECK(drain(h.observer(), none).empty());
    h.robotSetDouble(idle, 5.0, 200);
    CHECK(numbersOf(drain(h.observer(), none)) == std::vector<double>{5.0});

    // A topic nobody publishes at all.
    const NtSubscriber nobody = h.observer().subscribe(h.topic("nobody"), 8);
    CHECK(drain(h.observer(), nobody).empty());
}

NT_CONFORMANCE(subscribe_depth, "a subscriber keeps at most queueDepth updates, oldest first, and drains them once") {
    const std::string topic = h.topic("q");
    const NtSubscriber sub = h.observer().subscribe(topic, 2);
    REQUIRE(sub);
    h.robotSetDouble(topic, 1.0, 100);
    h.robotSetDouble(topic, 2.0, 200);
    h.robotSetDouble(topic, 3.0, 300);
    h.robotSetDouble(topic, 4.0, 400);
    const std::vector<NtValue> got = drain(h.observer(), sub);
    CHECK(numbersOf(got) == std::vector<double>{3.0, 4.0});  // the oldest updates fell out of the queue
    CHECK(drain(h.observer(), sub).empty());

    const NtSubscriber one = h.observer().subscribe(h.topic("q1"), 0);  // a depth of 0 still queues one update
    h.robotSetDouble(h.topic("q1"), 7.0, 100);
    h.robotSetDouble(h.topic("q1"), 8.0, 200);
    CHECK(numbersOf(drain(h.observer(), one)) == std::vector<double>{8.0});
}

NT_CONFORMANCE(robot_writes, "robot writes of every type are read back with their payload") {
    const std::string orientation = h.topic("robot_orientation_set");
    const std::string driver = h.topic("driverMode");
    const std::string pipeline = h.topic("pipelineIndexRequest");
    const std::string name = h.topic("name");
    const NtSubscriber so = h.observer().subscribe(orientation, kDefaultNtQueueDepth);
    const NtSubscriber sd = h.observer().subscribe(driver, kDefaultNtQueueDepth);
    const NtSubscriber sp = h.observer().subscribe(pipeline, kDefaultNtQueueDepth);
    const NtSubscriber sn = h.observer().subscribe(name, kDefaultNtQueueDepth);
    const double yaw[] = {45.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    h.robotSetDoubleArray(orientation, yaw, 5000);
    h.robotSetBoolean(driver, true, 6000);
    h.robotSetInteger(pipeline, 3, 6500);
    h.robotSetString(name, "front", 7000);

    const std::vector<NtValue> o = drain(h.observer(), so);
    REQUIRE(o.size() == 1);
    CHECK(o[0].type == NtType::DoubleArray);
    CHECK(o[0].numbers == std::vector<double>(std::begin(yaw), std::end(yaw)));
    CHECK(o[0].timeUs == 5000);
    const std::vector<NtValue> d = drain(h.observer(), sd);
    REQUIRE(d.size() == 1);
    CHECK(d[0].type == NtType::Boolean);
    CHECK(d[0].number == 1.0);
    const std::vector<NtValue> p = drain(h.observer(), sp);
    REQUIRE(p.size() == 1);
    CHECK(p[0].type == NtType::Integer);
    CHECK(p[0].number == 3.0);
    const std::vector<NtValue> n = drain(h.observer(), sn);
    REQUIRE(n.size() == 1);
    CHECK(n[0].text == "front");
}

NT_CONFORMANCE(robot_duplicates, "a robot that rewrites the same value publishes it only once") {
    const std::string topic = h.topic("ledMode");
    const NtSubscriber sub = h.observer().subscribe(topic, kDefaultNtQueueDepth);
    h.robotSetDouble(topic, 1.0, 100);
    h.robotSetDouble(topic, 1.0, 200);
    h.robotSetDouble(topic, 1.0, 300);
    h.robotSetDouble(topic, 3.0, 400);
    const std::vector<NtValue> got = drain(h.observer(), sub);
    CHECK(numbersOf(got) == std::vector<double>{1.0, 3.0});
}

// ---- scenarios: discovery ------------------------------------------------------------------------------------------

NT_CONFORMANCE(existence, "a topic exists only while it has a publisher") {
    const std::string topic = h.topic("t");
    const NtSubscriber sub = h.observer().subscribe(topic, 8);
    REQUIRE(sub);
    CHECK(listed(h.observer(), h.prefix()).empty());  // a subscriber alone does not make a topic exist
    CHECK_FALSE(h.bridge().hasForeignPublisher(topic));

    h.robotPublish(topic, NtType::Double);
    const std::vector<NtTopicInfo> topics = listed(h.observer(), h.prefix());
    REQUIRE(topics.size() == 1);
    CHECK(topics[0].name == topic);
    CHECK(h.bridge().hasForeignPublisher(topic));
}

NT_CONFORMANCE(topics_listing, "topics(prefix) lists the existing topics below it, with type string and properties") {
    h.robotPublish(h.topic("photonvision/version"), NtType::String);
    h.robotPublish(h.topic("photonvision/cam/rawBytes"), NtType::Raw, "photonstruct:R:1", R"({"message_uuid":"1"})");
    h.robotPublish(h.topic("photonvision/cam/driverModeRequest"), NtType::Boolean);
    h.robotPublish(h.topic("limelight/tv"), NtType::Double);
    REQUIRE(h.bridge().publish(h.topic("photonvision/cam/heartbeat"), NtType::Integer, options()));
    h.observer().subscribe(h.topic("photonvision/only-subscribed"), kDefaultNtQueueDepth);

    const std::string base = h.topic("photonvision/");
    const std::vector<NtTopicInfo> found = listed(h.observer(), base);
    REQUIRE(found.size() == 4);  // sorted by name: the order does not depend on who created what first
    CHECK(found[0].name == h.topic("photonvision/cam/driverModeRequest"));
    CHECK(found[0].typeString == "boolean");
    CHECK(found[0].propertiesJson == "{}");
    CHECK(found[1].name == h.topic("photonvision/cam/heartbeat"));
    CHECK(found[1].typeString == "int");
    CHECK(found[2].name == h.topic("photonvision/cam/rawBytes"));
    CHECK(found[2].typeString == "photonstruct:R:1");
    CHECK(found[2].propertiesJson == R"({"message_uuid":"1"})");
    CHECK(found[3].name == h.topic("photonvision/version"));
    CHECK(found[3].typeString == "string");

    CHECK(listed(h.observer(), h.prefix()).size() == 5);  // the whole harness, whoever published
    CHECK(listed(h.observer(), h.topic("nothing/")).empty());

    std::vector<NtTopicInfo> appended;
    appended.push_back(NtTopicInfo{"/kept", "x", "{}"});
    h.observer().topics(h.topic("limelight/"), appended);  // appends, it never clears
    REQUIRE(appended.size() == 2);
    CHECK(appended[0].name == "/kept");
    CHECK(appended[1].name == h.topic("limelight/tv"));
}

NT_CONFORMANCE(foreign_publisher,
               "only a topic that exists and that the bridge did not publish has a foreign publisher") {
    const std::string robotTopic = h.topic("robot");
    const std::string ownTopic = h.topic("own");
    CHECK_FALSE(h.bridge().hasForeignPublisher(robotTopic));  // nobody publishes it yet

    h.robotPublish(robotTopic, NtType::Raw, "photonstruct:R:1");
    CHECK(h.bridge().hasForeignPublisher(robotTopic));
    CHECK_FALSE(h.bridge().hasForeignPublisher(h.topic("unknown")));

    REQUIRE(h.bridge().publish(ownTopic, NtType::Double, options()));
    CHECK_FALSE(h.bridge().hasForeignPublisher(ownTopic));
    h.robotPublish(ownTopic, NtType::Double);  // a robot publisher that appears later cannot be told apart
    CHECK_FALSE(h.bridge().hasForeignPublisher(ownTopic));

    // Once the bridge publishes to the robot's topic too, ntcore cannot tell the two publishers apart any more.
    REQUIRE(h.bridge().publish(robotTopic, NtType::Raw, options("photonstruct:R:1")));
    CHECK_FALSE(h.bridge().hasForeignPublisher(robotTopic));
}

// ---- the real binding on its own -----------------------------------------------------------------------------------

TEST_CASE("nt real: the binding attaches to the ntcore of this process and reads its clock") {
    const std::unique_ptr<RealHarness> h = RealHarness::tryCreate();
    if (!h) return;
    CHECK(h->bridge().available());
    CHECK(h->bridge().unavailableReason().empty());
    const std::int64_t first = h->bridge().nowUs();
    CHECK(first > 0);  // NT_Now: microseconds on ntcore's own clock (the HAL FPGA clock inside a simulated robot)
    CHECK(h->bridge().nowUs() >= first);
    CHECK(h->observer().nowUs() >= first);
}

TEST_CASE("nt real: without the ntcore headers the binding is inert and says why") {
    if (nb::abiVerifiedAgainstHeaders()) {
        MESSAGE("this build pins the ntcore ABI with the real headers: nothing to check here");
        return;
    }
    const std::unique_ptr<INtApi> api = makeNtCoreApi();
    REQUIRE(api);
    CHECK_FALSE(api->available());
    CHECK(api->unavailableReason() == nb::kBuiltWithoutHeadersReason);
    CHECK_FALSE(api->publish("/t", NtType::Double, {}));
    CHECK(api->nowUs() == 0);
}

}  // namespace
