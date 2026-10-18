#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"

namespace {

using namespace wpiemu::bridges::vision;
using wpiemu::bridges::vision::testing::FakeNtApi;

NtPublishOptions options(std::string_view typeString = {}, std::string_view propertiesJson = {},
                         bool keepDuplicates = false) {
    NtPublishOptions o;
    o.typeString = typeString;
    o.propertiesJson = propertiesJson;
    o.keepDuplicates = keepDuplicates;
    return o;
}

TEST_CASE("fake nt: an unavailable api is inert") {
    FakeNtApi nt;
    nt.setAvailable(false);
    CHECK_FALSE(nt.available());
    CHECK(nt.nowUs() == 0);
    CHECK_FALSE(nt.publish("/a", NtType::Double, options()));
    CHECK_FALSE(nt.setDouble({}, 1, 1.0));
    CHECK_FALSE(nt.subscribe("/a", kDefaultNtQueueDepth));
    std::vector<NtTopicInfo> topics;
    nt.topics("/", topics);
    CHECK(topics.empty());
}

TEST_CASE("fake nt: stale timestamps are dropped silently, equal ones are kept") {
    FakeNtApi nt;
    const NtPublisher pub = nt.publish("/t", NtType::Double, options());
    REQUIRE(pub);
    CHECK(nt.setDouble(pub, 1000, 1.0));
    CHECK(nt.setDouble(pub, 1000, 2.0));  // equal time: accepted
    CHECK(nt.setDouble(pub, 900, 3.0));   // older: dropped, but the call still succeeds
    CHECK(nt.setDouble(pub, 1001, 4.0));
    const auto& history = nt.history("/t");
    REQUIRE(history.size() == 3);
    CHECK(history[0].number == 1.0);
    CHECK(history[1].number == 2.0);
    CHECK(history[2].number == 4.0);
    CHECK(nt.droppedValues() == 1);
}

TEST_CASE("fake nt: unchanged values are dropped unless the publisher keeps duplicates") {
    FakeNtApi nt;
    const NtPublisher plain = nt.publish("/plain", NtType::Double, options());
    const NtPublisher keep = nt.publish("/keep", NtType::Double, options({}, {}, true));
    for (int i = 1; i <= 3; ++i) {
        nt.setDouble(plain, i, 5.0);
        nt.setDouble(keep, i, 5.0);
    }
    CHECK(nt.history("/plain").size() == 1);
    CHECK(nt.history("/keep").size() == 3);
}

TEST_CASE("fake nt: topic properties come from the first publisher and must be a JSON object") {
    FakeNtApi nt;
    CHECK_FALSE(nt.publish("/p", NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":abc})")));  // unquoted
    CHECK_FALSE(nt.publish("/p", NtType::Raw, options("photonstruct:X:1", R"("abc")")));                   // not an object
    CHECK_FALSE(nt.exists("/p"));

    REQUIRE(nt.publish("/p", NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":"abc","a":1})")));
    CHECK(nt.propertiesJson("/p") == R"({"a":1,"message_uuid":"abc"})");

    REQUIRE(nt.publish("/p", NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":"other"})")));
    CHECK(nt.propertiesJson("/p") == R"({"a":1,"message_uuid":"abc"})");  // ignored: not the first publisher

    CHECK_FALSE(nt.setTopicProperty("/p", "b", "unquoted"));
    CHECK(nt.setTopicProperty("/p", "b", R"("quoted")"));
    CHECK(nt.propertiesJson("/p") == R"({"a":1,"b":"quoted","message_uuid":"abc"})");
}

TEST_CASE("fake nt: a second publisher with another type string is inactive") {
    FakeNtApi nt;
    nt.robotPublish("/photonvision/cam/rawBytes", NtType::Raw, "photonstruct:PhotonPipelineResult:aaaa");
    const NtPublisher ours = nt.publish("/photonvision/cam/rawBytes", NtType::Raw,
                                        options("photonstruct:PhotonPipelineResult:bbbb"));
    REQUIRE(ours);
    const std::uint8_t bytes[] = {1, 2, 3};
    CHECK_FALSE(nt.setRaw(ours, 10, bytes));
    CHECK(nt.history("/photonvision/cam/rawBytes").empty());
    CHECK(nt.typeString("/photonvision/cam/rawBytes") == "photonstruct:PhotonPipelineResult:aaaa");
}

TEST_CASE("fake nt: a value of the wrong type for the publisher is refused") {
    FakeNtApi nt;
    const NtPublisher pub = nt.publish("/t", NtType::Integer, options());
    CHECK_FALSE(nt.setDouble(pub, 1, 1.0));
    CHECK_FALSE(nt.setInteger({}, 1, 1));
    CHECK(nt.setInteger(pub, 1, 7));
    CHECK(nt.latest("/t")->number == 7.0);
    CHECK(nt.typeString("/t") == "int");
}

TEST_CASE("fake nt: a topic exists only with a publisher and our own never count as foreign") {
    FakeNtApi nt;
    const NtSubscriber sub = nt.subscribe("/t", kDefaultNtQueueDepth);
    REQUIRE(sub);
    CHECK_FALSE(nt.exists("/t"));  // a subscriber alone does not make a topic exist
    CHECK_FALSE(nt.hasForeignPublisher("/t"));

    nt.robotPublish("/t", NtType::Double);
    CHECK(nt.exists("/t"));
    CHECK(nt.hasForeignPublisher("/t"));

    REQUIRE(nt.publish("/own", NtType::Double, options()));
    CHECK_FALSE(nt.hasForeignPublisher("/own"));
    nt.robotPublish("/own", NtType::Double);
    CHECK_FALSE(nt.hasForeignPublisher("/own"));  // once we published, ntcore cannot tell the difference
}

TEST_CASE("fake nt: a subscriber gets the latest value first, then updates, bounded by its depth") {
    FakeNtApi nt;
    nt.robotSetDouble("/pipeline", 1.0, 100);
    const NtSubscriber sub = nt.subscribe("/pipeline", 2);
    std::vector<NtValue> got;
    nt.read(sub, got);
    REQUIRE(got.size() == 1);
    CHECK(got[0].number == 1.0);
    CHECK(got[0].timeUs == 100);

    nt.robotSetDouble("/pipeline", 2.0, 200);
    nt.robotSetDouble("/pipeline", 3.0, 300);
    nt.robotSetDouble("/pipeline", 4.0, 400);
    got.clear();
    nt.read(sub, got);
    REQUIRE(got.size() == 2);  // the oldest update fell out of the queue
    CHECK(got[0].number == 3.0);
    CHECK(got[1].number == 4.0);
    got.clear();
    nt.read(sub, got);
    CHECK(got.empty());
}

TEST_CASE("fake nt: robot writes of every type are read back with their payload") {
    FakeNtApi nt;
    const NtSubscriber sub = nt.subscribe("/cam/robot_orientation_set", kDefaultNtQueueDepth);
    nt.robotSetDoubleArray("/cam/robot_orientation_set", {45.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 5000);
    nt.robotSetBoolean("/cam/driverMode", true, 6000);
    nt.robotSetString("/cam/name", "front", 7000);
    std::vector<NtValue> got;
    nt.read(sub, got);
    REQUIRE(got.size() == 1);
    CHECK(got[0].type == NtType::DoubleArray);
    CHECK(got[0].numbers.size() == 6);
    CHECK(got[0].numbers[0] == 45.0);
    CHECK(nt.latest("/cam/driverMode")->number == 1.0);
    CHECK(nt.latest("/cam/name")->text == "front");
}

TEST_CASE("fake nt: topics() lists existing topics below a prefix with type string and properties") {
    FakeNtApi nt;
    nt.publish("/photonvision/version", NtType::String, options());
    nt.publish("/photonvision/cam/rawBytes", NtType::Raw, options("photonstruct:R:1", R"({"message_uuid":"1"})"));
    nt.publish("/limelight/tv", NtType::Double, options());
    nt.subscribe("/photonvision/only-subscribed", kDefaultNtQueueDepth);

    std::vector<NtTopicInfo> found;
    nt.topics("/photonvision/", found);
    REQUIRE(found.size() == 2);
    CHECK(found[0].name == "/photonvision/cam/rawBytes");
    CHECK(found[0].typeString == "photonstruct:R:1");
    CHECK(found[0].propertiesJson == R"({"message_uuid":"1"})");
    CHECK(found[1].name == "/photonvision/version");
    CHECK(found[1].typeString == "string");
    CHECK(found[1].propertiesJson == "{}");
}

}  // namespace
