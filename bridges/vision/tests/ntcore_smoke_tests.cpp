// Smoke test of the ntcore binding against the real ntcore, in-process. The test executable
// links ntcore, so the module is already loaded and the binding attaches to it exactly like it does in a robot.
#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#if WPIEMU_WPILIB_YEAR >= 2027
#include <wpi/nt/ntcore_c.h>
#else
#include <ntcore_c.h>
#endif

#include "wpiemu/bridges/vision/nt_binding.h"
#include "wpiemu/bridges/vision/photon_encoding.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace vis = wpiemu::bridges::vision;
namespace sdk = wpiemu::sdk;

namespace {

WPI_String str(const std::string& s) { return WPI_String{s.data(), s.size()}; }

// A value read straight from the default instance through the ntcore C API.
struct Direct {
    NT_Type type = NT_UNASSIGNED;
    std::int64_t time = 0;
    double d = 0;
    std::int64_t i = 0;
    std::vector<std::uint8_t> raw;
    std::string typeString;
};

Direct readDirect(const std::string& path) {
    Direct out;
    const WPI_String name = str(path);
    const NT_Entry entry = NT_GetEntry(NT_GetDefaultInstance(), &name);
    NT_Value v;
    std::memset(&v, 0, sizeof v);
    NT_GetEntryValue(entry, &v);
    out.type = v.type;
    out.time = v.last_change;
    if (v.type == NT_DOUBLE) out.d = v.data.v_double;
    if (v.type == NT_INTEGER) out.i = v.data.v_int;
    if (v.type == NT_RAW) out.raw.assign(v.data.v_raw.data, v.data.v_raw.data + v.data.v_raw.size);
    NT_DisposeValue(&v);
    WPI_String typeString{};
    NT_GetTopicTypeString(NT_GetTopic(NT_GetDefaultInstance(), &name), &typeString);
    if (typeString.str != nullptr) out.typeString.assign(typeString.str, typeString.len);
    WPI_FreeString(&typeString);
    return out;
}

std::string topicProperty(const std::string& path, const std::string& property) {
    const WPI_String name = str(path);
    const WPI_String prop = str(property);
    WPI_String value{};
    NT_GetTopicProperty(NT_GetTopic(NT_GetDefaultInstance(), &name), &prop, &value);
    std::string out = value.str != nullptr ? std::string(value.str, value.len) : std::string();
    WPI_FreeString(&value);
    return out;
}

// Publishes from "the robot" with the C API, like a robot program's NetworkTableEntry.setX would.
void robotWriteInteger(const std::string& path, std::int64_t value) {
    const WPI_String name = str(path);
    const WPI_String type = str("int");
    NT_PubSubOptions options;
    std::memset(&options, 0, sizeof options);
    options.structSize = sizeof options;
    const NT_Publisher pub = NT_Publish(NT_GetTopic(NT_GetDefaultInstance(), &name), NT_INTEGER, &type, &options);
    REQUIRE(pub != 0);
    NT_SetInteger(pub, 0, value);
}

void robotWriteDouble(const std::string& path, double value) {
    const WPI_String name = str(path);
    const WPI_String type = str("double");
    NT_PubSubOptions options;
    std::memset(&options, 0, sizeof options);
    options.structSize = sizeof options;
    const NT_Publisher pub = NT_Publish(NT_GetTopic(NT_GetDefaultInstance(), &name), NT_DOUBLE, &type, &options);
    REQUIRE(pub != 0);
    NT_SetDouble(pub, 0, value);
}

}  // namespace

TEST_CASE("vision ntcore binding: attaches to the loaded ntcore and reports the robot time base") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    // No HAL in this process: the time base is NT_Now().
    const std::int64_t before = NT_Now();
    const std::int64_t now = backend->nowUs();
    CHECK(now >= before);
    CHECK(now - before < 5'000'000);
}

TEST_CASE("vision ntcore binding: a missing ntcore module is unavailable and never fatal") {
    vis::NtcoreBackendOptions options;
    options.ntcoreModule = "wpiemu_no_such_ntcore.dll";
    auto backend = vis::makeNtcoreBackend(options);
    CHECK_FALSE(backend->available());
    CHECK_FALSE(backend->publish("/wpiemu_test/x", vis::ntDouble(1.0), 10).ok());
    CHECK_FALSE(backend->read("/wpiemu_test/x").has_value());
    CHECK(backend->nowUs() == 0);
    backend->unpublish("/wpiemu_test/");
}

TEST_CASE("vision ntcore binding: doubles keep their explicit timestamps") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    REQUIRE(backend->publish("/wpiemu_test/double", vis::ntDouble(1.5), 123'456).ok());
    const Direct d = readDirect("/wpiemu_test/double");
    CHECK(d.type == NT_DOUBLE);
    CHECK(d.d == 1.5);
    CHECK(d.time == 123'456);
    CHECK(d.typeString == "double");

    // The same value again is still a new sample with the new timestamp.
    REQUIRE(backend->publish("/wpiemu_test/double", vis::ntDouble(1.5), 223'456).ok());
    CHECK(readDirect("/wpiemu_test/double").time == 223'456);
    CHECK(vis::ntAsDouble(backend->read("/wpiemu_test/double")).value_or(-1) == 1.5);
}

TEST_CASE("vision ntcore binding: raw topics carry their type string and bytes") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    const vis::NtBytes bytes = {1, 2, 3, 0, 255};
    const std::string typeString(vis::photonPipelineResultTypeString());
    REQUIRE(backend->publish("/wpiemu_test/raw", vis::ntRaw(bytes), 777'000, typeString).ok());
    const Direct d = readDirect("/wpiemu_test/raw");
    CHECK(d.type == NT_RAW);
    CHECK(d.raw == bytes);
    CHECK(d.time == 777'000);
    CHECK(d.typeString == typeString);
    REQUIRE(backend->setTopicProperty("/wpiemu_test/raw", "message_uuid", "\"abc\"").ok());
    CHECK(topicProperty("/wpiemu_test/raw", "message_uuid") == "\"abc\"");
    CHECK_FALSE(backend->setTopicProperty("/wpiemu_test/not_published", "x", "1").ok());
    const std::optional<vis::NtValue> back = backend->read("/wpiemu_test/raw");
    REQUIRE(back.has_value());
    CHECK(std::get<vis::NtBytes>(*back) == bytes);
}

TEST_CASE("vision ntcore binding: every value kind round-trips") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    REQUIRE(backend->publish("/wpiemu_test/bool", vis::ntBool(true), 10).ok());
    REQUIRE(backend->publish("/wpiemu_test/int", vis::ntInt(-42), 10).ok());
    REQUIRE(backend->publish("/wpiemu_test/string", vis::ntString("hello"), 10).ok());
    REQUIRE(backend->publish("/wpiemu_test/doubles", vis::ntDoubles({1.0, 2.5, -3.0}), 10).ok());
    CHECK(vis::ntAsBool(backend->read("/wpiemu_test/bool")).value_or(false));
    CHECK(vis::ntAsInt(backend->read("/wpiemu_test/int")).value_or(0) == -42);
    CHECK(std::get<std::string>(*backend->read("/wpiemu_test/string")) == "hello");
    CHECK(std::get<vis::NtDoubles>(*backend->read("/wpiemu_test/doubles")) == vis::NtDoubles{1.0, 2.5, -3.0});
    CHECK(readDirect("/wpiemu_test/int").typeString == "int");

    // A topic keeps the type of its first value.
    CHECK_FALSE(backend->publish("/wpiemu_test/int", vis::ntDouble(1.0), 11).ok());
    CHECK_FALSE(backend->read("/wpiemu_test/never_written").has_value());
}

TEST_CASE("vision ntcore binding: reads values written by the robot") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    robotWriteInteger("/wpiemu_test/robot/int", 5);
    robotWriteDouble("/wpiemu_test/robot/double", 2.25);
    CHECK(vis::ntAsInt(backend->read("/wpiemu_test/robot/int")).value_or(0) == 5);
    CHECK(vis::ntAsDouble(backend->read("/wpiemu_test/robot/double")).value_or(0) == 2.25);
    // An integer reads as a number, as a double subscriber would see it.
    CHECK(vis::ntAsDouble(backend->read("/wpiemu_test/robot/int")).value_or(0) == 5.0);
}

TEST_CASE("vision ntcore binding: unpublish drops the publishers below a prefix") {
    auto backend = vis::makeNtcoreBackend();
    REQUIRE(backend->available());
    REQUIRE(backend->publish("/wpiemu_test/drop/a", vis::ntDouble(1.0), 10).ok());
    backend->unpublish("/wpiemu_test/drop/");
    // Publishing again creates a fresh publisher.
    REQUIRE(backend->publish("/wpiemu_test/drop/a", vis::ntDouble(2.0), 20).ok());
    CHECK(readDirect("/wpiemu_test/drop/a").d == 2.0);
}

TEST_CASE("vision ntcore binding: the limelight and photonvision bridges publish through ntcore") {
    sdk::VisionBridgeRegistry registry;
    REQUIRE(vis::registerNtVisionBridges(registry).ok());
    auto limelight = registry.create("vision.limelight");
    auto photon = registry.create("vision.photonvision");
    REQUIRE(limelight.ok());
    REQUIRE(photon.ok());
    sdk::NullLogger logger;
    REQUIRE(limelight.value()->start(logger).ok());
    REQUIRE(photon.value()->start(logger).ok());

    sdk::VisionSetup setup;
    sdk::CameraConfig ll;
    ll.kind = sdk::CameraKind::Limelight;
    ll.name = "smoke";
    sdk::CameraConfig pv;
    pv.kind = sdk::CameraKind::PhotonVision;
    pv.name = "smoke";
    setup.cameras = {sdk::CameraSetup{ll}, sdk::CameraSetup{pv}};
    limelight.value()->configure(setup);
    photon.value()->configure(setup);
    limelight.value()->poll(0);
    photon.value()->poll(0);

    sdk::TagDetection tag;
    tag.id = 7;
    tag.yawDeg = 4.0;
    tag.pitchDeg = -2.0;
    tag.areaPercent = 1.25;
    tag.cameraToTag = sdk::Pose3({2.f, 0.f, 0.f}, sdk::Quat::fromYaw(3.14159265f));
    std::vector<sdk::CameraFrame> frames(2);
    frames[0].cameraIndex = 0;
    frames[0].tags = {tag};
    frames[0].captureTimeUs = 1'000'000;
    frames[0].publishTimeUs = 1'040'000;
    frames[1] = frames[0];
    frames[1].cameraIndex = 1;
    limelight.value()->apply(frames, 1'040'000);
    photon.value()->apply(frames, 1'040'000);

    const Direct tv = readDirect("/limelight-smoke/tv");
    CHECK(tv.type == NT_DOUBLE);
    CHECK(tv.d == 1.0);
    CHECK(readDirect("/limelight-smoke/tx").d == 4.0);
    const Direct raw = readDirect("/photonvision/smoke/rawBytes");
    CHECK(raw.type == NT_RAW);
    CHECK(raw.typeString == vis::photonPipelineResultTypeString());
    CHECK(topicProperty("/photonvision/smoke/rawBytes", "message_uuid") == "\"4b2ff16a964b5e2bf04be0c1454d91c4\"");
    CHECK(raw.raw.size() == 32 + 1 + 294 + 1);  // one target with two four-corner lists, no multitag
    CHECK(readDirect("/photonvision/smoke/heartbeat").type == NT_INTEGER);

    // The robot writes a pipeline index; the next poll mirrors it.
    robotWriteDouble("/limelight-smoke/pipeline", 3.0);
    robotWriteInteger("/photonvision/smoke/pipelineIndexRequest", 2);
    limelight.value()->poll(20'000);
    photon.value()->poll(20'000);
    CHECK(readDirect("/limelight-smoke/getpipe").d == 3.0);
    CHECK(readDirect("/photonvision/smoke/pipelineIndexState").i == 2);

    limelight.value()->stop();
    photon.value()->stop();
}
