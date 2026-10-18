#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "photon_test_support.h"
#include "vision_frame_support.h"
#include "wpiemu/bridges/vision/camera_publisher.h"
#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/bridges/vision/photon_profile.h"
#include "wpiemu/bridges/vision/photon_vendor.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"

namespace {

namespace sdk = wpiemu::sdk;
namespace support = wpiemu::tests::vision;
using namespace wpiemu::bridges::vision;
using wpiemu::bridges::vision::testing::FakeNtApi;
using wpiemu::tests::photon::CapturingLogger;
using wpiemu::tests::photon::RecordingNt;

constexpr const char* kUuid = "4b2ff16a964b5e2bf04be0c1454d91c4";
constexpr std::int64_t kStartUs = 1'000'000;
const std::string kTable = "/photonvision/photon-front/";

// Everything a camera test needs: a fake ntcore behind a recorder, a logger and the vendor under test.
struct Rig {
    FakeNtApi fake;
    RecordingNt nt{fake};
    CapturingLogger log;
    std::unique_ptr<IVisionVendor> vendor = makePhotonVendor();

    Rig() { fake.setNowUs(kStartUs); }

    static sdk::CameraSetup setup(std::string ntName = "photon-front", std::string id = "front") {
        sdk::CameraSetup s;
        s.cameraId = std::move(id);
        s.vendor = sdk::CameraVendor::PhotonVision;
        s.ntName = std::move(ntName);
        s.widthPx = 1280;
        s.heightPx = 800;
        s.fxPx = 640.0;
        s.fyPx = 641.5;
        s.cxPx = 639.5;
        s.cyPx = 399.5;
        s.distortion = {0.1, -0.2, 0.001, 0.002, 0.03};
        s.frameRateHz = 30.0;
        s.latencyUs = 25000;
        return s;
    }

    std::unique_ptr<ICameraPublisher> camera(const sdk::CameraSetup& s) {
        std::unique_ptr<ICameraPublisher> publisher = vendor->createPublisher(nt, log, s);
        REQUIRE(publisher != nullptr);
        return publisher;
    }
    std::unique_ptr<ICameraPublisher> startedCamera(const sdk::CameraSetup& s = setup()) {
        std::unique_ptr<ICameraPublisher> publisher = camera(s);
        publisher->start();
        return publisher;
    }
};

// A frame of the sample scene (robot at (2, 4) facing +X, PhotonVision sample mount, tags of the test layout).
sdk::VisionFrame frameFor(const sdk::CameraSetup& setup, std::vector<int> tags, std::uint64_t sequence,
                          std::int64_t latencyUs = 25000) {
    support::FrameSpec spec;
    spec.cameraId = setup.cameraId;
    spec.ntName = setup.ntName;
    spec.fieldToRobot = support::sampleRobotPose();
    spec.robotToCamera = support::sampleFrontMount();
    spec.tagIds = std::move(tags);
    spec.sequence = sequence;
    spec.latencyUs = latencyUs;
    return support::makeFrame(spec, support::sampleLayout());
}

const PhotonCodec& codec() {
    static const PhotonCodec instance = [] {
        auto created = PhotonCodec::create(newestPhotonProfile());
        REQUIRE(created.ok());
        return std::move(created).value();
    }();
    return instance;
}

PhotonValue decodeResult(const NtValue& value) {
    REQUIRE(value.type == NtType::Raw);
    auto decoded = codec().decode(kPhotonResultMessage, value.raw);
    REQUIRE_MESSAGE(decoded.ok(), wpiemu::tests::photon::why(decoded));
    return std::move(decoded).value();
}

PhotonValue latestResult(const FakeNtApi& fake, const std::string& table = kTable) {
    const auto raw = fake.latest(table + "rawBytes");
    REQUIRE(raw.has_value());
    return decodeResult(*raw);
}

double number(const FakeNtApi& fake, const std::string& topic) {
    const auto value = fake.latest(topic);
    REQUIRE_MESSAGE(value.has_value(), "no value on " << topic);
    return value->number;
}

std::vector<std::string> topicNames(FakeNtApi& fake, std::string_view prefix) {
    std::vector<NtTopicInfo> topics;
    fake.topics(prefix, topics);
    std::vector<std::string> names;
    for (const NtTopicInfo& topic : topics) names.push_back(topic.name);
    return names;
}

// ---- topics --------------------------------------------------------------------------------------------------

TEST_CASE("photon publisher: start creates exactly the topics of a PhotonVision coprocessor, with their types") {
    Rig rig;
    const auto camera = rig.startedCamera();

    std::vector<NtTopicInfo> topics;
    rig.fake.topics("/photonvision/", topics);
    std::vector<std::pair<std::string, std::string>> found;
    for (const NtTopicInfo& topic : topics) found.emplace_back(topic.name, topic.typeString);

    const std::string result = std::string("photonstruct:PhotonPipelineResult:") + kUuid;
    const std::vector<std::pair<std::string, std::string>> expected = {
        {"/photonvision/ledModeState", "int"},
        {kTable + "cameraDistortion", "double[]"},
        {kTable + "cameraIntrinsics", "double[]"},
        {kTable + "driverMode", "boolean"},
        {kTable + "fps", "double"},
        {kTable + "fpsLimit", "int"},
        {kTable + "hasTarget", "boolean"},
        {kTable + "heartbeat", "int"},
        {kTable + "latencyMillis", "double"},
        {kTable + "pipelineIndexState", "int"},
        {kTable + "rawBytes", result},
        {kTable + "targetArea", "double"},
        {kTable + "targetPitch", "double"},
        {kTable + "targetPixelsX", "double"},
        {kTable + "targetPixelsY", "double"},
        {kTable + "targetPose", "struct:Transform3d"},
        {kTable + "targetSkew", "double"},
        {kTable + "targetYaw", "double"},
        {"/photonvision/version", "string"},
    };
    CHECK(found == expected);
    CHECK(camera->robotState().status.empty());
    CHECK(camera->robotState().publishing);
}

TEST_CASE(
    "photon publisher: the result stream carries the UUID in its type string AND its message_uuid property, set "
    "atomically") {
    Rig rig;
    const auto camera = rig.startedCamera();

    CHECK(rig.fake.typeString(kTable + "rawBytes") == std::string("photonstruct:PhotonPipelineResult:") + kUuid);
    const std::string expectedProperties = std::string("{\"message_uuid\":\"") + kUuid + "\"}";
    CHECK(rig.fake.propertiesJson(kTable + "rawBytes") == expectedProperties);

    // Properties go in the publish call itself (NT_PublishEx), not in a later setTopicProperty that a version check
    // could race.
    const RecordingNt::Publish* raw = rig.nt.publishOf(kTable + "rawBytes");
    REQUIRE(raw != nullptr);
    CHECK(raw->type == NtType::Raw);
    CHECK(raw->propertiesJson == expectedProperties);
    CHECK(raw->sendAll);
    CHECK(raw->keepDuplicates);
    CHECK(raw->periodicS == doctest::Approx(0.01));

    // Every other topic has plain options and the canonical type string.
    for (const RecordingNt::Publish& publish : rig.nt.publishes) {
        if (publish.topic == kTable + "rawBytes") continue;
        CAPTURE(publish.topic);
        CHECK_FALSE(publish.sendAll);
        CHECK_FALSE(publish.keepDuplicates);
        CHECK(publish.propertiesJson.empty());
    }
    const RecordingNt::Publish* pose = rig.nt.publishOf(kTable + "targetPose");
    REQUIRE(pose != nullptr);
    CHECK(pose->typeString == "struct:Transform3d");
}

TEST_CASE("photon publisher: before the first frame a robot sees a coprocessor that is there, idle and calibrated") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);

    const auto version = rig.fake.latest("/photonvision/version");
    REQUIRE(version.has_value());
    CHECK_FALSE(version->text.empty());  // PhotonLib's C++ camera treats an empty version as "no coprocessor"
    CHECK(version->text == newestPhotonProfile().version);
    CHECK(number(rig.fake, "/photonvision/ledModeState") == -1.0);  // VisionLEDMode.kDefault

    CHECK(number(rig.fake, kTable + "heartbeat") == 0.0);
    CHECK(number(rig.fake, kTable + "pipelineIndexState") == 0.0);
    CHECK(number(rig.fake, kTable + "driverMode") == 0.0);
    CHECK(number(rig.fake, kTable + "fpsLimit") == -1.0);
    CHECK(number(rig.fake, kTable + "hasTarget") == 0.0);
    CHECK(number(rig.fake, kTable + "latencyMillis") == 0.0);
    CHECK(number(rig.fake, kTable + "fps") == 0.0);
    for (const char* leaf :
         {"targetPitch", "targetYaw", "targetArea", "targetSkew", "targetPixelsX", "targetPixelsY"}) {
        CAPTURE(leaf);
        CHECK(number(rig.fake, kTable + leaf) == 0.0);
    }
    const auto intrinsics = rig.fake.latest(kTable + "cameraIntrinsics");
    REQUIRE(intrinsics.has_value());
    CHECK(intrinsics->numbers ==
          std::vector<double>{640.0, 0.0, 639.5, 0.0, 641.5, 399.5, 0.0, 0.0, 1.0});  // row-major
    const auto distortion = rig.fake.latest(kTable + "cameraDistortion");
    REQUIRE(distortion.has_value());
    CHECK(distortion->numbers == std::vector<double>{0.1, -0.2, 0.001, 0.002, 0.03});

    // The target pose of "no target" is Transform3d(): the identity, 56 bytes.
    const auto pose = rig.fake.latest(kTable + "targetPose");
    REQUIRE(pose.has_value());
    const auto decoded = codec().decode("Transform3d", pose->raw);
    REQUIRE(decoded.ok());
    CHECK(decoded.value().find("rotation")->find("q")->find("w")->realValue() == 1.0);
    CHECK(rig.fake.history(kTable + "rawBytes").empty());  // no result before the first frame
}

// ---- frames --------------------------------------------------------------------------------------------------

TEST_CASE(
    "photon publisher: a frame is published with one clock reading, one set per topic, the result before the "
    "heartbeat") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    rig.nt.sets.clear();
    rig.nt.nowCalls = 0;
    rig.fake.setNowUs(2'000'000);

    camera->publish(frameFor(setup, {1, 2, 3}, 17, 25000));

    CHECK(rig.nt.nowCalls == 1);
    const std::vector<std::string> leaves = {"rawBytes",      "latencyMillis", "fps",        "hasTarget",
                                             "targetPitch",   "targetYaw",     "targetArea", "targetSkew",
                                             "targetPixelsX", "targetPixelsY", "targetPose", "heartbeat"};
    REQUIRE(rig.nt.sets.size() == leaves.size());
    for (std::size_t i = 0; i < leaves.size(); ++i) {
        CAPTURE(leaves[i]);
        CHECK(rig.nt.sets[i].topic == kTable + leaves[i]);  // this order: the result first, the heartbeat last
        CHECK(rig.nt.sets[i].timeUs == 2'000'000);          // the same explicit time on every value of the frame
    }
    CHECK(rig.nt.setsOf("/photonvision/version") == 0);  // root and calibration topics are not rewritten per frame
    CHECK(rig.nt.setsOf(kTable + "cameraIntrinsics") == 0);
    CHECK(camera->robotState().framesPublished == 1);
}

TEST_CASE("photon publisher: the result carries the capture time (publish - latency), the sequence and the pong time") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    rig.fake.setNowUs(2'000'000);
    camera->publish(frameFor(setup, {1, 2, 3}, 17, 25000));

    const PhotonValue result = latestResult(rig.fake);
    const PhotonValue& metadata = *result.find("metadata");
    CHECK(metadata.find("sequenceID")->intValue() == 17);
    CHECK(metadata.find("publishTimestampMicros")->intValue() == 2'000'000);
    CHECK(metadata.find("captureTimestampMicros")->intValue() == 2'000'000 - 25000);
    CHECK(metadata.find("timeSinceLastPong")->intValue() == 10000);
    CHECK(rig.fake.latest(kTable + "rawBytes")->timeUs == 2'000'000);
    CHECK(number(rig.fake, kTable + "latencyMillis") == 25.0);
    CHECK(number(rig.fake, kTable + "fps") == 30.0);

    // A frame older than "now" is still stamped at publish time minus its own latency.
    rig.fake.setNowUs(2'100'000);
    camera->publish(frameFor(setup, {1, 2, 3}, 18, 40000));
    const PhotonValue second = latestResult(rig.fake);
    CHECK(second.find("metadata")->find("captureTimestampMicros")->intValue() == 2'100'000 - 40000);
    CHECK(number(rig.fake, kTable + "latencyMillis") == 40.0);
}

TEST_CASE("photon publisher: the timestamps never go backwards, even when the clock does") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    rig.fake.setNowUs(2'000'000);
    camera->publish(frameFor(setup, {1, 2, 3}, 1));
    rig.fake.setNowUs(1'500'000);  // HAL time is never restarted; if it ever were, ntcore would drop the older stamp
    camera->publish(frameFor(setup, {1, 2, 3}, 2));

    const auto& history = rig.fake.history(kTable + "rawBytes");
    REQUIRE(history.size() == 2);
    CHECK(history[0].timeUs == 2'000'000);
    CHECK(history[1].timeUs == 2'000'000);
    CHECK(decodeResult(history[1]).find("metadata")->find("captureTimestampMicros")->intValue() == 2'000'000 - 25000);
    CHECK(rig.fake.history(kTable + "heartbeat").size() == 3);  // 0, 1, 2: nothing was dropped
}

TEST_CASE("photon publisher: every frame is published, also one without a target, and the heartbeat counts them") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);

    for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
        rig.fake.advanceUs(33'000);
        camera->publish(frameFor(setup, sequence == 2 ? std::vector<int>{} : std::vector<int>{1, 2}, sequence));
    }
    CHECK(rig.fake.history(kTable + "rawBytes").size() == 3);
    CHECK(camera->robotState().framesPublished == 3);
    const auto& heartbeat = rig.fake.history(kTable + "heartbeat");
    REQUIRE(heartbeat.size() == 4);  // the initial 0, then 1 2 3
    for (std::size_t i = 0; i < heartbeat.size(); ++i) CHECK(heartbeat[i].number == static_cast<double>(i));

    // The middle frame: a result with nothing in it.
    const PhotonValue empty = decodeResult(rig.fake.history(kTable + "rawBytes")[1]);
    CHECK(empty.find("targets")->children().empty());
    CHECK(empty.find("multitagResult")->isAbsent());
    CHECK(empty.find("metadata")->find("sequenceID")->intValue() == 2);
    const auto& hasTarget = rig.fake.history(kTable + "hasTarget");
    REQUIRE(hasTarget.size() == 4);  // false at start, then true, false, true: each one a change
    CHECK(hasTarget[1].number == 1.0);
    CHECK(hasTarget[2].number == 0.0);
    CHECK(hasTarget[3].number == 1.0);
}

TEST_CASE("photon publisher: the heartbeat changes with every frame even if a caller repeats a sequence number") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    for (const std::uint64_t sequence : {5ULL, 5ULL, 5ULL, 100ULL, 100ULL}) {
        rig.fake.advanceUs(1000);
        camera->publish(frameFor(setup, {1}, sequence));
    }
    const auto& heartbeat = rig.fake.history(kTable + "heartbeat");
    REQUIRE(heartbeat.size() == 6);
    const std::vector<double> expected = {0, 5, 6, 7, 100, 101};
    for (std::size_t i = 0; i < expected.size(); ++i) CHECK(heartbeat[i].number == expected[i]);
}

TEST_CASE("photon publisher: the legacy single-target topics follow the best (first) target of the result") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    const sdk::VisionFrame frame = frameFor(setup, {1, 2, 3, 4}, 9);
    rig.fake.setNowUs(3'000'000);
    camera->publish(frame);

    const PhotonValue result = latestResult(rig.fake);
    const PhotonValue& best = result.find("targets")->children().front();
    CHECK(number(rig.fake, kTable + "hasTarget") == 1.0);
    CHECK(number(rig.fake, kTable + "targetYaw") == best.find("yaw")->realValue());
    CHECK(number(rig.fake, kTable + "targetPitch") == best.find("pitch")->realValue());
    CHECK(number(rig.fake, kTable + "targetArea") == best.find("area")->realValue());
    CHECK(number(rig.fake, kTable + "targetSkew") == 0.0);
    CHECK(number(rig.fake, kTable + "targetPixelsX") == frame.observations.front().centerPx.x);
    CHECK(number(rig.fake, kTable + "targetPixelsY") == frame.observations.front().centerPx.y);
    const auto pose = codec().decode("Transform3d", rig.fake.latest(kTable + "targetPose")->raw);
    REQUIRE(pose.ok());
    CHECK(pose.value() == *best.find("bestCameraToTarget"));

    // The frame after has no target: the topics go back to zeros.
    rig.fake.advanceUs(33'000);
    camera->publish(frameFor(setup, {}, 10));
    CHECK(number(rig.fake, kTable + "hasTarget") == 0.0);
    CHECK(number(rig.fake, kTable + "targetYaw") == 0.0);
    CHECK(number(rig.fake, kTable + "targetArea") == 0.0);
    CHECK(number(rig.fake, kTable + "targetPixelsX") == 0.0);
    CHECK(rig.fake.latest(kTable + "targetPose")->raw.size() == 56);
}

TEST_CASE("photon publisher: the multi-tag result is published with the frame's tags") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    rig.fake.advanceUs(1000);
    camera->publish(frameFor(setup, {1, 2, 3}, 1));
    const PhotonValue result = latestResult(rig.fake);
    const PhotonValue& multi = *result.find("multitagResult");
    REQUIRE(multi.kind() == PhotonValue::Kind::Struct);
    const PhotonValue& ids = *multi.find("fiducialIDsUsed");
    REQUIRE(ids.children().size() == 3);
    const PhotonValue& pose = *multi.find("estimatedPose")->find("best")->find("translation");
    CHECK(pose.find("x")->realValue() == doctest::Approx(2.30).epsilon(1e-6));  // the camera position in the field
    CHECK(pose.find("y")->realValue() == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(pose.find("z")->realValue() == doctest::Approx(0.5).epsilon(1e-6));

    rig.fake.advanceUs(1000);
    camera->publish(frameFor(setup, {1}, 2));  // one tag: no multi-tag solution
    CHECK(latestResult(rig.fake).find("multitagResult")->isAbsent());
}

TEST_CASE("photon publisher: no more than 127 targets and ids are published") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    sdk::VisionFrame frame = frameFor(setup, {1, 2}, 1);
    const sdk::FiducialObservation first = frame.observations.front();
    frame.observations.assign(130, first);
    frame.multiTag.tagIds.assign(130, 7);
    rig.fake.advanceUs(1000);
    camera->publish(frame);
    const PhotonValue result = latestResult(rig.fake);
    CHECK(result.find("targets")->children().size() == 127);
    CHECK(result.find("multitagResult")->find("fiducialIDsUsed")->children().size() == 127);
}

TEST_CASE("photon publisher: a frame the profile cannot carry is skipped whole and logged once") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.startedCamera(setup);
    rig.fake.advanceUs(1000);
    sdk::VisionFrame bad = frameFor(setup, {1, 2}, 1);
    bad.multiTag.tagIds = {40000};  // an int16 list: no such tag id can be written
    const std::size_t before = rig.nt.sets.size();
    camera->publish(bad);
    camera->publish(bad);
    CHECK(rig.nt.sets.size() == before);  // not even a heartbeat: the camera does not claim a frame it cannot deliver
    CHECK(camera->robotState().framesPublished == 0);
    CHECK(rig.log.count(sdk::LogLevel::Warn, "cannot be encoded") == 1);

    camera->publish(frameFor(setup, {1, 2}, 3));
    CHECK(camera->robotState().framesPublished == 1);
    CHECK(rig.fake.history(kTable + "rawBytes").size() == 1);
}

TEST_CASE("photon publisher: frames of another camera, before start or without ntcore are ignored") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    const auto camera = rig.camera(setup);
    camera->publish(frameFor(setup, {1, 2}, 1));  // not started
    CHECK(rig.nt.sets.empty());

    camera->start();
    const std::size_t afterStart = rig.nt.sets.size();
    sdk::VisionFrame other = frameFor(setup, {1, 2}, 2);
    other.cameraId = "rear";
    camera->publish(other);
    CHECK(rig.nt.sets.size() == afterStart);

    rig.fake.setAvailable(false);
    camera->publish(frameFor(setup, {1, 2}, 3));
    rig.fake.setAvailable(true);
    CHECK(rig.nt.sets.size() == afterStart);
    CHECK(camera->robotState().framesPublished == 0);
}

// ---- what the robot writes -----------------------------------------------------------------------------------

TEST_CASE("photon publisher: the robot's requests are echoed into the state topics and reported") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    rig.fake.robotSetInteger(kTable + "pipelineIndexRequest", 0, 500);  // the robot constructor made its topics
    rig.fake.robotSetBoolean(kTable + "driverModeRequest", false, 500);
    rig.fake.robotPublish(kTable + "fpsLimitRequest", NtType::Integer);
    rig.fake.robotPublish("/photonvision/ledModeRequest", NtType::Integer);
    const auto camera = rig.startedCamera(setup);

    camera->poll();
    sdk::CameraRobotState state = camera->robotState();
    CHECK(state.present);
    CHECK(state.pipelineIndex == 0);  // requests made before start are read at the first poll
    CHECK(state.driverMode == false);
    CHECK_FALSE(state.fpsLimit.has_value());
    CHECK_FALSE(state.ledMode.has_value());
    CHECK(state.ntName == "photon-front");
    CHECK(state.cameraId == "front");
    CHECK(state.vendor == sdk::CameraVendor::PhotonVision);
    CHECK(state.publishing);
    CHECK(state.status.empty());

    rig.fake.advanceUs(20'000);
    rig.fake.robotSetInteger(kTable + "pipelineIndexRequest", 1, rig.fake.nowUs());
    rig.fake.robotSetBoolean(kTable + "driverModeRequest", true, rig.fake.nowUs());
    rig.fake.robotSetInteger(kTable + "fpsLimitRequest", 15, rig.fake.nowUs());
    rig.fake.robotSetInteger("/photonvision/ledModeRequest", 2, rig.fake.nowUs());
    camera->poll();

    CHECK(number(rig.fake, kTable + "pipelineIndexState") == 1.0);
    CHECK(rig.fake.latest(kTable + "driverMode")->type == NtType::Boolean);
    CHECK(number(rig.fake, kTable + "driverMode") == 1.0);
    CHECK(number(rig.fake, kTable + "fpsLimit") == 15.0);
    CHECK(number(rig.fake, "/photonvision/ledModeState") == 2.0);
    CHECK(rig.fake.latest(kTable + "pipelineIndexState")->timeUs == rig.fake.nowUs());
    state = camera->robotState();
    CHECK(state.pipelineIndex == 1);
    CHECK(state.driverMode == true);
    CHECK(state.fpsLimit == 15.0);
    CHECK(state.ledMode == 2);
}

TEST_CASE("photon publisher: several requests in one poll leave one echo, a negative pipeline index is ignored") {
    Rig rig;
    const auto camera = rig.startedCamera();
    camera->poll();
    for (const int index : {1, 2, 3}) {
        rig.fake.advanceUs(1000);
        rig.fake.robotSetInteger(kTable + "pipelineIndexRequest", index, rig.fake.nowUs());
    }
    camera->poll();
    const auto& state = rig.fake.history(kTable + "pipelineIndexState");
    REQUIRE(state.size() == 2);  // the initial 0 and the final 3
    CHECK(state.back().number == 3.0);

    rig.fake.advanceUs(1000);
    rig.fake.robotSetInteger(kTable + "pipelineIndexRequest", -5, rig.fake.nowUs());
    camera->poll();
    CHECK(rig.fake.history(kTable + "pipelineIndexState").size() == 2);
    CHECK(camera->robotState().pipelineIndex == 3);  // PhotonVision ignores an index below 0
}

TEST_CASE("photon publisher: the robot uses the camera once its PhotonCamera created a request topic") {
    Rig rig;
    const auto camera = rig.startedCamera();
    camera->poll();
    CHECK_FALSE(camera->robotState().present);
    CHECK_FALSE(camera->robotState().pipelineIndex.has_value());

    rig.fake.robotPublish(kTable + "driverModeRequest", NtType::Boolean);  // a topic without a value is enough
    camera->poll();
    CHECK(camera->robotState().present);
    CHECK_FALSE(camera->robotState().driverMode.has_value());
    camera->poll();
    CHECK(camera->robotState().present);  // and it stays so

    // The root LED request alone does not say that THIS camera is used.
    Rig other;
    other.fake.robotPublish("/photonvision/ledModeRequest", NtType::Integer);
    const auto lonely = other.startedCamera();
    lonely->poll();
    CHECK_FALSE(lonely->robotState().present);
}

TEST_CASE("photon publisher: a camera in driver mode keeps reporting, with empty results") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    rig.fake.robotPublish(kTable + "driverModeRequest", NtType::Boolean);
    const auto camera = rig.startedCamera(setup);
    rig.fake.advanceUs(1000);
    camera->publish(frameFor(setup, {1, 2, 3}, 1));
    CHECK(latestResult(rig.fake).find("targets")->children().size() == 3);

    rig.fake.robotSetBoolean(kTable + "driverModeRequest", true, rig.fake.nowUs());
    camera->poll();
    rig.fake.advanceUs(1000);
    camera->publish(frameFor(setup, {1, 2, 3}, 2));
    const PhotonValue driver = latestResult(rig.fake);
    CHECK(driver.find("targets")->children().empty());
    CHECK(driver.find("multitagResult")->isAbsent());
    CHECK(driver.find("metadata")->find("sequenceID")->intValue() == 2);
    CHECK(number(rig.fake, kTable + "hasTarget") == 0.0);
    CHECK(camera->robotState().framesPublished == 2);

    rig.fake.robotSetBoolean(kTable + "driverModeRequest", false, rig.fake.nowUs());
    camera->poll();
    rig.fake.advanceUs(1000);
    camera->publish(frameFor(setup, {1, 2, 3}, 3));
    CHECK(latestResult(rig.fake).find("targets")->children().size() == 3);
}

// ---- cameras that must not publish ---------------------------------------------------------------------------

TEST_CASE(
    "photon publisher: an unknown message UUID refuses the camera: nothing is published, the requests are still read") {
    for (const char* uuid : {"deadbeefdeadbeefdeadbeefdeadbeef", "unverified"}) {
        CAPTURE(uuid);
        Rig rig;
        sdk::CameraSetup setup = Rig::setup();
        setup.photonMessageUuid = uuid;
        rig.fake.robotSetInteger(kTable + "pipelineIndexRequest", 2, 500);
        const auto camera = rig.startedCamera(setup);
        camera->poll();

        sdk::CameraRobotState state = camera->robotState();
        CHECK(state.status == camera_status::kPhotonUuidUnsupported);
        CHECK_FALSE(state.publishing);
        CHECK(state.present);
        CHECK(state.pipelineIndex == 2);  // still reported
        CHECK(rig.log.count(sdk::LogLevel::Warn, std::string(uuid)) == 1);

        camera->publish(frameFor(setup, {1, 2, 3}, 1));
        camera->poll();
        CHECK(camera->robotState().framesPublished == 0);
        for (const std::string& name : topicNames(rig.fake, "/photonvision/")) {
            CAPTURE(name);
            CHECK(name == kTable + "pipelineIndexRequest");  // the only topic is the robot's own
        }
        CHECK(rig.nt.publishes.empty());  // not even the version, the LED state or the heartbeat
        CHECK(rig.nt.sets.empty());
    }
}

TEST_CASE(
    "photon publisher: a result topic that already has a publisher (a robot-side PhotonCameraSim) is left alone") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    rig.fake.robotPublish(kTable + "rawBytes", NtType::Raw, std::string("photonstruct:PhotonPipelineResult:") + kUuid);
    rig.fake.robotSetBoolean(kTable + "driverModeRequest", true, 500);
    const auto camera = rig.startedCamera(setup);
    camera->poll();

    const sdk::CameraRobotState state = camera->robotState();
    CHECK(state.status == camera_status::kForeignPublisher);
    CHECK_FALSE(state.publishing);
    CHECK(state.driverMode == true);  // the requests are still read
    CHECK(rig.log.count(sdk::LogLevel::Warn, "rawBytes") == 1);

    camera->publish(frameFor(setup, {1, 2, 3}, 1));
    CHECK(rig.nt.publishes.empty());
    CHECK(rig.nt.sets.empty());
    CHECK(rig.fake.history(kTable + "rawBytes").empty());
    CHECK_FALSE(rig.fake.exists("/photonvision/version"));
    CHECK_FALSE(rig.fake.exists(kTable + "heartbeat"));
}

TEST_CASE("photon publisher: ntcore that is not there yet is reported, and start works when it is") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    rig.fake.setAvailable(false);
    const auto camera = rig.camera(setup);
    camera->start();
    CHECK(camera->robotState().status == camera_status::kNtcoreUnavailable);
    CHECK_FALSE(camera->robotState().publishing);
    camera->poll();
    camera->publish(frameFor(setup, {1, 2}, 1));
    CHECK(camera->robotState().framesPublished == 0);

    rig.fake.setAvailable(true);
    camera->start();
    CHECK(camera->robotState().status.empty());
    CHECK(camera->robotState().publishing);
    CHECK(rig.fake.exists(kTable + "rawBytes"));
    camera->start();  // a second start is harmless
    CHECK(rig.nt.publishOf(kTable + "rawBytes") != nullptr);
    CHECK(std::count_if(rig.nt.publishes.begin(), rig.nt.publishes.end(),
                        [](const RecordingNt::Publish& p) { return p.topic == kTable + "rawBytes"; }) == 1);
}

// ---- several cameras, the vendor -----------------------------------------------------------------------------

TEST_CASE("photon publisher: the root topics are created once however many cameras there are") {
    Rig rig;
    const sdk::CameraSetup front = Rig::setup("photon-front", "front");
    const sdk::CameraSetup rear = Rig::setup("photon-rear", "rear");
    const auto a = rig.startedCamera(front);
    const auto b = rig.startedCamera(rear);

    CHECK(rig.fake.publisherCount("/photonvision/version") == 1);
    CHECK(rig.fake.publisherCount("/photonvision/ledModeState") == 1);
    CHECK(rig.fake.history("/photonvision/version").size() == 1);
    CHECK(rig.fake.publisherCount("/photonvision/photon-front/rawBytes") == 1);
    CHECK(rig.fake.publisherCount("/photonvision/photon-rear/rawBytes") == 1);

    // One LED request serves both cameras and is echoed once.
    rig.fake.robotSetInteger("/photonvision/ledModeRequest", 3, rig.fake.nowUs());
    a->poll();
    b->poll();
    CHECK(rig.fake.history("/photonvision/ledModeState").size() == 2);  // -1, then 3
    CHECK(number(rig.fake, "/photonvision/ledModeState") == 3.0);
    CHECK(a->robotState().ledMode == 3);
    CHECK(b->robotState().ledMode == 3);

    // Frames only touch their own camera's topics.
    rig.fake.advanceUs(1000);
    a->publish(frameFor(front, {1, 2, 3}, 1));
    CHECK(rig.fake.history("/photonvision/photon-front/rawBytes").size() == 1);
    CHECK(rig.fake.history("/photonvision/photon-rear/rawBytes").empty());
    b->publish(frameFor(rear, {1}, 1));
    CHECK(rig.fake.history("/photonvision/photon-rear/rawBytes").size() == 1);
    CHECK(rig.fake.history("/photonvision/version").size() == 1);
}

TEST_CASE("photon publisher: a refused camera creates no root topics, a working neighbour still does") {
    Rig rig;
    sdk::CameraSetup refused = Rig::setup("photon-front", "front");
    refused.photonMessageUuid = "deadbeefdeadbeefdeadbeefdeadbeef";
    const auto a = rig.startedCamera(refused);
    CHECK_FALSE(rig.fake.exists("/photonvision/version"));
    CHECK_FALSE(rig.fake.exists("/photonvision/ledModeState"));

    const auto b = rig.startedCamera(Rig::setup("photon-rear", "rear"));
    CHECK(rig.fake.exists("/photonvision/version"));
    CHECK(rig.fake.exists("/photonvision/ledModeState"));
    CHECK_FALSE(rig.fake.exists("/photonvision/photon-front/heartbeat"));

    // An LED request that arrived before the root topics existed is still what ledModeState starts with.
    Rig late;
    late.fake.robotSetInteger("/photonvision/ledModeRequest", 1, 500);
    sdk::CameraSetup first = Rig::setup("photon-front", "front");
    first.photonMessageUuid = "unverified";
    const auto c = late.startedCamera(first);
    c->poll();  // read (but nobody publishes yet)
    const auto d = late.startedCamera(Rig::setup("photon-rear", "rear"));
    CHECK(number(late.fake, "/photonvision/ledModeState") == 1.0);
}

TEST_CASE(
    "photon vendor: no UUID means the newest profile, an explicit UUID is honoured, another vendor is not served") {
    Rig rig;
    CHECK(rig.vendor->vendor() == sdk::CameraVendor::PhotonVision);
    CHECK(rig.vendor->topicPrefix() == "/photonvision/");

    sdk::CameraSetup explicitUuid = Rig::setup("a", "a");
    explicitUuid.photonMessageUuid = kUuid;
    const auto a = rig.startedCamera(explicitUuid);
    const auto b = rig.startedCamera(Rig::setup("b", "b"));
    CHECK(rig.fake.typeString("/photonvision/a/rawBytes") == rig.fake.typeString("/photonvision/b/rawBytes"));

    sdk::CameraSetup limelight = Rig::setup("limelight", "ll");
    limelight.vendor = sdk::CameraVendor::Limelight;
    CHECK(rig.vendor->createPublisher(rig.nt, rig.log, limelight) == nullptr);
}

TEST_CASE("photon vendor: a camera without a network name uses its id") {
    Rig rig;
    sdk::CameraSetup setup = Rig::setup("", "front");
    const auto camera = rig.startedCamera(setup);
    CHECK(rig.fake.exists("/photonvision/front/rawBytes"));
    CHECK(camera->setup().ntName == "front");
    CHECK(camera->robotState().ntName == "front");
}

TEST_CASE("photon vendor: a camera rebuilt for the same table continues its heartbeat and timestamps") {
    Rig rig;
    const sdk::CameraSetup setup = Rig::setup();
    {
        const auto first = rig.startedCamera(setup);
        for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
            rig.fake.advanceUs(33'000);
            first->publish(frameFor(setup, {1, 2}, sequence));
        }
    }
    const std::int64_t lastPublishUs = rig.fake.latest(kTable + "heartbeat")->timeUs;
    rig.fake.setNowUs(lastPublishUs -
                      10'000);  // even a clock that went back must not make the new camera's values stale

    const auto rebuilt =
        rig.startedCamera(setup);  // a reconnect brought a new setup: the bridge builds the camera again
    CHECK(number(rig.fake, kTable + "heartbeat") == 3.0);  // not reset to 0
    CHECK(rig.fake.history(kTable + "heartbeat").back().timeUs >= lastPublishUs);
    CHECK(rig.fake.publisherCount("/photonvision/version") == 1);  // the root topics stay shared

    rig.fake.advanceUs(1000);
    rebuilt->publish(frameFor(setup, {1, 2}, 1));  // the engine's sequence may start over: the heartbeat does not
    CHECK(number(rig.fake, kTable + "heartbeat") == 4.0);
    CHECK(rig.fake.latest(kTable + "rawBytes")->timeUs >= lastPublishUs);
}

// ---- discovery -----------------------------------------------------------------------------------------------

TEST_CASE("photon vendor: discovery finds the cameras whose request topics the ROBOT created, names verbatim") {
    Rig rig;
    rig.fake.robotPublish("/photonvision/photon-front/pipelineIndexRequest", NtType::Integer);
    rig.fake.robotPublish("/photonvision/photon-front/driverModeRequest", NtType::Boolean);
    rig.fake.robotPublish("/photonvision/photon-front/fpsLimitRequest", NtType::Integer);
    rig.fake.robotPublish("/photonvision/Arducam OV9281 #2 (left)/driverModeRequest", NtType::Boolean);
    rig.fake.robotPublish("/photonvision/cam.with-odd_chars/fpsLimitRequest", NtType::Integer);
    rig.fake.robotPublish("/photonvision/\xC3\x84rducam \xC2\xB1\xCF\x80/pipelineIndexRequest",
                          NtType::Integer);                                                 // UTF-8
    rig.fake.robotPublish("/photonvision/team/cam/pipelineIndexRequest", NtType::Integer);  // a nickname with a '/'
    // Things that are not a camera the robot uses:
    rig.fake.robotPublish("/photonvision/ledModeRequest", NtType::Integer);  // root, shared
    rig.fake.robotPublish("/photonvision/ledModeState", NtType::Integer);
    rig.fake.robotPublish("/photonvision/version", NtType::String);
    rig.fake.robotPublish("/photonvision/other/rawBytes", NtType::Raw);
    rig.fake.robotPublish("/photonvision/other/pipelineIndexState",
                          NtType::Integer);  // our own state topics do not count
    rig.fake.robotPublish("/photonvision/.timesync/host", NtType::Raw);
    rig.fake.robotPublish("/photonvision//pipelineIndexRequest", NtType::Integer);  // no camera name
    rig.fake.robotPublish("/limelight/pipeline", NtType::Double);

    std::vector<NtTopicInfo> topics;
    rig.fake.topics(rig.vendor->topicPrefix(), topics);
    std::vector<std::string> names;
    rig.vendor->discoverRobotCameras(topics, names);

    std::vector<std::string> expected = {"Arducam OV9281 #2 (left)", "cam.with-odd_chars", "photon-front", "team/cam",
                                         "\xC3\x84rducam \xC2\xB1\xCF\x80"};
    std::sort(names.begin(), names.end());
    std::sort(expected.begin(), expected.end());
    CHECK(names == expected);

    // Names already known are not repeated, and the call appends.
    std::vector<std::string> known = {"photon-front", "unrelated"};
    rig.vendor->discoverRobotCameras(topics, known);
    CHECK(known.size() == 2 + expected.size() - 1);
    CHECK(std::count(known.begin(), known.end(), "photon-front") == 1);
}

TEST_CASE("photon vendor: a camera we publish ourselves is not discovered as one the robot uses") {
    Rig rig;
    const auto camera = rig.startedCamera();  // publishes /photonvision/photon-front/..., never a *Request topic
    std::vector<NtTopicInfo> topics;
    rig.fake.topics(rig.vendor->topicPrefix(), topics);
    REQUIRE_FALSE(topics.empty());
    std::vector<std::string> names;
    rig.vendor->discoverRobotCameras(topics, names);
    CHECK(names.empty());

    rig.fake.robotPublish(kTable + "pipelineIndexRequest", NtType::Integer);  // now the robot made one
    topics.clear();
    rig.fake.topics(rig.vendor->topicPrefix(), topics);
    rig.vendor->discoverRobotCameras(topics, names);
    CHECK(names == std::vector<std::string>{"photon-front"});
}

}  // namespace
