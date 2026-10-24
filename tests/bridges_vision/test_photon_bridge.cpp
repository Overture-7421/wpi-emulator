#include <cmath>
#include <string>

#include "vision_test_rig.h"
#include "wpiemu/bridges/vision/photon_encoding.h"

using namespace vision_test;
using doctest::Approx;
using vis::ntBool;
using vis::ntInt;

namespace {

const std::string kCam = "/photonvision/front";

constexpr std::size_t kMetadataBytes = 32;
constexpr std::size_t kTargetBytes = 294;  // 4 f64 + 3 x 4 + 2 Transform3d + f64 + 2 corner lists of 4

const vis::NtBytes& rawBytes(const Rig& rig, const std::string& cam = kCam) {
    static const vis::NtBytes kEmpty;
    const vis::NtBytes* b = rig.nt->lastAs<vis::NtBytes>(cam + "/rawBytes");
    return b != nullptr ? *b : kEmpty;
}

void configurePhoton(Rig& rig, std::string version = {}) {
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::PhotonVision, "front")}, std::move(version)));
}

}  // namespace

TEST_CASE("vision photon: topics are created under photonvision slash camera name") {
    Rig rig;
    configurePhoton(rig);
    rig.poll(0);
    for (const char* topic : {"/rawBytes", "/pipelineIndexState", "/driverMode", "/fpsLimit", "/heartbeat",
                              "/cameraIntrinsics", "/cameraDistortion", "/hasTarget", "/latencyMillis"}) {
        INFO(topic);
        CHECK(rig.nt->has(kCam + topic));
    }
    CHECK(*rig.nt->lastAs<std::string>("/photonvision/version") == "v2026.3.4");
    CHECK(rig.nt->has("/photonvision/ledModeState"));
    CHECK(rig.nt->topic(kCam + "/rawBytes")->typeString ==
          "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4");
    // PhotonLib's version check reads this JSON string property of the topic.
    CHECK(rig.nt->topic(kCam + "/rawBytes")->properties.at("message_uuid") == "\"4b2ff16a964b5e2bf04be0c1454d91c4\"");
    // The raw topic exists with a first empty result so PhotonLib's version check can read its type string.
    CHECK(rawBytes(rig).size() == kMetadataBytes + 2);
}

TEST_CASE("vision photon: intrinsics are a 3x3 row-major matrix and distortion is zero") {
    Rig rig;
    configurePhoton(rig);
    rig.poll(0);
    const std::vector<double>& k = rig.nt->lastDoubles(kCam + "/cameraIntrinsics");
    REQUIRE(k.size() == 9);
    CHECK(k[0] == Approx(640.0));              // fx = (w/2) / tan(hFov/2), hFov = 90 deg
    CHECK(k[4] == Approx(400.0 / std::tan(30.0 * kPi / 180.0)));
    CHECK(k[2] == 640.0);
    CHECK(k[5] == 400.0);
    CHECK(k[8] == 1.0);
    CHECK(k[1] == 0.0);
    CHECK(rig.nt->lastDoubles(kCam + "/cameraDistortion") == std::vector<double>(8, 0.0));
}

TEST_CASE("vision photon: the pinned 2027 client gets its own schema and nanosecond capture age") {
    Rig rig;
    configurePhoton(rig, "dev-v2027.0.0-alpha-2-86-gda55d785");
    rig.nt->nowTimeUs = 9'000'000;
    rig.poll(0);
    rig.apply(twoTagFrame(), 1'040'000);
    CHECK(rig.nt->topic(kCam + "/rawBytes")->typeString ==
          "photonstruct:PhotonPipelineResult:fd353b11f3a818e9c5b457c7dc5a8f5c");
    CHECK(rig.nt->topic(kCam + "/rawBytes")->properties.at("message_uuid") ==
          "\"fd353b11f3a818e9c5b457c7dc5a8f5c\"");
    CHECK(readI64(rawBytes(rig), 8) == 8'960'000'000);
    CHECK(readI64(rawBytes(rig), 16) == 9'000'000'000);
    CHECK(rawBytes(rig)[32] == 2);
    rig.nt->setRobotValue(kCam + "/enabledRequest", vis::ntBool(false));
    rig.poll(1'050'000);
    rig.apply(twoTagFrame(), 1'080'000);
    CHECK_FALSE(*rig.nt->lastAs<bool>(kCam + "/enabled"));
    CHECK(rawBytes(rig)[32] == 0);
    rig.nt->setRobotValue(kCam + "/enabledRequest", vis::ntBool(true));
    rig.poll(1'090'000);
    rig.apply(twoTagFrame(), 1'120'000);
    CHECK(*rig.nt->lastAs<bool>(kCam + "/enabled"));
    CHECK(rawBytes(rig)[32] == 2);
    rig.photon->reset();
    CHECK(rawBytes(rig)[32] == 0);
}

TEST_CASE("vision photon: a frame publishes the pipeline result with latency compensated metadata") {
    Rig rig;
    configurePhoton(rig);
    rig.nt->nowTimeUs = 9'000'000;
    rig.apply(twoTagFrame(), 1'040'000);

    const vis::NtBytes& b = rawBytes(rig);
    REQUIRE(b.size() == kMetadataBytes + 1 + 2 * kTargetBytes + 1 + 56 * 2 + 24 + 1 + 4);
    CHECK(readI64(b, 0) == 7);                    // sequenceID
    CHECK(readI64(b, 8) == 9'000'000 - 40'000);   // capture = publish - (publish - capture)
    CHECK(readI64(b, 16) == 9'000'000);           // publish
    CHECK(readI64(b, 24) == 0);                   // timeSinceLastPong
    CHECK(b[32] == 2);                            // two targets, the larger one (id 7) first
    CHECK(readF64(b, 33) == Approx(5.0));         // yaw
    CHECK(readF64(b, 41) == Approx(-3.0));        // pitch
    CHECK(readF64(b, 49) == Approx(2.0));         // area
    CHECK(readF64(b, 57) == 0.0);                 // skew
    CHECK(b[65] == 7);                            // fiducialId (little endian int32)
    CHECK(b[66] == 0);
    CHECK(b[69] == 0xff);                         // objDetectId = -1
    CHECK(readF64(b, 33 + 44) == Approx(2.0));    // bestCameraToTarget.x
    CHECK(readF64(b, 33 + 52) == Approx(0.2));    // y
    CHECK(rig.nt->topic(kCam + "/rawBytes")->last().timestampUs == 9'000'000);

    // Multi-tag result: estimated field->camera transform and the ids used.
    const std::size_t multi = kMetadataBytes + 1 + 2 * kTargetBytes;
    CHECK(b[multi] == 1);
    // Robot at (2,3) facing +Y, camera 0.3 m ahead of the robot origin: the camera is at (2, 3.3, 0.5).
    CHECK(readF64(b, multi + 1) == Approx(2.0));
    CHECK(readF64(b, multi + 9) == Approx(3.3));
    CHECK(readF64(b, multi + 17) == Approx(0.5));
    const std::size_t ids = multi + 1 + 2 * 56 + 24;
    CHECK(b[ids] == 2);
    CHECK(b[ids + 1] == 3);
    CHECK(b[ids + 3] == 7);
}

TEST_CASE("vision photon: a single tag has no multitag result and a no-tag frame is empty") {
    Rig rig;
    configurePhoton(rig);
    rig.apply({makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})}, 1'040'000);
    const vis::NtBytes single = rawBytes(rig);
    REQUIRE(single.size() == kMetadataBytes + 1 + kTargetBytes + 1);
    CHECK(single.back() == 0);
    CHECK(*rig.nt->lastAs<bool>(kCam + "/hasTarget"));
    CHECK(rig.nt->lastDouble(kCam + "/targetYaw") == Approx(5.0));
    CHECK(rig.nt->lastDouble(kCam + "/latencyMillis") == Approx(40.0));

    rig.apply({makeFrame(0, {})}, 1'100'000);
    const vis::NtBytes& none = rawBytes(rig);
    REQUIRE(none.size() == kMetadataBytes + 2);
    CHECK(none[32] == 0);
    CHECK(none[33] == 0);
    CHECK_FALSE(*rig.nt->lastAs<bool>(kCam + "/hasTarget"));
}

TEST_CASE("vision photon: every frame is a new raw sample") {
    Rig rig;
    configurePhoton(rig);
    rig.apply({makeFrame(0, {})}, 1'040'000);
    rig.apply({makeFrame(0, {})}, 1'070'000);
    CHECK(rig.nt->topic(kCam + "/rawBytes")->history.size() == 3);  // initial + two frames
}

TEST_CASE("vision photon: robot requests are mirrored and driver mode publishes empty results") {
    Rig rig;
    configurePhoton(rig);
    rig.poll(0);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/pipelineIndexState") == 0);
    CHECK_FALSE(*rig.nt->lastAs<bool>(kCam + "/driverMode"));
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/fpsLimit") == -1);

    rig.nt->setRobotValue(kCam + "/pipelineIndexRequest", ntInt(2));
    rig.nt->setRobotValue(kCam + "/driverModeRequest", ntBool(true));
    rig.nt->setRobotValue(kCam + "/fpsLimitRequest", ntInt(15));
    rig.nt->setRobotValue("/photonvision/ledModeRequest", ntInt(1));
    rig.poll(10'000);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/pipelineIndexState") == 2);
    CHECK(*rig.nt->lastAs<bool>(kCam + "/driverMode"));
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/fpsLimit") == 15);
    CHECK(*rig.nt->lastAs<std::int64_t>("/photonvision/ledModeState") == 1);

    rig.apply({makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})}, 1'040'000);
    CHECK(rawBytes(rig).size() == kMetadataBytes + 2);
    CHECK_FALSE(*rig.nt->lastAs<bool>(kCam + "/hasTarget"));

    rig.nt->setRobotValue(kCam + "/driverModeRequest", ntBool(false));
    rig.poll(20'000);
    rig.apply({makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})}, 1'070'000);
    CHECK(rawBytes(rig).size() == kMetadataBytes + 1 + kTargetBytes + 1);
}

TEST_CASE("vision photon: heartbeat increments at 10 Hz of simulated time") {
    Rig rig;
    configurePhoton(rig);
    rig.poll(0);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/heartbeat") == 1);
    rig.poll(50'000);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/heartbeat") == 1);
    rig.poll(100'000);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/heartbeat") == 2);
    rig.poll(300'000);
    CHECK(*rig.nt->lastAs<std::int64_t>(kCam + "/heartbeat") == 3);
}

TEST_CASE("vision photon: no photon cameras and an unverified version is not an error") {
    Rig rig;
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::Limelight, "")}, "v1.0.0"));
    rig.poll(0);
    CHECK(rig.logger.count(sdk::LogLevel::Error) == 0);
}

TEST_CASE("vision photon: limelight cameras and unknown camera indices are ignored") {
    Rig rig;
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::Limelight, "ll"), makeCamera(sdk::CameraKind::PhotonVision, "front")}));
    rig.poll(0);
    CHECK_FALSE(rig.nt->has("/photonvision/ll/rawBytes"));
    CHECK(rig.nt->has(kCam + "/rawBytes"));
    const std::size_t before = rig.nt->topic(kCam + "/rawBytes")->history.size();
    rig.apply({makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})}), makeFrame(9, {})}, 1'040'000);
    CHECK(rig.nt->topic(kCam + "/rawBytes")->history.size() == before);
    rig.apply({makeFrame(1, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})}, 1'040'000);
    CHECK(rig.nt->topic(kCam + "/rawBytes")->history.size() == before + 1);
}

TEST_CASE("vision photon: an unavailable ntcore is skipped quietly and retried") {
    Rig rig;
    configurePhoton(rig);
    rig.nt->isAvailable = false;
    rig.poll(0);
    rig.apply(twoTagFrame(), 1'040'000);
    CHECK(rig.nt->topics.empty());
    CHECK(rig.logger.count(sdk::LogLevel::Warn) == 0);
    CHECK(rig.logger.count(sdk::LogLevel::Error) == 0);

    rig.nt->isAvailable = true;
    rig.poll(5'000);
    CHECK(rig.nt->has(kCam + "/rawBytes"));
    rig.apply(twoTagFrame(), 1'040'000);
    CHECK(rawBytes(rig).size() > kMetadataBytes + 2);
}

TEST_CASE("vision photon: reset publishes an empty result") {
    Rig rig;
    configurePhoton(rig);
    rig.apply(twoTagFrame(), 1'040'000);
    REQUIRE(rawBytes(rig).size() > kMetadataBytes + 2);
    rig.nt->nowTimeUs = 6'000'000;
    rig.photon->reset();
    const vis::NtBytes& b = rawBytes(rig);
    REQUIRE(b.size() == kMetadataBytes + 2);
    CHECK(readI64(b, 16) == 6'000'000);
    CHECK(b[32] == 0);
    CHECK_FALSE(*rig.nt->lastAs<bool>(kCam + "/hasTarget"));
}

TEST_CASE("vision photon: stop releases the camera topics") {
    Rig rig;
    configurePhoton(rig);
    rig.poll(0);
    REQUIRE(rig.nt->has(kCam + "/rawBytes"));
    rig.photon->stop();
    CHECK_FALSE(rig.nt->has(kCam + "/rawBytes"));
}

TEST_CASE("vision photon: an unverified PhotonLib version warns once and still publishes with the default schema") {
    Rig rig;
    configurePhoton(rig, "v2026.99.9");
    CHECK(rig.logger.count(sdk::LogLevel::Warn) == 1);
    CHECK(rig.logger.count(sdk::LogLevel::Error) == 0);
    configurePhoton(rig, "v2026.99.9");   // a reconfigure with the same version does not repeat the warning
    CHECK(rig.logger.count(sdk::LogLevel::Warn) == 1);
    rig.poll(0);
    rig.apply(twoTagFrame(), 1'040'000);
    CHECK(rig.nt->topic(kCam + "/rawBytes")->typeString == std::string("photonstruct:PhotonPipelineResult:") +
                                                              std::string(vis::photonPipelineResultUuid(vis::photonDefaultSchema())));
    CHECK(rawBytes(rig)[32] == 2);
}
