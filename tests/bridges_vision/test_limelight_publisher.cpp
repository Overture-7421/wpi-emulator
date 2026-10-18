#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "test_limelight_support.h"
#include "wpiemu/bridges/vision/camera_publisher.h"

namespace {

using namespace wpiemu::tests::limelight;
namespace status = wpiemu::bridges::vision::camera_status;
using wpiemu::bridges::vision::NtValue;
using wpiemu::sdk::CameraRobotState;
using wpiemu::sdk::VisionFrame;

constexpr std::int64_t kNow = 2'000'000;       // the clock at the first frame
constexpr std::int64_t kLatency = 32'000;      // rearFrame's latency
constexpr std::int64_t kCapture = kNow - kLatency;

double elem(const std::vector<double>& a, std::size_t i) { return a.at(i); }

// The rear Limelight at (2, 4) seeing tags 31 (primary) and 32.
VisionFrame bothTags(const wpiemu::sdk::Pose3& robot = robotAt(2.0, 4.0, 0.0)) { return rearFrame({31, 32}, robot); }

// ---- topics, types, defaults ------------------------------------------------------------------------------------

TEST_CASE("limelight publisher: start creates every per-frame topic with the device's type and a sane default") {
    Rig rig;
    rig.camera->start();
    FakeNtApi& nt = rig.nt;

    for (const std::string_view key : ll::kPerFrameKeys) CHECK_MESSAGE(nt.exists(topic(key)), key);
    CHECK(nt.exists(topic(ll::kHw)));

    for (const std::string_view key : {"tv", "tx", "ty", "txnc", "tync", "ta", "tl", "cl", "tid", "hb", "getpipe"})
        CHECK_MESSAGE(nt.typeString(topic(key)) == "double", key);
    CHECK(nt.typeString(topic("getpipetype")) == "string");
    for (const std::string_view key : kBotposeKeys) CHECK_MESSAGE(nt.typeString(topic(key)) == "double[]", key);
    for (const std::string_view key : {"rawfiducials", "camerapose_robotspace", "hw"})
        CHECK_MESSAGE(nt.typeString(topic(key)) == "double[]", key);

    CHECK(number(nt, "tv") == 0.0);
    CHECK(number(nt, "tx") == 0.0);
    CHECK(number(nt, "ty") == 0.0);
    CHECK(number(nt, "ta") == 0.0);
    CHECK(number(nt, "tl") == 0.0);
    CHECK(number(nt, "cl") == 0.0);
    CHECK(number(nt, "tid") == -1.0);
    CHECK(number(nt, "hb") == 0.0);
    CHECK(number(nt, "getpipe") == 0.0);
    CHECK(nt.latest(topic("getpipetype"))->text == "pipe_fiducial");
    for (const std::string_view key : kBotposeKeys)
        CHECK_MESSAGE(array(nt, key) == std::vector<double>(11, 0.0), key);
    CHECK(array(nt, "rawfiducials").empty());

    // the believed mount defaults to the configured one, in Limelight robot space
    const std::vector<double> mount = array(nt, "camerapose_robotspace");
    REQUIRE(mount.size() == 6);
    CHECK(mount[0] == doctest::Approx(-0.30).epsilon(1e-6));
    CHECK(mount[1] == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(mount[2] == doctest::Approx(0.45).epsilon(1e-6));
    CHECK(mount[4] == doctest::Approx(25.0).epsilon(1e-6));
    CHECK(std::fabs(mount[5]) == doctest::Approx(180.0).epsilon(1e-6));

    const std::vector<double> hw = array(nt, "hw");
    REQUIRE(hw.size() == ll::kHwSize);
    CHECK(hw[ll::kHwFps] == 30.0);

    const CameraRobotState state = rig.camera->robotState();
    CHECK(state.status.empty());
    CHECK(state.publishing);
    CHECK_FALSE(state.present);  // the robot has not touched the camera
    CHECK(state.framesPublished == 0);
    CHECK(state.ntName == kTable);
    CHECK(state.vendor == wpiemu::sdk::CameraVendor::Limelight);
    CHECK(state.cameraId == "ll-rear");
}

TEST_CASE("limelight publisher: the empty ntName is the table limelight") {
    wpiemu::sdk::CameraSetup setup = rearSetup();
    setup.ntName.clear();
    Rig rig(setup);
    rig.camera->start();
    CHECK(rig.nt.exists("/limelight/tv"));
    CHECK(rig.nt.exists("/limelight/botpose_wpiblue"));
    CHECK(rig.camera->robotState().ntName == "limelight");
}

TEST_CASE("limelight publisher: start is idempotent and publishes nothing but the defaults") {
    Rig rig;
    rig.camera->start();
    const std::size_t accepted = rig.nt.acceptedValues();
    rig.camera->start();
    rig.camera->poll();
    CHECK(rig.nt.acceptedValues() == accepted);
    CHECK(rig.camera->robotState().framesPublished == 0);
}

// ---- one frame --------------------------------------------------------------------------------------------------

TEST_CASE("limelight publisher: a frame sets every per-frame topic exactly once, all with one timestamp") {
    Rig rig;
    rig.camera->start();
    const VisionFrame frame = bothTags();
    rig.publishAt(kNow, frame);

    for (const std::string_view key : ll::kPerFrameKeys) CHECK_MESSAGE(countAt(rig.nt, key, kNow) == 1, key);
    CHECK(rig.nt.history(topic(ll::kHw)).size() == 1);  // static: written at start only

    CHECK(number(rig.nt, "tv") == 1.0);
    CHECK(number(rig.nt, "tid") == static_cast<double>(frame.observations[0].id));
    CHECK(number(rig.nt, "tx") == doctest::Approx(frame.observations[0].yawRad * 180.0 / ll::kPi));
    CHECK(number(rig.nt, "txnc") == number(rig.nt, "tx"));
    CHECK(number(rig.nt, "ty") == doctest::Approx(frame.observations[0].pitchRad * 180.0 / ll::kPi));
    CHECK(number(rig.nt, "tync") == number(rig.nt, "ty"));
    CHECK(number(rig.nt, "ta") == doctest::Approx(frame.observations[0].areaPct));
    CHECK(number(rig.nt, "hb") == 1.0);
    CHECK(number(rig.nt, "getpipe") == 0.0);

    for (const std::string_view key : kBotposeKeys)
        CHECK_MESSAGE(array(rig.nt, key).size() == ll::kBotposeHeaderSize + 2 * ll::kBotposeTagStride, key);
    CHECK(array(rig.nt, "rawfiducials").size() == 2 * ll::kRawFiducialsStride);

    CHECK(rig.camera->robotState().framesPublished == 1);
}

TEST_CASE("limelight publisher: cl and tl split the latency and the capture time is the publish time minus it") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, bothTags());  // latency 32 ms = cl 12 ms + tl 20 ms
    CHECK(number(rig.nt, "cl") == doctest::Approx(12.0));
    CHECK(number(rig.nt, "tl") == doctest::Approx(20.0));
    const std::vector<double> pose = array(rig.nt, "botpose_wpiblue");
    CHECK(elem(pose, ll::kBotposeLatencyMs) == doctest::Approx(32.0));

    // what LimelightHelpers computes: NT value time - latency(ms) = the capture time of the frame
    const std::int64_t valueTimeUs = rig.nt.latest(topic("botpose_wpiblue"))->timeUs;
    const double capture = static_cast<double>(valueTimeUs) - elem(pose, ll::kBotposeLatencyMs) * 1000.0;
    CHECK(capture == doctest::Approx(static_cast<double>(kCapture)));
}

TEST_CASE("limelight publisher: identical frames are all published (keepDuplicates), the timestamp keeps moving") {
    Rig rig;
    rig.camera->start();
    const VisionFrame frame = bothTags();
    rig.publishAt(2'000'000, frame);
    rig.publishAt(2'033'000, frame);
    rig.publishAt(2'066'000, frame);
    // default + three frames, although the three arrays are identical
    CHECK(rig.nt.history(topic("botpose_wpiblue")).size() == 4);
    CHECK(rig.nt.history(topic("botpose_orb_wpiblue")).size() == 4);
    CHECK(rig.nt.history(topic("rawfiducials")).size() == 4);
    CHECK(rig.nt.history(topic("tv")).size() == 4);
    CHECK(rig.nt.latest(topic("botpose_wpiblue"))->timeUs == 2'066'000);
    CHECK(rig.nt.droppedValues() == 0);
    CHECK(number(rig.nt, "hb") == 3.0);
}

TEST_CASE("limelight publisher: the publish time never goes back, whatever the clock does") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(5'000'000, bothTags());
    rig.publishAt(4'000'000, bothTags());  // the clock went backwards: ntcore would drop an older stamp silently
    CHECK(rig.nt.droppedValues() == 0);
    CHECK(rig.nt.latest(topic("tv"))->timeUs == 5'000'000);
    CHECK(rig.nt.history(topic("botpose_wpiblue")).size() == 3);
    CHECK(number(rig.nt, "hb") == 2.0);
    rig.publishAt(5'500'000, bothTags());
    CHECK(rig.nt.latest(topic("tv"))->timeUs == 5'500'000);
}

TEST_CASE("limelight publisher: a frame without tags still publishes: tv 0, no tag, zero arrays, a heartbeat") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, bothTags());
    rig.publishAt(kNow + 33'000, rearFrame({}));
    CHECK(number(rig.nt, "tv") == 0.0);
    CHECK(number(rig.nt, "tid") == -1.0);
    CHECK(number(rig.nt, "tx") == 0.0);
    CHECK(number(rig.nt, "ta") == 0.0);
    CHECK(number(rig.nt, "hb") == 2.0);
    for (const std::string_view key : kBotposeKeys) {
        const std::vector<double> a = array(rig.nt, key);
        REQUIRE_MESSAGE(a.size() == ll::kBotposeHeaderSize, key);
        CHECK_MESSAGE(elem(a, ll::kBotposeTagCount) == 0.0, key);
        CHECK_MESSAGE(elem(a, ll::kBotposeX) == 0.0, key);
        CHECK_MESSAGE(elem(a, ll::kBotposeLatencyMs) == doctest::Approx(32.0), key);
    }
    CHECK(array(rig.nt, "rawfiducials").empty());
    CHECK(rig.nt.latest(topic("tv"))->timeUs == kNow + 33'000);  // and it carries its own time
    CHECK(rig.camera->robotState().framesPublished == 2);
}

// ---- MegaTag1 and the believed mount ----------------------------------------------------------------------------

TEST_CASE("limelight publisher: megatag1 is the exact robot pose when the believed mount is the physical one") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, bothTags());
    const std::vector<double> blue = array(rig.nt, "botpose_wpiblue");
    CHECK(elem(blue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(elem(blue, ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));
    CHECK(elem(blue, ll::kBotposeZ) == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(angleDiffDeg(elem(blue, ll::kBotposeYawDeg), 0.0) < 1e-3);
    CHECK(elem(blue, ll::kBotposeTagCount) == 2.0);

    // a robot that stands elsewhere and is turned: (5, 2) facing 30 degrees
    rig.publishAt(kNow + 33'000, rearFrame({31, 32}, robotAt(5.0, 2.0, 30.0)));
    const std::vector<double> moved = array(rig.nt, "botpose_wpiblue");
    CHECK(elem(moved, ll::kBotposeX) == doctest::Approx(5.0).epsilon(1e-5));
    CHECK(elem(moved, ll::kBotposeY) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(angleDiffDeg(elem(moved, ll::kBotposeYawDeg), 30.0) < 1e-3);
}

TEST_CASE("limelight publisher: the written mount is echoed and skews megatag1 like a wrong mount on hardware") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, bothTags());
    CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));

    // the robot writes the sample mount with 0.10 m too much "forward": [-0.20, 0, 0.45, 0, 25, -180]
    robotSetMount(rig.nt, {-0.20, -0.0, 0.45, -0.0, 25.0, -180.0}, kNow);
    rig.nt.setNowUs(kNow + 1000);
    rig.camera->poll();
    const std::vector<double> echo = array(rig.nt, "camerapose_robotspace");  // at once, not with the next frame
    REQUIRE(echo.size() == 6);
    CHECK(echo[ll::kPose6Forward] == -0.20);
    CHECK(echo[ll::kPose6PitchDeg] == 25.0);
    CHECK(echo[ll::kPose6YawDeg] == -180.0);
    REQUIRE(rig.camera->robotState().cameraPoseOverride.has_value());
    CHECK(rig.camera->robotState().cameraPoseOverride->at(ll::kPose6Forward) == -0.20);

    rig.publishAt(kNow + 33'000, bothTags());
    const std::vector<double> skewed = array(rig.nt, "botpose_wpiblue");
    CHECK(elem(skewed, ll::kBotposeX) == doctest::Approx(2.0 - 0.10).epsilon(1e-5));  // truth - delta, closed form
    CHECK(elem(skewed, ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));
    CHECK(array(rig.nt, "camerapose_robotspace") == std::vector<double>{-0.20, -0.0, 0.45, -0.0, 25.0, -180.0});

    // "forgot the pitch": [-0.30, 0, 0.45, 0, 0, -180] gives truth + (m - Ry(25) m), pitched by 25 degrees
    robotSetMount(rig.nt, {-0.30, -0.0, 0.45, -0.0, 0.0, -180.0}, kNow + 40'000);
    rig.publishAt(kNow + 66'000, bothTags());
    const std::vector<double> tilted = array(rig.nt, "botpose_wpiblue");
    const double c = std::cos(25.0 * kDeg), s = std::sin(25.0 * kDeg);
    CHECK(elem(tilted, ll::kBotposeX) == doctest::Approx(2.0 + (-0.30 - (c * -0.30 + s * 0.45))).epsilon(1e-5));
    CHECK(elem(tilted, ll::kBotposeZ) == doctest::Approx(0.45 - (-s * -0.30 + c * 0.45)).epsilon(1e-5));
    CHECK(elem(tilted, ll::kBotposePitchDeg) == doctest::Approx(25.0).epsilon(1e-4));
}

TEST_CASE("limelight publisher: a mount the robot wrote in its constructor is used by the very first frame") {
    Rig rig;
    robotSetMount(rig.nt, {-0.20, -0.0, 0.45, -0.0, 25.0, -180.0}, 100'000);  // before start(), like a constructor
    rig.camera->start();
    CHECK(array(rig.nt, "camerapose_robotspace") == std::vector<double>{-0.20, -0.0, 0.45, -0.0, 25.0, -180.0});
    CHECK(rig.camera->robotState().present);
    rig.publishAt(kNow, bothTags());
    CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeX) == doctest::Approx(1.90).epsilon(1e-5));
}

TEST_CASE("limelight publisher: the alliance flavours are the same estimate in the red and field-centre frames") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, rearFrame({31, 32}, robotAt(5.0, 2.0, 30.0)));
    const std::vector<double> blue = array(rig.nt, "botpose_wpiblue");
    const std::vector<double> red = array(rig.nt, "botpose_wpired");
    const std::vector<double> centre = array(rig.nt, "botpose");
    CHECK(elem(blue, ll::kBotposeX) == doctest::Approx(5.0).epsilon(1e-5));
    CHECK(elem(red, ll::kBotposeX) == doctest::Approx(ll::kWpiFieldLengthM - 5.0).epsilon(1e-5));
    CHECK(elem(red, ll::kBotposeY) == doctest::Approx(ll::kWpiFieldWidthM - 2.0).epsilon(1e-5));
    CHECK(angleDiffDeg(elem(red, ll::kBotposeYawDeg), 210.0) < 1e-3);  // 30 + 180
    CHECK(elem(centre, ll::kBotposeX) == doctest::Approx(5.0 - ll::kWpiFieldLengthM / 2.0).epsilon(1e-5));
    CHECK(elem(centre, ll::kBotposeY) == doctest::Approx(2.0 - ll::kWpiFieldWidthM / 2.0).epsilon(1e-5));
    CHECK(angleDiffDeg(elem(centre, ll::kBotposeYawDeg), 30.0) < 1e-3);
    // z, roll, pitch and every statistic are shared
    for (const std::size_t i : {ll::kBotposeZ, ll::kBotposeLatencyMs, ll::kBotposeTagCount, ll::kBotposeTagSpanM,
                                ll::kBotposeAvgTagDistM}) {
        CHECK(elem(red, i) == elem(blue, i));
        CHECK(elem(centre, i) == elem(blue, i));
    }
    CHECK(red.size() == blue.size());

    // MegaTag2 has the same three flavours
    robotSetOrientation(rig.nt, 30.0, 1'900'000);
    rig.publishAt(kNow + 33'000, rearFrame({31, 32}, robotAt(5.0, 2.0, 30.0)));
    const std::vector<double> orbBlue = array(rig.nt, "botpose_orb_wpiblue");
    const std::vector<double> orbRed = array(rig.nt, "botpose_orb_wpired");
    CHECK(elem(orbBlue, ll::kBotposeX) == doctest::Approx(5.0).epsilon(1e-5));
    CHECK(elem(orbRed, ll::kBotposeX) == doctest::Approx(ll::kWpiFieldLengthM - 5.0).epsilon(1e-5));
    CHECK(angleDiffDeg(elem(orbRed, ll::kBotposeYawDeg), 210.0) < 1e-3);
}

// ---- MegaTag2 and the robot's orientation ------------------------------------------------------------------------

// The 2-tag rear frame with the robot at (2, 4) facing +X, published at kNow (capture kCapture); MegaTag2's x and y.
struct Mt2 {
    double x, y, z, yaw;
};
Mt2 mt2Of(Rig& rig, const VisionFrame& frame, std::int64_t nowUs = kNow) {
    rig.publishAt(nowUs, frame);
    const std::vector<double> a = array(rig.nt, "botpose_orb_wpiblue");
    return {elem(a, ll::kBotposeX), elem(a, ll::kBotposeY), elem(a, ll::kBotposeZ), elem(a, ll::kBotposeYawDeg)};
}

TEST_CASE("limelight publisher: megatag2 is the truth with the right yaw and reproduces the spike's 45 degree skew") {
    {
        Rig rig;
        rig.camera->start();
        robotSetOrientation(rig.nt, 0.0, 1'900'000);
        const Mt2 right = mt2Of(rig, bothTags());
        CHECK(right.x == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(right.y == doctest::Approx(4.0).epsilon(1e-5));
        CHECK(right.z == doctest::Approx(0.0).epsilon(1e-5));
        CHECK(right.yaw == 0.0);
    }
    {
        Rig rig;
        rig.camera->start();
        robotSetOrientation(rig.nt, 45.0, 1'900'000);  // the sample's joystick axis adds 45 degrees
        const Mt2 wrong = mt2Of(rig, bothTags());
        CHECK(wrong.x == doctest::Approx(1.2368).epsilon(1e-4));  // the spike measured MT2 = 1.237, 5.334, 45.00
        CHECK(wrong.y == doctest::Approx(5.3340).epsilon(1e-4));
        CHECK(wrong.z == doctest::Approx(0.0).epsilon(1e-5));
        CHECK(wrong.yaw == 45.0);
        const auto expected = expectedMegaTag2Xy(kTagX, kTag31Y, 2.0, 4.0, 0.0, 45.0);
        CHECK(wrong.x == doctest::Approx(expected[0]).epsilon(1e-5));
        CHECK(wrong.y == doctest::Approx(expected[1]).epsilon(1e-5));
        // MegaTag1 is not affected by it
        CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));
    }
    {
        Rig rig;  // a robot that really faces 30 degrees and says so
        rig.camera->start();
        robotSetOrientation(rig.nt, 30.0, 1'900'000);
        const Mt2 right = mt2Of(rig, bothTags(robotAt(2.0, 4.0, 30.0)));
        CHECK(right.x == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(right.y == doctest::Approx(4.0).epsilon(1e-5));
        CHECK(right.yaw == 30.0);
    }
}

TEST_CASE("limelight publisher: megatag2 interpolates the robot's orientation to the capture time") {
    Rig rig;
    rig.camera->start();
    // The frame published at 2.000 s was captured at 1.968 s. The robot wrote 30 degrees at 1.953 s and 50 degrees at
    // 1.973 s, so the capture lies 15 ms into the 20 ms between the writes: the device uses 30 + 0.75 * 20 = 45 deg.
    robotSetOrientation(rig.nt, 30.0, 1'953'000);
    robotSetOrientation(rig.nt, 50.0, 1'973'000);
    const Mt2 mid = mt2Of(rig, bothTags());
    CHECK(mid.yaw == doctest::Approx(45.0).epsilon(1e-9));
    CHECK(mid.x == doctest::Approx(1.2368).epsilon(1e-4));  // the same skew as a flat 45 degrees: the spike's numbers
    CHECK(mid.y == doctest::Approx(5.3340).epsilon(1e-4));
}

TEST_CASE("limelight publisher: megatag2 holds the latest write at or before the capture and does not smear a step") {
    {
        Rig rig;  // one write long ago: it holds
        rig.camera->start();
        robotSetOrientation(rig.nt, 45.0, 1'000'000);
        const Mt2 held = mt2Of(rig, bothTags());
        CHECK(held.yaw == 45.0);
        CHECK(held.x == doctest::Approx(1.2368).epsilon(1e-4));
    }
    {
        Rig rig;  // a step written 980 ms after the previous write and after the capture: the frame sees the old value
        rig.camera->start();
        robotSetOrientation(rig.nt, 0.0, 1'000'000);
        robotSetOrientation(rig.nt, 90.0, 1'980'000);
        const Mt2 old = mt2Of(rig, bothTags());
        CHECK(old.yaw == 0.0);
        CHECK(old.x == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(old.y == doctest::Approx(4.0).epsilon(1e-5));
    }
}

TEST_CASE("limelight publisher: megatag2 interpolates the yaw along the shortest arc") {
    Rig rig;
    rig.camera->start();
    robotSetOrientation(rig.nt, 170.0, 1'953'000);
    robotSetOrientation(rig.nt, -170.0, 1'973'000);
    const Mt2 wrapped = mt2Of(rig, bothTags());
    CHECK(wrapped.yaw == doctest::Approx(-175.0).epsilon(1e-6));  // 15 of 20 ms: 170 + 0.75 * 20 = 185 = -175
    const auto expected = expectedMegaTag2Xy(kTagX, kTag31Y, 2.0, 4.0, 0.0, -175.0);
    CHECK(wrapped.x == doctest::Approx(expected[0]).epsilon(1e-5));
    CHECK(wrapped.y == doctest::Approx(expected[1]).epsilon(1e-5));
}

TEST_CASE("limelight publisher: before the first orientation write megatag2 solves with yaw zero") {
    const VisionFrame turned = bothTags(robotAt(2.0, 4.0, 30.0));  // the robot really faces 30 degrees
    const auto expected = expectedMegaTag2Xy(kTagX, kTag31Y, 2.0, 4.0, 30.0, 0.0);

    {
        Rig rig;  // the robot never calls SetRobotOrientation
        rig.camera->start();
        const Mt2 none = mt2Of(rig, turned);
        CHECK(none.yaw == 0.0);
        CHECK(none.x == doctest::Approx(expected[0]).epsilon(1e-5));
        CHECK(none.y == doctest::Approx(expected[1]).epsilon(1e-5));
        CHECK(std::hypot(none.x - 2.0, none.y - 4.0) > 0.5);  // badly wrong: the symptom of a forgotten orientation
        // MegaTag1 still has the vision yaw: the truth
        CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
        CHECK(angleDiffDeg(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeYawDeg), 30.0) < 1e-3);
    }
    {
        Rig rig;  // the first write comes after the capture: the capture still predates it
        rig.camera->start();
        robotSetOrientation(rig.nt, 30.0, 1'990'000);
        const Mt2 late = mt2Of(rig, turned);
        CHECK(late.yaw == 0.0);
        CHECK(late.x == doctest::Approx(expected[0]).epsilon(1e-5));
    }
}

TEST_CASE("limelight publisher: the orientation carries pitch, roll and rates and a yaw-only array is accepted") {
    Rig rig;
    rig.camera->start();
    robotSetArray6(rig.nt, ll::kRobotOrientationSet, {45.0, 10.0, 2.0, 3.0, 4.0, 5.0}, 1'900'000);
    rig.nt.setNowUs(1'950'000);
    rig.camera->poll();
    REQUIRE(rig.camera->robotState().robotOrientationDeg.has_value());
    const auto& o = *rig.camera->robotState().robotOrientationDeg;
    CHECK(o[ll::kOrientationYawDeg] == 45.0);
    CHECK(o[ll::kOrientationYawRateDegPerS] == 10.0);
    CHECK(o[ll::kOrientationPitchDeg] == 2.0);
    CHECK(o[ll::kOrientationPitchRateDegPerS] == 3.0);
    CHECK(o[ll::kOrientationRollDeg] == 4.0);
    CHECK(o[ll::kOrientationRollRateDegPerS] == 5.0);

    Rig other;  // [yaw] only, as raw NetworkTables code writes it
    other.camera->start();
    other.nt.robotSetDoubleArray(topic(ll::kRobotOrientationSet), {45.0}, 1'900'000);
    const Mt2 yawOnly = mt2Of(other, bothTags());
    CHECK(yawOnly.yaw == 45.0);
    CHECK(yawOnly.x == doctest::Approx(1.2368).epsilon(1e-4));
}

// ---- what else the robot can ask ----------------------------------------------------------------------------------

TEST_CASE("limelight publisher: the pipeline request is echoed to getpipe at once and on every frame") {
    Rig rig;
    rig.camera->start();
    robotSetNumber(rig.nt, ll::kPipeline, 1.0, 1'500'000);
    rig.nt.setNowUs(1'600'000);
    rig.camera->poll();
    CHECK(number(rig.nt, "getpipe") == 1.0);
    CHECK(rig.nt.latest(topic("getpipe"))->timeUs == 1'600'000);
    REQUIRE(rig.camera->robotState().pipelineIndex.has_value());
    CHECK(*rig.camera->robotState().pipelineIndex == 1);

    rig.publishAt(kNow, bothTags());
    CHECK(number(rig.nt, "getpipe") == 1.0);
    robotSetNumber(rig.nt, ll::kPipeline, 0.0, 2'010'000);
    rig.publishAt(kNow + 33'000, bothTags());
    CHECK(number(rig.nt, "getpipe") == 0.0);
}

TEST_CASE("limelight publisher: the driver camera mode publishes empty results and vision comes back with it") {
    Rig rig;
    rig.camera->start();
    robotSetNumber(rig.nt, ll::kCamMode, 1.0, 1'500'000);
    rig.publishAt(kNow, bothTags());
    CHECK(number(rig.nt, "tv") == 0.0);
    CHECK(number(rig.nt, "hb") == 1.0);  // the camera still delivers frames
    CHECK(array(rig.nt, "rawfiducials").empty());
    CHECK(array(rig.nt, "botpose_wpiblue").size() == ll::kBotposeHeaderSize);
    REQUIRE(rig.camera->robotState().driverMode.has_value());
    CHECK(*rig.camera->robotState().driverMode);

    robotSetNumber(rig.nt, ll::kCamMode, 0.0, 2'010'000);
    rig.publishAt(kNow + 33'000, bothTags());
    CHECK(number(rig.nt, "tv") == 1.0);
    CHECK_FALSE(*rig.camera->robotState().driverMode);
}

TEST_CASE("limelight publisher: throttle processes one frame after N skipped ones, skipped frames publish nothing") {
    Rig rig;
    rig.camera->start();
    robotSetNumber(rig.nt, ll::kThrottleSet, 2.0, 1'500'000);
    for (int i = 0; i < 6; ++i) rig.publishAt(kNow + i * 33'000, bothTags());
    // frames 3 and 6 are processed
    CHECK(rig.camera->robotState().framesPublished == 2);
    CHECK(number(rig.nt, "hb") == 2.0);
    CHECK(rig.nt.history(topic("tv")).size() == 3);  // the default and the two processed frames
    CHECK(rig.nt.latest(topic("tv"))->timeUs == kNow + 5 * 33'000);
    REQUIRE(rig.camera->robotState().throttle.has_value());
    CHECK(*rig.camera->robotState().throttle == 2.0);

    robotSetNumber(rig.nt, ll::kThrottleSet, 0.0, 2'300'000);  // back to full rate
    rig.publishAt(2'400'000, bothTags());
    rig.publishAt(2'433'000, bothTags());
    CHECK(rig.camera->robotState().framesPublished == 4);
}

TEST_CASE("limelight publisher: the priority tag decides tx and tid, the id filter decides which tags localize") {
    Rig rig;
    rig.camera->start();
    robotSetNumber(rig.nt, ll::kPriorityId, 32.0, 1'500'000);
    rig.nt.robotSetDoubleArray(topic(ll::kFiducialIdFiltersSet), {32.0}, 1'500'000);
    rig.publishAt(kNow, bothTags());
    CHECK(number(rig.nt, "tid") == 32.0);
    CHECK(number(rig.nt, "tx") == doctest::Approx(6.4236).epsilon(1e-4));
    CHECK(array(rig.nt, "rawfiducials").size() == 2 * ll::kRawFiducialsStride);
    const std::vector<double> mt1 = array(rig.nt, "botpose_wpiblue");
    CHECK(elem(mt1, ll::kBotposeTagCount) == 1.0);
    CHECK(mt1.size() == ll::kBotposeHeaderSize + ll::kBotposeTagStride);
    CHECK(elem(mt1, ll::kBotposeHeaderSize + ll::kTagId) == 32.0);

    rig.nt.robotSetDoubleArray(topic(ll::kFiducialIdFiltersSet), {}, 2'010'000);  // an empty list lifts the filter
    robotSetNumber(rig.nt, ll::kPriorityId, -1.0, 2'010'000);
    rig.publishAt(kNow + 33'000, bothTags());
    CHECK(number(rig.nt, "tid") == 31.0);
    CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeTagCount) == 2.0);
}

TEST_CASE("limelight publisher: robotState reports everything the robot asked for") {
    Rig rig;
    rig.camera->start();
    CHECK_FALSE(rig.camera->robotState().pipelineIndex.has_value());
    CHECK_FALSE(rig.camera->robotState().ledMode.has_value());
    CHECK_FALSE(rig.camera->robotState().imuMode.has_value());
    CHECK_FALSE(rig.camera->robotState().driverMode.has_value());
    CHECK_FALSE(rig.camera->robotState().throttle.has_value());
    CHECK_FALSE(rig.camera->robotState().robotOrientationDeg.has_value());
    CHECK_FALSE(rig.camera->robotState().cameraPoseOverride.has_value());

    robotSetNumber(rig.nt, ll::kPipeline, 3.0, 1'100'000);
    robotSetNumber(rig.nt, ll::kLedMode, 3.0, 1'100'000);
    robotSetNumber(rig.nt, ll::kImuModeSet, 1.0, 1'100'000);
    robotSetNumber(rig.nt, ll::kThrottleSet, 5.0, 1'100'000);
    robotSetMount(rig.nt, {-0.3, -0.0, 0.45, -0.0, 25.0, -180.0}, 1'100'000);
    robotSetOrientation(rig.nt, 12.5, 1'100'000);
    rig.camera->poll();

    const CameraRobotState state = rig.camera->robotState();
    CHECK(state.present);
    CHECK(*state.pipelineIndex == 3);
    CHECK(*state.ledMode == 3);
    CHECK(*state.imuMode == 1);
    CHECK(*state.throttle == 5.0);
    CHECK_FALSE(state.fpsLimit.has_value());  // a Limelight has none
    REQUIRE(state.cameraPoseOverride.has_value());
    CHECK((*state.cameraPoseOverride)[ll::kPose6PitchDeg] == 25.0);
    REQUIRE(state.robotOrientationDeg.has_value());
    CHECK((*state.robotOrientationDeg)[ll::kOrientationYawDeg] == 12.5);
    CHECK(state.publishing);
    CHECK(state.status.empty());
}

TEST_CASE("limelight publisher: a camera the robot never touches is not present, one it created topics for is") {
    {
        Rig rig;
        rig.camera->start();
        for (int i = 0; i < 200; ++i) rig.camera->poll();
        CHECK_FALSE(rig.camera->robotState().present);
    }
    {
        Rig rig;  // the robot created its publisher for pipeline but has not written yet
        rig.nt.robotPublish(topic(ll::kPipeline), NtType::Double);
        rig.camera->start();
        CHECK(rig.camera->robotState().present);
    }
    {
        Rig rig;  // the robot starts using the camera later: a topic appears, a poll notices it within a moment
        rig.camera->start();
        rig.nt.robotPublish(topic(ll::kCrop), NtType::DoubleArray);
        for (int i = 0; i < 100; ++i) rig.camera->poll();
        CHECK(rig.camera->robotState().present);
    }
    {
        Rig rig;  // our own per-frame topics never make the camera "present"
        rig.camera->start();
        rig.publishAt(kNow, bothTags());
        for (int i = 0; i < 100; ++i) rig.camera->poll();
        CHECK_FALSE(rig.camera->robotState().present);
    }
}

TEST_CASE("limelight publisher: malformed or non-finite requests are ignored") {
    Rig rig;
    rig.camera->start();
    rig.nt.robotSetString(topic(ll::kPipeline), "one", 1'100'000);                        // wrong type
    rig.nt.robotSetDouble(topic(ll::kLedMode), std::nan(""), 1'100'000);                   // not a number
    rig.nt.robotSetDoubleArray(topic(ll::kCameraPoseRobotSpaceSet), {1.0, 2.0, 3.0}, 1'100'000);  // too short
    rig.nt.robotSetDoubleArray(topic(ll::kRobotOrientationSet), {std::nan("")}, 1'100'000);
    rig.camera->poll();
    const CameraRobotState state = rig.camera->robotState();
    CHECK_FALSE(state.pipelineIndex.has_value());
    CHECK_FALSE(state.ledMode.has_value());
    CHECK_FALSE(state.cameraPoseOverride.has_value());
    CHECK_FALSE(state.robotOrientationDeg.has_value());

    // the mount stays the configured one, the orientation the default: the pose is still the truth
    rig.publishAt(kNow, bothTags());
    CHECK(elem(array(rig.nt, "botpose_wpiblue"), ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
}

// ---- refusing and recovering -------------------------------------------------------------------------------------

TEST_CASE("limelight publisher: a robot-side simulation makes the camera inert, its requests are still read") {
    Rig rig;
    rig.nt.robotPublish(topic(ll::kBotposeWpiBlue), NtType::DoubleArray);  // PhotonCameraSim style: robot publishes
    rig.camera->start();

    CameraRobotState state = rig.camera->robotState();
    CHECK(state.status == std::string(status::kForeignPublisher));
    CHECK_FALSE(state.publishing);
    CHECK(rig.log.has(wpiemu::sdk::LogLevel::Warn, "already exists"));
    CHECK_FALSE(rig.nt.exists(topic("tv")));  // nothing of ours was created
    CHECK(rig.nt.publisherCount(topic(ll::kBotposeWpiBlue)) == 1);

    rig.publishAt(kNow, bothTags());
    CHECK(rig.camera->robotState().framesPublished == 0);
    CHECK_FALSE(rig.nt.exists(topic("tv")));
    CHECK(rig.nt.history(topic(ll::kBotposeWpiBlue)).empty());

    robotSetNumber(rig.nt, ll::kPipeline, 2.0, 1'500'000);  // still reading what the robot asks
    rig.camera->poll();
    state = rig.camera->robotState();
    CHECK(state.present);
    REQUIRE(state.pipelineIndex.has_value());
    CHECK(*state.pipelineIndex == 2);
    CHECK(state.status == std::string(status::kForeignPublisher));
}

TEST_CASE("limelight publisher: a foreign publisher of tv alone is enough to refuse") {
    Rig rig;
    rig.nt.robotPublish(topic(ll::kTv), NtType::Double);
    rig.camera->start();
    CHECK(rig.camera->robotState().status == std::string(status::kForeignPublisher));
    CHECK_FALSE(rig.nt.exists(topic("botpose_wpiblue")));
}

TEST_CASE("limelight publisher: without ntcore the camera is inert and starts by itself once ntcore is there") {
    Rig rig;
    rig.nt.setAvailable(false);
    rig.camera->start();
    CHECK(rig.camera->robotState().status == std::string(status::kNtcoreUnavailable));
    CHECK_FALSE(rig.camera->robotState().publishing);
    rig.publishAt(kNow, bothTags());  // harmless
    CHECK(rig.camera->robotState().framesPublished == 0);

    rig.nt.setAvailable(true);
    rig.camera->poll();  // the bridge keeps polling: the camera starts when ntcore appears
    CHECK(rig.camera->robotState().status.empty());
    CHECK(rig.camera->robotState().publishing);
    CHECK(rig.nt.exists(topic("tv")));
    rig.publishAt(kNow + 33'000, bothTags());
    CHECK(rig.camera->robotState().framesPublished == 1);
}

TEST_CASE("limelight publisher: a camera rebuilt after a reconnect reuses its own topics instead of refusing them") {
    Rig rig;
    rig.camera->start();
    rig.publishAt(kNow, bothTags());
    REQUIRE(rig.camera->robotState().status.empty());

    auto again = rig.vendor->createPublisher(rig.nt, rig.log, rearSetup());  // the bridge rebuilds on a new handshake
    again->start();
    CHECK(again->robotState().status.empty());
    CHECK(again->robotState().publishing);
    rig.nt.setNowUs(kNow + 33'000);
    again->publish(bothTags());
    CHECK(number(rig.nt, "tv") == 1.0);
    CHECK(rig.nt.latest(topic("tv"))->timeUs == kNow + 33'000);
}

TEST_CASE("limelight publisher: two cameras publish into their own tables") {
    Rig rig;
    wpiemu::sdk::CameraSetup other = rearSetup();
    other.cameraId = "ll-left";
    other.ntName = "limelight-left";
    auto left = rig.vendor->createPublisher(rig.nt, rig.log, other);
    rig.camera->start();
    left->start();
    rig.publishAt(kNow, bothTags());
    CHECK(rig.nt.exists("/limelight-left/tv"));
    CHECK(rig.nt.history("/limelight-left/botpose_wpiblue").size() == 1);  // only its default, no frame yet
    CHECK(rig.nt.history(topic("botpose_wpiblue")).size() == 2);
    CHECK(left->robotState().framesPublished == 0);
    CHECK(rig.camera->robotState().framesPublished == 1);

    VisionFrame leftFrame = bothTags();
    leftFrame.cameraId = "ll-left";
    left->publish(leftFrame);
    CHECK(rig.nt.history("/limelight-left/botpose_wpiblue").size() == 2);
    CHECK(rig.nt.history(topic("botpose_wpiblue")).size() == 2);
}

}  // namespace
