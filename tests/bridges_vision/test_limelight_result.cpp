#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "test_limelight_support.h"
#include "wpiemu/bridges/vision/limelight_result.h"

namespace {

using namespace wpiemu::tests::limelight;
using ll::LimelightResult;
using ll::ResultSettings;

constexpr double kTagZ = 0.55245;
constexpr double kTagSpan = kTag32Y - kTag31Y;  // tags 31 and 32 are 0.4318 m apart

ResultSettings settingsFor(const wpiemu::sdk::Pose3& assumed = rearMount()) {
    ResultSettings s;
    s.assumedRobotToCamera = ll::toPose3d(assumed);
    return s;
}

double at(const std::vector<double>& array, std::size_t index) { return array.at(index); }
double tagValue(const std::vector<double>& array, std::size_t tag, std::size_t field) {
    return array.at(ll::kBotposeHeaderSize + tag * ll::kBotposeTagStride + field);
}
double rawValue(const std::vector<double>& raw, std::size_t tag, std::size_t field) {
    return raw.at(tag * ll::kRawFiducialsStride + field);
}
std::size_t tagCountOf(const std::vector<double>& array) {
    return static_cast<std::size_t>(at(array, ll::kBotposeTagCount));
}

// The six botpose arrays of a result: MegaTag1 then MegaTag2, centred, blue, red.
std::array<const std::vector<double>*, 6> botposes(const LimelightResult& r) {
    return {&r.botpose, &r.botposeWpiBlue, &r.botposeWpiRed, &r.botposeOrb, &r.botposeOrbWpiBlue, &r.botposeOrbWpiRed};
}

void checkLayout(const std::vector<double>& array) {
    REQUIRE(array.size() >= ll::kBotposeHeaderSize);
    CHECK(array.size() == ll::kBotposeHeaderSize + ll::kBotposeTagStride * tagCountOf(array));
}

TEST_CASE("limelight result: a frame without tags is a complete result, not a missing one") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({});
    const LimelightResult r = ll::computeResult(frame, settingsFor());
    CHECK_FALSE(r.hasTarget);
    CHECK(r.txDeg == 0.0);
    CHECK(r.tyDeg == 0.0);
    CHECK(r.taPct == 0.0);
    CHECK(r.tid == -1.0);
    CHECK(r.tlMs == doctest::Approx(20.0));
    CHECK(r.clMs == doctest::Approx(12.0));
    CHECK(r.rawFiducials.empty());

    for (const auto* array : botposes(r)) {
        REQUIRE(array->size() == ll::kBotposeHeaderSize);  // 11 values: LimelightHelpers accepts it, with tagCount 0
        for (std::size_t i = 0; i < array->size(); ++i) {
            if (i == ll::kBotposeLatencyMs)
                CHECK((*array)[i] == doctest::Approx(32.0));
            else
                CHECK((*array)[i] == 0.0);  // also the red and centred flavours: there is no pose to move
        }
    }
}

TEST_CASE("limelight result: one tag fills the primary target, both estimates and the raw list") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31});
    REQUIRE(frame.observations.size() == 1);
    const wpiemu::sdk::FiducialObservation& o = frame.observations.front();
    const LimelightResult r = ll::computeResult(frame, settingsFor());

    CHECK(r.hasTarget);
    CHECK(r.txDeg == doctest::Approx(-9.1617).epsilon(1e-4));  // the spike's ll.tx = -9.16
    CHECK(r.tyDeg == doctest::Approx(-21.5348).epsilon(1e-4));  // ll.ty = -21.53
    CHECK(r.taPct == doctest::Approx(0.4711).epsilon(1e-3));    // ll.ta = 0.471
    CHECK(r.tid == 31.0);
    CHECK(r.txDeg == doctest::Approx(o.yawRad * 180.0 / ll::kPi));

    for (const auto* array : botposes(r)) {
        checkLayout(*array);
        CHECK(tagCountOf(*array) == 1);
        CHECK(at(*array, ll::kBotposeLatencyMs) == doctest::Approx(32.0));
        CHECK(at(*array, ll::kBotposeTagSpanM) == 0.0);  // one tag has no span
        CHECK(at(*array, ll::kBotposeAvgTagDistM) == doctest::Approx(o.distanceM));
        CHECK(at(*array, ll::kBotposeAvgTagAreaPct) == doctest::Approx(o.areaPct));
        CHECK(tagValue(*array, 0, ll::kTagId) == 31.0);
        CHECK(tagValue(*array, 0, ll::kTagTxnc) == doctest::Approx(r.txDeg));
        CHECK(tagValue(*array, 0, ll::kTagTync) == doctest::Approx(r.tyDeg));
        CHECK(tagValue(*array, 0, ll::kTagTa) == doctest::Approx(o.areaPct));
        CHECK(tagValue(*array, 0, ll::kTagDistToCameraM) == doctest::Approx(o.distanceM));
        CHECK(tagValue(*array, 0, ll::kTagAmbiguity) == 0.0);
    }
    // distance from the robot to the tag, by hand: the believed mount equals the real one, the robot stands at (2, 4)
    const double distToRobot =
        std::sqrt((kTagX - 2.0) * (kTagX - 2.0) + (kTag31Y - 4.0) * (kTag31Y - 4.0) + kTagZ * kTagZ);
    CHECK(tagValue(r.botposeWpiBlue, 0, ll::kTagDistToRobotM) == doctest::Approx(distToRobot).epsilon(1e-5));

    REQUIRE(r.rawFiducials.size() == ll::kRawFiducialsStride);
    CHECK(rawValue(r.rawFiducials, 0, ll::kTagId) == 31.0);
    CHECK(rawValue(r.rawFiducials, 0, ll::kTagDistToRobotM) == doctest::Approx(distToRobot).epsilon(1e-5));

    // MegaTag1 in the three flavours: the truth (2, 4, facing +X) is exact
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeZ) == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(angleDiffDeg(at(r.botposeWpiBlue, ll::kBotposeYawDeg), 0.0) < 1e-3);
    CHECK(at(r.botposeWpiRed, ll::kBotposeX) == doctest::Approx(ll::kWpiFieldLengthM - 2.0).epsilon(1e-5));
    CHECK(at(r.botposeWpiRed, ll::kBotposeY) == doctest::Approx(ll::kWpiFieldWidthM - 4.0).epsilon(1e-5));
    CHECK(angleDiffDeg(at(r.botposeWpiRed, ll::kBotposeYawDeg), 180.0) < 1e-3);
    CHECK(at(r.botpose, ll::kBotposeX) == doctest::Approx(2.0 - ll::kWpiFieldLengthM / 2.0).epsilon(1e-5));
    CHECK(at(r.botpose, ll::kBotposeY) == doctest::Approx(4.0 - ll::kWpiFieldWidthM / 2.0).epsilon(1e-5));
}

TEST_CASE("limelight result: two tags use the multi-tag solution and report span, mean distance and mean area") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});
    REQUIRE(frame.observations.size() == 2);
    REQUIRE(frame.multiTag.valid);
    const LimelightResult r = ll::computeResult(frame, settingsFor());

    checkLayout(r.botposeWpiBlue);
    CHECK(tagCountOf(r.botposeWpiBlue) == 2);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeTagSpanM) == doctest::Approx(kTagSpan).epsilon(1e-5));
    const double meanDist = 0.5 * (frame.observations[0].distanceM + frame.observations[1].distanceM);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeAvgTagDistM) == doctest::Approx(meanDist));
    const double meanArea = 0.5 * (frame.observations[0].areaPct + frame.observations[1].areaPct);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeAvgTagAreaPct) == doctest::Approx(meanArea));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));

    REQUIRE(r.rawFiducials.size() == 2 * ll::kRawFiducialsStride);
    CHECK(rawValue(r.rawFiducials, 0, ll::kTagId) == static_cast<double>(frame.observations[0].id));
    CHECK(rawValue(r.rawFiducials, 1, ll::kTagId) == static_cast<double>(frame.observations[1].id));
    // the per-tag entries of the arrays are the raw list
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t f = 0; f < ll::kRawFiducialsStride; ++f)
            CHECK(tagValue(r.botposeWpiBlue, i, f) == rawValue(r.rawFiducials, i, f));
    // tx is the first tag's
    CHECK(r.tid == static_cast<double>(frame.observations[0].id));
}

TEST_CASE("limelight result: without a valid multi-tag solution megatag1 falls back to the largest tag alone") {
    vision::FrameSpec spec;
    spec.cameraId = "ll-rear";
    spec.vendor = wpiemu::sdk::CameraVendor::Limelight;
    spec.fieldToRobot = robotAt(2.0, 4.0, 0.0);
    spec.robotToCamera = rearMount();
    spec.tagIds = {31, 32};
    spec.multiTag = false;  // the camera is configured without multi-tag
    const wpiemu::sdk::VisionFrame frame = vision::makeFrame(spec, sceneLayout());
    REQUIRE_FALSE(frame.multiTag.valid);

    const LimelightResult r = ll::computeResult(frame, settingsFor());
    CHECK(tagCountOf(r.botposeWpiBlue) == 1);  // MegaTag1 used one tag ...
    checkLayout(r.botposeWpiBlue);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(r.rawFiducials.size() == 2 * ll::kRawFiducialsStride);  // ... while the camera still sees two
    CHECK(tagCountOf(r.botposeOrbWpiBlue) == 2);                  // and MegaTag2 reports every allowed tag
}

TEST_CASE("limelight result: latency is split into cl and tl and clamped") {
    LimelightResult r = ll::computeResult(rearFrame({31}, robotAt(2, 4, 0), 32000, 12000), settingsFor());
    CHECK(r.clMs == doctest::Approx(12.0));
    CHECK(r.tlMs == doctest::Approx(20.0));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeLatencyMs) == doctest::Approx(r.tlMs + r.clMs));

    r = ll::computeResult(rearFrame({31}, robotAt(2, 4, 0), 10000, 25000), settingsFor());  // capture > latency
    CHECK(r.clMs == doctest::Approx(10.0));
    CHECK(r.tlMs == doctest::Approx(0.0));

    r = ll::computeResult(rearFrame({31}, robotAt(2, 4, 0), -5, -7), settingsFor());
    CHECK(r.clMs == 0.0);
    CHECK(r.tlMs == 0.0);
}

TEST_CASE("limelight result: a priority tag decides the primary target, localization and raw fiducials ignore it") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});
    ResultSettings s = settingsFor();
    s.priorityId = 32;
    LimelightResult r = ll::computeResult(frame, s);
    CHECK(r.hasTarget);
    CHECK(r.tid == 32.0);
    CHECK(r.txDeg == doctest::Approx(6.4236).epsilon(1e-4));
    CHECK(tagCountOf(r.botposeWpiBlue) == 2);
    CHECK(r.rawFiducials.size() == 2 * ll::kRawFiducialsStride);

    s.priorityId = 17;  // not in view: no valid target for tx/ty, but the pose estimates are untouched
    r = ll::computeResult(frame, s);
    CHECK_FALSE(r.hasTarget);
    CHECK(r.tid == -1.0);
    CHECK(r.txDeg == 0.0);
    CHECK(tagCountOf(r.botposeWpiBlue) == 2);
    CHECK(r.rawFiducials.size() == 2 * ll::kRawFiducialsStride);
}

TEST_CASE("limelight result: the id filter restricts localization only") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});
    const std::vector<int> only32{32};
    ResultSettings s = settingsFor();
    s.idFilter = only32;
    LimelightResult r = ll::computeResult(frame, s);
    CHECK(r.hasTarget);                       // targeting is not filtered
    CHECK(r.tid == 31.0);
    CHECK(r.rawFiducials.size() == 2 * ll::kRawFiducialsStride);  // "unfiltered"
    checkLayout(r.botposeWpiBlue);
    CHECK(tagCountOf(r.botposeWpiBlue) == 1);  // tag 31 is part of the multi-tag solution, so tag 32 alone
    CHECK(tagValue(r.botposeWpiBlue, 0, ll::kTagId) == 32.0);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(tagCountOf(r.botposeOrbWpiBlue) == 1);
    // MegaTag2 solves from tag 32 now: the closed form with a 45 degree yaw error and tag 32 at (0.0081, 4.1775)
    s.orientation.yawDeg = 45.0;
    r = ll::computeResult(frame, s);
    const auto expected = expectedMegaTag2Xy(kTagX, kTag32Y, 2.0, 4.0, 0.0, 45.0);
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeX) == doctest::Approx(expected[0]).epsilon(1e-5));
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeY) == doctest::Approx(expected[1]).epsilon(1e-5));

    const std::vector<int> both{31, 32};  // every tag of the solution is allowed: the multi-tag pose is back
    s.idFilter = both;
    r = ll::computeResult(frame, s);
    CHECK(tagCountOf(r.botposeWpiBlue) == 2);

    const std::vector<int> none{99};  // nothing may localize, although a tag is in view
    s.idFilter = none;
    r = ll::computeResult(frame, s);
    CHECK(r.hasTarget);
    CHECK(tagCountOf(r.botposeWpiBlue) == 0);
    CHECK(r.botposeWpiBlue.size() == ll::kBotposeHeaderSize);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == 0.0);
    CHECK(tagCountOf(r.botposeOrbWpiBlue) == 0);
}

TEST_CASE("limelight result: driver mode processes no vision at all") {
    ResultSettings s = settingsFor();
    s.driverMode = true;
    const LimelightResult r = ll::computeResult(rearFrame({31, 32}), s);
    CHECK_FALSE(r.hasTarget);
    CHECK(r.tid == -1.0);
    CHECK(r.rawFiducials.empty());
    CHECK(r.botposeWpiBlue.size() == ll::kBotposeHeaderSize);
    CHECK(tagCountOf(r.botposeWpiBlue) == 0);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeLatencyMs) == doctest::Approx(32.0));  // the frame is still a frame
}

TEST_CASE("limelight result: a believed mount that differs from the physical one skews megatag1 by the closed form") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});

    // believed 0.10 m further forward: est = truth - (0.10, 0, 0)
    wpiemu::sdk::Pose3 forward = rearMount();
    forward.position.x += 0.10f;
    LimelightResult r = ll::computeResult(frame, settingsFor(forward));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(1.90).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeY) == doctest::Approx(4.0).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeZ) == doctest::Approx(0.0).epsilon(1e-5));

    // the believed mount lacks the 25 degree upward tilt ("forgot the pitch"): est = truth + (m - Ry(25) m), pitch +25
    wpiemu::sdk::Pose3 flat = rearMount();
    flat.rotation = wpiemu::sdk::Quat::fromYaw(static_cast<float>(vision::kPi));
    r = ll::computeResult(frame, settingsFor(flat));
    const double c = std::cos(25.0 * kDeg), s = std::sin(25.0 * kDeg);
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) ==
          doctest::Approx(2.0 + (-0.30 - (c * -0.30 + s * 0.45))).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposeZ) == doctest::Approx(0.45 - (-s * -0.30 + c * 0.45)).epsilon(1e-5));
    CHECK(at(r.botposeWpiBlue, ll::kBotposePitchDeg) == doctest::Approx(25.0).epsilon(1e-4));
    // the distance to the robot follows the believed mount, the distance to the camera does not
    CHECK(tagValue(r.botposeWpiBlue, 0, ll::kTagDistToCameraM) == doctest::Approx(frame.observations[0].distanceM));
    CHECK(tagValue(r.botposeWpiBlue, 0, ll::kTagDistToRobotM) !=
          doctest::Approx(tagValue(ll::computeResult(frame, settingsFor()).botposeWpiBlue, 0, ll::kTagDistToRobotM)));
}

TEST_CASE("limelight result: a degenerate solve is no estimate and never NaN on the robot's side") {
    wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});
    frame.multiTag.fieldToCamera.position.x = std::numeric_limits<float>::quiet_NaN();
    ResultSettings s = settingsFor();
    s.orientation.yawDeg = 45.0;
    const LimelightResult r = ll::computeResult(frame, s);

    REQUIRE(r.botposeWpiBlue.size() == ll::kBotposeHeaderSize);  // MegaTag1 relies on the broken multi-tag pose
    CHECK(tagCountOf(r.botposeWpiBlue) == 0);
    for (const auto* array : {&r.botpose, &r.botposeWpiBlue, &r.botposeWpiRed})
        for (const double v : *array) CHECK(std::isfinite(v));
    CHECK(tagCountOf(r.botposeOrbWpiBlue) == 2);  // MegaTag2 does not: it keeps working
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeX) == doctest::Approx(1.2368).epsilon(1e-4));
}

TEST_CASE("limelight result: megatag2 follows the supplied orientation in every flavour") {
    const wpiemu::sdk::VisionFrame frame = rearFrame({31, 32});  // primary: tag 31
    ResultSettings s = settingsFor();
    s.orientation.yawDeg = 45.0;
    const LimelightResult r = ll::computeResult(frame, s);

    // the spike's numbers: robot (2, 4) facing +X, 45 degrees written -> (1.237, 5.334, yaw 45.00)
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeX) == doctest::Approx(1.2368).epsilon(1e-4));
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeY) == doctest::Approx(5.3340).epsilon(1e-4));
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeZ) == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(at(r.botposeOrbWpiBlue, ll::kBotposeYawDeg) == 45.0);
    // red and centred derive from it
    CHECK(at(r.botposeOrbWpiRed, ll::kBotposeX) == doctest::Approx(ll::kWpiFieldLengthM - 1.2368).epsilon(1e-4));
    CHECK(at(r.botposeOrbWpiRed, ll::kBotposeY) == doctest::Approx(ll::kWpiFieldWidthM - 5.3340).epsilon(1e-4));
    CHECK(at(r.botposeOrbWpiRed, ll::kBotposeYawDeg) == doctest::Approx(-135.0));
    CHECK(at(r.botposeOrb, ll::kBotposeX) == doctest::Approx(1.2368 - ll::kWpiFieldLengthM / 2.0).epsilon(1e-4));
    CHECK(at(r.botposeOrb, ll::kBotposeYawDeg) == 45.0);
    // MegaTag1 does not use it
    CHECK(at(r.botposeWpiBlue, ll::kBotposeX) == doctest::Approx(2.0).epsilon(1e-5));
    // it reports both tags like the device does
    CHECK(tagCountOf(r.botposeOrbWpiBlue) == 2);
    checkLayout(r.botposeOrbWpiBlue);
}

}  // namespace
