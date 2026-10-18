#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "test_limelight_support.h"
#include "wpiemu/bridges/vision/limelight_math.h"

namespace {

using namespace wpiemu::tests::limelight;
using wpiemu::sdk::Pose3;
using wpiemu::sdk::Quat;
using wpiemu::sdk::Vec3;

constexpr double kTol = 1e-9;

double quatDistance(const ll::Quatd& a, const ll::Quatd& b) {
    // 0 when both describe the same rotation (q and -q do); the inputs may carry float rounding in their norm
    const ll::Quatd na = ll::normalized(a);
    const ll::Quatd nb = ll::normalized(b);
    const double d = std::fabs(na.x * nb.x + na.y * nb.y + na.z * nb.z + na.w * nb.w);
    return 1.0 - std::min(d, 1.0);
}

TEST_CASE("limelight math: angles wrap to the range minus 180 to 180 and interpolate along the shortest arc") {
    CHECK(ll::wrapDegrees(0.0) == 0.0);
    CHECK(ll::wrapDegrees(45.5) == 45.5);
    CHECK(ll::wrapDegrees(180.0) == 180.0);
    CHECK(ll::wrapDegrees(-180.0) == doctest::Approx(180.0));
    CHECK(ll::wrapDegrees(190.0) == doctest::Approx(-170.0));
    CHECK(ll::wrapDegrees(-190.0) == doctest::Approx(170.0));
    CHECK(ll::wrapDegrees(720.0) == doctest::Approx(0.0).epsilon(kTol));
    CHECK(ll::wrapDegrees(359.0) == doctest::Approx(-1.0));

    CHECK(ll::lerpAngleDegrees(10.0, 20.0, 0.5) == doctest::Approx(15.0));
    CHECK(ll::lerpAngleDegrees(170.0, -170.0, 0.5) == doctest::Approx(180.0));    // through 180, not through 0
    CHECK(ll::lerpAngleDegrees(-170.0, 170.0, 0.25) == doctest::Approx(-175.0));  // the short way is backwards
    CHECK(ll::lerpAngleDegrees(10.0, 20.0, 2.0) == doctest::Approx(20.0));        // fraction is clamped
    CHECK(ll::lerpAngleDegrees(10.0, 20.0, -1.0) == doctest::Approx(10.0));
}

TEST_CASE("limelight math: Euler angles follow WPILib Rotation3d and the SDK quaternion") {
    const double roll = 10.0 * kDeg, pitch = 20.0 * kDeg, yaw = 30.0 * kDeg;
    const ll::Quatd q = ll::quatFromEulerRad(roll, pitch, yaw);
    const ll::EulerRad e = ll::eulerFromQuat(q);
    CHECK(e.roll == doctest::Approx(roll).epsilon(kTol));
    CHECK(e.pitch == doctest::Approx(pitch).epsilon(kTol));
    CHECK(e.yaw == doctest::Approx(yaw).epsilon(kTol));

    const Quat sdkQuat = Quat::fromEuler(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw));
    CHECK(quatDistance(q, {sdkQuat.x, sdkQuat.y, sdkQuat.z, sdkQuat.w}) < 1e-12);

    // Counter-clockwise yaw: +X turns into +Y; a positive pitch tilts +X down (WPILib), so a camera tilted UP is < 0.
    const ll::Vec3d turned = ll::rotate(ll::quatFromEulerRad(0.0, 0.0, 90.0 * kDeg), {1.0, 0.0, 0.0});
    CHECK(turned.x == doctest::Approx(0.0).epsilon(kTol));
    CHECK(turned.y == doctest::Approx(1.0));
    const ll::Vec3d nosedDown = ll::rotate(ll::quatFromEulerRad(0.0, 30.0 * kDeg, 0.0), {1.0, 0.0, 0.0});
    CHECK(nosedDown.z < 0.0);
}

TEST_CASE("limelight math: a pose composed with its inverse is the identity") {
    const ll::Pose3d p{{1.5, -2.0, 0.7}, ll::quatFromEulerRad(0.1, -0.4, 2.0)};
    const ll::Pose3d identity = p * ll::inverse(p);
    CHECK(ll::length(identity.position) < 1e-12);
    CHECK(quatDistance(identity.rotation, ll::Quatd{}) < 1e-12);

    const ll::Pose3d q{{0.3, 0.2, -1.0}, ll::quatFromEulerRad(-0.2, 0.3, -1.0)};
    const ll::Pose3d composed = p * q;  // q expressed in the frame of p
    const ll::Vec3d viaRotation = p.position + ll::rotate(p.rotation, q.position);
    CHECK(ll::distance(composed.position, viaRotation) < 1e-12);
}

TEST_CASE("limelight math: the sample mount maps to the array the sample robot sends and back") {
    const ll::Pose3d mount = ll::toPose3d(rearMount());
    const ll::RobotSpacePose array = ll::toLimelightRobotSpace(mount);
    // [x, -y, z, -roll, -pitch, -yaw] of (-0.30, 0, 0.45, roll 0, pitch -25, yaw 180) = [-0.30, 0, 0.45, 0, 25, -180]
    CHECK(array[ll::kPose6Forward] == doctest::Approx(-0.30).epsilon(1e-6));
    CHECK(array[ll::kPose6Right] == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(array[ll::kPose6Up] == doctest::Approx(0.45).epsilon(1e-6));
    CHECK(array[ll::kPose6RollDeg] == doctest::Approx(0.0).epsilon(1e-4));
    CHECK(array[ll::kPose6PitchDeg] == doctest::Approx(25.0).epsilon(1e-6));
    CHECK(std::fabs(array[ll::kPose6YawDeg]) == doctest::Approx(180.0).epsilon(1e-6));  // +-180: the same rotation

    // and the other way: what the robot writes becomes the WPILib mount
    const ll::RobotSpacePose written = {-0.30, -0.0, 0.45, -0.0, 25.0, -180.0};
    const ll::Pose3d decoded = ll::fromLimelightRobotSpace(written);
    CHECK(decoded.position.x == doctest::Approx(-0.30));
    CHECK(decoded.position.y == doctest::Approx(0.0).epsilon(kTol));
    CHECK(decoded.position.z == doctest::Approx(0.45));
    CHECK(quatDistance(decoded.rotation, mount.rotation) < 1e-6);
}

TEST_CASE("limelight math: Limelight robot space is +Y right with negated angles and round-trips") {
    // WPILib: 0.20 m forward, 0.15 m LEFT, 0.40 m up, roll 5, pitch -10 (up), yaw 30 (left)
    const ll::Pose3d wpilib{{0.20, 0.15, 0.40}, ll::quatFromEulerRad(5.0 * kDeg, -10.0 * kDeg, 30.0 * kDeg)};
    const ll::RobotSpacePose array = ll::toLimelightRobotSpace(wpilib);
    CHECK(array[ll::kPose6Forward] == doctest::Approx(0.20));
    CHECK(array[ll::kPose6Right] == doctest::Approx(-0.15));  // left of the robot = negative "right"
    CHECK(array[ll::kPose6Up] == doctest::Approx(0.40));
    CHECK(array[ll::kPose6RollDeg] == doctest::Approx(-5.0));
    CHECK(array[ll::kPose6PitchDeg] == doctest::Approx(10.0));
    CHECK(array[ll::kPose6YawDeg] == doctest::Approx(-30.0));

    const ll::Pose3d back = ll::fromLimelightRobotSpace(array);
    CHECK(ll::distance(back.position, wpilib.position) < kTol);
    CHECK(quatDistance(back.rotation, wpilib.rotation) < 1e-12);

    const ll::RobotSpacePose again = ll::toLimelightRobotSpace(back);
    for (std::size_t i = 0; i < ll::kPose6Size; ++i) CHECK(again[i] == doctest::Approx(array[i]).epsilon(1e-9));

    // a zero stays "0" (no -0.0 in the arrays or the logs)
    const ll::RobotSpacePose zero = ll::toLimelightRobotSpace(ll::Pose3d{});
    for (const double v : zero) CHECK_FALSE(std::signbit(v));
}

TEST_CASE("limelight math: the red origin is the field rotated 180 degrees about its centre") {
    const ll::PoseDeg blue{2.0, 4.0, 0.1, 1.0, 2.0, 30.0};
    const ll::PoseDeg red = ll::toWpiRed(blue);
    CHECK(red.x == doctest::Approx(16.541 - 2.0));
    CHECK(red.y == doctest::Approx(8.069 - 4.0));
    CHECK(red.z == doctest::Approx(0.1));
    CHECK(red.rollDeg == doctest::Approx(1.0));
    CHECK(red.pitchDeg == doctest::Approx(2.0));
    CHECK(red.yawDeg == doctest::Approx(-150.0));

    const ll::PoseDeg again = ll::toWpiRed(red);  // an involution
    CHECK(again.x == doctest::Approx(blue.x));
    CHECK(again.y == doctest::Approx(blue.y));
    CHECK(again.yawDeg == doctest::Approx(blue.yawDeg));

    // the centre of the field is a fixed point, facing 0 turns to 180
    const ll::PoseDeg centre = ll::toWpiRed({16.541 / 2.0, 8.069 / 2.0, 0.0, 0.0, 0.0, 0.0});
    CHECK(centre.x == doctest::Approx(16.541 / 2.0));
    CHECK(centre.y == doctest::Approx(8.069 / 2.0));
    CHECK(centre.yawDeg == doctest::Approx(180.0));

    ll::FieldSize custom;
    custom.lengthM = 10.0;
    custom.widthM = 6.0;
    CHECK(ll::toWpiRed({1.0, 1.0, 0.0, 0.0, 0.0, 0.0}, custom).x == doctest::Approx(9.0));
    CHECK(ll::toWpiRed({1.0, 1.0, 0.0, 0.0, 0.0, 0.0}, custom).y == doctest::Approx(5.0));
}

TEST_CASE("limelight math: the field-centre origin only moves the origin") {
    const ll::PoseDeg blue{2.0, 4.0, 0.1, 1.0, 2.0, 30.0};
    const ll::PoseDeg centred = ll::toFieldCentered(blue);
    CHECK(centred.x == doctest::Approx(2.0 - 16.541 / 2.0));
    CHECK(centred.y == doctest::Approx(4.0 - 8.069 / 2.0));
    CHECK(centred.z == doctest::Approx(0.1));
    CHECK(centred.yawDeg == doctest::Approx(30.0));
}

TEST_CASE("limelight math: megatag1 is exact for the right mount and shifted by the mount error otherwise") {
    const Pose3 robot = robotAt(2.0, 4.0, 0.0);
    const ll::Pose3d fieldToCamera = ll::toPose3d(robot * rearMount());
    const ll::Pose3d physical = ll::toPose3d(rearMount());

    const ll::PoseDeg exact = ll::toPoseDeg(ll::megaTag1RobotPose(fieldToCamera, physical));
    CHECK(exact.x == doctest::Approx(2.0).epsilon(1e-6));
    CHECK(exact.y == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(exact.z == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(exact.yawDeg == doctest::Approx(0.0).epsilon(1e-4));

    // The device believes the camera sits 0.10 m further FORWARD than it does. The image still fixes the camera, so
    // the robot lands 0.10 m behind the truth: est = F * (P * A^-1) = F * T(-delta).
    ll::Pose3d believed = physical;
    believed.position.x += 0.10;
    const ll::PoseDeg shifted = ll::toPoseDeg(ll::megaTag1RobotPose(fieldToCamera, believed));
    CHECK(shifted.x == doctest::Approx(2.0 - 0.10).epsilon(1e-6));
    CHECK(shifted.y == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(shifted.z == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(shifted.yawDeg == doctest::Approx(0.0).epsilon(1e-4));

    // The believed mount is missing the 25 degree upward tilt: est = F * (m - Ry(25 deg) m, Ry(25 deg)), by hand.
    ll::Pose3d flat = physical;
    flat.rotation = ll::quatFromEulerRad(0.0, 0.0, 180.0 * kDeg);
    const ll::PoseDeg tilted = ll::toPoseDeg(ll::megaTag1RobotPose(fieldToCamera, flat));
    const double c = std::cos(25.0 * kDeg), s = std::sin(25.0 * kDeg);
    const double mx = -0.30, mz = 0.45;
    CHECK(tilted.x == doctest::Approx(2.0 + (mx - (c * mx + s * mz))).epsilon(1e-6));
    CHECK(tilted.y == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(tilted.z == doctest::Approx(mz - (-s * mx + c * mz)).epsilon(1e-6));
    CHECK(tilted.pitchDeg == doctest::Approx(25.0).epsilon(1e-5));
    CHECK(tilted.yawDeg == doctest::Approx(0.0).epsilon(1e-4));
}

TEST_CASE("limelight math: megatag2 reproduces the truth for the right orientation and skews with a wrong yaw") {
    const Pose3 robot = robotAt(2.0, 4.0, 0.0);
    const Pose3 tag31({0.0080772f, 3.7457126f, 0.55245f}, Quat::identity());
    const Pose3 cameraToField = (robot * rearMount()).inverse();
    const Vec3 tagInCamera = cameraToField.transformPoint(tag31.position);
    const ll::Pose3d fieldToTag = ll::toPose3d(tag31);
    const ll::Vec3d measured{tagInCamera.x, tagInCamera.y, tagInCamera.z};
    const ll::Pose3d mount = ll::toPose3d(rearMount());

    ll::RobotOrientation orientation;  // yaw 0 = the truth
    const ll::PoseDeg right = ll::megaTag2RobotPose(fieldToTag, measured, mount, orientation);
    CHECK(right.x == doctest::Approx(2.0).epsilon(1e-6));
    CHECK(right.y == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(right.z == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(right.rollDeg == 0.0);
    CHECK(right.pitchDeg == 0.0);
    CHECK(right.yawDeg == 0.0);

    // The spike's measurement: a yaw of 45 degrees written by the robot -> (1.237, 5.334, yaw 45.00), tag 31 primary.
    orientation.yawDeg = 45.0;
    const ll::PoseDeg wrong = ll::megaTag2RobotPose(fieldToTag, measured, mount, orientation);
    CHECK(wrong.x == doctest::Approx(1.2368).epsilon(1e-4));
    CHECK(wrong.y == doctest::Approx(5.3340).epsilon(1e-4));
    CHECK(wrong.z == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(wrong.yawDeg == 45.0);
    // ... and it is the closed form p - Rz(psi - theta)(p - r)
    const auto expected = expectedMegaTag2Xy(kTagX, kTag31Y, 2.0, 4.0, 0.0, 45.0);
    CHECK(wrong.x == doctest::Approx(expected[0]).epsilon(1e-6));
    CHECK(wrong.y == doctest::Approx(expected[1]).epsilon(1e-6));

    // yaw is reported wrapped
    orientation.yawDeg = 190.0;
    CHECK(ll::megaTag2RobotPose(fieldToTag, measured, mount, orientation).yawDeg == doctest::Approx(-170.0));
}

TEST_CASE("limelight math: megatag2 carries the supplied roll and pitch and is exact when they are right") {
    // A robot standing on a 5 degree slope (pitch +5, nose down) 0.10 m above the floor, yawed 20 degrees.
    const Pose3 robot({2.0f, 4.0f, 0.10f},
                      Quat::fromEuler(0.f, static_cast<float>(5.0 * kDeg), static_cast<float>(20.0 * kDeg)));
    const Pose3 tag({0.0080772f, 3.9f, 0.55245f}, Quat::identity());
    const Vec3 inCamera = (robot * rearMount()).inverse().transformPoint(tag.position);

    ll::RobotOrientation orientation;
    orientation.yawDeg = 20.0;
    orientation.pitchDeg = 5.0;
    const ll::PoseDeg pose = ll::megaTag2RobotPose(ll::toPose3d(tag), {inCamera.x, inCamera.y, inCamera.z},
                                                   ll::toPose3d(rearMount()), orientation);
    CHECK(pose.x == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(pose.y == doctest::Approx(4.0).epsilon(1e-5));
    CHECK(pose.z == doctest::Approx(0.10).epsilon(1e-5));
    CHECK(pose.rollDeg == 0.0);
    CHECK(pose.pitchDeg == 5.0);
    CHECK(pose.yawDeg == 20.0);

    // forgetting the pitch moves the estimate (z and x change with it)
    orientation.pitchDeg = 0.0;
    const ll::PoseDeg flat = ll::megaTag2RobotPose(ll::toPose3d(tag), {inCamera.x, inCamera.y, inCamera.z},
                                                   ll::toPose3d(rearMount()), orientation);
    CHECK(std::fabs(flat.z - 0.10) > 0.01);
}

TEST_CASE("limelight math: megatag2 uses the believed mount rotation, not the physical one") {
    const Pose3 robot = robotAt(2.0, 4.0, 0.0);
    const Pose3 tag({0.0080772f, 3.7457126f, 0.55245f}, Quat::identity());
    const Vec3 inCamera = (robot * rearMount()).inverse().transformPoint(tag.position);
    const ll::Vec3d measured{inCamera.x, inCamera.y, inCamera.z};

    // believed mount: the same position and yaw but no 25 degree tilt. With the right robot yaw the camera orientation
    // is Rz(0) * believed rotation: the camera is at tag - R_believed * t and the robot that minus the mount offset.
    ll::Pose3d believed = ll::toPose3d(rearMount());
    believed.rotation = ll::quatFromEulerRad(0.0, 0.0, 180.0 * kDeg);
    const ll::PoseDeg pose = ll::megaTag2RobotPose(ll::toPose3d(tag), measured, believed, {});

    const Quat believedRotation = Quat::fromYaw(static_cast<float>(ll::kPi));
    const Vec3 camera = tag.position - believedRotation.rotate(inCamera);
    const Vec3 expectedRobot = camera - Vec3(-0.30f, 0.0f, 0.45f);  // an unrotated robot: the offset is not rotated
    CHECK(pose.x == doctest::Approx(expectedRobot.x).epsilon(1e-5));
    CHECK(pose.y == doctest::Approx(expectedRobot.y).epsilon(1e-5));
    CHECK(pose.z == doctest::Approx(expectedRobot.z).epsilon(1e-5));
    CHECK(std::fabs(pose.x - 2.0) > 0.01);  // and it is wrong, as on a real robot
}

}  // namespace
