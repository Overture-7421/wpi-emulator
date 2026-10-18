#include "wpiemu/bridges/vision/limelight_math.h"

#include <algorithm>

namespace wpiemu::bridges::vision::limelight {

namespace {

// -x without producing -0.0 (so a zero stays "0" in the arrays and in logs).
double negated(double v) { return v == 0.0 ? 0.0 : -v; }

}  // namespace

double wrapDegrees(double deg) {
    if (deg > -180.0 && deg <= 180.0) return deg;
    if (!std::isfinite(deg)) return deg;
    double wrapped = std::fmod(deg + 180.0, 360.0);
    if (wrapped <= 0.0) wrapped += 360.0;
    return wrapped - 180.0;
}

double lerpAngleDegrees(double fromDeg, double toDeg, double fraction) {
    const double t = std::clamp(fraction, 0.0, 1.0);
    const double delta = wrapDegrees(toDeg - fromDeg);
    return wrapDegrees(fromDeg + delta * t);
}

Quatd normalized(const Quatd& q) {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return n > 0.0 ? Quatd{q.x / n, q.y / n, q.z / n, q.w / n} : Quatd{};
}

Vec3d rotate(const Quatd& q, const Vec3d& v) {
    const Vec3d axis{q.x, q.y, q.z};
    const Vec3d t = Vec3d{axis.y * v.z - axis.z * v.y, axis.z * v.x - axis.x * v.z, axis.x * v.y - axis.y * v.x} * 2.0;
    const Vec3d axisCrossT{axis.y * t.z - axis.z * t.y, axis.z * t.x - axis.x * t.z, axis.x * t.y - axis.y * t.x};
    return v + t * q.w + axisCrossT;
}

Pose3d operator*(const Pose3d& a, const Pose3d& b) {
    return {a.position + rotate(a.rotation, b.position), normalized(a.rotation * b.rotation)};
}

Pose3d inverse(const Pose3d& pose) {
    const Quatd inv = conjugate(pose.rotation);
    return {rotate(inv, pose.position) * -1.0, inv};
}

Pose3d toPose3d(const sdk::Pose3& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            normalized({pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w})};
}

Quatd quatFromEulerRad(double rollRad, double pitchRad, double yawRad) {
    const double cr = std::cos(rollRad * 0.5), sr = std::sin(rollRad * 0.5);
    const double cp = std::cos(pitchRad * 0.5), sp = std::sin(pitchRad * 0.5);
    const double cy = std::cos(yawRad * 0.5), sy = std::sin(yawRad * 0.5);
    return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy};
}

EulerRad eulerFromQuat(const Quatd& q) {
    const double sinPitch = 2.0 * (q.w * q.y - q.z * q.x);
    EulerRad e;
    e.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    e.pitch = std::fabs(sinPitch) >= 1.0 ? std::copysign(kPi / 2.0, sinPitch) : std::asin(sinPitch);
    e.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return e;
}

PoseDeg toPoseDeg(const Pose3d& pose) {
    const EulerRad e = eulerFromQuat(pose.rotation);
    return {pose.position.x, pose.position.y, pose.position.z, radToDeg(e.roll), radToDeg(e.pitch), radToDeg(e.yaw)};
}

RobotSpacePose toLimelightRobotSpace(const Pose3d& robotToCamera) {
    const EulerRad e = eulerFromQuat(robotToCamera.rotation);
    RobotSpacePose out{};
    out[kPose6Forward] = robotToCamera.position.x;
    out[kPose6Right] = negated(robotToCamera.position.y);
    out[kPose6Up] = robotToCamera.position.z;
    out[kPose6RollDeg] = negated(radToDeg(e.roll));
    out[kPose6PitchDeg] = negated(radToDeg(e.pitch));
    out[kPose6YawDeg] = negated(radToDeg(e.yaw));
    return out;
}

Pose3d fromLimelightRobotSpace(const RobotSpacePose& values) {
    return {{values[kPose6Forward], -values[kPose6Right], values[kPose6Up]},
            quatFromEulerRad(degToRad(-values[kPose6RollDeg]), degToRad(-values[kPose6PitchDeg]),
                             degToRad(-values[kPose6YawDeg]))};
}

PoseDeg toWpiRed(const PoseDeg& wpiBlue, const FieldSize& field) {
    PoseDeg red = wpiBlue;
    red.x = field.lengthM - wpiBlue.x;
    red.y = field.widthM - wpiBlue.y;
    red.yawDeg = wrapDegrees(wpiBlue.yawDeg + 180.0);
    return red;
}

PoseDeg toFieldCentered(const PoseDeg& wpiBlue, const FieldSize& field) {
    PoseDeg centered = wpiBlue;
    centered.x = wpiBlue.x - 0.5 * field.lengthM;
    centered.y = wpiBlue.y - 0.5 * field.widthM;
    return centered;
}

Pose3d fieldToCameraFromTag(const Pose3d& fieldToTag, const Pose3d& cameraToTag) {
    return fieldToTag * inverse(cameraToTag);
}

Pose3d megaTag1RobotPose(const Pose3d& fieldToCamera, const Pose3d& assumedRobotToCamera) {
    return fieldToCamera * inverse(assumedRobotToCamera);
}

PoseDeg megaTag2RobotPose(const Pose3d& fieldToTag, const Vec3d& tagInCamera, const Pose3d& assumedRobotToCamera,
                          const RobotOrientation& orientation) {
    const Quatd robotRotation =
        quatFromEulerRad(degToRad(orientation.rollDeg), degToRad(orientation.pitchDeg), degToRad(orientation.yawDeg));
    const Quatd cameraRotation = robotRotation * assumedRobotToCamera.rotation;
    const Vec3d cameraPosition = fieldToTag.position - rotate(cameraRotation, tagInCamera);
    const Vec3d robotPosition = cameraPosition - rotate(robotRotation, assumedRobotToCamera.position);
    return {robotPosition.x, robotPosition.y,  robotPosition.z,
            orientation.rollDeg, orientation.pitchDeg, wrapDegrees(orientation.yawDeg)};
}

}  // namespace wpiemu::bridges::vision::limelight
