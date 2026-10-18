#pragma once

// Pure geometry of the Limelight publisher: conversions between the WPILib frames the emulator works in and the
// conventions of the Limelight arrays, and the pose models of MegaTag1 and MegaTag2. No NetworkTables, no state; unit
// tested on its own (tests/bridges_vision/test_limelight_math.cpp).
//
// The SDK geometry is float (Box3D) and the arrays are doubles, so this header carries its own double types (Vec3d,
// Quatd, Pose3d) and converts once at the boundary (toPose3d).
//
// Conventions
//  * WPILib / SDK frames (inputs): robot and camera frames are X forward, Y left, Z up; the field is the blue-origin
//    frame (+X toward the red wall, +Y left). Euler angles are WPILib Rotation3d's: roll about X, pitch about Y, yaw
//    about Z, applied extrinsically in that order (R = Rz(yaw) Ry(pitch) Rx(roll)), counter-clockwise positive. A
//    camera tilted UP has a NEGATIVE pitch.
//  * Limelight robot space (camerapose_robotspace[_set]): X forward, Y RIGHT, Z up, origin on the floor under the
//    robot [CS]. A WPILib robot-to-camera transform maps to [x, -y, z, -roll, -pitch, -yaw] (degrees), which is what
//    the sample robots send (samples/vision_robot/README.md): the pitch of a camera tilted up is positive. The sample
//    pins this convention for its yaw 180 mount; the signs for other mounts follow the same rule but are not compared
//    with hardware.
//  * Every pose the device publishes comes in three field flavours [CS]: WPILib blue (the layout's origin), WPILib red
//    (the field rotated by 180 degrees about its centre) and the field-centre origin of the map generator.

#include <array>
#include <cmath>
#include <cstddef>

#include "wpiemu/bridges/vision/limelight_keys.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::bridges::vision::limelight {

inline constexpr double kPi = 3.14159265358979323846;
constexpr double degToRad(double deg) { return deg * kPi / 180.0; }
constexpr double radToDeg(double rad) { return rad * 180.0 / kPi; }

// Wraps to (-180, 180]; a value already inside is returned untouched.
double wrapDegrees(double deg);
// From `fromDeg` to `toDeg` along the shortest arc (`fraction` clamped to [0, 1]), wrapped to (-180, 180].
double lerpAngleDegrees(double fromDeg, double toDeg, double fraction);

// ---- double precision geometry ---------------------------------------------------------------------------------
struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
inline Vec3d operator+(const Vec3d& a, const Vec3d& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3d operator-(const Vec3d& a, const Vec3d& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3d operator*(const Vec3d& v, double s) { return {v.x * s, v.y * s, v.z * s}; }
inline double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double length(const Vec3d& v) { return std::sqrt(dot(v, v)); }
inline double distance(const Vec3d& a, const Vec3d& b) { return length(a - b); }

// Unit quaternion, Hamilton convention, component order like sdk::Quat.
struct Quatd {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};
// (a * b) applies b first, then a.
inline Quatd operator*(const Quatd& a, const Quatd& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quatd conjugate(const Quatd& q) { return {-q.x, -q.y, -q.z, q.w}; }
Quatd normalized(const Quatd& q);
Vec3d rotate(const Quatd& q, const Vec3d& v);

// The pose of a child frame in its parent frame.
struct Pose3d {
    Vec3d position;
    Quatd rotation;
};
// (a * b) is the pose b expressed in the parent frame of a: a.position + a.rotation * b.position, a.rotation *
// b.rotation.
Pose3d operator*(const Pose3d& a, const Pose3d& b);
Pose3d inverse(const Pose3d& pose);
Pose3d toPose3d(const sdk::Pose3& pose);

// ---- Euler angles (WPILib Rotation3d) --------------------------------------------------------------------------
struct EulerRad {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};
Quatd quatFromEulerRad(double rollRad, double pitchRad, double yawRad);
EulerRad eulerFromQuat(const Quatd& q);

// A pose the way the arrays carry it: metres and degrees.
struct PoseDeg {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
};
PoseDeg toPoseDeg(const Pose3d& pose);

// ---- Limelight robot space -------------------------------------------------------------------------------------
using RobotSpacePose = std::array<double, kPose6Size>;  // [forward, right, up, rollDeg, pitchDeg, yawDeg]
RobotSpacePose toLimelightRobotSpace(const Pose3d& robotToCamera);
Pose3d fromLimelightRobotSpace(const RobotSpacePose& values);

// ---- field flavours --------------------------------------------------------------------------------------------
struct FieldSize {
    double lengthM = kWpiFieldLengthM;
    double widthM = kWpiFieldWidthM;
};
// The field rotated by 180 degrees about its centre: (x, y, yaw) -> (L - x, W - y, yaw + 180); z, roll and pitch stay.
PoseDeg toWpiRed(const PoseDeg& wpiBlue, const FieldSize& field = {});
// The origin moved to the field centre, same axes: (x, y) -> (x - L / 2, y - W / 2).
PoseDeg toFieldCentered(const PoseDeg& wpiBlue, const FieldSize& field = {});

// ---- MegaTag ---------------------------------------------------------------------------------------------------
// What the robot wrote to robot_orientation_set (degrees, degrees per second); yaw is in the WPILib blue frame.
struct RobotOrientation {
    double yawDeg = 0.0;
    double yawRateDegPerS = 0.0;
    double pitchDeg = 0.0;
    double pitchRateDegPerS = 0.0;
    double rollDeg = 0.0;
    double rollRateDegPerS = 0.0;
};

// The field-to-camera pose implied by one tag: the layout's tag pose and the tag pose measured in the camera frame.
Pose3d fieldToCameraFromTag(const Pose3d& fieldToTag, const Pose3d& cameraToTag);

// MegaTag1: the robot pose the DEVICE reports. `fieldToCamera` comes from the observations of the physical camera,
// while the robot is placed with the mount the device BELIEVES (`assumedRobotToCamera`): an assumed mount that differs
// from the physical one skews the result exactly as on a real robot.
Pose3d megaTag1RobotPose(const Pose3d& fieldToCamera, const Pose3d& assumedRobotToCamera);

// MegaTag2 [MT2]: the orientation is not solved from the image but supplied by the robot. The camera orientation
// becomes (robot orientation) * (assumed mount rotation); the camera position is the tag's field position minus the
// tag's measured position in the camera frame (`tagInCamera`, from the physical camera) rotated into the field with
// that orientation; the robot position is the camera position minus the rotated assumed mount offset. A wrong yaw
// rotates the camera-to-tag vector and skews the translation, a correct one reproduces the true pose. The result
// carries the supplied roll, pitch and yaw (yaw wrapped to (-180, 180]).
PoseDeg megaTag2RobotPose(const Pose3d& fieldToTag, const Vec3d& tagInCamera, const Pose3d& assumedRobotToCamera,
                          const RobotOrientation& orientation);

}  // namespace wpiemu::bridges::vision::limelight
