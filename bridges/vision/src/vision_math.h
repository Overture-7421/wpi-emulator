#pragma once

// Pose helpers shared by the vision bridges (internal).

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::bridges::vision::detail {

struct EulerDeg {
    double rollDeg = 0, pitchDeg = 0, yawDeg = 0;
};

// Inverse of sdk::Quat::fromEuler (intrinsic Z-Y-X, WPILib Rotation3d roll/pitch/yaw).
inline EulerDeg eulerDeg(const sdk::Quat& q) {
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return {roll * sdk::kRadToDeg, pitch * sdk::kRadToDeg, yaw * sdk::kRadToDeg};
}

inline double wrapDeg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg > 180.0) deg -= 360.0;
    if (deg <= -180.0) deg += 360.0;
    return deg;
}

// Limelight pose array: x, y, z [m], roll, pitch, yaw [deg].
inline std::vector<double> pose6(const sdk::Pose3& p) {
    const EulerDeg e = eulerDeg(p.rotation);
    return {p.position.x, p.position.y, p.position.z, wrapDeg(e.rollDeg), wrapDeg(e.pitchDeg), wrapDeg(e.yawDeg)};
}

enum class FieldOrigin { Blue, Red, Center };

// Re-expresses a pose of our blue-origin field frame (origin at the blue-alliance right corner, +X toward red)
// in another Limelight field convention: Red = origin at the opposite corner, axes rotated by 180 degrees
// (x' = L - x, y' = W - y, yaw + 180); Center = origin at the field center, same axes.
inline sdk::Pose3 inFieldOrigin(const sdk::Pose3& blue, FieldOrigin origin, double fieldLengthM,
                                double fieldWidthM) {
    switch (origin) {
        case FieldOrigin::Blue:
            return blue;
        case FieldOrigin::Center:
            return {{static_cast<float>(blue.position.x - 0.5 * fieldLengthM),
                     static_cast<float>(blue.position.y - 0.5 * fieldWidthM), blue.position.z},
                    blue.rotation};
        case FieldOrigin::Red: {
            const sdk::Quat turn = sdk::Quat::fromYaw(sdk::kPiF);
            return {{static_cast<float>(fieldLengthM - blue.position.x),
                     static_cast<float>(fieldWidthM - blue.position.y), blue.position.z},
                    (turn * blue.rotation).normalized()};
        }
    }
    return blue;
}

}  // namespace wpiemu::bridges::vision::detail
