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

// Limelight pose array: x, y, z [m], roll, pitch, yaw [deg].
inline std::vector<double> pose6(const sdk::Pose3& p) {
    const sdk::Euler e = p.rotation.toEuler();
    const auto deg = [](float rad) { return sdk::wrapAngleDeg(sdk::radToDeg(static_cast<double>(rad))); };
    return {p.position.x, p.position.y, p.position.z, deg(e.rollRad), deg(e.pitchRad), deg(e.yawRad)};
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
