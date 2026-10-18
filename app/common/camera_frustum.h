#pragma once

// Geometry of a camera's field of view, shared by the editor viewport and the run view: the apex and the four far corners of the viewing
// pyramid in the world frame. The camera frame follows the SDK (X forward / Y left / Z up, vision_types.h); the corners are listed as seen
// through the lens: top-left, top-right, bottom-right, bottom-left. No Qt.

#include <array>
#include <cmath>
#include <cstddef>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::app::camera {

using sdk::Pose3;
using sdk::Vec3;

struct Frustum {
    Vec3 apex;
    std::array<Vec3, 4> farCorners;  // TL, TR, BR, BL on the plane `depthM` along the optical axis
};

// `cameraInWorld` is the camera frame in the world frame (+X = optical axis); the angles are the FULL horizontal and vertical field of view.
inline Frustum frustum(const Pose3& cameraInWorld, double hfovRad, double vfovRad, double depthM) {
    const float depth = static_cast<float>(depthM);
    const float halfWidth = static_cast<float>(depthM * std::tan(0.5 * hfovRad));
    const float halfHeight = static_cast<float>(depthM * std::tan(0.5 * vfovRad));
    // +Y is the camera's left and +Z its up, so the top-left corner has positive y and z.
    const Vec3 local[4] = {{depth, halfWidth, halfHeight},
                           {depth, -halfWidth, halfHeight},
                           {depth, -halfWidth, -halfHeight},
                           {depth, halfWidth, -halfHeight}};
    Frustum f;
    f.apex = cameraInWorld.position;
    for (std::size_t i = 0; i < 4; ++i) f.farCorners[i] = cameraInWorld.transformPoint(local[i]);
    return f;
}

}  // namespace wpiemu::app::camera
