#pragma once

// AprilTag geometry shared by visibility, the pose solver and the vision system.
//
// Tag frame (WPILib AprilTagFieldLayout convention): origin at the tag centre, +X the outward
// normal (out of the face), +Z up, +Y the viewer's right when looking at the face. The four corners
// are always listed BL, BR, TR, TL as seen from the tag front (PhotonVision TargetModel order).
//   BL (0,-s/2,-s/2)   BR (0,+s/2,-s/2)   TR (0,+s/2,+s/2)   TL (0,-s/2,+s/2)

#include <array>
#include <optional>

#include "wpiemu/core/vision/camera_model.h"
#include "wpiemu/core/vision/vision_math.h"

namespace wpiemu::core::vision {

inline std::array<Vec3d, 4> tagCornersInTagFrame(double sizeM) {
    const double h = 0.5 * sizeM;
    return {{{0.0, -h, -h}, {0.0, h, -h}, {0.0, h, h}, {0.0, -h, h}}};
}

// Outward normal (tag +X) of a tag in the frame `tagPose` maps into.
inline Vec3d tagNormal(const Pose3d& tagPose) { return tagPose.applyRotation({1.0, 0.0, 0.0}); }

// The four corners in the frame `tagPose` maps into (BL, BR, TR, TL).
inline std::array<Vec3d, 4> tagCornersIn(const Pose3d& tagPose, double sizeM) {
    std::array<Vec3d, 4> out = tagCornersInTagFrame(sizeM);
    for (Vec3d& c : out) c = tagPose.apply(c);
    return out;
}

// Pixels of the four tag corners seen by a camera, `cameraToTag` being the tag pose in the camera
// frame. nullopt when any corner is not in front of the camera.
inline std::optional<std::array<sdk::PixelPoint, 4>> projectTagCorners(const CameraModel& camera,
                                                                       const Pose3d& cameraToTag,
                                                                       double sizeM) {
    const std::array<Vec3d, 4> corners = tagCornersIn(cameraToTag, sizeM);
    std::array<sdk::PixelPoint, 4> px{};
    if (!projectPolygon(camera, corners, px)) return std::nullopt;
    return px;
}

}  // namespace wpiemu::core::vision
