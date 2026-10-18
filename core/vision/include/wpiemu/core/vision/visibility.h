#pragma once

// Which AprilTags a camera sees (ADR 0017, contract in the vision spec, section E).
//
// A tag is visible when, in this order:
//   1. its centre and all four corners are in front of the camera,
//   2. it faces the camera: dot(tag normal, direction tag -> camera) > 0 and >= cos(maxViewAngle),
//   3. all four projected corners lie inside the image [0,w] x [0,h],
//   4. its shortest projected edge is at least minTagPixels,
//   5. (occlusion on) a ray from the camera origin toward the tag centre does not hit a
//      layer::Field shape nearer than kOcclusionStandoffM short of the centre, other than the
//      collider the tag is mounted on (the own robot is excluded: it is not on that layer).

#include <array>
#include <cstdint>

#include "wpiemu/core/vision/camera_model.h"
#include "wpiemu/core/vision/tag_geometry.h"
#include "wpiemu/core/vision/vision_math.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::core::vision {

// Field colliders are modelled with tolerances and a tag may coincide with, or sit slightly inside,
// the collider face it is mounted on. Only an obstruction at least this much nearer to the camera
// than the tag centre occludes the tag.
inline constexpr double kOcclusionStandoffM = 0.08;

// A tag can also sit deeper inside its mounting collider than any standoff allows: the 2026 trench
// tags are at the mid-plane of the 1.19 m deep trench-arm box. The shapes within this distance of
// the tag centre are the structure the tag is mounted on, part of the tag rather than obstructions.
inline constexpr double kMountProbeRadiusM = 0.01;

enum class TagRejection : std::uint8_t {
    None = 0,  // visible
    Behind,    // centre or a corner is not in front of the camera
    Facing,    // the tag faces away or is seen at more than maxViewAngle
    OutOfImage,  // a projected corner is outside the image
    TooSmall,    // shortest projected edge below minTagPixels
    Occluded,    // a field shape blocks the line of sight
};

struct VisibilityLimits {
    double minTagPixels = 12.0;
    double maxViewAngleRad = 1.309;
    bool occlusion = true;
};

struct TagVisibility {
    TagRejection rejection = TagRejection::None;
    // Valid for every rejection except Behind.
    std::array<sdk::PixelPoint, 4> cornersPx{};
    double minEdgePx = 0.0;
    double facingCos = 0.0;  // dot(tag normal, unit direction tag -> camera)
    double distanceM = 0.0;  // camera origin to tag centre

    bool visible() const { return rejection == TagRejection::None; }
};

// `fieldToCamera` / `fieldToTag`: the camera frame (X forward, Y left, Z up) and the tag frame in
// the field frame. `world` is only used for the line of sight; null skips that check.
TagVisibility evaluateTagVisibility(const CameraModel& camera, const VisibilityLimits& limits,
                                    const Pose3d& fieldToCamera, const Pose3d& fieldToTag,
                                    double tagSizeM, const sdk::IPhysicsWorld* world);

// True when the ray from the camera origin toward the tag centre hits a layer::Field shape before
// it comes within kOcclusionStandoffM of the centre, unless that shape is one the tag is mounted
// on (it lies within kMountProbeRadiusM of the centre).
bool isLineOfSightBlocked(const sdk::IPhysicsWorld& world, const Vec3d& cameraPosition,
                          const Vec3d& tagCenter);

}  // namespace wpiemu::core::vision
