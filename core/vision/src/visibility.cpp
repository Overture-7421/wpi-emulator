#include "wpiemu/core/vision/visibility.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace wpiemu::core::vision {

bool isLineOfSightBlocked(const sdk::IPhysicsWorld& world, const Vec3d& cameraPosition,
                          const Vec3d& tagCenter) {
    const Vec3d direction = tagCenter - cameraPosition;
    const double reachM = norm(direction) - kOcclusionStandoffM;
    if (!(reachM > 0.0)) return false;  // closer than the standoff: nothing can stand in between
    const sdk::RayHit hit = world.rayCastClosest(toSdk(cameraPosition), toSdk(direction),
                                                 static_cast<float>(reachM), sdk::layer::Field);
    if (!hit.hit) return false;
    // The structure the tag is mounted on, or sits inside of, is part of the tag.
    std::vector<sdk::ShapeId> mount;
    world.overlapSphere(toSdk(tagCenter), static_cast<float>(kMountProbeRadiusM), sdk::layer::Field,
                        mount);
    return std::find(mount.begin(), mount.end(), hit.shape) == mount.end();
}

TagVisibility evaluateTagVisibility(const CameraModel& camera, const VisibilityLimits& limits,
                                    const Pose3d& fieldToCamera, const Pose3d& fieldToTag,
                                    double tagSizeM, const sdk::IPhysicsWorld* world) {
    TagVisibility out;
    const Pose3d cameraToTag = fieldToCamera.inverse() * fieldToTag;
    out.distanceM = norm(cameraToTag.t);

    const std::array<Vec3d, 4> cornersInCamera = tagCornersIn(cameraToTag, tagSizeM);
    bool inFront = cameraToTag.t.x >= CameraModel::kMinDepthM;
    for (const Vec3d& c : cornersInCamera) inFront = inFront && c.x >= CameraModel::kMinDepthM;
    if (!inFront) {
        out.rejection = TagRejection::Behind;
        return out;
    }

    const Vec3d normal = tagNormal(fieldToTag);
    const Vec3d toCamera = fieldToCamera.t - fieldToTag.t;
    const double toCameraLength = norm(toCamera);
    out.facingCos = toCameraLength > 0.0 ? dot(normal, toCamera) / toCameraLength : 0.0;
    projectPolygon(camera, cornersInCamera, out.cornersPx);
    out.minEdgePx = shortestEdgePx(out.cornersPx);
    if (!(out.facingCos > 0.0) || out.facingCos < std::cos(limits.maxViewAngleRad)) {
        out.rejection = TagRejection::Facing;
        return out;
    }
    for (const sdk::PixelPoint& px : out.cornersPx) {
        if (!camera.inImage(px)) {
            out.rejection = TagRejection::OutOfImage;
            return out;
        }
    }
    if (out.minEdgePx < limits.minTagPixels) {
        out.rejection = TagRejection::TooSmall;
        return out;
    }
    if (limits.occlusion && world != nullptr &&
        isLineOfSightBlocked(*world, fieldToCamera.t, fieldToTag.t))
        out.rejection = TagRejection::Occluded;
    return out;
}

}  // namespace wpiemu::core::vision
