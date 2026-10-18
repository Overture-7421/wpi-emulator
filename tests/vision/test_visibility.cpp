#include <doctest/doctest.h>

#include <cmath>

#include "vision_test_support.h"
#include "wpiemu/core/vision/visibility.h"

using namespace vision_test;
using doctest::Approx;

namespace {

constexpr double kTagM = 0.1651;

vis::CameraModel camera90() {
    sdk::CameraIntrinsics in;
    in.widthPx = 1280;
    in.heightPx = 800;
    in.hfovRad = kPi / 2.0;
    return vis::CameraModel(in);
}

vis::VisibilityLimits limits(bool occlusion = false) { return {12.0, 1.309, occlusion}; }

// The camera sits at the origin looking along +X; tags are placed in front of it.
Pose3d camera() { return Pose3d{}; }
Pose3d tagAtYaw(double x, double y, double z, double yawRad) { return poseYpr({x, y, z}, yawRad); }

vis::TagRejection reasonFor(const Pose3d& tag, double size = kTagM,
                            const vis::VisibilityLimits& l = limits()) {
    return vis::evaluateTagVisibility(camera90(), l, camera(), tag, size, nullptr).rejection;
}

}  // namespace

TEST_CASE("vision visibility: a tag facing the camera inside the image is visible and measured") {
    const auto v = vis::evaluateTagVisibility(camera90(), limits(), camera(),
                                              tagAtYaw(2.0, 0.0, 0.0, kPi), kTagM, nullptr);
    CHECK(v.visible());
    CHECK(v.facingCos == Approx(1.0));
    CHECK(v.distanceM == Approx(2.0));
    CHECK(v.minEdgePx == Approx(640.0 * kTagM / 2.0));
    CHECK(v.cornersPx[0].x < v.cornersPx[1].x);  // BL left of BR
    CHECK(v.cornersPx[0].y > v.cornersPx[3].y);  // BL below TL
}

TEST_CASE("vision visibility: a tag behind the camera or reaching behind it is rejected") {
    CHECK(reasonFor(tagAtYaw(-2.0, 0.0, 0.0, 0.0)) == vis::TagRejection::Behind);
    // Centre in front but the plane of the (huge) tag runs through the camera: one corner is behind.
    CHECK(reasonFor(tagAtYaw(0.5, 0.0, 0.0, kPi / 2.0), 4.0) == vis::TagRejection::Behind);
}

TEST_CASE("vision visibility: a tag facing away is rejected") {
    const auto v = vis::evaluateTagVisibility(camera90(), limits(), camera(),
                                              tagAtYaw(2.0, 0.0, 0.0, 0.0), kTagM, nullptr);
    CHECK(v.rejection == vis::TagRejection::Facing);
    CHECK(v.facingCos == Approx(-1.0));
}

TEST_CASE("vision visibility: the view angle limit is cos(maxViewAngle) on dot(normal, direction to camera)") {
    // The normal of a tag at 2 m with yaw (pi - theta) is theta away from the line of sight.
    auto atViewAngle = [](double thetaDeg) { return tagAtYaw(2.0, 0.0, 0.0, kPi - thetaDeg * kPi / 180.0); };
    vis::VisibilityLimits l = limits();
    l.minTagPixels = 0.0;  // isolate the angle rule
    l.maxViewAngleRad = 75.0 * kPi / 180.0;
    CHECK(reasonFor(atViewAngle(70.0), kTagM, l) == vis::TagRejection::None);
    CHECK(reasonFor(atViewAngle(74.0), kTagM, l) == vis::TagRejection::None);
    CHECK(reasonFor(atViewAngle(76.0), kTagM, l) == vis::TagRejection::Facing);
    CHECK(reasonFor(atViewAngle(80.0), kTagM, l) == vis::TagRejection::Facing);

    // Beyond a right angle the rule is the facing test itself (dot > 0).
    l.maxViewAngleRad = kPi;
    CHECK(reasonFor(atViewAngle(89.0), kTagM, l) == vis::TagRejection::None);
    CHECK(reasonFor(atViewAngle(91.0), kTagM, l) == vis::TagRejection::Facing);
}

TEST_CASE("vision visibility: every corner must be inside the image") {
    // 2 m ahead the 90 degree field of view is 2 m wide on each side and 1.25 m high.
    CHECK(reasonFor(tagAtYaw(2.0, -1.9, 0.0, kPi)) == vis::TagRejection::None);
    CHECK(reasonFor(tagAtYaw(2.0, -2.0, 0.0, kPi)) == vis::TagRejection::OutOfImage);  // right edge
    CHECK(reasonFor(tagAtYaw(2.0, 2.0, 0.0, kPi)) == vis::TagRejection::OutOfImage);   // left edge
    CHECK(reasonFor(tagAtYaw(2.0, 0.0, 1.1, kPi)) == vis::TagRejection::None);
    CHECK(reasonFor(tagAtYaw(2.0, 0.0, 1.2, kPi)) == vis::TagRejection::OutOfImage);   // top edge
    CHECK(reasonFor(tagAtYaw(2.0, 0.0, -1.2, kPi)) == vis::TagRejection::OutOfImage);  // bottom edge
}

TEST_CASE("vision visibility: the shortest projected edge must reach minTagPixels") {
    // 640 * 0.1651 / distance pixels
    CHECK(reasonFor(tagAtYaw(8.0, 0.0, 0.0, kPi)) == vis::TagRejection::None);  // 13.2 px
    CHECK(reasonFor(tagAtYaw(8.9, 0.0, 0.0, kPi)) == vis::TagRejection::TooSmall);  // 11.9 px
    CHECK(reasonFor(tagAtYaw(20.0, 0.0, 0.0, kPi)) == vis::TagRejection::TooSmall);

    vis::VisibilityLimits l = limits();
    l.minTagPixels = 5.0;
    CHECK(reasonFor(tagAtYaw(20.0, 0.0, 0.0, kPi), kTagM, l) == vis::TagRejection::None);  // 5.3 px
    // The size passed in is the size used: a 0.3 m tag is large enough at 12 m.
    CHECK(reasonFor(tagAtYaw(12.0, 0.0, 0.0, kPi), 0.3) == vis::TagRejection::None);
    CHECK(reasonFor(tagAtYaw(12.0, 0.0, 0.0, kPi), kTagM) == vis::TagRejection::TooSmall);
}

TEST_CASE("vision visibility: a field shape on the line of sight occludes the tag") {
    BoxWorld w;
    const Pose3d tag = tagAtYaw(6.0, 0.0, 0.0, kPi);
    auto reason = [&](const vis::VisibilityLimits& l, const sdk::IPhysicsWorld* world) {
        return vis::evaluateTagVisibility(camera90(), l, camera(), tag, kTagM, world).rejection;
    };
    CHECK(reason(limits(true), w.world.get()) == vis::TagRejection::None);  // open line of sight

    w.addBox({3.f, 0.f, 0.f}, {0.05f, 2.f, 2.f});  // a wall across the view
    CHECK(reason(limits(true), w.world.get()) == vis::TagRejection::Occluded);
    // Not looked at without the occlusion switch or without a world.
    CHECK(reason(limits(false), w.world.get()) == vis::TagRejection::None);
    CHECK(reason(limits(true), nullptr) == vis::TagRejection::None);
}

TEST_CASE("vision visibility: only layer::Field blocks, and only between the camera and the tag") {
    const Pose3d tag = tagAtYaw(6.0, 0.0, 0.0, kPi);
    auto occluded = [&](BoxWorld& w) {
        return vis::evaluateTagVisibility(camera90(), limits(true), camera(), tag, kTagM, w.world.get())
                   .rejection == vis::TagRejection::Occluded;
    };
    {
        BoxWorld w;  // the robot's own body (any non-Field category) never blocks
        w.addBox({3.f, 0.f, 0.f}, {0.05f, 2.f, 2.f}, sdk::layer::Robot);
        CHECK_FALSE(occluded(w));
    }
    {
        BoxWorld w;  // the wall the tag hangs on: its face is flush with the tag
        w.addBox({6.5f, 0.f, 0.f}, {0.5f, 3.f, 3.f});
        CHECK_FALSE(occluded(w));
    }
    {
        BoxWorld w;  // beside the line of sight
        w.addBox({3.f, 1.5f, 0.f}, {0.05f, 0.5f, 2.f});
        CHECK_FALSE(occluded(w));
    }
    {
        BoxWorld w;  // behind the camera
        w.addBox({-1.f, 0.f, 0.f}, {0.05f, 2.f, 2.f});
        CHECK_FALSE(occluded(w));
    }
}

TEST_CASE("vision visibility: a collider the tag sits slightly inside does not occlude it, a nearer one does") {
    // The ray runs from the camera toward the tag centre and only counts a hit nearer than the
    // centre minus kOcclusionStandoffM (8 cm), because colliders are modelled with tolerances.
    CHECK(vis::kOcclusionStandoffM == Approx(0.08));
    const Pose3d tag = tagAtYaw(6.0, 0.0, 0.0, kPi);
    auto blockedBy = [&](float frontFaceX) {
        BoxWorld w;  // a deep wall whose camera-side face is at x = frontFaceX
        w.addBox({frontFaceX + 0.5f, 0.f, 0.f}, {0.5f, 3.f, 3.f});
        return vis::isLineOfSightBlocked(*w.world, {0.0, 0.0, 0.0}, tag.t);
    };
    CHECK_FALSE(blockedBy(6.00f));  // flush with the tag centre
    CHECK_FALSE(blockedBy(5.95f));  // the tag centre 5 cm inside the box (the 2026 trench tags)
    CHECK_FALSE(blockedBy(5.93f));  // 7 cm in front: still within the standoff
    CHECK(blockedBy(5.90f));        // 10 cm nearer than the tag: an obstruction
    CHECK(blockedBy(3.00f));

    // end to end: the tag embedded 5 cm in its wall is visible, a plate 10 cm in front hides it
    {
        BoxWorld w;
        w.addBox({6.45f, 0.f, 0.f}, {0.5f, 3.f, 3.f});  // front face at x = 5.95
        CHECK(vis::evaluateTagVisibility(camera90(), limits(true), camera(), tag, kTagM, w.world.get())
                  .visible());
    }
    {
        BoxWorld w;
        w.addBox({5.90f, 0.f, 0.f}, {0.005f, 1.f, 1.f});
        CHECK(vis::evaluateTagVisibility(camera90(), limits(true), camera(), tag, kTagM, w.world.get())
                  .rejection == vis::TagRejection::Occluded);
    }
    // a camera closer to the tag than the standoff cannot be blocked
    BoxWorld empty;
    CHECK_FALSE(vis::isLineOfSightBlocked(*empty.world, tag.t + Vec3d{-0.05, 0.0, 0.0}, tag.t));
}
