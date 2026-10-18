#include <doctest/doctest.h>

#include <cmath>

#include "common/robot_placement.h"

using namespace wpiemu::app;
using namespace wpiemu::app::placement;
namespace sdk = wpiemu::sdk;

namespace {

constexpr float kPi = static_cast<float>(sdk::kPi);

// A camera ray that passes through `target` coming from above and from the side.
Ray rayThrough(const Vec3& target, const Vec3& offset = Vec3{0.f, -3.f, 4.f}) {
    const Vec3 origin = target + offset;
    return {origin, sdk::normalized(target - origin)};
}

bool near(const Vec3& a, const Vec3& b, float eps = 1e-3f) { return sdk::length(a - b) < eps; }

}  // namespace

TEST_CASE("placement: a camera ray meets a horizontal plane where it should and misses it when it cannot") {
    const Ray down{Vec3{0.f, 0.f, 2.f}, sdk::normalized(Vec3{1.f, 0.f, -1.f})};
    REQUIRE(intersectHorizontalPlane(down, 0.f));
    CHECK(near(*intersectHorizontalPlane(down, 0.f), Vec3{2.f, 0.f, 0.f}));
    CHECK(near(*intersectHorizontalPlane(down, 1.5f), Vec3{0.5f, 0.f, 1.5f}));   // any height, not just the floor
    CHECK_FALSE(intersectHorizontalPlane({Vec3{0.f, 0.f, 2.f}, Vec3{1.f, 0.f, 0.f}}, 0.f));   // parallel
    CHECK_FALSE(intersectHorizontalPlane({Vec3{0.f, 0.f, 2.f}, sdk::normalized(Vec3{1.f, 0.f, 1.f})}, 0.f));   // looking away
    CHECK_FALSE(intersectHorizontalPlane(down, 3.f));                                                     // plane behind the camera

    // through the real camera: the pixel of a floor point maps back onto that point
    OrbitCamera camera;
    const Vec3 floorPoint{0.6f, -0.4f, 0.f};
    const auto pixel = camera.project(floorPoint, 800.f, 600.f);
    REQUIRE(pixel);
    const auto hit = intersectHorizontalPlane(camera.rayAt(pixel->first, pixel->second, 800.f, 600.f), 0.f);
    REQUIRE(hit);
    CHECK(near(*hit, floorPoint));
}

TEST_CASE("placement: moving keeps the grabbed spot under the cursor, never touches the heading and snaps to the grid") {
    const StartPose start{1.f, 2.f, 0.5f};
    FloorDrag drag = FloorDrag::move(start, Vec3{1.2f, 2.1f, 0.3f});   // grabbed 0.3 m above the floor
    CHECK(drag.mode() == FloorDrag::Mode::Move);
    CHECK(drag.planeZM() == 0.3f);
    CHECK(drag.pose() == start);

    auto pose = drag.update(rayThrough(Vec3{3.2f, 1.1f, 0.3f}));
    REQUIRE(pose);
    CHECK(pose->xM == doctest::Approx(3.0f));
    CHECK(pose->yM == doctest::Approx(1.0f));
    CHECK(pose->yawRad == 0.5f);
    CHECK(drag.pose() == *pose);

    Snap snap;
    snap.moveStepM = 0.5f;
    snap.yawStepRad = kPi / 12.f;
    pose = drag.update(rayThrough(Vec3{3.5f, 1.1f, 0.3f}), snap);
    REQUIRE(pose);
    CHECK(pose->xM == doctest::Approx(3.5f));   // 1 + (3.5 - 1.2) = 3.3 -> the nearer grid line
    CHECK(pose->yM == doctest::Approx(1.0f));
    CHECK(pose->yawRad == 0.5f);                // a move does not snap (or change) the heading
    CHECK(snap.active());
    CHECK_FALSE(Snap{}.active());

    // dragging everywhere is derived from the start, so small steps are not eaten by the snapping
    drag = FloorDrag::move(start, Vec3{1.f, 2.f, 0.f});
    snap = Snap{0.5f, 0.f};
    CHECK(drag.update(rayThrough(Vec3{1.2f, 2.f, 0.f}), snap)->xM == doctest::Approx(1.0f));
    CHECK(drag.update(rayThrough(Vec3{1.3f, 2.f, 0.f}), snap)->xM == doctest::Approx(1.5f));

    // a ray that cannot reach the plane keeps the last pose
    const Ray level{Vec3{0.f, 0.f, 0.3f}, Vec3{1.f, 0.f, 0.f}};
    const StartPose before = drag.pose();
    CHECK_FALSE(drag.update(level));
    CHECK(drag.pose() == before);
}

TEST_CASE("placement: rotating makes the pose face the cursor, snaps the heading and ignores the pivot") {
    const StartPose start{1.f, 1.f, 0.f};
    FloorDrag drag = FloorDrag::rotate(start, 0.f);
    CHECK(drag.mode() == FloorDrag::Mode::Rotate);

    auto pose = drag.update(rayThrough(Vec3{1.f, 3.f, 0.f}));
    REQUIRE(pose);
    CHECK(pose->yawRad == doctest::Approx(kPi / 2.f));
    CHECK(pose->xM == 1.f);   // turning never moves the pose
    CHECK(pose->yM == 1.f);

    pose = drag.update(rayThrough(Vec3{0.f, 1.f, 0.f}));   // facing -X is +-pi, wrapped into (-pi, pi]
    REQUIRE(pose);
    CHECK(std::fabs(pose->yawRad) == doctest::Approx(kPi));

    Snap snap;
    snap.yawStepRad = kPi / 12.f;                                               // 15 degrees
    pose = drag.update(rayThrough(Vec3{2.f, 1.9f, 0.f}), snap);                // atan2(0.9, 1) = 42 degrees
    REQUIRE(pose);
    CHECK(pose->yawRad == doctest::Approx(kPi / 4.f).epsilon(1e-4));            // -> 45 degrees

    const StartPose last = drag.pose();
    CHECK_FALSE(drag.update(rayThrough(Vec3{1.0005f, 1.f, 0.f})));              // on the pivot: no direction
    CHECK(drag.pose() == last);
}

TEST_CASE("placement: turning a moving pose pivots about the spot under the cursor") {
    FloorDrag drag = FloorDrag::move(StartPose{2.f, 0.f, 0.f}, Vec3{3.f, 0.f, 0.f});   // grabbed 1 m ahead of the origin
    REQUIRE(drag.update(rayThrough(Vec3{3.f, 0.f, 0.f})));

    StartPose turned = drag.rotateBy(kPi / 2.f);
    CHECK(turned.xM == doctest::Approx(3.f).epsilon(1e-4));
    CHECK(turned.yM == doctest::Approx(-1.f).epsilon(1e-4));   // the origin swung about the grabbed spot
    CHECK(turned.yawRad == doctest::Approx(kPi / 2.f));
    CHECK(drag.pose() == turned);

    // the next cursor movement continues from the turned pose
    auto pose = drag.update(rayThrough(Vec3{3.f, 0.f, 0.f}));
    REQUIRE(pose);
    CHECK(pose->xM == doctest::Approx(3.f).epsilon(1e-4));
    CHECK(pose->yM == doctest::Approx(-1.f).epsilon(1e-4));
    pose = drag.update(rayThrough(Vec3{4.f, 0.5f, 0.f}));
    REQUIRE(pose);
    CHECK(pose->xM == doctest::Approx(4.f).epsilon(1e-4));
    CHECK(pose->yM == doctest::Approx(-0.5f).epsilon(1e-4));
    CHECK(pose->yawRad == doctest::Approx(kPi / 2.f));

    // snapping rounds the resulting heading, and angles keep wrapping
    Snap snap;
    snap.yawStepRad = kPi / 12.f;
    turned = drag.rotateBy(0.3f, snap);
    CHECK(turned.yawRad == doctest::Approx(kPi / 2.f + kPi / 12.f).epsilon(1e-4));   // 90 + 17 degrees -> 105
    turned = drag.rotateBy(kPi, Snap{});
    CHECK(turned.yawRad < 0.f);   // 105 + 180 degrees wraps to -75
    CHECK(turned.yawRad > -kPi);
}

TEST_CASE("placement: the picking test sees a robot's box where the camera sees it, from any side") {
    LocalBox robot = LocalBox::empty();
    CHECK_FALSE(robot.valid());
    CHECK_FALSE(rayHitDistance(rayThrough(Vec3{}), sdk::Pose3{}, robot));   // nothing to hit
    robot.include(Vec3{-0.4f, -0.4f, -0.2f});
    robot.include(Vec3{0.4f, 0.4f, 0.6f});
    REQUIRE(robot.valid());

    const sdk::Pose3 pose{Vec3{5.f, 5.f, 0.2f}, sdk::Quat::fromYaw(0.7f)};
    const Vec3 top = pose.transformPoint(Vec3{0.1f, 0.f, 0.6f});
    const Ray fromAbove = rayThrough(top, Vec3{0.f, -3.f, 4.f});
    const auto hit = rayHitDistance(fromAbove, pose, robot);
    REQUIRE(hit);
    CHECK(near(fromAbove.origin + fromAbove.direction * *hit, top));                                // enters through the top face
    CHECK_FALSE(rayHitDistance(rayThrough(pose.position + Vec3{2.f, 0.f, 0.f}), pose, robot));     // beside the robot
    CHECK_FALSE(rayHitDistance(rayThrough(pose.position + Vec3{0.f, 0.f, 1.5f}), pose, robot));    // above it, looking past

    const Ray inside{pose.position, Vec3{0.f, 0.f, -1.f}};
    REQUIRE(rayHitDistance(inside, pose, robot));
    CHECK(*rayHitDistance(inside, pose, robot) == 0.f);                                              // starts inside

    // the box turns with the robot: a ray along +X at y = 0.5 just misses the yaw-0 box (half width 0.4) ...
    const sdk::Pose3 square{Vec3{5.f, 5.f, 0.2f}, sdk::Quat{}};
    const Ray along{Vec3{0.f, 5.5f, 0.2f}, Vec3{1.f, 0.f, 0.f}};
    CHECK_FALSE(rayHitDistance(along, square, robot));
    const Ray alongInside{Vec3{0.f, 5.3f, 0.2f}, Vec3{1.f, 0.f, 0.f}};   // parallel to a slab, inside it
    CHECK(*rayHitDistance(alongInside, square, robot) == doctest::Approx(4.6f));
    // ... and a margin makes it forgiving
    CHECK(rayHitDistance(along, square, robot.grown(0.15f)));
}
