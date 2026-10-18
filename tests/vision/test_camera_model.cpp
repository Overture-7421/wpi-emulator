#include <doctest/doctest.h>

#include <cmath>

#include "vision_test_support.h"

using namespace vision_test;
using doctest::Approx;

namespace {

sdk::CameraIntrinsics intrinsics90() {
    sdk::CameraIntrinsics in;
    in.widthPx = 1280;
    in.heightPx = 800;
    in.hfovRad = kPi / 2.0;
    return in;
}

}  // namespace

TEST_CASE("vision camera: the pinhole parameters come from the field of view or the explicit values") {
    sdk::CameraIntrinsics in = intrinsics90();
    const vis::CameraModel derived(in);
    CHECK(derived.widthPx() == 1280);
    CHECK(derived.heightPx() == 800);
    CHECK(derived.fxPx() == Approx(640.0));
    CHECK(derived.fyPx() == Approx(640.0));
    CHECK(derived.cxPx() == Approx(640.0));
    CHECK(derived.cyPx() == Approx(400.0));
    CHECK(derived.hfovRad() == Approx(kPi / 2.0));
    CHECK(derived.vfovRad() == Approx(2.0 * std::atan(400.0 / 640.0)));

    in.fxPx = 900.0;
    in.fyPx = 910.0;
    in.cxPx = 650.0;
    in.cyPx = 390.0;
    const vis::CameraModel explicitModel(in);
    CHECK(explicitModel.fxPx() == 900.0);
    CHECK(explicitModel.fyPx() == 910.0);
    CHECK(explicitModel.cxPx() == 650.0);
    CHECK(explicitModel.cyPx() == 390.0);
    CHECK(explicitModel.hfovRad() == Approx(2.0 * std::atan(640.0 / 900.0)));
    CHECK(explicitModel.vfovRad() == Approx(2.0 * std::atan(400.0 / 910.0)));
}

TEST_CASE("vision camera: a point ahead and to the left lands left of centre, a point above lands above") {
    const vis::CameraModel m(intrinsics90());
    const auto centre = m.project({1.0, 0.0, 0.0});
    REQUIRE(centre);
    CHECK(centre->x == Approx(640.0));
    CHECK(centre->y == Approx(400.0));

    // 1 m ahead, 0.5 m to the LEFT (+Y): u = cx - fx * 0.5
    const auto left = m.project({1.0, 0.5, 0.0});
    REQUIRE(left);
    CHECK(left->x == Approx(320.0));
    CHECK(left->y == Approx(400.0));
    CHECK(left->x < centre->x);

    // 2 m ahead, 0.5 m UP (+Z): v = cy - fy * 0.25, image y grows downwards
    const auto up = m.project({2.0, 0.0, 0.5});
    REQUIRE(up);
    CHECK(up->x == Approx(640.0));
    CHECK(up->y == Approx(240.0));
    CHECK(up->y < centre->y);

    CHECK_FALSE(m.project({0.0, 1.0, 0.0}));   // on the image plane
    CHECK_FALSE(m.project({-1.0, 0.0, 0.0}));  // behind
}

TEST_CASE("vision camera: yaw is positive to the right of the axis and pitch positive upwards") {
    const vis::CameraModel m(intrinsics90());
    const auto right = m.project({2.0, -1.0, 0.0});  // to the right (-Y)
    const auto left = m.project({2.0, 1.0, 0.0});
    const auto up = m.project({2.0, 0.0, 1.0});
    const auto down = m.project({2.0, 0.0, -1.0});
    REQUIRE((right && left && up && down));
    CHECK(m.yawRad(*right) == Approx(std::atan(0.5)));
    CHECK(m.yawRad(*left) == Approx(-std::atan(0.5)));
    CHECK(m.pitchRad(*up) == Approx(std::atan(0.5)));
    CHECK(m.pitchRad(*down) == Approx(-std::atan(0.5)));
    CHECK(m.yawRad({640.0, 400.0}) == 0.0);
    CHECK(m.pitchRad({640.0, 400.0}) == 0.0);
}

TEST_CASE("vision camera: unprojecting a projected point returns it and rays are unit vectors") {
    sdk::CameraIntrinsics in = intrinsics90();
    in.fxPx = 700.0;
    in.fyPx = 690.0;
    in.cxPx = 655.0;
    in.cyPx = 395.0;
    const vis::CameraModel m(in);
    const Vec3d points[] = {{1.0, 0.0, 0.0}, {3.0, -0.7, 0.4}, {0.5, 0.2, -0.1}, {8.0, 3.0, 2.0}};
    for (const Vec3d& p : points) {
        const auto px = m.project(p);
        REQUIRE(px);
        const Vec3d back = m.unproject(*px, p.x);
        CHECK(vis::norm(back - p) < 1e-9);
        const Vec3d ray = m.rayDirection(*px);
        CHECK(vis::norm(ray) == Approx(1.0));
        CHECK(vis::norm(vis::cross(ray, vis::normalized(p))) < 1e-12);
    }
}

TEST_CASE("vision camera: the image spans [0,w] x [0,h] with the edges included") {
    const vis::CameraModel m(intrinsics90());
    CHECK(m.inImage({0.0, 0.0}));
    CHECK(m.inImage({1280.0, 800.0}));
    CHECK(m.inImage({640.0, 400.0}));
    CHECK_FALSE(m.inImage({-0.001, 400.0}));
    CHECK_FALSE(m.inImage({1280.001, 400.0}));
    CHECK_FALSE(m.inImage({640.0, -0.001}));
    CHECK_FALSE(m.inImage({640.0, 800.001}));
}

TEST_CASE("vision camera: polygon area, shortest edge and quad centre") {
    const sdk::PixelPoint square[] = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}};
    const sdk::PixelPoint reversed[] = {{0.0, 100.0}, {100.0, 100.0}, {100.0, 0.0}, {0.0, 0.0}};
    CHECK(vis::polygonAreaPx(square) == Approx(10000.0));
    CHECK(vis::polygonAreaPx(reversed) == Approx(10000.0));

    const sdk::PixelPoint rect[] = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 40.0}, {0.0, 40.0}};
    CHECK(vis::shortestEdgePx(rect) == Approx(40.0));
    CHECK(vis::shortestEdgePx(std::span<const sdk::PixelPoint>{}) == 0.0);

    // The diagonals of a projected square meet at the image of its centre, not at the corner mean.
    const vis::CameraModel m(intrinsics90());
    const Pose3d cameraToTag = poseYpr({2.0, 0.3, 0.1}, kPi - 0.9);
    const auto corners = vis::projectTagCorners(m, cameraToTag, 0.5);
    REQUIRE(corners);
    const sdk::PixelPoint centre = vis::quadCenterPx(*corners);
    const auto truth = m.project(cameraToTag.t);
    REQUIRE(truth);
    CHECK(centre.x == Approx(truth->x).epsilon(1e-12));
    CHECK(centre.y == Approx(truth->y).epsilon(1e-12));
    const double meanX = ((*corners)[0].x + (*corners)[1].x + (*corners)[2].x + (*corners)[3].x) / 4.0;
    CHECK(std::abs(meanX - truth->x) > 1e-3);
}

TEST_CASE("vision camera: projecting a polygon fails as soon as a point is behind the camera") {
    const vis::CameraModel m(intrinsics90());
    const Vec3d ahead[] = {{1.0, 0.0, 0.0}, {2.0, 0.5, 0.0}};
    sdk::PixelPoint out[2];
    CHECK(vis::projectPolygon(m, ahead, out));
    CHECK(out[1].x == Approx(640.0 - 640.0 * 0.25));
    const Vec3d mixed[] = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}};
    CHECK_FALSE(vis::projectPolygon(m, mixed, out));
}

TEST_CASE("vision tag: corners are BL, BR, TR, TL as seen from the tag front") {
    const auto local = vis::tagCornersInTagFrame(0.2);
    CHECK(local[0].y == Approx(-0.1));  // BL
    CHECK(local[0].z == Approx(-0.1));
    CHECK(local[1].y == Approx(0.1));  // BR
    CHECK(local[1].z == Approx(-0.1));
    CHECK(local[2].y == Approx(0.1));  // TR
    CHECK(local[2].z == Approx(0.1));
    CHECK(local[3].y == Approx(-0.1));  // TL
    CHECK(local[3].z == Approx(0.1));
    for (const Vec3d& c : local) CHECK(c.x == 0.0);

    // A tag 2 m ahead that faces the camera (yaw pi): its viewer's right is the image's right.
    const vis::CameraModel m(intrinsics90());
    const auto px = vis::projectTagCorners(m, poseYpr({2.0, 0.0, 0.0}, kPi), 0.2);
    REQUIRE(px);
    CHECK((*px)[0].x == Approx(608.0));  // BL
    CHECK((*px)[0].y == Approx(432.0));
    CHECK((*px)[1].x == Approx(672.0));  // BR
    CHECK((*px)[1].y == Approx(432.0));
    CHECK((*px)[2].x == Approx(672.0));  // TR
    CHECK((*px)[2].y == Approx(368.0));
    CHECK((*px)[3].x == Approx(608.0));  // TL
    CHECK((*px)[3].y == Approx(368.0));
    CHECK(vis::tagNormal(poseYpr({2.0, 0.0, 0.0}, kPi)).x == Approx(-1.0));
}
