#include <cmath>

#include "common/camera_frustum.h"
#include "test_support.h"

using namespace wpiemu::app;
using wpiemu::app::editor::testing::approx;
namespace sdk = wpiemu::sdk;

TEST_CASE("camera frustum: far rectangle spans the field of view along +X") {
    sdk::CameraConfig camera;
    camera.hFovDeg = 90.0;
    camera.vFovDeg = 60.0;
    const FrustumGeometry g = cameraFrustum(camera, 2.f);

    CHECK(approx(g.pts[kFrustumApex], {0.f, 0.f, 0.f}));
    CHECK(approx(g.lengthM, 2.f));
    const float halfZ = 2.f * std::tan(30.f * 3.14159265f / 180.f);
    CHECK(approx(g.pts[kFrustumTopLeft], {2.f, 2.f, halfZ}));
    CHECK(approx(g.pts[kFrustumTopRight], {2.f, -2.f, halfZ}));
    CHECK(approx(g.pts[kFrustumBottomRight], {2.f, -2.f, -halfZ}));
    CHECK(approx(g.pts[kFrustumBottomLeft], {2.f, 2.f, -halfZ}));
}

TEST_CASE("camera frustum: display length clamps the range and absurd fields of view stay finite") {
    sdk::CameraConfig camera;
    camera.maxRangeM = 50.0;
    CHECK(frustumDisplayLengthM(camera) == kFrustumMaxDisplayLengthM);
    camera.maxRangeM = 1.5;
    CHECK(approx(frustumDisplayLengthM(camera), 1.5f));
    camera.maxRangeM = 0.0;
    CHECK(frustumDisplayLengthM(camera) == kFrustumMinDisplayLengthM);

    camera.hFovDeg = 400.0;
    camera.vFovDeg = -5.0;
    camera.maxRangeM = 50.0;
    const FrustumGeometry g = cameraFrustum(camera);
    for (const auto& p : g.pts) CHECK(std::isfinite(p.x + p.y + p.z));
    CHECK(std::fabs(g.pts[kFrustumTopLeft].y) <= kFrustumMaxHalfExtentM + 1e-3f);
}

TEST_CASE("camera frustum: triangles face outward") {
    sdk::CameraConfig camera;
    const FrustumGeometry g = cameraFrustum(camera, 1.f);
    const sdk::Vec3 centre = (g.pts[0] + g.pts[1] + g.pts[2] + g.pts[3] + g.pts[4]) / 5.f;
    for (const auto& t : kFrustumTriangles) {
        const sdk::Vec3 a = g.pts[t[0]];
        const sdk::Vec3 n = sdk::cross(g.pts[t[1]] - a, g.pts[t[2]] - a);
        CHECK(sdk::dot(n, a - centre) > 0.f);
    }
}
