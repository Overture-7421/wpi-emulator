#include <doctest/doctest.h>

#include <cmath>

#include "common/camera_frustum.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::app::camera;
namespace sdk = wpiemu::sdk;

namespace {

bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = 1e-4f) { return sdk::length(a - b) < eps; }

}  // namespace

TEST_CASE("camera frustum: corners are listed top-left, top-right, bottom-right, bottom-left through the lens") {
    const sdk::Pose3 camera({1.f, 2.f, 3.f}, sdk::Quat::identity());  // facing +X
    const Frustum f = frustum(camera, sdk::kPi / 2.0, sdk::kPi / 3.0, 2.0);
    const float h = 2.f * std::tan(static_cast<float>(sdk::kPi) / 6.f);
    CHECK(near(f.apex, {1.f, 2.f, 3.f}));
    CHECK(near(f.farCorners[0], {3.f, 4.f, 3.f + h}));  // the camera's left is +Y, its up is +Z
    CHECK(near(f.farCorners[1], {3.f, 0.f, 3.f + h}));
    CHECK(near(f.farCorners[2], {3.f, 0.f, 3.f - h}));
    CHECK(near(f.farCorners[3], {3.f, 4.f, 3.f - h}));
}

TEST_CASE("camera frustum: a turned camera carries its pyramid with it") {
    const sdk::Pose3 camera({0.f, 0.f, 0.f}, sdk::Quat::fromYaw(static_cast<float>(sdk::kPi) / 2.f));  // facing +Y
    const Frustum f = frustum(camera, sdk::kPi / 2.0, sdk::kPi / 2.0, 1.0);
    CHECK(near(f.farCorners[0], {-1.f, 1.f, 1.f}));  // left of a camera that faces +Y is -X
    CHECK(near(f.farCorners[2], {1.f, 1.f, -1.f}));
}
