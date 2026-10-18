#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "vision_test_support.h"
#include "wpiemu/core/vision/pose_solver.h"
#include "wpiemu/core/vision/visibility.h"
#include "wpiemu/sdk/core/rng.h"

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

// A tag in front of the camera (bearing within +-25 deg, elevation within +-15 deg), turned toward it
// and then tilted away by up to `maxTiltDeg` about a random axis, spun by a random in-plane roll.
Pose3d randomTag(sdk::Rng& rng, double maxTiltDeg, double minDistM = 0.8, double maxDistM = 6.0) {
    const double d = rng.uniform(minDistM, maxDistM);
    const Vec3d p{d, -d * std::tan(rng.uniform(-0.43, 0.43)), d * std::tan(rng.uniform(-0.26, 0.26))};
    const Vec3d toCamera = vis::normalized(-p);
    const Vec3d a = vis::normalized(vis::cross(toCamera, {0.0, 0.0, 1.0}));
    const Vec3d b = vis::cross(toCamera, a);
    const double tilt = rng.uniform(0.0, maxTiltDeg * kPi / 180.0);
    const double dir = rng.uniform(0.0, 2.0 * kPi);
    const Vec3d normal = vis::normalized(toCamera * std::cos(tilt) +
                                         (a * std::cos(dir) + b * std::sin(dir)) * std::sin(tilt));
    return {rotationWithXAxis(normal, rng.uniform(-kPi, kPi)), p};
}

std::array<sdk::PixelPoint, 4> cornersOf(const vis::CameraModel& m, const Pose3d& cameraToTag,
                                         double size) {
    const auto px = vis::projectTagCorners(m, cameraToTag, size);
    REQUIRE(px);
    return *px;
}

}  // namespace

TEST_CASE("vision solver: exact corners give the exact tag pose over many views") {
    const vis::CameraModel m = camera90();
    sdk::Rng rng(42);
    double worstPosM = 0.0, worstRotRad = 0.0, worstErrPx = 0.0;
    bool allExact = true;
    for (int i = 0; i < 600; ++i) {
        const double size = i % 2 == 0 ? kTagM : 0.5;
        const Pose3d truth = randomTag(rng, 70.0);
        const auto corners = cornersOf(m, truth, size);
        const vis::SquarePnpResult r = vis::solveSquarePnp(m, size, corners);
        REQUIRE(r.valid);
        worstPosM = std::max(worstPosM, vis::norm(r.best.cameraToTag.t - truth.t));
        worstRotRad = std::max(worstRotRad, vis::rotationDistanceRad(r.best.cameraToTag.R, truth.R));
        worstErrPx = std::max(worstErrPx, r.best.reprojErrPx);
        allExact = allExact && r.ambiguity == 0.0 && r.alt.reprojErrPx >= r.best.reprojErrPx;
    }
    CHECK(worstPosM < 1e-8);
    CHECK(worstRotRad < 1e-8);
    CHECK(worstErrPx < 1e-9);
    CHECK(allExact);  // an exact fit is unambiguous
}

TEST_CASE("vision solver: a tag seen head-on has one pose and no ambiguity when exact") {
    const vis::CameraModel m = camera90();
    const Vec3d positions[] = {{3.0, 0.0, 0.0}, {3.0, 0.4, 0.2}, {1.2, -0.5, 0.3}};
    for (const Vec3d& p : positions) {
        // the normal points straight at the camera centre
        const Pose3d truth{rotationWithXAxis(-p, 0.3), p};
        const auto r = vis::solveSquarePnp(m, kTagM, cornersOf(m, truth, kTagM));
        REQUIRE(r.valid);
        CHECK(vis::norm(r.best.cameraToTag.t - truth.t) < 1e-9);
        CHECK(vis::rotationDistanceRad(r.best.cameraToTag.R, truth.R) < 1e-8);
        CHECK(r.best.reprojErrPx < 1e-9);
        CHECK(r.ambiguity == 0.0);
        // both solutions coincide
        CHECK(vis::norm(r.alt.cameraToTag.t - r.best.cameraToTag.t) < 1e-6);
        CHECK(vis::rotationDistanceRad(r.alt.cameraToTag.R, r.best.cameraToTag.R) < 1e-6);
    }
}

TEST_CASE("vision solver: the camera-to-tag pose is the tag frame expressed in the camera frame") {
    const vis::CameraModel m = camera90();
    // A tag 2 m ahead facing the camera: its outward normal (+X) points back along -X, its up axis
    // stays up, and the viewer's right (+Y) is the camera's right (-Y).
    const Pose3d truth = poseYpr({2.0, 0.0, 0.0}, kPi);
    const auto r = vis::solveSquarePnp(m, kTagM, cornersOf(m, truth, kTagM));
    REQUIRE(r.valid);
    const Pose3d& got = r.best.cameraToTag;
    CHECK(got.t.x == Approx(2.0));
    CHECK(got.t.y == Approx(0.0).epsilon(1e-9));
    CHECK(got.t.z == Approx(0.0).epsilon(1e-9));
    CHECK(got.R.column(0).x == Approx(-1.0));  // tag +X (normal)
    CHECK(got.R.column(1).y == Approx(-1.0));  // tag +Y (viewer's right)
    CHECK(got.R.column(2).z == Approx(1.0));   // tag +Z (up)

    // A tag to the camera's left and above it sits at +Y / +Z in the camera frame.
    const Pose3d lefthigh = poseYpr({3.0, 0.5, 0.25}, kPi);
    const auto r2 = vis::solveSquarePnp(m, kTagM, cornersOf(m, lefthigh, kTagM));
    REQUIRE(r2.valid);
    CHECK(r2.best.cameraToTag.t.y == Approx(0.5));
    CHECK(r2.best.cameraToTag.t.z == Approx(0.25));
}

TEST_CASE("vision solver: an oblique tag has a second pose that does not fit and a zero ambiguity when exact") {
    const vis::CameraModel m = camera90();
    const double size = 0.5;  // big and close: strong perspective, a clearly different alternative
    const Pose3d truth{rotationWithXAxis({-1.0, 0.8, 0.0}, 0.0), {1.6, 0.0, 0.0}};
    const auto r = vis::solveSquarePnp(m, size, cornersOf(m, truth, size));
    REQUIRE(r.valid);
    CHECK(vis::norm(r.best.cameraToTag.t - truth.t) < 1e-8);
    CHECK(vis::rotationDistanceRad(r.best.cameraToTag.R, truth.R) < 1e-8);
    CHECK(r.ambiguity == 0.0);
    // the alternative is another pose with the tag in front of the camera and a real error
    CHECK(r.alt.reprojErrPx > 1e-3);
    CHECK(vis::rotationDistanceRad(r.alt.cameraToTag.R, r.best.cameraToTag.R) > 0.1);
    CHECK(r.alt.cameraToTag.t.x > 0.0);
    CHECK(r.alt.cameraToTag.R.determinant() == Approx(1.0));
}

TEST_CASE("vision solver: ambiguity is the best over the alternative error, in [0,1]") {
    const vis::CameraModel m = camera90();
    sdk::Rng rng(7);
    int oblique = 0;
    double obliqueSum = 0.0;
    for (int i = 0; i < 200; ++i) {
        const Pose3d truth = randomTag(rng, 70.0, 1.0, 4.0);
        auto corners = cornersOf(m, truth, kTagM);
        for (auto& c : corners) {  // 0.3 px of corner noise
            c.x += 0.3 * (rng.nextDouble() - 0.5) * 2.0;
            c.y += 0.3 * (rng.nextDouble() - 0.5) * 2.0;
        }
        const auto r = vis::solveSquarePnp(m, kTagM, corners);
        REQUIRE(r.valid);
        CHECK(r.ambiguity >= 0.0);
        CHECK(r.ambiguity <= 1.0);
        CHECK(r.best.reprojErrPx <= r.alt.reprojErrPx);
        if (r.best.reprojErrPx > 1e-9)
            CHECK(r.ambiguity == Approx(r.best.reprojErrPx / r.alt.reprojErrPx));
        ++oblique;
        obliqueSum += r.ambiguity;
    }
    CHECK(oblique == 200);
    CHECK(obliqueSum / oblique <= 1.0);
}

TEST_CASE("vision solver: reprojection error is the RMS per coordinate") {
    const vis::CameraModel m = camera90();
    const Pose3d truth = poseYpr({2.0, 0.2, 0.1}, kPi - 0.3);
    auto corners = cornersOf(m, truth, kTagM);
    CHECK(vis::squareReprojectionErrorPx(m, truth, kTagM, corners) < 1e-12);
    for (auto& c : corners) c.x += 1.0;  // 1 px along x for every corner: sqrt(4 / 8)
    CHECK(vis::squareReprojectionErrorPx(m, truth, kTagM, corners) == Approx(std::sqrt(0.5)));
    // a pose behind the camera has no finite error
    CHECK(std::isinf(vis::squareReprojectionErrorPx(m, poseYpr({-2.0, 0.0, 0.0}, 0.0), kTagM, corners)));
}

TEST_CASE("vision solver: degenerate corners are refused") {
    const vis::CameraModel m = camera90();
    const std::array<sdk::PixelPoint, 4> collinear{{{100.0, 100.0}, {200.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}}};
    CHECK_FALSE(vis::solveSquarePnp(m, kTagM, collinear).valid);
    const std::array<sdk::PixelPoint, 4> nan{{{std::nan(""), 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}};
    CHECK_FALSE(vis::solveSquarePnp(m, kTagM, nan).valid);
    const auto good = cornersOf(m, poseYpr({2.0, 0.0, 0.0}, kPi), kTagM);
    CHECK_FALSE(vis::solveSquarePnp(m, 0.0, good).valid);
    CHECK_FALSE(vis::solveSquarePnp(m, -1.0, good).valid);
}

TEST_CASE("vision solver: single-tag fit to noisy corners reaches the least-squares minimum") {
    const vis::CameraModel m = camera90();
    sdk::Rng rng(11);
    const double sigma = 1.0;
    double sumSquares = 0.0;
    int n = 0;
    for (int i = 0; i < 300; ++i) {
        const Pose3d truth = randomTag(rng, 45.0, 1.0, 3.0);
        auto corners = cornersOf(m, truth, 0.3);
        for (auto& c : corners) {
            // Box-Muller
            const double u1 = rng.nextDouble(), u2 = rng.nextDouble();
            const double r = std::sqrt(-2.0 * std::log(1.0 - u1));
            c.x += sigma * r * std::cos(2.0 * kPi * u2);
            c.y += sigma * r * std::sin(2.0 * kPi * u2);
        }
        const auto r = vis::solveSquarePnp(m, 0.3, corners);
        REQUIRE(r.valid);
        sumSquares += r.best.reprojErrPx * r.best.reprojErrPx;
        ++n;
    }
    // 8 residuals, 6 fitted parameters: E[RMS^2] = sigma^2 * 2 / 8.
    CHECK(std::sqrt(sumSquares / n) == Approx(0.5 * sigma).epsilon(0.12));
}

TEST_CASE("vision multi-tag: exact corners of several tags give the exact camera pose") {
    const vis::CameraModel m = camera90();
    const Pose3d fieldToCamera = poseYpr({3.0, 2.0, 0.6}, 0.5, 0.15, 0.05);
    const std::vector<Pose3d> tagPoses = {
        poseYpr({8.0, 1.5, 0.5}, kPi), poseYpr({8.0, 2.5, 0.8}, kPi),
        poseYpr({8.0, 3.5, 0.4}, kPi), poseYpr({6.0, 5.5, 0.6}, -kPi / 2.0)};
    const vis::VisibilityLimits limits{12.0, 1.309, false};
    std::vector<vis::TagCornerObservation> seen;
    for (const Pose3d& tag : tagPoses) {
        const auto v = vis::evaluateTagVisibility(m, limits, fieldToCamera, tag, kTagM, nullptr);
        REQUIRE(v.visible());
        seen.push_back({tag, kTagM, v.cornersPx});
    }

    for (const std::size_t count : {std::size_t{2}, std::size_t{3}, std::size_t{4}}) {
        const std::span<const vis::TagCornerObservation> subset(seen.data(), count);
        const vis::MultiTagPnpResult r = vis::solveMultiTagPnp(m, subset);
        REQUIRE(r.valid);
        CHECK(vis::norm(r.fieldToCamera.t - fieldToCamera.t) < 1e-9);
        CHECK(vis::rotationDistanceRad(r.fieldToCamera.R, fieldToCamera.R) < 1e-9);
        CHECK(r.reprojErrPx < 1e-9);
    }
}

TEST_CASE("vision multi-tag: precomputed seeds give the same answer as the internal ones") {
    const vis::CameraModel m = camera90();
    const Pose3d fieldToCamera = poseYpr({1.0, -1.0, 0.5}, -0.4, 0.1);
    const std::vector<Pose3d> tagPoses = {poseYpr({6.0, -2.0, 0.5}, kPi), poseYpr({6.0, -0.5, 0.9}, kPi)};
    const vis::VisibilityLimits limits{12.0, 1.309, false};
    std::vector<vis::TagCornerObservation> seen;
    std::vector<vis::SquarePnpResult> seeds;
    for (const Pose3d& tag : tagPoses) {
        const auto v = vis::evaluateTagVisibility(m, limits, fieldToCamera, tag, kTagM, nullptr);
        REQUIRE(v.visible());
        seen.push_back({tag, kTagM, v.cornersPx});
        seeds.push_back(vis::solveSquarePnp(m, kTagM, v.cornersPx));
    }
    const auto a = vis::solveMultiTagPnp(m, seen);
    const auto b = vis::solveMultiTagPnp(m, seen, seeds);
    REQUIRE((a.valid && b.valid));
    CHECK(vis::norm(a.fieldToCamera.t - b.fieldToCamera.t) < 1e-9);
    CHECK(vis::norm(a.fieldToCamera.t - fieldToCamera.t) < 1e-9);
}

TEST_CASE("vision multi-tag: needs two tags and usable corners") {
    const vis::CameraModel m = camera90();
    const Pose3d tag = poseYpr({4.0, 0.0, 0.0}, kPi);
    const vis::TagCornerObservation one{tag, kTagM, cornersOf(m, tag, kTagM)};
    CHECK_FALSE(vis::solveMultiTagPnp(m, std::span<const vis::TagCornerObservation>(&one, 1)).valid);
    CHECK_FALSE(vis::solveMultiTagPnp(m, std::span<const vis::TagCornerObservation>{}).valid);

    vis::TagCornerObservation flat = one;
    flat.cornersPx = {{{100.0, 100.0}, {200.0, 100.0}, {300.0, 100.0}, {400.0, 100.0}}};
    const vis::TagCornerObservation both[] = {flat, flat};
    CHECK_FALSE(vis::solveMultiTagPnp(m, both).valid);
}

TEST_CASE("vision multi-tag: noisy corners are fitted to the least-squares minimum and beat a single tag") {
    const vis::CameraModel m = camera90();
    sdk::Rng rng(5);
    const double sigma = 0.8;
    const Pose3d fieldToCamera = poseYpr({0.0, 0.0, 0.6}, 0.0);
    const std::vector<Pose3d> tagPoses = {poseYpr({5.0, -1.5, 0.5}, kPi), poseYpr({5.5, 0.2, 0.9}, kPi),
                                          poseYpr({5.0, 1.6, 0.4}, kPi)};
    const vis::VisibilityLimits limits{12.0, 1.309, false};
    double sumSquares = 0.0, multiErrM = 0.0, singleErrM = 0.0;
    const int trials = 200;
    for (int i = 0; i < trials; ++i) {
        std::vector<vis::TagCornerObservation> seen;
        for (const Pose3d& tag : tagPoses) {
            auto v = vis::evaluateTagVisibility(m, limits, fieldToCamera, tag, kTagM, nullptr);
            REQUIRE(v.visible());
            for (auto& c : v.cornersPx) {
                const double u1 = rng.nextDouble(), u2 = rng.nextDouble();
                const double r = std::sqrt(-2.0 * std::log(1.0 - u1));
                c.x += sigma * r * std::cos(2.0 * kPi * u2);
                c.y += sigma * r * std::sin(2.0 * kPi * u2);
            }
            seen.push_back({tag, kTagM, v.cornersPx});
        }
        const auto multi = vis::solveMultiTagPnp(m, seen);
        REQUIRE(multi.valid);
        sumSquares += multi.reprojErrPx * multi.reprojErrPx;
        multiErrM += vis::norm(multi.fieldToCamera.t - fieldToCamera.t);

        const auto single = vis::solveSquarePnp(m, kTagM, seen[1].cornersPx);
        REQUIRE(single.valid);
        const Pose3d fromSingle = seen[1].fieldToTag * single.best.cameraToTag.inverse();
        singleErrM += vis::norm(fromSingle.t - fieldToCamera.t);
    }
    // 24 residuals, 6 parameters: E[RMS^2] = sigma^2 * 18 / 24.
    CHECK(std::sqrt(sumSquares / trials) == Approx(sigma * std::sqrt(0.75)).epsilon(0.08));
    CHECK(multiErrM < singleErrM);
    CHECK(multiErrM / trials < 0.3);  // metres, at 5 m with 0.8 px of noise: the depth is the weak axis
}
