#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "wpiemu/sdk/physics/remote_spec_limits.h"

using namespace wpiemu::sdk;

namespace {

ChassisSpec chassis() {
    ChassisSpec c;
    c.sizeM = {0.7f, 0.6f, 0.2f};
    c.comHeightM = 0.2f;
    c.comOffsetM = {0.05f, -0.1f, 0.f};
    return c;
}

Pose3 at(float x, float y, float z) { return {Vec3{x, y, z}, Quat{}}; }

constexpr float kEps = 1e-6f;

}  // namespace

TEST_CASE("zone pose limits: the table has the documented reach margin and height") {
    CHECK(kRemoteSpecLimits.zoneReachMarginM == doctest::Approx(0.75));
    CHECK(kRemoteSpecLimits.zoneMaxHeightAboveFloorM == doctest::Approx(1.5));
}

TEST_CASE("zone pose limits: the envelope is the frame box grown by the reach margin, floor to 1.5 m") {
    const ChassisSpec c = chassis();
    const ZoneEnvelope box = remoteZoneEnvelope(c);
    // The frame box is centred at -comOffset in the chassis frame (the origin is the COM).
    CHECK(box.min.x == doctest::Approx(-0.05f - 0.35f - 0.75f));
    CHECK(box.max.x == doctest::Approx(-0.05f + 0.35f + 0.75f));
    CHECK(box.min.y == doctest::Approx(0.1f - 0.3f - 0.75f));
    CHECK(box.max.y == doctest::Approx(0.1f + 0.3f + 0.75f));
    CHECK(box.min.z == doctest::Approx(-0.2f));
    CHECK(box.max.z == doctest::Approx(1.5f - 0.2f));
}

TEST_CASE("zone pose limits: a pose inside the envelope is returned unchanged") {
    const ChassisSpec c = chassis();
    for (const Pose3& p : {at(0, 0, 0), at(0.9f, 0.4f, 1.0f), at(-0.8f, -0.9f, -0.19f), Pose3{Vec3{0.3f, 0.2f, 0.4f}, Quat::fromEuler(0.1f, 0.2f, 3.f)}}) {
        const ClampedZonePose r = clampRemoteZonePose(c, p);
        CHECK_FALSE(r.changed);
        CHECK_FALSE(r.nonFinite);
        CHECK(r.pose.position.x == p.position.x);
        CHECK(r.pose.position.y == p.position.y);
        CHECK(r.pose.position.z == p.position.z);
        CHECK(length(Vec3{r.pose.rotation.x - p.rotation.x, r.pose.rotation.y - p.rotation.y, r.pose.rotation.z - p.rotation.z}) < 1e-6f);
    }
}

TEST_CASE("zone pose limits: positions are clamped per axis at the box faces") {
    const ChassisSpec c = chassis();
    const ZoneEnvelope box = remoteZoneEnvelope(c);
    ClampedZonePose r = clampRemoteZonePose(c, at(50.f, 0.f, 0.f));
    CHECK(r.changed);
    CHECK(r.pose.position.x == box.max.x);
    CHECK(r.pose.position.y == 0.f);
    r = clampRemoteZonePose(c, at(0.f, -50.f, 0.f));
    CHECK(r.pose.position.y == box.min.y);
    CHECK(r.pose.position.x == 0.f);
    r = clampRemoteZonePose(c, at(0.f, 0.f, 9.f));
    CHECK(r.pose.position.z == box.max.z);
    r = clampRemoteZonePose(c, at(0.f, 0.f, -9.f));
    CHECK(r.pose.position.z == box.min.z);
    r = clampRemoteZonePose(c, at(-30.f, 30.f, 30.f));
    CHECK(r.changed);
    CHECK(r.pose.position.x == box.min.x);
    CHECK(r.pose.position.y == box.max.y);
    CHECK(r.pose.position.z == box.max.z);
}

TEST_CASE("zone pose limits: a pose exactly on the boundary is not changed, just past it is") {
    const ChassisSpec c = chassis();
    const ZoneEnvelope box = remoteZoneEnvelope(c);
    CHECK_FALSE(clampRemoteZonePose(c, at(box.max.x, box.min.y, box.max.z)).changed);
    CHECK_FALSE(clampRemoteZonePose(c, at(box.min.x, box.max.y, box.min.z)).changed);
    const float past = std::nextafter(box.max.x, 100.f);
    const ClampedZonePose r = clampRemoteZonePose(c, at(past, 0.f, 0.f));
    CHECK(r.changed);
    CHECK(r.pose.position.x == box.max.x);
    CHECK(std::fabs(r.pose.position.x - past) < kEps * 10);
}

TEST_CASE("zone pose limits: the envelope follows the chassis size, COM offset and COM height") {
    ChassisSpec small = chassis();
    small.sizeM = {0.4f, 0.4f, 0.1f};
    small.comOffsetM = {};
    small.comHeightM = 0.05f;
    const ZoneEnvelope a = remoteZoneEnvelope(small);
    CHECK(a.max.x == doctest::Approx(0.2f + 0.75f));
    CHECK(a.min.x == doctest::Approx(-0.2f - 0.75f));
    CHECK(a.min.z == doctest::Approx(-0.05f));
    CHECK(a.max.z == doctest::Approx(1.45f));
    ChassisSpec tall = small;
    tall.comHeightM = 0.9f;
    CHECK(remoteZoneEnvelope(tall).min.z == doctest::Approx(-0.9f));
    CHECK(remoteZoneEnvelope(tall).max.z == doctest::Approx(0.6f));
}

TEST_CASE("zone pose limits: the rotation is normalised, never limited") {
    const ChassisSpec c = chassis();
    const Quat q = Quat::fromEuler(0.4f, -1.2f, 2.9f);
    const Pose3 scaled{Vec3{}, Quat{q.x * 4.f, q.y * 4.f, q.z * 4.f, q.w * 4.f}};
    const ClampedZonePose r = clampRemoteZonePose(c, scaled);
    CHECK_FALSE(r.changed);
    const Quat& o = r.pose.rotation;
    CHECK(std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w) == doctest::Approx(1.f).epsilon(1e-6));
    CHECK(std::fabs(o.x - q.x) < 1e-5f);
    CHECK(std::fabs(o.w - q.w) < 1e-5f);
    // A fully inverted zone (pointing straight down) is a legal pose.
    const Pose3 down{Vec3{}, Quat::fromEuler(0.f, 1.5707963f, 0.f)};
    CHECK_FALSE(clampRemoteZonePose(c, down).changed);
}

TEST_CASE("zone pose limits: non-finite input is flagged and replaced by a finite pose") {
    const ChassisSpec c = chassis();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const ZoneEnvelope box = remoteZoneEnvelope(c);
    for (const Pose3& p : {at(nan, 0, 0), at(0, inf, 0), at(0, 0, -inf)}) {
        const ClampedZonePose r = clampRemoteZonePose(c, p);
        CHECK(r.nonFinite);
        CHECK(r.changed);
        CHECK(finite(r.pose));
    }
    const ClampedZonePose x = clampRemoteZonePose(c, at(nan, 0, 0));
    CHECK(x.pose.position.x == doctest::Approx(0.5f * (box.min.x + box.max.x)));
    const ClampedZonePose q = clampRemoteZonePose(c, Pose3{Vec3{}, Quat{nan, 0.f, 0.f, 1.f}});
    CHECK(q.nonFinite);
    CHECK(finite(q.pose));
    CHECK(q.pose.rotation.w == 1.f);
    // A zero quaternion is not finite-bad but unusable: replaced by the identity and flagged as changed only.
    const ClampedZonePose zero = clampRemoteZonePose(c, Pose3{Vec3{}, Quat{0.f, 0.f, 0.f, 0.f}});
    CHECK_FALSE(zero.nonFinite);
    CHECK(zero.changed);
    CHECK(zero.pose.rotation.w == 1.f);
}
