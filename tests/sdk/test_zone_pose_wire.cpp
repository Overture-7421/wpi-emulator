#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
#include "wpiemu/sdk/match/zone_pose_wire.h"

using namespace wpiemu::sdk;
using namespace wpiemu::sdk::match;

namespace {

constexpr float kTestPi = std::numbers::pi_v<float>;

// Geodesic angle between two unit rotations (well conditioned near 0, unlike acos of the dot product).
double rotationErrorRad(const Quat& a, const Quat& b) {
    const Quat d = a.normalized() * b.normalized().conjugate();
    const double v = std::sqrt(static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y + static_cast<double>(d.z) * d.z);
    return 2.0 * std::atan2(v, std::fabs(static_cast<double>(d.w)));
}

struct Lcg {
    std::uint32_t s = 12345;
    float next() {  // [0, 1)
        s = s * 1664525u + 1013904223u;
        return static_cast<float>((s >> 8) & 0xFFFFFF) / static_cast<float>(1 << 24);
    }
    float range(float lo, float hi) { return lo + (hi - lo) * next(); }
};

Pose3 poseOf(float x, float y, float z, float roll, float pitch, float yaw) {
    return {Vec3{x, y, z}, Quat::fromEuler(roll, pitch, yaw)};
}

}  // namespace

TEST_CASE("zone pose wire: positions round trip within half a millimetre") {
    for (const double m : {0.0, 0.0004, 0.0005001, -0.0004, 0.1234, -0.7499, 1.5, -2.25, 10.0, 32.0, -32.0, 32.767, -32.767}) {
        const Pose3 p{Vec3{static_cast<float>(m), static_cast<float>(-m), static_cast<float>(0.5 * m)}, Quat{}};
        const Pose3 back = toPose3(makeZonePoseInput(3, p));
        CAPTURE(m);
        CHECK(std::fabs(back.position.x - p.position.x) <= kMaxZonePosePositionErrorM + 1e-5);
        CHECK(std::fabs(back.position.y - p.position.y) <= kMaxZonePosePositionErrorM + 1e-5);
        CHECK(std::fabs(back.position.z - p.position.z) <= kMaxZonePosePositionErrorM + 1e-5);
    }
    Lcg rng;
    for (int i = 0; i < 2000; ++i) {
        const Pose3 p{Vec3{rng.range(-5.f, 5.f), rng.range(-5.f, 5.f), rng.range(-2.f, 2.f)}, Quat{}};
        const Pose3 back = toPose3(makeZonePoseInput(0, p));
        CHECK(std::fabs(back.position.x - p.position.x) <= kMaxZonePosePositionErrorM + 1e-5);
        CHECK(std::fabs(back.position.y - p.position.y) <= kMaxZonePosePositionErrorM + 1e-5);
        CHECK(std::fabs(back.position.z - p.position.z) <= kMaxZonePosePositionErrorM + 1e-5);
    }
}

TEST_CASE("zone pose wire: rotations round trip within the documented bound, gimbal lock included") {
    Lcg rng;
    double worst = 0;
    for (int i = 0; i < 4000; ++i) {
        const Pose3 p = poseOf(0, 0, 0, rng.range(-kTestPi, kTestPi), rng.range(-1.5707f, 1.5707f), rng.range(-kTestPi, kTestPi));
        worst = std::max(worst, rotationErrorRad(toPose3(makeZonePoseInput(0, p)).rotation, p.rotation));
    }
    for (const float pitch : {kTestPi / 2, -kTestPi / 2, kTestPi / 2 - 1e-4f, -kTestPi / 2 + 1e-4f}) {
        for (const float roll : {0.f, 0.7f, -2.f}) {
            const Pose3 p = poseOf(0, 0, 0, roll, pitch, 1.1f);
            worst = std::max(worst, rotationErrorRad(toPose3(makeZonePoseInput(0, p)).rotation, p.rotation));
        }
    }
    // Quarter turns and the identity are the poses a game client sends most.
    for (const float yaw : {0.f, kTestPi / 2, kTestPi, -kTestPi / 2}) {
        const Pose3 p = poseOf(0, 0, 0, 0, 0, yaw);
        worst = std::max(worst, rotationErrorRad(toPose3(makeZonePoseInput(0, p)).rotation, p.rotation));
    }
    CAPTURE(worst);
    CHECK(worst <= kMaxZonePoseRotationErrorRad + 1e-5);
}

TEST_CASE("zone pose wire: the identity pose is all zeros but the index") {
    const ZonePoseInput z = makeZonePoseInput(7, Pose3{});
    CHECK(z == ZonePoseInput{7, 0, 0, 0, 0, 0, 0});
}

TEST_CASE("zone pose wire: positions beyond +-32.767 m saturate") {
    const ZonePoseInput hi = makeZonePoseInput(0, {Vec3{1000.f, 40.f, 32.767f}, Quat{}});
    CHECK(hi.xMm == 32767);
    CHECK(hi.yMm == 32767);
    CHECK(hi.zMm == 32767);
    const ZonePoseInput lo = makeZonePoseInput(0, {Vec3{-1e9f, -33.f, -32.768f}, Quat{}});
    CHECK(lo.xMm == -32768);
    CHECK(lo.yMm == -32768);
    CHECK(lo.zMm == -32768);
    CHECK(toPose3(hi).position.x == doctest::Approx(32.767f));
    CHECK(toPose3(lo).position.x == doctest::Approx(-32.768f));
}

TEST_CASE("zone pose wire: non-finite and degenerate poses map to a defined safe value") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const ZonePoseInput badPosition = makeZonePoseInput(1, {Vec3{nan, inf, -inf}, Quat{}});
    CHECK(badPosition == ZonePoseInput{1, 0, 0, 0, 0, 0, 0});
    for (const Quat& q : {Quat{nan, 0.f, 0.f, 1.f}, Quat{0.f, inf, 0.f, 1.f}, Quat{0.f, 0.f, 0.f, 0.f}}) {
        const ZonePoseInput z = makeZonePoseInput(2, {Vec3{0.1f, 0.2f, 0.3f}, q});
        CHECK(z == ZonePoseInput{2, 100, 200, 300, 0, 0, 0});
    }
    // A non-unit quaternion is normalised first.
    const ZonePoseInput scaled = makeZonePoseInput(0, {Vec3{}, Quat{0.f, 0.f, 3.f * std::sin(0.25f), 3.f * std::cos(0.25f)}});
    CHECK(scaled == makeZonePoseInput(0, {Vec3{}, Quat::fromEuler(0.f, 0.f, 0.5f)}));
}

TEST_CASE("zone pose wire: any bit pattern decodes to a finite pose with a unit rotation") {
    Lcg rng;
    for (int i = 0; i < 5000; ++i) {
        ZonePoseInput z;
        z.zoneIndex = static_cast<std::uint8_t>(rng.next() * 256.f);
        z.xMm = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        z.yMm = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        z.zMm = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        z.yawQ = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        z.pitchQ = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        z.rollQ = static_cast<std::int16_t>(rng.next() * 65536.f - 32768.f);
        const Pose3 p = toPose3(z);
        REQUIRE(finite(p));
        const Quat& q = p.rotation;
        CHECK(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) == doctest::Approx(1.0).epsilon(1e-5));
    }
    for (const std::int16_t extreme : {std::int16_t{-32768}, std::int16_t{32767}, std::int16_t{0}}) {
        CHECK(finite(toPose3({0, extreme, extreme, extreme, extreme, extreme, extreme})));
    }
}

TEST_CASE("zone pose wire: the index saturates and is then refused as out of range") {
    CHECK(makeZonePoseInput(5, Pose3{}).zoneIndex == 5);
    CHECK(makeZonePoseInput(1000, Pose3{}).zoneIndex == 255);
    CHECK_FALSE(zonePosesWellFormed({makeZonePoseInput(1000, Pose3{})}));
    CHECK_FALSE(zonePosesWellFormed({makeZonePoseInput(kMaxZonesPerRobot, Pose3{})}));
    CHECK(zonePosesWellFormed({makeZonePoseInput(kMaxZonesPerRobot - 1, Pose3{})}));
}

TEST_CASE("zone pose wire: well formed lists are strictly increasing, in range and within the cap") {
    const auto at = [](std::size_t i) { return makeZonePoseInput(i, Pose3{}); };
    CHECK(zonePosesWellFormed({}));
    CHECK(zonePosesWellFormed({at(0), at(1), at(15)}));
    CHECK_FALSE(zonePosesWellFormed({at(1), at(1)}));
    CHECK_FALSE(zonePosesWellFormed({at(2), at(1)}));
    CHECK_FALSE(zonePosesWellFormed({at(0), at(5), at(4)}));
    std::vector<ZonePoseInput> all;
    for (std::size_t i = 0; i < kMaxZonesPerRobot; ++i) all.push_back(at(i));
    CHECK(zonePosesWellFormed(all));
    CHECK_FALSE(zonePosesWellFormed(all, kMaxSnapshotZonePosesPerRobot));
    all.resize(kMaxSnapshotZonePosesPerRobot);
    CHECK(zonePosesWellFormed(all, kMaxSnapshotZonePosesPerRobot));
}

TEST_CASE("zone pose wire: fitsLimits covers the zone poses of inputs and snapshots") {
    MatchInput input;
    CHECK(fitsLimits(input));
    input.zonePoses = {makeZonePoseInput(1, Pose3{}), makeZonePoseInput(1, Pose3{})};
    CHECK_FALSE(fitsLimits(input));
    MatchInputPacket packet;
    packet.inputs = {MatchInput{}, input};
    CHECK_FALSE(fitsLimits(packet));
    input.zonePoses.pop_back();
    packet.inputs[1] = input;
    CHECK(fitsLimits(packet));

    MatchSnapshot snapshot;
    snapshot.robots.assign(2, RobotWireState{});
    CHECK(fitsLimits(snapshot));
    for (std::size_t i = 0; i <= kMaxSnapshotZonePosesPerRobot; ++i) snapshot.robots[1].zonePoses.push_back(makeZonePoseInput(i, Pose3{}));
    CHECK_FALSE(fitsLimits(snapshot));
    snapshot.robots[1].zonePoses.pop_back();
    CHECK(fitsLimits(snapshot));
}

TEST_CASE("zone pose wire: setZonePose keeps the list ordered and replaces an earlier entry") {
    MatchInput input;
    CHECK(setZonePose(input, 5, Pose3{}));
    CHECK(setZonePose(input, 1, poseOf(0.1f, 0, 0, 0, 0, 0)));
    CHECK(setZonePose(input, 9, Pose3{}));
    REQUIRE(input.zonePoses.size() == 3);
    CHECK(input.zonePoses[0].zoneIndex == 1);
    CHECK(input.zonePoses[1].zoneIndex == 5);
    CHECK(input.zonePoses[2].zoneIndex == 9);
    CHECK(setZonePose(input, 5, poseOf(0.5f, 0, 0, 0, 0, 0)));
    REQUIRE(input.zonePoses.size() == 3);
    CHECK(input.zonePoses[1].xMm == 500);
    const std::vector<ZonePoseInput> before = input.zonePoses;
    CHECK_FALSE(setZonePose(input, kMaxZonesPerRobot, Pose3{}));
    CHECK(input.zonePoses == before);
    CHECK(fitsLimits(input));
}

TEST_CASE("selectSnapshotZonePoses: only zones away from their rest pose are listed") {
    const std::vector<Pose3> rest = {poseOf(0.3f, 0, 0.5f, 0, 0, 0), poseOf(-0.3f, 0, 0.5f, 0, 0, kTestPi), poseOf(0, 0.3f, 0.2f, 0, 0, 0)};
    CHECK(selectSnapshotZonePoses(rest, rest, 0).empty());
    CHECK(selectSnapshotZonePoses(rest, rest, 0xFFFFu).empty());

    std::vector<Pose3> now = rest;
    now[1].position.x += 0.4f;
    const auto one = selectSnapshotZonePoses(rest, now, 0);
    REQUIRE(one.size() == 1);
    CHECK(one[0] == makeZonePoseInput(1, now[1]));

    now = rest;
    now[2].rotation = Quat::fromEuler(0.f, 0.f, 1.f);
    now[0].position.z += 0.1f;
    const auto two = selectSnapshotZonePoses(rest, now, 0);
    REQUIRE(two.size() == 2);
    CHECK(two[0].zoneIndex == 0);
    CHECK(two[1].zoneIndex == 2);
}

TEST_CASE("selectSnapshotZonePoses: a change below the wire resolution is no change") {
    const std::vector<Pose3> rest = {poseOf(0.3f, 0.1f, 0.5f, 0, 0, 0.5f)};
    std::vector<Pose3> now = rest;
    now[0].position.x += 0.0001f;
    CHECK(selectSnapshotZonePoses(rest, now, 1).empty());
    now[0].position.x += 0.001f;
    CHECK(selectSnapshotZonePoses(rest, now, 1).size() == 1);
}

TEST_CASE("selectSnapshotZonePoses: past the cap active zones win, then lower indices, listed by index") {
    std::vector<Pose3> rest(8);
    std::vector<Pose3> now(8);
    for (std::size_t i = 0; i < 8; ++i) now[i].position.x = 0.1f * static_cast<float>(i + 1);

    // None active: the four lowest indices.
    auto picked = selectSnapshotZonePoses(rest, now, 0);
    REQUIRE(picked.size() == kMaxSnapshotZonePosesPerRobot);
    for (std::size_t i = 0; i < picked.size(); ++i) CHECK(picked[i].zoneIndex == i);

    // Zones 6 and 7 active: they take two of the four places, the lowest inactive indices (0 and 1) fill the rest.
    picked = selectSnapshotZonePoses(rest, now, (1u << 6) | (1u << 7));
    REQUIRE(picked.size() == 4);
    CHECK(picked[0].zoneIndex == 0);
    CHECK(picked[1].zoneIndex == 1);
    CHECK(picked[2].zoneIndex == 6);
    CHECK(picked[3].zoneIndex == 7);

    // More than four active: the lowest four active ones.
    picked = selectSnapshotZonePoses(rest, now, 0xFFu);
    REQUIRE(picked.size() == 4);
    for (std::size_t i = 0; i < picked.size(); ++i) CHECK(picked[i].zoneIndex == i);

    // An active zone at rest does not take a place.
    now[5] = rest[5];
    picked = selectSnapshotZonePoses(rest, now, (1u << 5) | (1u << 4));
    REQUIRE(picked.size() == 4);
    CHECK(picked[0].zoneIndex == 0);
    CHECK(picked[1].zoneIndex == 1);
    CHECK(picked[2].zoneIndex == 2);
    CHECK(picked[3].zoneIndex == 4);
    CHECK(zonePosesWellFormed(picked, kMaxSnapshotZonePosesPerRobot));
}

TEST_CASE("selectSnapshotZonePoses: mismatched spans and zones past the cap are not considered") {
    const std::vector<Pose3> rest(3);
    std::vector<Pose3> now(2);
    now[1].position.y = 1.f;
    const auto picked = selectSnapshotZonePoses(rest, now, 0);
    REQUIRE(picked.size() == 1);
    CHECK(picked[0].zoneIndex == 1);
    CHECK(selectSnapshotZonePoses({}, now, 0).empty());
    CHECK(selectSnapshotZonePoses(rest, {}, 0).empty());

    std::vector<Pose3> many(kMaxZonesPerRobot + 4);
    std::vector<Pose3> moved = many;
    for (Pose3& p : moved) p.position.x = 1.f;
    const auto capped = selectSnapshotZonePoses(many, moved, 0xFFFFFFFFu);
    REQUIRE(capped.size() == kMaxSnapshotZonePosesPerRobot);
    for (const ZonePoseInput& z : capped) CHECK(z.zoneIndex < kMaxZonesPerRobot);
}
