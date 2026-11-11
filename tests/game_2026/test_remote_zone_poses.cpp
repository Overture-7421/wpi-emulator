// Client-movable zones end to end: the real SimulationSession (Box3D world, 2026 field and game state) with remote robots
// whose inputs carry zone poses. A remote robot simulates no mechanisms, so its zones are moved by its client; the plugin
// collects fuel at the moved pickup volume and fires along the moved shooter zone. The host robot's zones keep following
// its mechanisms and are published with their resolved pose.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "multirobot_session_rig.h"
#include "test_support.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26session::Options;
using g26session::Rig;
using sdk::proto::AllianceStation;

namespace {

constexpr float kMidX = dim::kFieldLengthM * 0.5f;
constexpr float kMidY = dim::kFieldWidthM * 0.5f;
constexpr float kHalfPi = 1.5707963f;

using PoseList = std::vector<std::pair<std::size_t, sdk::Pose3>>;

sdk::Pose3 poseAt(float x, float y, float z, float yaw = 0.f) { return {sdk::Vec3{x, y, z}, sdk::Quat::fromEuler(0.f, 0.f, yaw)}; }

// Sends stationary inputs with zone flags and poses, the way a game client holds a pose: every 4 ticks, each input listing
// all the poses it wants held.
class PoseClient {
public:
    PoseClient(Rig& rig, sdk::ParticipantId id) : rig_(rig), id_(id) {}

    sdk::match::MatchInput make(std::uint64_t tick, const PoseList& poses, std::uint32_t flags) {
        sdk::match::MatchInput in = g26session::rt::inputOf(++seq_, tick, std::vector<sdk::WheelSetpoint>(4), flags);
        for (const auto& [index, pose] : poses) REQUIRE(sdk::match::setZonePose(in, index, pose));
        return in;
    }
    void run(int ticks, const PoseList& poses, std::uint32_t flags) {
        for (int i = 0; i < ticks; ++i) {
            if (i % 4 == 0) REQUIRE(rig_.session->submitRemoteInput(id_, make(rig_.session->clock().tickIndex(), poses, flags)).ok());
            rig_.session->tick(rig_.frame, false);
        }
    }

private:
    Rig& rig_;
    sdk::ParticipantId id_;
    std::uint32_t seq_ = 0;
};

std::vector<sdk::Vec3> fuelPositions(const sdk::ISimSnapshot& snap) {
    for (const sdk::EntityGroupSample& g : snap.entityGroups())
        if (g.group == "fuel") return g.positions;
    return {};
}

int fuelHeld(Rig& rig, sdk::ParticipantId id) {
    const auto snap = rig.snapshot();
    const sdk::RobotSample* sample = Rig::sampleOf(*snap, id);
    REQUIRE(sample != nullptr);
    return Rig::hudInt(*sample, hud::kFuelCount);
}

// The neutral zone ball with the largest x, after the staged fuel has settled.
sdk::Vec3 edgeBall(Rig& rig) {
    rig.run(100, {});
    const std::vector<sdk::Vec3> fuel = fuelPositions(*rig.snapshot());
    REQUIRE(fuel.size() >= 360);
    sdk::Vec3 best = fuel.front();
    for (const sdk::Vec3& p : fuel)
        if (p.x > best.x && p.x < kMidX + 2.5f) best = p;
    return best;
}

}  // namespace

TEST_CASE("zone poses e2e: a pickup volume at rest misses fuel that its client moves it onto") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot());
    const sdk::Vec3 ball = edgeBall(rig);
    // 1 m beyond the cluster edge, facing away from it: the intake (0.45 m ahead at rest) points away from every ball.
    rig.teleport(1, {ball.x + 1.0f, ball.y, 0.04f});
    PoseClient client(rig, 1);

    client.run(60, {}, 0b01);
    CHECK(fuelHeld(rig, 1) == 8);   // active, but nothing in reach
    const int onField = Rig::fuelOnField(*rig.snapshot());

    // The client slides the intake behind the robot, onto the ball: chassis frame, 1 m back, low.
    client.run(60, {{0, poseAt(-1.0f, 0.f, 0.f)}}, 0b01);
    CHECK(fuelHeld(rig, 1) > 8);
    CHECK(Rig::fuelOnField(*rig.snapshot()) < onField);

    // Back at rest nothing more is collected.
    const int held = fuelHeld(rig, 1);
    client.run(40, {}, 0b01);
    CHECK(fuelHeld(rig, 1) == held);
}

TEST_CASE("zone poses e2e: the moved pickup volume only collects while its zone is active") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot());
    const sdk::Vec3 ball = edgeBall(rig);
    rig.teleport(1, {ball.x + 1.0f, ball.y, 0.04f});
    PoseClient client(rig, 1);
    client.run(60, {{0, poseAt(-1.0f, 0.f, 0.f)}}, 0);
    CHECK(fuelHeld(rig, 1) == 8);
    client.run(60, {{0, poseAt(-1.0f, 0.f, 0.f)}}, 0b01);
    CHECK(fuelHeld(rig, 1) > 8);
}

TEST_CASE("zone poses e2e: a pose past the robot's reach is clamped and collects nothing out of reach") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot());
    const sdk::Vec3 ball = edgeBall(rig);
    rig.teleport(1, {ball.x + 1.5f, ball.y, 0.04f});
    PoseClient client(rig, 1);

    // Unclamped, the volume at -1.5 m would sit on the edge ball; the envelope stops it at the reach margin.
    client.run(80, {{0, poseAt(-1.5f, 0.f, 0.f)}}, 0b01);
    CHECK(fuelHeld(rig, 1) == 8);
    const core::runtime::RemoteRobotInfo info = *rig.session->remoteRobotInfo(1);
    CHECK(info.stats.zonePosesClamped > 0);
    CHECK(info.stats.zonePosesApplied == info.stats.zonePosesClamped);
    CHECK(info.zonePoses[0].position.x == doctest::Approx(-(0.5f * 0.7f + 0.75f)).epsilon(1e-3));

    // Far beyond the wire range too.
    client.run(40, {{0, poseAt(-500.f, 0.f, 0.f)}}, 0b01);
    CHECK(fuelHeld(rig, 1) == 8);
    CHECK(rig.session->remoteRobotInfo(1)->zonePoses[0].position.x == doctest::Approx(-1.1f).epsilon(1e-3));
}

namespace {

// Mean horizontal displacement of the fuel on the field from the robot (at rest, facing +X), after a shooter fired on an empty
// field (so every ball is a shot). Subtract the muzzle position to get the shots' own displacement.
sdk::Vec3 shotDirection(const PoseList& poses, int workerCount = 1) {
    Options options;
    options.neutralFuel = 0;
    options.depotFuel = 0;
    options.workerCount = workerCount;
    Rig rig(options);
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX, kMidY, 0.04f});
    PoseClient client(rig, 1);
    client.run(60, poses, 0b10);
    const sdk::Pose3 robot = rig.session->world().pose(rig.session->robots().find(1)->body);
    const std::vector<sdk::Vec3> shots = fuelPositions(*rig.snapshot());
    REQUIRE(shots.size() >= 2);
    sdk::Vec3 mean;
    for (const sdk::Vec3& p : shots) mean += sdk::Vec3{p.x - robot.position.x, p.y - robot.position.y, 0.f};
    return mean * (1.f / static_cast<float>(shots.size()));
}

}  // namespace

TEST_CASE("zone poses e2e: a shooter's shots follow its zone orientation") {
    const sdk::Vec3 muzzle{0.25f, 0.f, 0.f};   // the shooter zone's local position, unless a pose moves it
    const sdk::Vec3 ahead = shotDirection({}) - muzzle;
    CHECK(ahead.x > 0.3f);
    CHECK(std::fabs(ahead.y) < 0.2f * ahead.x);

    // The same zone turned 90 degrees to the left in the chassis frame fires to the left.
    const sdk::Vec3 left = shotDirection({{1, poseAt(0.25f, 0.f, 0.55f, kHalfPi)}}) - muzzle;
    CHECK(left.y > 0.3f);
    CHECK(std::fabs(left.x) < 0.2f * left.y);

    const sdk::Vec3 right = shotDirection({{1, poseAt(0.25f, 0.f, 0.55f, -kHalfPi)}}) - muzzle;
    CHECK(right.y < -0.3f);
    CHECK(std::fabs(right.x) < 0.2f * -right.y);

    const sdk::Vec3 behind = shotDirection({{1, poseAt(0.25f, 0.f, 0.55f, 3.14159265f)}}) - muzzle;
    CHECK(behind.x < -0.3f);
}

TEST_CASE("zone poses e2e: a moved shooter muzzle moves where the shots start") {
    const sdk::Vec3 near = shotDirection({{1, poseAt(0.0f, 0.f, 0.55f)}});
    const sdk::Vec3 far = shotDirection({{1, poseAt(0.9f, 0.f, 0.55f)}});
    CHECK(far.x > near.x + 0.4f);   // the same speed from a muzzle 0.9 m further forward
}

namespace {

struct Outcome {
    std::uint64_t hash = 0;
    int fuelOnField = 0;
    std::vector<int> held;
    std::vector<int> shots;
    std::vector<std::vector<sdk::Pose3>> poses;
    bool operator==(const Outcome& o) const {
        if (hash != o.hash || fuelOnField != o.fuelOnField || held != o.held || shots != o.shots || poses.size() != o.poses.size())
            return false;
        for (std::size_t i = 0; i < poses.size(); ++i) {
            if (poses[i].size() != o.poses[i].size()) return false;
            for (std::size_t j = 0; j < poses[i].size(); ++j)
                if (!(poses[i][j].position == o.poses[i][j].position && poses[i][j].rotation == o.poses[i][j].rotation)) return false;
        }
        return true;
    }
};

// A mover robot (pickup volume slid over the neutral fuel in steps) and a turret robot (shooter zone swept around), inputs
// scripted in advance and handed to the session in `order`.
Outcome scenario(int workerCount, const std::vector<std::size_t>& order) {
    Options options;
    options.workerCount = workerCount;
    Rig rig(options);
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.add(2, AllianceStation::Blue2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX - 0.2f, kMidY, 0.04f});
    rig.teleport(2, {kMidX + 3.5f, kMidY - 1.5f, 0.04f}, 3.14159f);
    PoseClient first(rig, 1);
    PoseClient second(rig, 2);
    std::vector<std::pair<sdk::ParticipantId, sdk::match::MatchInput>> inputs;
    for (int k = 0; k < 10; ++k) {
        const float f = static_cast<float>(k);
        const std::uint64_t tick = static_cast<std::uint64_t>(5 * k);
        inputs.push_back({1, first.make(tick, {{0, poseAt(0.45f - 0.18f * f, 0.1f * std::sin(f), 0.f)}}, 0b01)});
        inputs.push_back({2, second.make(tick, {{1, poseAt(0.25f, 0.f, 0.55f, 0.35f * f)}}, 0b10)});
    }
    for (const std::size_t i : order) REQUIRE(rig.session->submitRemoteInput(inputs[i].first, inputs[i].second).ok());
    for (int t = 0; t < 120; ++t) rig.session->tick(rig.frame, false);

    const auto snap = rig.snapshot();
    Outcome out;
    out.hash = rig.session->world().stateHash();
    out.fuelOnField = Rig::fuelOnField(*snap);
    for (const sdk::ParticipantId id : {sdk::ParticipantId{1}, sdk::ParticipantId{2}}) {
        const sdk::RobotSample* sample = Rig::sampleOf(*snap, id);
        REQUIRE(sample != nullptr);
        out.held.push_back(Rig::hudInt(*sample, hud::kFuelCount));
        out.shots.push_back(Rig::hudInt(*sample, hud::kShots));
        std::vector<sdk::Pose3> poses;
        for (const sdk::ZoneState& z : sample->zones) {
            REQUIRE(z.poseInChassis.has_value());
            poses.push_back(*z.poseInChassis);
        }
        out.poses.push_back(std::move(poses));
    }
    return out;
}

std::vector<std::size_t> orderOf(std::size_t count, unsigned seed) {
    std::vector<std::size_t> order(count);
    for (std::size_t i = 0; i < count; ++i) order[i] = i;
    for (std::size_t i = count; i > 1; --i) {   // a fixed pseudo-random shuffle
        seed = seed * 1664525u + 1013904223u;
        std::swap(order[i - 1], order[(seed >> 8) % i]);
    }
    return order;
}

}  // namespace

TEST_CASE("zone poses e2e: the game state is identical for 1, 2 and 4 physics workers and for any arrival order") {
    std::vector<std::size_t> sorted(20);
    for (std::size_t i = 0; i < 20; ++i) sorted[i] = i;
    const Outcome reference = scenario(1, sorted);
    CHECK(reference.held[0] > 8);   // the moving pickup really collects
    CHECK(reference.shots[1] > 0);  // and the swept shooter really fires
    CHECK(scenario(1, sorted) == reference);
    CHECK(scenario(2, sorted) == reference);
    CHECK(scenario(4, sorted) == reference);
    std::vector<std::size_t> reversed(sorted.rbegin(), sorted.rend());
    CHECK(scenario(1, reversed) == reference);
    CHECK(scenario(2, orderOf(20, 7)) == reference);
    CHECK(scenario(4, orderOf(20, 99)) == reference);
}

TEST_CASE("zone poses e2e: a remote robot's published zone pose follows its override and returns to rest") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot());
    PoseClient client(rig, 1);
    const sdk::Pose3 moved = poseAt(0.9f, 0.1f, 0.7f, 0.5f);

    client.run(8, {}, 0);
    auto snap = rig.snapshot();
    auto sample = Rig::sampleOf(*snap, 1);
    REQUIRE(sample != nullptr);
    REQUIRE(sample->zones.size() == 2);
    REQUIRE(sample->zones[0].poseInChassis.has_value());
    CHECK(sample->zones[0].poseInChassis->position.x == doctest::Approx(0.45f).epsilon(1e-4));

    client.run(8, {{0, moved}}, 0);
    snap = rig.snapshot();
    sample = Rig::sampleOf(*snap, 1);
    CHECK(sample->zones[0].poseInChassis->position.x == doctest::Approx(0.9f).epsilon(1e-3));
    CHECK(sample->zones[0].poseInChassis->position.z == doctest::Approx(0.7f).epsilon(1e-3));
    CHECK(sample->zones[1].poseInChassis->position.x == doctest::Approx(0.25f).epsilon(1e-4));   // untouched

    client.run(8, {}, 0);
    snap = rig.snapshot();
    sample = Rig::sampleOf(*snap, 1);
    CHECK(sample->zones[0].poseInChassis->position.x == doctest::Approx(0.45f).epsilon(1e-4));
}

TEST_CASE("zone poses e2e: the host robot's published zone pose follows its mechanism link") {
    using g26test::makeZone;
    using g26test::ScriptedMechanism;
    using g26test::Session;
    Session s;
    ScriptedMechanism& arm = s.addScripted("arm");
    arm.links.push_back({"tip", sdk::Pose3{{0.f, 1.f, 0.f}, sdk::Quat{}}});
    sdk::MechanismConfig armConfig;
    armConfig.id = "arm";
    armConfig.type = "test.scripted";
    s.env.config.mechanisms.push_back(armConfig);
    sdk::Value props = sdk::Value::makeObject();
    props["intakeRateHz"] = 0.0;
    sdk::ZoneConfig onArm = makeZone("onArm", kPickupKind, props);
    onArm.attachment.parentMechanismId = "arm";
    onArm.attachment.parentLink = "tip";
    s.addZone(onArm);
    s.addZone(makeZone("onChassis", kPickupKind, props));
    s.start();

    const auto published = [&] {
        sdk::SnapshotBuilder out;
        s.game->publish(out);
        return out.data().zones;
    };
    s.tick(2);
    auto zones = published();
    REQUIRE(zones.size() == 2);
    REQUIRE(zones[0].poseInChassis.has_value());
    REQUIRE(zones[1].poseInChassis.has_value());
    CHECK(zones[0].poseInChassis->position.x == doctest::Approx(0.45f));
    CHECK(zones[0].poseInChassis->position.y == doctest::Approx(1.f));
    CHECK(zones[0].poseInChassis->position.z == doctest::Approx(0.2f));
    CHECK(zones[1].poseInChassis->position.y == doctest::Approx(0.f));

    arm.links.front().pose.position = {0.f, -1.f, 0.f};
    arm.links.front().pose.rotation = sdk::Quat::fromEuler(0.f, 0.f, kHalfPi);
    s.tick(1);
    zones = published();
    // The zone's local 0.45 m ahead is now 0.45 m to the left of a link that turned 90 degrees, at y = -1 m.
    CHECK(zones[0].poseInChassis->position.y == doctest::Approx(-1.f + 0.45f).epsilon(1e-4));
    CHECK(std::fabs(zones[0].poseInChassis->position.x) < 1e-4f);
    CHECK(zones[0].poseInChassis->rotation.z == doctest::Approx(std::sin(kHalfPi * 0.5f)).epsilon(1e-4));
    CHECK(zones[1].poseInChassis->position.y == doctest::Approx(0.f));   // the chassis zone stays
}

TEST_CASE("zone poses e2e: a host zone published before its first update is at its configured local pose") {
    using g26test::makeZone;
    using g26test::Session;
    Session s;
    sdk::ZoneConfig zone = makeZone("intake", kPickupKind);
    zone.local = poseAt(0.3f, -0.2f, 0.6f, 0.4f);
    s.addZone(zone);
    s.start();
    sdk::SnapshotBuilder out;
    s.game->publish(out);
    REQUIRE(out.data().zones.size() == 1);
    REQUIRE(out.data().zones[0].poseInChassis.has_value());
    CHECK(out.data().zones[0].poseInChassis->position.x == doctest::Approx(0.3f));
    CHECK(out.data().zones[0].poseInChassis->position.y == doctest::Approx(-0.2f));
}
