// Client-movable zones of remote robots in SimulationSession: a MatchInput's zonePoses apply on the input's target tick with
// the wheels and the flags, are clamped to the robot's envelope and show in RobotView, RemoteRobotInfo and the snapshot.
// Over the real Box3D world (remote_robot_rig.h); the plugin end to end is in tests/game_2026.
#include <doctest/doctest.h>

#include <cmath>
#include <tuple>
#include <vector>

#include "remote_robot_rig.h"
#include "wpiemu/sdk/match/zone_pose_wire.h"

using namespace remote_test;
using proto::AllianceStation;

namespace {

constexpr float kHalfPi = 1.5707963f;

Pose3 poseAt(float x, float y, float z, float yaw = 0.f) { return {Vec3{x, y, z}, Quat::fromEuler(0.f, 0.f, yaw)}; }

// A swerve robot with three zones at distinct rest poses (the third one is hung on a mechanism link the remote robot
// does not have, so its rest pose is its local pose).
RobotConfig zonedRobot() {
    RobotConfig c = swerveRobot("Z", 3);
    c.zones[0].local = poseAt(0.3f, 0.f, 0.4f);
    c.zones[1].local = poseAt(-0.3f, 0.2f, 0.5f, kHalfPi);
    c.zones[2].local = poseAt(0.f, -0.1f, 0.3f);
    c.zones[2].attachment.parentMechanismId = "arm";
    return c;
}

bool samePose(const Pose3& a, const Pose3& b, float tol = 2e-3f) {
    const float dot = a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y + a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w;
    return std::fabs(a.position.x - b.position.x) <= tol && std::fabs(a.position.y - b.position.y) <= tol &&
           std::fabs(a.position.z - b.position.z) <= tol && std::fabs(dot) > 1.f - 1e-5f;
}

struct ZoneInputBuilder {
    Rig& rig;
    ParticipantId id;
    std::uint32_t seq = 0;
    std::vector<WheelSetpoint> idle;

    ZoneInputBuilder(Rig& r, ParticipantId i, const RobotConfig& spec) : rig(r), id(i) {
        ScriptedClient client(r, i, spec);
        idle = client.setpointsFor({});
    }
    match::MatchInput make(std::uint64_t tick, const std::vector<std::pair<std::size_t, Pose3>>& poses, std::uint32_t flags = 0) {
        match::MatchInput in = inputOf(++seq, tick, idle, flags);
        for (const auto& [index, pose] : poses) REQUIRE(match::setZonePose(in, index, pose));
        return in;
    }
    Result<RemoteInputOutcome> send(std::uint64_t tick, const std::vector<std::pair<std::size_t, Pose3>>& poses, std::uint32_t flags = 0) {
        return rig.session->submitRemoteInput(id, make(tick, poses, flags));
    }
};

std::vector<Pose3> viewPoses(Rig& rig, ParticipantId id) {
    const RobotView* v = rig.session->robots().find(id);
    REQUIRE(v != nullptr);
    return {v->zonePoses.begin(), v->zonePoses.end()};
}

}  // namespace

TEST_CASE("zone poses: without an input every zone is at its spec pose, and the host robot has none") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    rig.tick();
    const RobotConfig spec = zonedRobot();
    const RobotView* view = rig.session->robots().find(1);
    REQUIRE(view != nullptr);
    REQUIRE(view->zonePoses.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        CHECK(samePose(view->zonePoses[i], spec.zones[i].local, 1e-6f));
        REQUIRE(view->zonePose(i).has_value());
        CHECK(samePose(*view->zonePose(i), spec.zones[i].local, 1e-6f));
    }
    CHECK_FALSE(view->zonePose(3).has_value());
    CHECK_FALSE(view->zonePose(1000).has_value());

    const RobotView* host = rig.session->robots().find(match::kHostParticipantId);
    REQUIRE(host != nullptr);
    CHECK(host->zonePoses.empty());
    CHECK_FALSE(host->zonePose(0).has_value());

    const RemoteRobotInfo info = *rig.session->remoteRobotInfo(1);
    REQUIRE(info.zonePoses.size() == 3);
    CHECK(samePose(info.zonePoses[1], spec.zones[1].local, 1e-6f));
    CHECK(info.stats.zonePosesApplied == 0);
}

TEST_CASE("zone poses: an override applies exactly on its target tick") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const Pose3 moved = poseAt(0.9f, -0.2f, 0.6f, 0.5f);
    REQUIRE(client.send(10, {{0, moved}}).value() == RemoteInputOutcome::Queued);
    const Pose3 rest = zonedRobot().zones[0].local;
    for (int t = 0; t < 10; ++t) {   // ticks 0..9
        rig.tick();
        CAPTURE(t);
        CHECK(samePose(viewPoses(rig, 1)[0], rest, 1e-6f));
        CHECK(rig.session->remoteRobotInfo(1)->stats.zonePosesApplied == 0);
    }
    rig.tick();   // tick 10
    CHECK(samePose(viewPoses(rig, 1)[0], moved));
    CHECK(samePose(rig.session->remoteRobotInfo(1)->zonePoses[0], moved));
    CHECK(samePose(viewPoses(rig, 1)[1], zonedRobot().zones[1].local, 1e-6f));   // the others stay at rest
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.zonePosesApplied == 1);
    CHECK(stats.zonePosesClamped == 0);
    CHECK(stats.zonePosesIgnored == 0);
}

TEST_CASE("zone poses: an override of a mechanism-attached zone moves it from its pinned local pose") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    rig.tick();
    CHECK(samePose(viewPoses(rig, 1)[2], zonedRobot().zones[2].local, 1e-6f));
    const Pose3 out = poseAt(0.8f, 0.f, 0.45f);
    REQUIRE(client.send(rig.nextTick(), {{2, out}}).ok());
    rig.tick();
    CHECK(samePose(viewPoses(rig, 1)[2], out));
}

TEST_CASE("zone poses: each input is self-contained, a zone it does not list returns to its spec pose") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const RobotConfig spec = zonedRobot();
    const Pose3 a = poseAt(0.7f, 0.1f, 0.5f);
    const Pose3 b = poseAt(-0.6f, -0.3f, 0.7f, 1.f);

    REQUIRE(client.send(rig.nextTick(), {{0, a}, {1, b}}).ok());
    rig.tick();
    CHECK(samePose(viewPoses(rig, 1)[0], a));
    CHECK(samePose(viewPoses(rig, 1)[1], b));

    REQUIRE(client.send(rig.nextTick(), {{1, b}}).ok());   // zone 0 not listed: back at rest
    rig.tick();
    CHECK(samePose(viewPoses(rig, 1)[0], spec.zones[0].local, 1e-6f));
    CHECK(samePose(viewPoses(rig, 1)[1], b));

    REQUIRE(client.send(rig.nextTick(), {}).ok());   // an empty list: everything at rest
    rig.tick();
    for (std::size_t i = 0; i < 3; ++i) CHECK(samePose(viewPoses(rig, 1)[i], spec.zones[i].local, 1e-6f));
    CHECK(rig.session->remoteRobotInfo(1)->stats.zonePosesApplied == 3);
}

TEST_CASE("zone poses: when several inputs are due only the newest sequence decides the poses") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const Pose3 first = poseAt(0.6f, 0.f, 0.5f);
    const Pose3 second = poseAt(0.5f, 0.2f, 0.4f);
    REQUIRE(client.send(3, {{0, first}, {1, first}}).ok());
    REQUIRE(client.send(3, {{0, second}}).ok());
    rig.tick(4);
    CHECK(samePose(viewPoses(rig, 1)[0], second));
    CHECK(samePose(viewPoses(rig, 1)[1], zonedRobot().zones[1].local, 1e-6f));   // not first's: superseded
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.superseded == 1);
    CHECK(stats.zonePosesApplied == 1);
}

TEST_CASE("zone poses: arrival order does not change the poses or the counters") {
    const auto run = [](const std::vector<int>& order) {
        Rig rig;
        rig.add(1, AllianceStation::Blue1, zonedRobot());
        ZoneInputBuilder client(rig, 1, zonedRobot());
        std::vector<match::MatchInput> inputs;
        for (int i = 1; i <= 8; ++i) {
            const float f = static_cast<float>(i);
            inputs.push_back(client.make(static_cast<std::uint64_t>(3 * i),
                                         {{static_cast<std::size_t>(i % 3), poseAt(0.1f * f, 0.05f * f, 0.2f + 0.05f * f, 0.2f * f)},
                                          {static_cast<std::size_t>((i + 1) % 3), poseAt(40.f, -0.1f * f, 0.4f)}}));
            // The list must be sorted by index: setZonePose keeps it so.
        }
        for (const int i : order) REQUIRE(rig.session->submitRemoteInput(1, inputs[static_cast<std::size_t>(i)]).ok());
        std::vector<Pose3> history;
        for (int t = 0; t < 30; ++t) {
            rig.tick();
            for (const Pose3& p : viewPoses(rig, 1)) history.push_back(p);
        }
        const RemoteInputStats s = rig.session->remoteRobotInfo(1)->stats;
        return std::make_tuple(history, s.zonePosesApplied, s.zonePosesClamped, s.superseded, s.applied);
    };
    const auto inOrder = run({0, 1, 2, 3, 4, 5, 6, 7});
    const auto reversed = run({7, 6, 5, 4, 3, 2, 1, 0});
    const auto shuffled = run({3, 0, 6, 1, 7, 2, 5, 4});
    CHECK(std::get<1>(inOrder) == 16);
    CHECK(std::get<2>(inOrder) == 8);   // every 40 m pose is clamped
    for (const auto* other : {&reversed, &shuffled}) {
        const auto& h0 = std::get<0>(inOrder);
        const auto& h1 = std::get<0>(*other);
        REQUIRE(h0.size() == h1.size());
        bool same = true;
        for (std::size_t i = 0; i < h0.size(); ++i) same = same && samePose(h0[i], h1[i], 0.f);
        CHECK(same);
        CHECK(std::get<1>(*other) == std::get<1>(inOrder));
        CHECK(std::get<2>(*other) == std::get<2>(inOrder));
        CHECK(std::get<3>(*other) == std::get<3>(inOrder));
        CHECK(std::get<4>(*other) == std::get<4>(inOrder));
    }
}

TEST_CASE("zone poses: poses outside the envelope are clamped and counted") {
    Rig rig;
    const RobotConfig spec = zonedRobot();
    rig.add(1, AllianceStation::Blue1, spec);
    ZoneInputBuilder client(rig, 1, spec);
    const ZoneEnvelope box = remoteZoneEnvelope(spec.chassis);
    REQUIRE(client.send(rig.nextTick(), {{0, poseAt(25.f, 0.f, 0.5f)}, {1, poseAt(0.f, -25.f, -5.f)}, {2, poseAt(0.2f, 0.f, 0.5f)}}).ok());
    rig.tick();
    const std::vector<Pose3> poses = viewPoses(rig, 1);
    CHECK(poses[0].position.x == doctest::Approx(box.max.x).epsilon(1e-4));
    CHECK(poses[0].position.y == doctest::Approx(0.f).epsilon(1e-4));
    CHECK(poses[1].position.y == doctest::Approx(box.min.y).epsilon(1e-4));
    CHECK(poses[1].position.z == doctest::Approx(box.min.z).epsilon(1e-4));
    CHECK(samePose(poses[2], poseAt(0.2f, 0.f, 0.5f)));
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.zonePosesApplied == 3);
    CHECK(stats.zonePosesClamped == 2);
    CHECK(stats.zonePosesIgnored == 0);
    // The wire saturates at 32.767 m: a pose beyond that is clamped too, never wrapped.
    REQUIRE(client.send(rig.nextTick(), {{0, poseAt(1000.f, -1000.f, 1000.f)}}).ok());
    rig.tick();
    CHECK(viewPoses(rig, 1)[0].position.x == doctest::Approx(box.max.x).epsilon(1e-4));
    CHECK(viewPoses(rig, 1)[0].position.y == doctest::Approx(box.min.y).epsilon(1e-4));
    CHECK(viewPoses(rig, 1)[0].position.z == doctest::Approx(box.max.z).epsilon(1e-4));
}

TEST_CASE("zone poses: entries for a zone the robot does not have are ignored and counted, the rest apply") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());   // three zones
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const Pose3 a = poseAt(0.5f, 0.f, 0.5f);
    REQUIRE(client.send(rig.nextTick(), {{1, a}, {3, a}, {15, a}}).ok());
    rig.tick();
    CHECK(samePose(viewPoses(rig, 1)[1], a));
    CHECK(viewPoses(rig, 1).size() == 3);
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.zonePosesApplied == 1);
    CHECK(stats.zonePosesIgnored == 2);
}

TEST_CASE("zone poses: a robot without zones ignores every entry") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, swerveRobot());
    ZoneInputBuilder client(rig, 1, swerveRobot());
    REQUIRE(client.send(rig.nextTick(), {{0, poseAt(0.5f, 0.f, 0.5f)}}).ok());
    rig.tick();
    CHECK(rig.session->robots().find(1)->zonePoses.empty());
    CHECK(rig.session->remoteRobotInfo(1)->stats.zonePosesIgnored == 1);
}

TEST_CASE("zone poses: a malformed list is rejected before it is queued") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const auto zone = [](unsigned index) { return match::makeZonePoseInput(index, Pose3{}); };
    const std::vector<std::vector<match::ZonePoseInput>> bad = {
        {zone(2), zone(1)}, {zone(1), zone(1)}, {zone(static_cast<unsigned>(match::kMaxZonesPerRobot))}, {zone(255)}};
    std::uint64_t rejected = 0;
    for (const auto& list : bad) {
        match::MatchInput in = client.make(rig.nextTick(), {});
        in.zonePoses = list;
        const Result<RemoteInputOutcome> r = rig.session->submitRemoteInput(1, in);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == ErrorCode::InvalidArgument);
        ++rejected;
    }
    std::vector<match::ZonePoseInput> tooMany;
    for (std::size_t i = 0; i <= match::kMaxZonesPerRobot; ++i) tooMany.push_back(zone(static_cast<unsigned>(i)));
    match::MatchInput in = client.make(rig.nextTick(), {});
    in.zonePoses = tooMany;
    CHECK_FALSE(rig.session->submitRemoteInput(1, in).ok());
    ++rejected;
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.rejected == rejected);
    CHECK(stats.queueDepth == 0);
}

TEST_CASE("zone poses: reset and a fresh join clear the overrides") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const RobotConfig spec = zonedRobot();
    REQUIRE(client.send(rig.nextTick(), {{0, poseAt(0.8f, 0.f, 0.5f)}}).ok());
    rig.tick();
    REQUIRE(samePose(viewPoses(rig, 1)[0], poseAt(0.8f, 0.f, 0.5f)));

    rig.session->reset();
    CHECK(samePose(viewPoses(rig, 1)[0], spec.zones[0].local, 1e-6f));
    CHECK(samePose(rig.session->remoteRobotInfo(1)->zonePoses[0], spec.zones[0].local, 1e-6f));
    CHECK(rig.session->remoteRobotInfo(1)->stats.zonePosesApplied == 0);
    rig.tick(5);
    CHECK(samePose(viewPoses(rig, 1)[0], spec.zones[0].local, 1e-6f));   // nothing re-applies after the reset

    ZoneInputBuilder again(rig, 1, spec);
    REQUIRE(again.send(rig.nextTick(), {{2, poseAt(0.5f, 0.f, 0.6f)}}).ok());
    rig.tick();
    REQUIRE(samePose(viewPoses(rig, 1)[2], poseAt(0.5f, 0.f, 0.6f)));
    REQUIRE(rig.session->removeRemoteRobot(1).ok());
    rig.add(1, AllianceStation::Blue1, spec);   // joins again
    rig.tick();
    for (std::size_t i = 0; i < 3; ++i) CHECK(samePose(viewPoses(rig, 1)[i], spec.zones[i].local, 1e-6f));
}

TEST_CASE("zone poses: silence and a disabled robot keep the last poses while the flags clear") {
    Rig rig;
    rig.add(1, AllianceStation::Blue1, zonedRobot());
    ZoneInputBuilder client(rig, 1, zonedRobot());
    const Pose3 held = poseAt(0.7f, 0.2f, 0.55f, 0.3f);
    REQUIRE(client.send(rig.nextTick(), {{1, held}}, 0b010u).ok());
    rig.tick();
    CHECK(rig.session->robots().find(1)->zoneFlags == 0b010u);

    rig.setEnabled(false);
    rig.tick();
    CHECK(rig.session->robots().find(1)->zoneFlags == 0u);
    CHECK(samePose(viewPoses(rig, 1)[1], held));
    rig.setEnabled(true);

    const int timeoutTicks = static_cast<int>(match::kInputTimeoutUs / rig.session->clock().fixedStepUs());
    rig.tick(timeoutTicks + 20);
    CHECK(rig.session->robots().find(1)->zoneFlags == 0u);   // timed out
    CHECK(samePose(viewPoses(rig, 1)[1], held));              // but the zone stays where it was put
}

TEST_CASE("zone poses: flooding with maximal pose lists stays bounded") {
    Rig rig;
    RobotConfig spec = swerveRobot("Many", match::kMaxZonesPerRobot);
    rig.add(1, AllianceStation::Blue1, spec);
    ZoneInputBuilder client(rig, 1, spec);
    std::vector<std::pair<std::size_t, Pose3>> all;
    for (std::size_t i = 0; i < match::kMaxZonesPerRobot; ++i) all.push_back({i, poseAt(0.1f * static_cast<float>(i), 0.f, 0.4f)});
    for (int i = 0; i < 3000; ++i) (void)client.send(rig.nextTick() + 150, all);
    const RemoteInputStats stats = rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.queueDepth <= kMaxQueuedRemoteInputs);
    CHECK(stats.evicted > 0);
    CHECK(stats.zonePosesApplied == 0);   // none of them is due
    rig.tick(3);
    CHECK(rig.session->remoteRobotInfo(1)->zonePoses.size() == match::kMaxZonesPerRobot);
}

TEST_CASE("zone poses: the snapshot sample of a remote robot carries each zone's effective pose") {
    RigOptions options;
    options.recordingGame = true;
    Rig rig(options);
    const RobotConfig spec = zonedRobot();
    rig.add(5, AllianceStation::Red3, spec);
    rig.add(6, AllianceStation::Blue1, tankRobot("Tk"));
    ZoneInputBuilder client(rig, 5, spec);
    const Pose3 moved = poseAt(0.6f, 0.1f, 0.6f, 0.4f);
    REQUIRE(client.send(rig.nextTick(), {{1, moved}}, 0b010u).ok());
    rig.tick(3);

    const auto snap = rig.session->buildSnapshot(false, true);
    REQUIRE(snap->remoteRobots().size() == 2);
    const RobotSample& a = snap->remoteRobots()[0];
    REQUIRE(a.zones.size() == 3);
    REQUIRE(a.zones[0].poseInChassis.has_value());
    REQUIRE(a.zones[1].poseInChassis.has_value());
    CHECK(samePose(*a.zones[0].poseInChassis, spec.zones[0].local, 1e-6f));
    CHECK(samePose(*a.zones[1].poseInChassis, moved));
    CHECK(a.zones[1].active);
    CHECK(samePose(*a.zones[2].poseInChassis, spec.zones[2].local, 1e-6f));
    CHECK(snap->remoteRobots()[1].zones.empty());
}
