#include "../runtime/remote_robot_rig.h"
#include "match_test_support.h"

using namespace match_test;
namespace rtest = remote_test;

namespace {

// The real session over Box3D, the match host over it, and a scripted client that drives its robot like a game client.
struct RealMatch {
    rtest::Rig rig;
    SimulationMatchSession adapter;
    Harness h;
    std::unique_ptr<TestClient> client;
    std::optional<DriveKinematics> kinematics;
    RobotConfig spec;
    std::uint32_t seq = 0;
    std::size_t consumed = 0;
    std::vector<wire::MatchSnapshot> snapshots;

    explicit RealMatch(RobotConfig robot = rtest::swerveRobot("Mine"), MatchHostConfig cfg = {})
        : adapter(*rig.session), h(adapter, std::move(cfg)), spec(std::move(robot)) {
        client = std::make_unique<TestClient>(h);
        wire::Join join;
        join.displayName = "Driver";
        join.gamePluginId = adapter.gamePluginId();
        join.wpilibYear = adapter.wpilibYear();
        join.robotSpecJson = specJson(spec);
        REQUIRE(client->join(join));
        auto kin = DriveKinematics::create(spec.drivetrain.spec);
        REQUIRE(kin.ok());
        kinematics.emplace(std::move(kin).value());
    }

    void sendInput(const ChassisSpeeds& speeds, std::uint64_t targetTick, std::uint64_t epoch = 0) {
        auto setpoints = kinematics->update(speeds);
        REQUIRE(setpoints.ok());
        wire::MatchInputPacket packet;
        packet.sessionId = client->accept->sessionId;
        packet.runEpoch = epoch != 0 ? epoch : client->accept->runEpoch;
        packet.inputs.push_back(rtest::inputOf(++seq, targetTick, setpoints.value()));
        REQUIRE(client->send(packet).ok());
    }

    // One sim tick as the runner does it: onPoll, session tick, onTickEnd; the client receives what was sent.
    void tick() {
        h.clock.advance(5ms);
        h.poll();
        rig.tick();
        h.tickEnd();
        REQUIRE(h.host->waitOutboundIdle(3s));
        client->drain();
        for (; consumed < client->log.size(); ++consumed)
            if (const auto* s = std::get_if<wire::MatchSnapshot>(&client->log[consumed])) snapshots.push_back(*s);
    }

    // Drives for `ticks` ticks, a fresh input every fourth tick (50 Hz, under the host's input rate limit).
    void drive(const ChassisSpeeds& speeds, int ticks, int lead = 0) {
        for (int i = 0; i < ticks; ++i) {
            if (i % 4 == 0) {
                const std::uint64_t before = h.host->stats().inputsSubmitted;
                sendInput(speeds, rig.nextTick() + static_cast<std::uint64_t>(lead));
                REQUIRE(h.pumpUntil([&] { return h.host->stats().inputsSubmitted > before; }));
            }
            tick();
        }
    }

    const wire::RobotWireState* robotOf(const wire::MatchSnapshot& s, wire::ParticipantId id) const {
        for (const wire::RobotWireState& r : s.robots)
            if (r.participantId == id) return &r;
        return nullptr;
    }
};

double distanceMm(const wire::RobotWireState& a, const wire::RobotWireState& b) {
    return std::hypot(static_cast<double>(a.xMm - b.xMm), static_cast<double>(a.yMm - b.yMm));
}

}  // namespace

TEST_CASE("match host with the real session: a scripted client drives its robot through the physics") {
    RealMatch m;
    REQUIRE(m.rig.session->remoteRobotIds() == std::vector<wire::ParticipantId>{1});
    m.tick();
    REQUIRE_FALSE(m.snapshots.empty());
    const wire::MatchSnapshot first = m.snapshots.back();
    const wire::RobotWireState* start = m.robotOf(first, 1);
    REQUIRE(start != nullptr);
    CHECK(start->wheels.size() == 4);
    const wire::RobotWireState* host = m.robotOf(first, wire::kHostParticipantId);
    REQUIRE(host != nullptr);
    CHECK(host->xMm == doctest::Approx(rtest::kHostStartX * 1000.f).epsilon(0.01));

    m.drive({1.5f, 0.f, 0.f}, 240);
    const wire::MatchSnapshot last = m.snapshots.back();
    const wire::RobotWireState* end = m.robotOf(last, 1);
    REQUIRE(end != nullptr);
    CHECK(distanceMm(*start, *end) > 500.0);   // it drove
    CHECK((std::abs(end->vxMmps) + std::abs(end->vyMmps)) > 300);

    // The client's reconciliation data: the newest applied input and the tick it applied on.
    CHECK(last.ackInputSeq + 2 >= m.seq);
    CHECK(last.ackInputSeq != wire::kNoInputSeq);
    REQUIRE(last.ackTick != proto::kNoAckTick);
    CHECK(last.ackTick + 12 >= last.tick);
    CHECK(last.ackTick <= last.tick);
    CHECK(last.inputLeadTicks == 0);   // each input was sent for the tick it arrived on
    CHECK(last.tick == m.rig.session->clock().tickIndex());
    CHECK(last.info.state == wire::MatchState::Running);
    CHECK(last.info.enabled);
    CHECK(last.simTimeUs == m.rig.session->clock().simTimeUs());

    // The wheels turn: the spin angle of at least one wheel changed.
    bool spun = false;
    for (std::size_t w = 0; w < end->wheels.size(); ++w) spun = spun || end->wheels[w].spinAngleQ != start->wheels[w].spinAngleQ;
    CHECK(spun);
    CHECK(m.h.host->stats().inputsRejected == 0);
    CHECK(m.h.host->stats().inputsRateLimited == 0);
}

TEST_CASE("match host with the real session: inputs scheduled ahead report their lead") {
    RealMatch m;
    m.tick();
    m.drive({1.f, 0.f, 0.f}, 120, 3);
    CHECK(m.snapshots.back().inputLeadTicks == 3);
    const rt::RemoteInputStats stats = m.rig.session->remoteRobotInfo(1)->stats;
    CHECK(stats.applied > 10);
    CHECK(stats.late == 0);
}

TEST_CASE("match host with the real session: a reset puts the robots back and drops the old epoch's inputs") {
    RealMatch m;
    m.tick();
    m.drive({1.5f, 0.f, 0.f}, 120);
    const wire::RobotWireState moved = *m.robotOf(m.snapshots.back(), 1);

    m.rig.session->reset();   // what the runner does on a Reset command
    m.h.obs.runEpoch = 2;
    m.tick();
    CHECK(m.h.host->runEpoch() == 2);
    REQUIRE(m.client->waitFor([&] {
        for (const auto& c : m.client->all<wire::MatchControl>())
            if (c.kind == wire::MatchControlKind::Reset && c.runEpoch == 2) return true;
        return false;
    }));
    const wire::RobotWireState back = *m.robotOf(m.snapshots.back(), 1);
    CHECK(distanceMm(moved, back) > 300.0);
    CHECK(m.snapshots.back().runEpoch == 2);
    CHECK(m.snapshots.back().ackInputSeq == wire::kNoInputSeq);   // the session forgot the old inputs

    // A packet of the old epoch is dropped, one of the new epoch is applied.
    m.sendInput({1.5f, 0.f, 0.f}, m.rig.nextTick(), 1);
    REQUIRE(m.h.pumpUntil([&] { return m.h.host->stats().framesStaleEpoch == 1; }));
    CHECK(m.h.host->stats().inputsRejected == 0);
    const std::uint64_t before = m.h.host->stats().inputsSubmitted;
    m.sendInput({1.5f, 0.f, 0.f}, m.rig.nextTick(), 2);
    REQUIRE(m.h.pumpUntil([&] { return m.h.host->stats().inputsSubmitted > before; }));
}

TEST_CASE("match host with the real session: spec clamps reach the physics and bad specs are refused") {
    rtest::Rig rig;
    SimulationMatchSession adapter(*rig.session);
    Harness h(adapter);
    TestClient c(h);
    RobotConfig heavy = rtest::swerveRobot("Heavy");
    heavy.chassis.massKg = 400.f;
    wire::Join join;
    join.gamePluginId = adapter.gamePluginId();
    join.wpilibYear = adapter.wpilibYear();
    join.robotSpecJson = specJson(heavy);
    REQUIRE(c.join(join));
    CHECK(rig.session->remoteRobotIds().size() == 1);

    TestClient bad(h);
    RobotConfig unknown = rtest::swerveRobot("Hover");
    unknown.drivetrain.spec.kind = "hovercraft";
    join.robotSpecJson = specJson(unknown);
    REQUIRE(bad.send(join).ok());
    REQUIRE(bad.waitFor([&] { return bad.count<wire::JoinReject>() > 0; }));
    CHECK(bad.last<wire::JoinReject>()->reason == wire::JoinRejectReason::BadSpec);
    CHECK(rig.session->remoteRobotIds().size() == 1);
}

TEST_CASE("match host with the real session: leaving and kicking remove the robot from the world") {
    RealMatch m;
    m.tick();
    REQUIRE(m.rig.session->remoteRobotIds().size() == 1);
    m.h.host->kick(1, "afk");
    m.h.poll();
    CHECK(m.rig.session->remoteRobotIds().empty());
    m.tick();   // the world keeps running without it
    CHECK(m.rig.session->buildSnapshot(false, true)->remoteRobots().empty());
    CHECK(m.client->all<wire::MatchControl>().back().kind == wire::MatchControlKind::Kick);

    RealMatch second;
    second.tick();
    wire::Leave leave;
    REQUIRE(second.client->send(leave).ok());
    REQUIRE(second.h.pumpUntil([&] { return second.rig.session->remoteRobotIds().empty(); }));
}

TEST_CASE("match host with the real session: two clients and the host share one world") {
    rtest::Rig rig;
    SimulationMatchSession adapter(*rig.session);
    Harness h(adapter);
    wire::Join join;
    join.gamePluginId = adapter.gamePluginId();
    join.wpilibYear = adapter.wpilibYear();
    join.robotSpecJson = specJson(rtest::swerveRobot("One"));
    TestClient a(h), b(h);
    REQUIRE(a.join(join));
    join.robotSpecJson = specJson(rtest::tankRobot("Two"));
    REQUIRE(b.join(join));
    CHECK(a.accept->station != b.accept->station);
    for (int i = 0; i < 60; ++i) {
        h.clock.advance(5ms);
        h.poll();
        rig.tick();
        h.tickEnd();
    }
    REQUIRE(h.host->waitOutboundIdle(3s));
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchSnapshot>() > 3; }));
    const wire::MatchSnapshot snap = *a.last<wire::MatchSnapshot>();
    CHECK(snap.robots.size() == 3);   // the host and both clients, identical for both
    REQUIRE(b.waitFor([&] { return b.count<wire::MatchSnapshot>() > 3; }));
    CHECK(b.last<wire::MatchSnapshot>()->robots.size() == 3);
}
