#include "client_test_support.h"

using namespace client_test;

namespace {

struct Rig {
    testing::ManualWallClock clock;
    ClientKit kit{clock};
    ScriptedHost host{kit.codec};
    std::vector<std::unique_ptr<ScriptedHost>> later;   // hosts of reconnects
    mc::MatchClientConfig config;
    std::unique_ptr<mc::MatchClient> client;
    RobotConfig robot = swerveSpec("Me");
    std::uint64_t tick = 0;

    explicit Rig(bool withReconnect = false, mc::MatchClientConfig cfg = {}) : config(std::move(cfg)) {
        config.displayName = "Me";
        mc::MatchClientServices services = kit.services();
        if (withReconnect)
            services.reconnect = [this]() -> Result<std::unique_ptr<proto::ITransport>> {
                later.push_back(std::make_unique<ScriptedHost>(kit.codec));
                later.back()->session = 0x2222 + later.size();
                return later.back()->clientEnd();
            };
        client = std::make_unique<mc::MatchClient>(config, std::move(services));
    }

    void connect() {
        const Status s = client->connect(host.clientEnd(), robot);
        REQUIRE_MESSAGE(s.ok(), (s.ok() ? "" : s.error().message));
    }
    // One 5 ms frame of a host running at the manual clock's pace.
    void frame(ScriptedHost& h, wire::MatchState state = wire::MatchState::Running, bool sendSnapshot = true) {
        clock.advance(5ms);
        if (state == wire::MatchState::Running) ++tick;
        h.answerPings(static_cast<std::int64_t>(tick) * 5000, tick);
        if (sendSnapshot) REQUIRE(h.send(h.snapshot(tick, state)).ok());
        client->update();
    }
    void frames(ScriptedHost& h, int n, wire::MatchState state = wire::MatchState::Running) {
        for (int i = 0; i < n; ++i) frame(h, state);
    }
    mc::ConnectionState state() const { return client->status().state; }
};

}  // namespace

TEST_CASE("match client join: the handshake reaches the lobby with the roster and the accepted spec") {
    Rig r;
    CHECK(r.state() == mc::ConnectionState::Disconnected);
    r.connect();
    CHECK(r.state() == mc::ConnectionState::Connecting);
    const std::optional<wire::Join> join = r.host.last<wire::Join>();
    REQUIRE(join.has_value());
    CHECK(join->displayName == "Me");
    CHECK(join->protocolVersion == wire::kMatchProtocolVersion);
    CHECK_FALSE(join->robotSpecJson.empty());
    CHECK_FALSE(wire::hasResumeToken(join->resumeToken));

    r.host.accept(wire::MatchState::Lobby);
    r.client->update();
    CHECK(r.state() == mc::ConnectionState::Lobby);
    const auto view = r.client->view();
    REQUIRE(view);
    CHECK(view->roster.size() == 2);
    CHECK(view->sessionId == r.host.session);
    REQUIRE(view->hasOwn);
    CHECK(view->own.id == 1);
    CHECK(view->own.pose.position.x == doctest::Approx(14.f));   // where the host put it, until a snapshot says otherwise
    CHECK_FALSE(view->own.predicted);
    CHECK(r.client->wheelCount() == 4);
    REQUIRE(r.client->acceptedSpec());
    CHECK(r.client->fieldDescription() != nullptr);
}

TEST_CASE("match client join: every refusal of the host is reported with its reason") {
    for (std::uint8_t code = 0; code <= static_cast<std::uint8_t>(wire::JoinRejectReason::BadResumeToken); ++code) {
        Rig r;
        r.connect();
        wire::JoinReject reject;
        reject.reason = static_cast<wire::JoinRejectReason>(code);
        reject.detail = "no because " + std::to_string(code);
        REQUIRE(r.host.send(reject).ok());
        r.client->update();
        const mc::ConnectionStatus s = r.client->status();
        CHECK(s.state == mc::ConnectionState::Disconnected);
        CHECK(s.reason == mc::DisconnectReason::JoinRejected);
        REQUIRE(s.rejectReason.has_value());
        CHECK(*s.rejectReason == reject.reason);
        CHECK(s.detail == reject.detail);
    }
}

TEST_CASE("match client join: a robot description the host's intake refuses never leaves the client") {
    Rig r;
    r.robot.drivetrain.spec.wheels.clear();
    const Status s = r.client->connect(r.host.clientEnd(), r.robot);
    CHECK_FALSE(s.ok());
    const mc::ConnectionStatus status = r.client->status();
    CHECK(status.state == mc::ConnectionState::Disconnected);
    CHECK(status.reason == mc::DisconnectReason::BadSpec);
    CHECK_FALSE(status.detail.empty());
    CHECK(r.host.all<wire::Join>().empty());
}

TEST_CASE("match client join: a robot outside the fairness limits is clamped like the host will clamp it, and told so") {
    Rig r;
    r.robot.chassis.massKg = 900.f;
    r.connect();
    r.host.accept();
    r.client->update();
    const auto view = r.client->view();
    CHECK_FALSE(view->specClamps.empty());
    REQUIRE(r.client->acceptedSpec());
    CHECK(r.client->acceptedSpec()->chassis.massKg <= static_cast<float>(sdk::kRemoteSpecLimits.massKg.max));
    // The Join still carries what the player has: the host does its own clamp.
    CHECK(r.host.last<wire::Join>()->robotSpecJson.find("900") != std::string::npos);
}

TEST_CASE("match client join: no answer for the join timeout disconnects") {
    Rig r;
    r.connect();
    r.clock.advance(4900ms);
    r.client->update();
    CHECK(r.state() == mc::ConnectionState::Connecting);
    r.clock.advance(200ms);
    r.client->update();
    CHECK(r.client->status().reason == mc::DisconnectReason::JoinTimeout);
    CHECK(r.state() == mc::ConnectionState::Disconnected);
}

TEST_CASE("match client state: lobby, live, paused and stalled follow the host and the world pauses with it") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Lobby);
    r.frames(r.host, 3, wire::MatchState::Lobby);
    CHECK(r.state() == mc::ConnectionState::Lobby);
    CHECK(r.client->view()->hud.state == wire::MatchState::Lobby);

    r.frames(r.host, 40);
    CHECK(r.state() == mc::ConnectionState::Live);
    CHECK(r.client->view()->own.enabled);

    r.host.control(wire::MatchControlKind::Pause);
    r.frames(r.host, 2, wire::MatchState::Paused);
    CHECK(r.state() == mc::ConnectionState::Paused);
    r.host.control(wire::MatchControlKind::Resume);
    r.frames(r.host, 10);
    CHECK(r.state() == mc::ConnectionState::Live);

    r.frames(r.host, 3, wire::MatchState::HostStalled);
    CHECK(r.state() == mc::ConnectionState::HostStalled);
    CHECK(r.client->view()->connection.state == mc::ConnectionState::HostStalled);
    r.frames(r.host, 10);
    CHECK(r.state() == mc::ConnectionState::Live);
}

TEST_CASE("match client state: Running snapshots whose sim time stands still are a stalled host") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 30);
    CHECK(r.state() == mc::ConnectionState::Live);
    // The host keeps sending snapshots that say Running but its world no longer advances.
    for (int i = 0; i < 100; ++i) {
        r.clock.advance(10ms);
        r.host.answerPings(r.tick * 5000, r.tick);
        REQUIRE(r.host.send(r.host.snapshot(r.tick, wire::MatchState::Running)).ok());
        r.client->update();
    }
    CHECK(r.state() == mc::ConnectionState::HostStalled);
    // ... and moves again.
    r.frames(r.host, 20);
    CHECK(r.state() == mc::ConnectionState::Live);
}

TEST_CASE("match client state: a kick, a closing server and leaving end the session with their reason") {
    {
        Rig r;
        r.connect();
        r.host.accept(wire::MatchState::Running);
        r.frames(r.host, 5);
        r.host.control(wire::MatchControlKind::Kick, "bad manners", 1);
        r.client->update();
        CHECK(r.client->status().reason == mc::DisconnectReason::Kicked);
        CHECK(r.client->status().detail == "bad manners");
        CHECK(r.state() == mc::ConnectionState::Disconnected);
    }
    {
        Rig r;
        r.connect();
        r.host.accept(wire::MatchState::Running);
        r.frames(r.host, 5);
        r.host.control(wire::MatchControlKind::ServerClosing, "bye");
        r.client->update();
        CHECK(r.client->status().reason == mc::DisconnectReason::ServerClosing);
    }
    {
        Rig r;
        r.connect();
        r.host.accept(wire::MatchState::Running);
        r.frames(r.host, 5);
        r.client->leave("done");
        CHECK(r.client->status().reason == mc::DisconnectReason::Left);
        CHECK(r.host.all<wire::Leave>().size() == 1);
        CHECK(r.host.last<wire::Leave>()->detail == "done");
    }
}

TEST_CASE("match client link: a silent host and a closed link are told apart") {
    {
        Rig r;
        r.connect();
        r.host.accept(wire::MatchState::Running);
        r.frames(r.host, 20);
        r.clock.advance(2900ms);
        r.client->update();
        CHECK(r.state() == mc::ConnectionState::Live);   // quiet, not yet dead
        r.clock.advance(200ms);
        r.client->update();
        CHECK(r.client->status().reason == mc::DisconnectReason::HostSilent);
    }
    {
        Rig r;
        r.connect();
        r.host.accept(wire::MatchState::Running);
        r.frames(r.host, 5);
        r.host.close();
        r.client->update();
        CHECK(r.client->status().reason == mc::DisconnectReason::TransportLost);
    }
}

TEST_CASE("match client link: a lost link is re-established with the resume token and the same identity") {
    Rig r(true);
    r.connect();
    r.host.accept(wire::MatchState::Running, 3);
    r.frames(r.host, 30);
    CHECK(r.client->view()->own.id == 3);
    r.host.close();
    r.client->update();
    mc::ConnectionStatus s = r.client->status();
    CHECK(s.state == mc::ConnectionState::Connecting);
    CHECK(s.reconnecting);
    REQUIRE(r.later.size() == 1);
    // The rejoin carries the token of the last JoinAccept.
    const std::optional<wire::Join> rejoin = r.later[0]->last<wire::Join>();
    REQUIRE(rejoin.has_value());
    CHECK(rejoin->resumeToken == r.host.token);
    r.later[0]->epoch = 1;
    r.later[0]->accept(wire::MatchState::Running, 3, r.tick);
    r.client->update();
    s = r.client->status();
    CHECK(s.state == mc::ConnectionState::Live);
    CHECK_FALSE(s.reconnecting);
    CHECK(r.client->view()->own.id == 3);
    CHECK(r.client->view()->sessionId == r.later[0]->session);
    // Input numbering continues across the reconnect, so the host's duplicate filter keeps working.
    r.frames(*r.later[0], 30);
    const auto packets = r.later[0]->all<wire::MatchInputPacket>();
    REQUIRE_FALSE(packets.empty());
    CHECK(packets.front().sessionId == r.later[0]->session);
}

TEST_CASE("match client link: giving up after the attempts leaves the client disconnected") {
    mc::MatchClientConfig cfg;
    cfg.maxReconnectAttempts = 3;
    cfg.reconnectInitialBackoff = 100ms;
    Rig r(true, cfg);
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 10);
    r.host.close();
    // Every new link dies at once.
    for (int i = 0; i < 100 && r.client->status().state != mc::ConnectionState::Disconnected; ++i) {
        r.client->update();
        for (auto& h : r.later) h->close();
        r.clock.advance(100ms);
    }
    r.client->update();
    CHECK(r.state() == mc::ConnectionState::Disconnected);
    CHECK(r.client->status().reason == mc::DisconnectReason::TransportLost);
    CHECK(r.later.size() <= 4);
}

TEST_CASE("match client frames: stale, foreign and out-of-order snapshots are dropped and counted") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 10);
    const std::uint64_t received = r.client->view()->diagnostics.snapshotsReceived;

    wire::MatchSnapshot foreign = r.host.snapshot(r.tick + 1, wire::MatchState::Running);
    foreign.sessionId += 1;
    REQUIRE(r.host.send(foreign).ok());
    wire::MatchSnapshot old = r.host.snapshot(r.tick + 1, wire::MatchState::Running);
    old.snapshotSeq = 2;   // behind the newest applied
    REQUIRE(r.host.send(old).ok());
    r.client->update();
    const auto d = r.client->view()->diagnostics;
    CHECK(d.snapshotsReceived == received);
    CHECK(d.snapshotsDropped == 1);
    CHECK(d.snapshotsStale == 1);
}

TEST_CASE("match client frames: a reset starts a new epoch, older frames are dropped and the timing starts over") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 40);
    CHECK(r.client->view()->runEpoch == 1);
    CHECK(r.client->view()->diagnostics.clockSynced);

    wire::MatchSnapshot stale = r.host.snapshot(r.tick + 1, wire::MatchState::Running);   // epoch 1, sent before the reset
    r.host.epoch = 2;
    r.host.control(wire::MatchControlKind::Reset);
    r.client->update();
    CHECK(r.client->view()->runEpoch == 2);
    CHECK(r.client->view()->diagnostics.resyncs >= 1);
    stale.runEpoch = 1;
    REQUIRE(r.host.send(stale).ok());
    r.client->update();
    CHECK(r.client->view()->diagnostics.snapshotsDropped == 1);

    // The new world restarts at tick 0.
    r.tick = 0;
    r.frames(r.host, 30);
    CHECK(r.state() == mc::ConnectionState::Live);
    CHECK(r.client->view()->diagnostics.clockSynced);
    CHECK(r.client->view()->renderSimTimeUs > 50000);
    CHECK(r.client->view()->renderSimTimeUs <= 160000);   // the new world is 150 ms old, drawn a little in the past
}

TEST_CASE("match client frames: a snapshot of a newer epoch than announced is adopted") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 20);
    r.host.epoch = 3;
    r.tick = 0;
    r.frames(r.host, 5);
    CHECK(r.client->view()->runEpoch == 3);
    CHECK(r.state() == mc::ConnectionState::Live);
}

TEST_CASE("match client clock: a host whose sim time runs 0.06 percent fast is tracked and the estimate stays inside 3 ms") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    std::int64_t sim = 0;
    double worstMs = 0;
    for (int i = 0; i < 1600; ++i) {   // 8 s
        r.clock.advance(5ms);
        sim += 5003;               // the host's second is 600 ppm longer than the client's
        r.tick = static_cast<std::uint64_t>(sim / 5000);
        r.host.answerPings(sim, r.tick);
        REQUIRE(r.host.send(r.host.snapshot(r.tick, wire::MatchState::Running)).ok());
        r.client->update();
        if (i > 1200) {
            const auto d = r.client->view()->diagnostics;
            worstMs = std::max(worstMs, std::fabs(static_cast<double>(d.hostSimEstimateUs - sim)) / 1000.0);
        }
    }
    const auto d = r.client->view()->diagnostics;
    CHECK(d.clockDriftPpm > 350.0);
    CHECK(d.clockDriftPpm < 850.0);
    CHECK(worstMs < 3.0);
}

TEST_CASE("match client acknowledgements: every applied snapshot is acknowledged, latest wins") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 12);
    const auto acks = r.host.all<wire::MatchSnapshotAck>();
    REQUIRE_FALSE(acks.empty());
    CHECK(acks.back().snapshotSeq == r.host.seq);
    CHECK(acks.back().sessionId == r.host.session);
    CHECK(acks.back().runEpoch == 1);
}

TEST_CASE("match client input: a steady 60 Hz stream, redundant by three, stamped ahead of the host tick") {
    Rig r;
    REQUIRE(plugins::game_2026::registerPlugin(r.kit.kit.plugins).ok());
    r.robot = g26session::Rig::fuelRobot();   // a pickup (zone 0) and a shooter (zone 1)
    r.robot.gamePluginId = std::string(plugins::game_2026::pluginId());
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 40);
    const std::uint64_t before = r.client->view()->diagnostics.inputsSent;
    r.frames(r.host, 200);   // one second
    const std::uint64_t idle = r.client->view()->diagnostics.inputsSent - before;
    CHECK(idle >= 55);
    CHECK(idle <= 70);

    const auto packets = r.host.all<wire::MatchInputPacket>();
    REQUIRE_FALSE(packets.empty());
    const wire::MatchInputPacket& p = packets.back();
    CHECK(p.sessionId == r.host.session);
    CHECK(p.runEpoch == 1);
    REQUIRE(p.inputs.size() == wire::kInputRedundancy);
    for (std::size_t i = 1; i < p.inputs.size(); ++i) {
        CHECK(p.inputs[i].inputSeq == p.inputs[i - 1].inputSeq + 1);
        CHECK(p.inputs[i].targetTick > p.inputs[i - 1].targetTick);
    }
    CHECK(p.inputs.back().wheels.size() == 4);
    const std::int64_t lead = static_cast<std::int64_t>(p.inputs.back().targetTick) - static_cast<std::int64_t>(r.tick);
    CHECK(lead >= 3);
    CHECK(lead <= 25);
    for (const wire::WheelInput& w : p.inputs.back().wheels) CHECK(w.wheelSpeedRadPerSec == 0.f);

    // A changed command goes out within a couple of ticks, with the zone flag and a moved zone.
    const std::size_t known = packets.size();
    REQUIRE(r.client->driveRobotRelative({1.0f, 0.f, 0.f}).ok());
    r.client->setZoneActive(0, true);
    REQUIRE(r.client->setZonePose(0, {{0.3f, 0.1f, 0.4f}, sdk::Quat{}}).ok());
    int frames = 0;
    std::optional<wire::MatchInput> changed;
    for (; frames < 4 && !changed; ++frames) {
        r.frame(r.host);
        const auto now = r.host.all<wire::MatchInputPacket>();
        for (std::size_t i = known; i < now.size() && !changed; ++i)
            for (const wire::MatchInput& in : now[i].inputs)
                if (in.wheels[0].wheelSpeedRadPerSec > 1.f) {
                    changed = in;
                    break;
                }
    }
    REQUIRE(changed.has_value());
    CHECK(frames <= 3);
    CHECK(changed->zoneFlags == 1u);
    REQUIRE(changed->zonePoses.size() == 1);
    CHECK(changed->zonePoses[0].zoneIndex == 0);
    CHECK(changed->zonePoses[0].xMm == 300);
    CHECK(changed->zonePoses[0].yMm == 100);
    // The robot's own picture moves at once, with no round trip.
    r.frames(r.host, 6);
    CHECK(r.client->view()->own.predicted);
    CHECK(r.client->view()->own.linearVelocity.x != 0.f);

    // A wrong count of setpoints is refused, a field-relative command goes through the same path.
    CHECK_FALSE(r.client->setWheelSetpoints(std::vector<sdk::WheelSetpoint>(3)).ok());
    CHECK(r.client->driveFieldRelative({0.5f, 0.f, 0.f}).ok());
}

TEST_CASE("match client input: nothing is sent while the host's world is not running") {
    Rig r;
    r.connect();
    r.host.accept(wire::MatchState::Running);
    r.frames(r.host, 40);
    r.host.control(wire::MatchControlKind::Pause);
    r.frames(r.host, 3, wire::MatchState::Paused);
    const std::uint64_t sent = r.client->view()->diagnostics.inputsSent;
    r.frames(r.host, 100, wire::MatchState::Paused);
    CHECK(r.client->view()->diagnostics.inputsSent == sent);
    r.host.control(wire::MatchControlKind::Resume);
    r.frames(r.host, 60);
    CHECK(r.client->view()->diagnostics.inputsSent > sent + 5);
}
