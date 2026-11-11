#include "match_test_support.h"

using namespace match_test;

namespace {

struct Joined {
    FakeSession s;
    Harness h;
    TestClient a;
    explicit Joined(MatchHostConfig cfg = {}) : h(s, std::move(cfg)), a(h) {
        REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    }
    // One sim tick: the session advances, the wall clock moves 5 ms, the host sends what is due.
    void tick(int count = 1) {
        for (int i = 0; i < count; ++i) {
            s.advance();
            h.clock.advance(5ms);
            h.tickEnd();
        }
    }
    bool submitted(std::uint64_t n) { return h.pumpUntil([&] { return h.host->stats().inputsSubmitted >= n; }); }
};

}  // namespace

TEST_CASE("match host inputs: each new input of a packet reaches the session once") {
    Joined j;
    REQUIRE(j.a.send(j.a.packetFor(1, 3, 10)).ok());
    REQUIRE(j.submitted(3));
    const auto& inputs = j.s.robots.at(1).inputs;
    REQUIRE(inputs.size() == 3);
    CHECK(inputs[0].inputSeq == 1);
    CHECK(inputs[2].inputSeq == 3);
    CHECK(inputs[0].wheels.size() == 4);

    REQUIRE(j.a.send(j.a.packetFor(1, 3, 10)).ok());   // a retransmission: nothing new
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().inputsRedundant >= 3; }));
    CHECK(j.h.host->stats().inputsSubmitted == 3);

    REQUIRE(j.a.send(j.a.packetFor(2, 3, 11)).ok());   // seq 2, 3 repeated, 4 new
    REQUIRE(j.submitted(4));
    CHECK(j.s.robots.at(1).inputs.back().inputSeq == 4);
    CHECK(j.s.robots.at(1).inputs.size() == 4);
}

TEST_CASE("match host inputs: the snapshot acknowledges the applied input and reports the lead") {
    Joined j;
    j.tick(10);
    REQUIRE(j.a.send(j.a.packetFor(1, 1, j.s.tick + 4)).ok());
    REQUIRE(j.submitted(1));
    j.tick(4);
    REQUIRE(j.h.host->waitOutboundIdle(3s));
    REQUIRE(j.a.waitFor([&] { return j.a.count<wire::MatchSnapshot>() > 0 && j.a.last<wire::MatchSnapshot>()->ackInputSeq == 1; }));
    const wire::MatchSnapshot snap = *j.a.last<wire::MatchSnapshot>();
    CHECK(snap.ackInputSeq == 1);
    CHECK(snap.ackTick == 10);
    CHECK(snap.inputLeadTicks == 4);
    CHECK(snap.sessionId == j.a.accept->sessionId);
    CHECK(snap.runEpoch == j.a.accept->runEpoch);
}

TEST_CASE("match host inputs: stale epochs and foreign sessions are dropped and counted") {
    Joined j;
    wire::MatchInputPacket foreign = j.a.packetFor(1, 1, 5);
    foreign.sessionId += 1;
    REQUIRE(j.a.send(foreign).ok());
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().framesForeignSession == 1; }));
    CHECK(j.h.host->stats().inputsSubmitted == 0);

    wire::MatchInputPacket old = j.a.packetFor(1, 1, 5);
    j.h.obs.runEpoch = 2;   // the world restarts
    j.h.poll();
    REQUIRE(j.a.send(old).ok());
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().framesStaleEpoch == 1; }));
    CHECK(j.h.host->stats().inputsSubmitted == 0);

    wire::MatchInputPacket current = j.a.packetFor(1, 1, 5);
    current.runEpoch = 2;
    REQUIRE(j.a.send(current).ok());
    REQUIRE(j.submitted(1));

    wire::MatchInputPacket future = j.a.packetFor(2, 1, 5);
    future.runEpoch = 9;   // an epoch the host never announced is a protocol violation
    REQUIRE(j.a.send(future).ok());
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().protocolViolations == 1; }));
}

TEST_CASE("match host: a reset starts a new epoch that every client follows") {
    FakeSession s;
    Harness h(s);
    TestClient a(h), b(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    REQUIRE(b.join(b.joinMessage("B", swerveSpec(), s)));
    h.poll();   // the first observation is the baseline
    CHECK(h.host->runEpoch() == 1);

    h.obs.runEpoch = 2;
    h.poll();
    CHECK(h.host->runEpoch() == 2);
    for (TestClient* c : {&a, &b}) {
        REQUIRE(c->waitFor([&] {
            for (const auto& m : c->all<wire::MatchControl>())
                if (m.kind == wire::MatchControlKind::Reset) return true;
            return false;
        }));
        const auto controls = c->all<wire::MatchControl>();
        CHECK(controls.back().kind == wire::MatchControlKind::Reset);
        CHECK(controls.back().runEpoch == 2);
        CHECK(controls.back().sessionId == c->accept->sessionId);
    }
    s.advance();
    h.tickEnd();
    REQUIRE(a.waitFor([&] { return a.last<wire::MatchSnapshot>() && a.last<wire::MatchSnapshot>()->runEpoch == 2; }));
    CHECK(h.host->stats().framesStaleEpoch == 0);
}

TEST_CASE("match host inputs: a flooding client is rate limited") {
    Joined j;
    constexpr std::uint32_t kPackets = 400;
    for (std::uint32_t i = 0; i < kPackets; ++i) REQUIRE(j.a.send(j.a.packetFor(1 + i, 1, 5)).ok());
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().inputPackets >= kPackets; }));
    const MatchHostConfig defaults;
    CHECK(j.h.host->stats().inputsSubmitted == defaults.inputBurst);   // the wall clock did not move: only the burst
    CHECK(j.h.host->stats().inputsRateLimited == kPackets - defaults.inputBurst);
    CHECK(j.h.host->participants()[0].inputsRateLimited == kPackets - defaults.inputBurst);

    j.h.clock.advance(1s);   // a second refills the bucket to the burst size
    for (std::uint32_t i = 0; i < 50; ++i) REQUIRE(j.a.send(j.a.packetFor(1000 + i, 1, 5)).ok());
    REQUIRE(j.h.pumpUntil([&] { return j.h.host->stats().inputPackets >= kPackets + 50; }));
    CHECK(j.h.host->stats().inputsSubmitted == 2 * defaults.inputBurst);
    CHECK(j.h.host->joinedCount() == 1);   // rate limiting is not a disconnect
}

TEST_CASE("match host: a normal 60 Hz client is never rate limited") {
    Joined j;
    for (std::uint32_t i = 0; i < 300; ++i) {
        j.h.clock.advance(16ms);
        REQUIRE(j.a.send(j.a.packetFor(1 + i, 1, 5)).ok());
        REQUIRE(j.submitted(i + 1));
    }
    CHECK(j.h.host->stats().inputsRateLimited == 0);
}

TEST_CASE("match host: malformed frames are dropped and the third drops the client") {
    Joined j;
    const std::array<std::byte, 12> garbage{std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF},
                                            std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0xFF}};
    REQUIRE(j.a.sendRaw(garbage).ok());
    REQUIRE(j.a.sendRaw(garbage).ok());
    REQUIRE(j.a.send(j.a.packetFor(1, 1, 5)).ok());   // a good frame between the bad ones still works
    REQUIRE(j.submitted(1));
    CHECK_FALSE(j.a.disconnected());
    CHECK(j.h.pumpUntil([&] { return j.h.host->stats().malformedFrames == 2; }));
    REQUIRE(j.a.sendRaw(garbage).ok());
    CHECK(j.a.waitFor([&] { return j.a.disconnected(); }));
    REQUIRE(j.h.pumpUntil([&] { return !j.h.host->participants()[0].connected; }));
    CHECK(j.h.host->stats().malformedFrames == 3);
    CHECK(j.s.robots.size() == 1);   // resumable
}

TEST_CASE("match host: a client cannot control anything but its own robot") {
    Joined j;
    for (const wire::MatchControlKind kind : {wire::MatchControlKind::Reset, wire::MatchControlKind::Pause, wire::MatchControlKind::Kick}) {
        wire::MatchControl ctl;
        ctl.kind = kind;
        ctl.sessionId = j.a.accept->sessionId;
        ctl.target = 0;
        REQUIRE(j.a.send(ctl).ok());
    }
    CHECK(j.a.waitFor([&] { return j.a.disconnected(); }));
    CHECK(j.h.host->stats().protocolViolations == 3);
    CHECK(j.h.host->runEpoch() == 1);
    CHECK(j.s.robots.size() == 1);
}

TEST_CASE("match host: the robot spec of a join is clamped, not trusted") {
    FakeSession s;
    Harness h(s);
    TestClient a(h);
    RobotConfig spec = swerveSpec("Heavy");
    spec.chassis.massKg = 500.f;   // an FRC robot is at most 100 kg in the limit table
    REQUIRE(a.join(a.joinMessage("Cheater", spec, s)));
    REQUIRE(s.robots.size() == 1);
    CHECK(s.robots.at(1).request.spec.chassis.massKg == doctest::Approx(kRemoteSpecLimits.massKg.max));
    CHECK(s.robots.at(1).request.spec.drivetrain.spec.wheels.size() == 4);
    bool logged = false;
    for (const auto& e : h.log.entries()) logged = logged || e.message.find("clamped robot spec") != std::string::npos;
    CHECK(logged);
}

TEST_CASE("match host: a slow client is dropped without ever blocking the sim thread") {
    MatchHostConfig cfg;
    cfg.peerTimeout = 1h;
    cfg.maxPingsPerSecond = 1000;   // the backlog below is built from ping replies
    cfg.pingBurst = 1000;
    FakeSession s;
    Harness h(s, cfg);
    ScriptedClient slow(h);
    REQUIRE(slow.client->join(slow.client->joinMessage("Slow", swerveSpec(), s)));
    TestClient healthy(h);
    REQUIRE(healthy.join(healthy.joinMessage("Fast", swerveSpec(), s)));
    REQUIRE(slow.transport != nullptr);
    slow.transport->blockSends = true;

    const auto start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration worst{};
    for (int i = 0; i < 200; ++i) {
        s.advance();
        h.clock.advance(5ms);
        const auto t0 = std::chrono::steady_clock::now();
        h.tickEnd();
        worst = std::max(worst, std::chrono::steady_clock::now() - t0);
    }
    CHECK(worst < 500ms);
    CHECK(std::chrono::steady_clock::now() - start < 5s);
    CHECK(h.host->participants()[0].connected);   // snapshots alone only get dropped oldest-first
    CHECK(h.host->stats().snapshotsDroppedSlow > 0);

    SUBCASE("a stuck send is detected by the wall clock") {
        REQUIRE(h.pumpUntil([&] {
            h.clock.advance(500ms);
            return !h.host->participants()[0].connected;
        }));
        CHECK(h.host->stats().slowPeersDropped >= 1);
    }
    SUBCASE("a reliable backlog overflows") {
        for (int i = 0; i < 80; ++i) {
            wire::MatchControl ping;
            ping.kind = wire::MatchControlKind::PingRequest;
            ping.sessionId = slow.client->accept->sessionId;
            REQUIRE(slow.client->send(ping).ok());
        }
        REQUIRE(h.pumpUntil([&] { return !h.host->participants()[0].connected; }));
        CHECK(h.host->stats().slowPeersDropped >= 1);
    }
    CHECK(h.host->participants()[1].connected);   // the healthy client was never affected
    h.tickEnd();
    CHECK(healthy.waitFor([&] { return healthy.count<wire::MatchSnapshot>() > 0; }));
}

TEST_CASE("match host: connections beyond the limit are refused") {
    FakeSession s;
    MatchHostConfig cfg;
    cfg.maxConnections = 3;
    Harness h(s, cfg);
    std::vector<std::unique_ptr<TestClient>> clients;
    for (int i = 0; i < 5; ++i) clients.push_back(std::make_unique<TestClient>(h));
    REQUIRE(h.pumpUntil([&] { return h.host->stats().connections + h.host->stats().connectionsRefused == 5; }));
    CHECK(h.host->stats().connections == 3);
    CHECK(h.host->stats().connectionsRefused == 2);
}

TEST_CASE("match host: threads adding hostile connections while the sim thread polls") {
    FakeSession s;
    MatchHostConfig cfg;
    cfg.maxConnections = 64;
    Harness h(s, cfg);
    std::atomic<int> finished{0};
    constexpr int kThreads = 4;
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            for (int i = 0; i < 6; ++i) {
                auto pair = testing::makeImpairedPair(h.clock, {}, static_cast<std::uint64_t>(t * 100 + i));
                h.host->addConnection(std::move(pair.a));
                proto::MessageChannel channel(std::move(pair.b), h.codec);
                if (i % 3 == 0) {
                    wire::Join join;
                    join.gamePluginId = "fake_game";
                    join.wpilibYear = 2099;
                    join.robotSpecJson = specJson(swerveSpec());
                    (void)channel.send(join);
                } else if (i % 3 == 1) {
                    const std::array<std::byte, 8> junk{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}, std::byte{7}, std::byte{8}};
                    for (int k = 0; k < 5; ++k) (void)channel.transport().send(junk);
                } else {
                    wire::MatchControl ping;
                    ping.kind = wire::MatchControlKind::PingRequest;
                    for (int k = 0; k < 20; ++k) (void)channel.send(ping);
                }
                std::this_thread::sleep_for(1ms);
                channel.close();
            }
            ++finished;
        });
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
        s.advance();
        h.clock.advance(5ms);
        h.poll();
        h.tickEnd();
        if (finished.load() == kThreads && h.host->stats().connections >= kThreads * 6u) break;
        std::this_thread::sleep_for(500us);
    }
    for (std::thread& th : threads) th.join();
    for (int i = 0; i < 50; ++i) h.poll();
    CHECK(finished.load() == kThreads);
    CHECK(h.host->joinedCount() <= wire::kMaxRobotsPerMatch - 1);
    CHECK(s.robots.size() == h.host->joinedCount());
    CHECK(h.host->stats().joinsAccepted + h.host->stats().joinsRejected >= 1);
}
