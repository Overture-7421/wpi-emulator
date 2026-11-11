#include "match_test_support.h"

using namespace match_test;

namespace {

wire::JoinReject rejectOf(TestClient& c) {
    REQUIRE(c.waitFor([&] { return c.count<wire::JoinReject>() > 0; }));
    return *c.last<wire::JoinReject>();
}

}  // namespace

TEST_CASE("match host: a client joins, is accepted and everyone gets the roster") {
    FakeSession s;
    Harness h(s);
    h.obs.matchState = wire::MatchState::Lobby;
    TestClient alice(h);
    REQUIRE(alice.join(alice.joinMessage("Alice", swerveSpec(), s)));
    const wire::JoinAccept& acc = *alice.accept;
    CHECK(acc.participantId == 1);
    CHECK(acc.sessionId != proto::kUnspecified);
    CHECK(acc.runEpoch == proto::kFirstRunEpoch);
    CHECK(acc.station == proto::AllianceStation::Blue1);   // the host is Red1: the first joiner balances the alliances
    CHECK(acc.stepUs == 5000);
    CHECK(acc.snapshotRateHz == wire::kSnapshotRateLanHz);
    CHECK(acc.inputRateHz == wire::kDefaultInputRateHz);
    CHECK(wire::hasResumeToken(acc.resumeToken));
    CHECK(acc.state == wire::MatchState::Lobby);
    CHECK(acc.startPose.position.x == doctest::Approx(1.f));
    REQUIRE(acc.roster.size() == 2);
    CHECK(acc.roster[0].isHost);
    CHECK(acc.roster[0].participantId == wire::kHostParticipantId);
    CHECK(acc.roster[0].name == "Host");
    CHECK(acc.roster[0].station == proto::AllianceStation::Red1);
    CHECK(acc.roster[1].participantId == 1);
    CHECK(acc.roster[1].name == "Alice");
    REQUIRE(s.robots.size() == 1);
    CHECK(s.robots.at(1).request.station == proto::AllianceStation::Blue1);

    TestClient bob(h);
    REQUIRE(bob.join(bob.joinMessage("Alice", swerveSpec(), s)));   // duplicate display names are fine
    CHECK(bob.accept->participantId == 2);
    CHECK(bob.accept->station == proto::AllianceStation::Red2);
    CHECK(bob.accept->sessionId != acc.sessionId);
    CHECK(bob.accept->resumeToken != acc.resumeToken);

    TestClient carol(h);
    REQUIRE(carol.join(carol.joinMessage("Carol", swerveSpec(), s)));
    CHECK(carol.accept->station == proto::AllianceStation::Blue2);

    REQUIRE(alice.waitFor([&] {
        const auto roster = alice.last<wire::Roster>();
        return roster && roster->entries.size() == 4;
    }));
    const wire::Roster roster = *alice.last<wire::Roster>();
    CHECK(roster.sessionId == acc.sessionId);
    CHECK(h.host->joinedCount() == 3);
    CHECK(h.host->stats().joinsAccepted == 3);
}

TEST_CASE("match host: the state in JoinAccept and snapshots follows the runner") {
    FakeSession s;
    Harness h(s);
    h.obs.matchState = wire::MatchState::Lobby;
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchSnapshot>() > 0; }));
    CHECK(a.last<wire::MatchSnapshot>()->info.state == wire::MatchState::Lobby);

    h.obs.matchState = wire::MatchState::HostStalled;
    h.clock.advance(200ms);
    REQUIRE(a.waitFor([&] { return a.last<wire::MatchSnapshot>()->info.state == wire::MatchState::HostStalled; }));
}

TEST_CASE("match host: every reject reason") {
    FakeSession s;
    Harness h(s);
    bool expectNoRobots = true;

    SUBCASE("bad protocol version") {
        TestClient c(h);
        wire::Join j = c.joinMessage("A", swerveSpec(), s);
        j.protocolVersion = wire::kMatchProtocolVersion + 5;
        REQUIRE(c.send(j).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::BadProtocol);
        CHECK(c.waitFor([&] { return c.disconnected(); }));
    }
    SUBCASE("wrong plugin") {
        TestClient c(h);
        wire::Join j = c.joinMessage("A", swerveSpec(), s);
        j.gamePluginId = "other_game";
        REQUIRE(c.send(j).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::WrongPlugin);
    }
    SUBCASE("wrong season") {
        TestClient c(h);
        wire::Join j = c.joinMessage("A", swerveSpec(), s);
        j.wpilibYear = 2001;
        REQUIRE(c.send(j).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::WrongSeason);
    }
    SUBCASE("the room is full") {
        std::vector<std::unique_ptr<TestClient>> clients;
        for (std::size_t i = 0; i < rt::kMaxRemoteRobots; ++i) {
            clients.push_back(std::make_unique<TestClient>(h));
            REQUIRE(clients.back()->join(clients.back()->joinMessage("P", swerveSpec(), s)));
        }
        TestClient late(h);
        REQUIRE(late.send(late.joinMessage("Late", swerveSpec(), s)).ok());
        CHECK(rejectOf(late).reason == wire::JoinRejectReason::RoomFull);
        CHECK(s.robots.size() == rt::kMaxRemoteRobots);
        expectNoRobots = false;
    }
    SUBCASE("a robot description that is not JSON") {
        TestClient c(h);
        wire::Join j = c.joinMessage("A", swerveSpec(), s);
        j.robotSpecJson = "{ not json";
        REQUIRE(c.send(j).ok());
        const wire::JoinReject r = rejectOf(c);
        CHECK(r.reason == wire::JoinRejectReason::BadSpec);
        CHECK_FALSE(r.detail.empty());
    }
    SUBCASE("a robot description of the maximum size that is not a robot") {
        TestClient c(h);
        wire::Join j = c.joinMessage("A", swerveSpec(), s);
        j.robotSpecJson.assign(wire::kMaxRobotSpecBytes, '[');
        REQUIRE(c.send(j).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::BadSpec);
    }
    SUBCASE("a wheel count the limits refuse") {
        TestClient c(h);
        RobotConfig spec = swerveSpec();
        spec.drivetrain.spec.wheels.resize(1);
        wire::Join j = c.joinMessage("A", spec, s);
        REQUIRE(c.send(j).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::BadSpec);
    }
    SUBCASE("a drivetrain kind the host does not know") {
        TestClient c(h);
        RobotConfig spec = swerveSpec();
        spec.drivetrain.spec.kind = "hovercraft";
        REQUIRE(c.send(c.joinMessage("A", spec, s)).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::BadSpec);
    }
    SUBCASE("the session refuses the robot") {
        s.addFailure = Error(ErrorCode::InvalidArgument, "zone count");
        TestClient c(h);
        REQUIRE(c.send(c.joinMessage("A", swerveSpec(), s)).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::BadSpec);
    }
    SUBCASE("the session reports the roster full") {
        s.addFailure = Error(ErrorCode::OutOfRange, "full");
        TestClient c(h);
        REQUIRE(c.send(c.joinMessage("A", swerveSpec(), s)).ok());
        CHECK(rejectOf(c).reason == wire::JoinRejectReason::RoomFull);
    }
    SUBCASE("the host is closing") {
        h.host->close("done");
        h.poll();
        TestClient c(h);
        CHECK(c.waitFor([&] { return c.disconnected(); }));
    }
    if (expectNoRobots) {
        CHECK(s.robots.empty());
        CHECK(h.host->joinedCount() == 0);
    }
}

TEST_CASE("match host: a join that never comes is dropped after the join timeout") {
    FakeSession s;
    Harness h(s);
    TestClient c(h);
    REQUIRE(h.pumpUntil([&] { return h.host->stats().connections == 1; }));
    h.clock.advance(1900ms);
    h.poll();
    CHECK_FALSE(c.disconnected());
    h.clock.advance(300ms);
    CHECK(c.waitFor([&] { return c.disconnected(); }));
    CHECK(h.host->stats().joinTimeouts == 1);
}

TEST_CASE("match host: messages before the join are strikes") {
    FakeSession s;
    Harness h(s);
    TestClient c(h);
    for (int i = 0; i < 3; ++i) REQUIRE(c.send(c.packetFor(1 + static_cast<std::uint32_t>(i), 1, 0)).ok());
    CHECK(c.waitFor([&] { return c.disconnected(); }));
    CHECK(h.host->stats().protocolViolations == 3);
}

TEST_CASE("match host: a client can rejoin with its resume token and keeps its robot") {
    FakeSession s;
    MatchHostConfig cfg;
    Harness h(s, cfg);
    auto first = std::make_unique<TestClient>(h);
    REQUIRE(first->join(first->joinMessage("A", swerveSpec(), s)));
    const wire::JoinAccept before = *first->accept;
    REQUIRE(s.robots.size() == 1);

    first->close();
    REQUIRE(h.pumpUntil([&] { return !h.host->participants().empty() && !h.host->participants()[0].connected; }));
    CHECK(s.robots.size() == 1);   // the robot stays while the participant may return
    CHECK(h.host->joinedCount() == 1);
    CHECK(h.host->constraints().peersConnected);

    SUBCASE("within the window") {
        h.clock.advance(20s);
        TestClient second(h);
        wire::Join j = second.joinMessage("A again", swerveSpec(), s);
        j.resumeToken = before.resumeToken;
        REQUIRE(second.join(j));
        const wire::JoinAccept& after = *second.accept;
        CHECK(after.participantId == before.participantId);
        CHECK(after.station == before.station);
        CHECK(after.sessionId != before.sessionId);
        CHECK(after.resumeToken != before.resumeToken);   // the token rotates on every join
        CHECK(s.robots.size() == 1);
        CHECK(s.removed.empty());
        CHECK(h.host->participants()[0].connected);
        CHECK(h.host->stats().resumes == 1);

        TestClient third(h);   // the old token is now useless: a fresh join
        wire::Join old = third.joinMessage("B", swerveSpec(), s);
        old.resumeToken = before.resumeToken;
        REQUIRE(third.join(old));
        CHECK(third.accept->participantId == 2);
    }
    SUBCASE("after the window the robot is gone and a fresh join happens") {
        h.clock.advance(31s);
        REQUIRE(h.pumpUntil([&] { return s.robots.empty(); }));
        CHECK(s.removed == std::vector<wire::ParticipantId>{1});
        TestClient second(h);
        wire::Join j = second.joinMessage("A", swerveSpec(), s);
        j.resumeToken = before.resumeToken;
        REQUIRE(second.join(j));
        CHECK(second.accept->participantId == 2);   // ids are never reused within a run
        CHECK(h.host->stats().resumes == 0);
    }
}

TEST_CASE("match host: resuming replaces a connection that is still open") {
    FakeSession s;
    Harness h(s);
    TestClient stale(h);
    REQUIRE(stale.join(stale.joinMessage("A", swerveSpec(), s)));
    TestClient fresh(h);
    wire::Join j = fresh.joinMessage("A", swerveSpec(), s);
    j.resumeToken = stale.accept->resumeToken;
    REQUIRE(fresh.join(j));
    CHECK(fresh.accept->participantId == stale.accept->participantId);
    CHECK(stale.waitFor([&] { return stale.disconnected(); }));
    CHECK(s.robots.size() == 1);
}

TEST_CASE("match host: leaving and timing out clean up the robot") {
    FakeSession s;
    Harness h(s);
    TestClient a(h), b(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    REQUIRE(b.join(b.joinMessage("B", swerveSpec(), s)));
    REQUIRE(s.robots.size() == 2);

    SUBCASE("Leave removes the robot at once and tells the others") {
        wire::Leave leave;
        leave.detail = "bye";
        REQUIRE(a.send(leave).ok());
        REQUIRE(h.pumpUntil([&] { return s.robots.size() == 1; }));
        CHECK(s.removed == std::vector<wire::ParticipantId>{1});
        CHECK(b.waitFor([&] { return b.last<wire::Roster>() && b.last<wire::Roster>()->entries.size() == 2; }));
        CHECK(h.host->stats().leaves == 1);
        CHECK(h.host->joinedCount() == 1);
    }
    SUBCASE("a silent client is dropped after the peer timeout and its robot after the resume window") {
        TestClient chatty(h);   // keeps talking so only `a` and `b` time out
        REQUIRE(chatty.join(chatty.joinMessage("C", swerveSpec(), s)));
        for (int i = 0; i < 6; ++i) {
            h.clock.advance(1s);
            wire::MatchControl ping;
            ping.kind = wire::MatchControlKind::PingRequest;
            ping.sessionId = chatty.accept->sessionId;
            REQUIRE(chatty.send(ping).ok());
            const std::uint64_t pings = h.host->stats().pings;
            REQUIRE(h.pumpUntil([&] { return h.host->stats().pings > pings; }));
        }
        CHECK(h.host->stats().peerTimeouts == 2);
        CHECK(a.waitFor([&] { return a.disconnected(); }));
        CHECK(s.robots.size() == 3);   // still resumable
        for (int i = 0; i < 31; ++i) {
            h.clock.advance(1s);
            wire::MatchControl ping;
            ping.kind = wire::MatchControlKind::PingRequest;
            ping.sessionId = chatty.accept->sessionId;
            REQUIRE(chatty.send(ping).ok());
            const std::uint64_t pings = h.host->stats().pings;
            REQUIRE(h.pumpUntil([&] { return h.host->stats().pings > pings; }));
        }
        CHECK(s.robots.size() == 1);
        CHECK(s.robots.count(3) == 1);
    }
}

TEST_CASE("match host: kick removes a participant and refuses its token") {
    FakeSession s;
    Harness h(s);
    TestClient a(h), b(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    REQUIRE(b.join(b.joinMessage("B", swerveSpec(), s)));
    const wire::ResumeToken token = a.accept->resumeToken;

    h.host->kick(a.accept->participantId, "no cheating");
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchControl>() > 0; }));
    const wire::MatchControl kick = *a.last<wire::MatchControl>();
    CHECK(kick.kind == wire::MatchControlKind::Kick);
    CHECK(kick.target == 1);
    CHECK(kick.detail == "no cheating");
    CHECK(kick.sessionId == a.accept->sessionId);
    CHECK(a.waitFor([&] { return a.disconnected(); }));
    CHECK(s.robots.size() == 1);
    CHECK(h.host->stats().kicks == 1);

    TestClient again(h);
    wire::Join j = again.joinMessage("A", swerveSpec(), s);
    j.resumeToken = token;
    REQUIRE(again.send(j).ok());
    CHECK(rejectOf(again).reason == wire::JoinRejectReason::Kicked);
}

TEST_CASE("match host: closing tells everyone, removes the robots and refuses new connections") {
    FakeSession s;
    Harness h(s);
    TestClient a(h), b(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    REQUIRE(b.join(b.joinMessage("B", swerveSpec(), s)));
    h.host->close("session over");
    h.poll();
    for (TestClient* c : {&a, &b}) {
        REQUIRE(c->waitFor([&] {
            for (const auto& m : c->all<wire::MatchControl>())
                if (m.kind == wire::MatchControlKind::ServerClosing) return true;
            return false;
        }));
        CHECK(c->waitFor([&] { return c->disconnected(); }));
    }
    CHECK(s.robots.empty());
    CHECK(h.host->joinedCount() == 0);
}

TEST_CASE("match host: pings are answered with the server tick and sim time") {
    FakeSession s;
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    s.advance(40);
    wire::MatchControl ping;
    ping.kind = wire::MatchControlKind::PingRequest;
    ping.sessionId = a.accept->sessionId;
    ping.pingSeq = 9;
    ping.clientTimeUs = 123456;
    REQUIRE(a.send(ping).ok());
    REQUIRE(a.waitFor([&] {
        for (const auto& m : a.all<wire::MatchControl>())
            if (m.kind == wire::MatchControlKind::PingReply) return true;
        return false;
    }));
    wire::MatchControl reply;
    for (const auto& m : a.all<wire::MatchControl>())
        if (m.kind == wire::MatchControlKind::PingReply) reply = m;
    CHECK(reply.pingSeq == 9);
    CHECK(reply.clientTimeUs == 123456);
    CHECK(reply.serverTick == 40);
    CHECK(reply.serverSimTimeUs == 40 * 5000);
    CHECK(reply.sessionId == a.accept->sessionId);

    SUBCASE("a ping of another session is ignored") {
        ping.sessionId = a.accept->sessionId + 1;
        const std::uint64_t pings = h.host->stats().pings;
        REQUIRE(a.send(ping).ok());
        REQUIRE(h.pumpUntil([&] { return h.host->stats().framesForeignSession > 0; }));
        CHECK(h.host->stats().pings == pings);
    }
}

TEST_CASE("match host: a ping flood is answered up to the burst and then dropped") {
    FakeSession s;
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    wire::MatchControl ping;
    ping.kind = wire::MatchControlKind::PingRequest;
    ping.sessionId = a.accept->sessionId;
    for (std::uint32_t i = 0; i < 50; ++i) {
        ping.pingSeq = i;
        REQUIRE(a.send(ping).ok());
    }
    REQUIRE(h.pumpUntil([&] { return h.host->stats().pings + h.host->stats().pingsRateLimited == 50; }));
    CHECK(h.host->stats().pings == 5);   // the burst: the wall clock did not move
    CHECK(h.host->stats().pingsRateLimited == 45);
    h.clock.advance(1s);
    ping.pingSeq = 99;
    REQUIRE(a.send(ping).ok());
    REQUIRE(h.pumpUntil([&] { return h.host->stats().pings == 6; }));
}

TEST_CASE("match host: joins beyond the per poll budget wait, a connection can queue only one") {
    MatchHostConfig cfg;
    cfg.joinsPerPoll = 1;
    FakeSession s;
    Harness h(s, cfg);
    TestClient a(h), b(h);
    const wire::Join ja = a.joinMessage("A", swerveSpec(), s);
    const wire::Join jb = b.joinMessage("B", swerveSpec(), s);
    REQUIRE(a.send(ja).ok());
    REQUIRE(b.send(jb).ok());
    REQUIRE(b.send(jb).ok());   // the second Join of one connection is a protocol violation, however busy the host is
    REQUIRE(a.waitFor([&] { return a.count<wire::JoinAccept>() > 0; }));
    REQUIRE(b.waitFor([&] { return b.count<wire::JoinAccept>() > 0; }));
    CHECK(h.host->stats().joinsAccepted == 2);
    CHECK(s.robots.size() == 2);
    CHECK(h.host->stats().protocolViolations >= 1);
}
TEST_CASE("match host: constraints are published while participants hold robots") {
    FakeSession s;
    Harness h(s);
    CHECK_FALSE(h.host->constraints().peersConnected);
    CHECK_FALSE(h.host->constraints().lockTimeScale);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    const HostConstraints c = h.host->constraints();
    CHECK(c.peersConnected);
    CHECK(c.lockTimeScale);
    CHECK(c.stepDisallowed);
}

TEST_CASE("match host: Pause and Resume are broadcast when the runner pauses") {
    FakeSession s;
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    h.obs.matchState = wire::MatchState::Paused;
    h.poll();
    h.obs.matchState = wire::MatchState::Running;
    h.poll();
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchControl>() >= 2; }));
    const auto controls = a.all<wire::MatchControl>();
    CHECK(controls[0].kind == wire::MatchControlKind::Pause);
    CHECK(controls[1].kind == wire::MatchControlKind::Resume);
}
