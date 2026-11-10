#include <cstdio>
#include <deque>

#include "match_test_support.h"
#include "wpiemu/sdk/match/entity_prediction.h"

using namespace match_test;

namespace {

using EntityMap = std::map<std::uint32_t, wire::EntityState>;

// What a game client does with entity deltas: keep the state of recent snapshots by snapshotSeq (every entity with the sim
// time of the snapshot that upserted it), apply each delta on the state named by baselineId, acknowledge what it applied.
// view() is the picture at the sim time of the newest snapshot: every entity extrapolated with the shared SDK model.
class DeltaClient {
public:
    bool apply(const wire::MatchSnapshot& s) {
        if (seen_ && !wire::seqNewer(s.snapshotSeq, latest_)) return false;   // latest wins
        std::map<std::uint16_t, HeldMap> next;
        for (const wire::EntityDelta& d : s.entityDeltas) {
            HeldMap base;
            if (d.baselineId != wire::kNoBaseline) {
                const auto it = states_.find(d.baselineId);
                if (it == states_.end()) return false;   // the baseline is gone: apply nothing, acknowledge nothing
                const auto g = it->second.find(d.groupId);
                if (g != it->second.end()) base = g->second;
            }
            for (const std::uint32_t id : d.removed) base.erase(id);
            for (const wire::EntityState& e : d.upserted) base[e.id] = {e, s.simTimeUs};
            next[d.groupId] = std::move(base);
        }
        states_[s.snapshotSeq] = std::move(next);
        order_.push_back(s.snapshotSeq);
        while (order_.size() > 32) {
            states_.erase(order_.front());
            order_.pop_front();
        }
        latest_ = s.snapshotSeq;
        latestTimeUs_ = s.simTimeUs;
        seen_ = true;
        return true;
    }
    EntityMap view(std::uint16_t group) const {
        EntityMap out;
        const auto it = states_.find(latest_);
        if (it == states_.end()) return out;
        const auto g = it->second.find(group);
        if (g == it->second.end()) return out;
        for (const auto& [id, held] : g->second) {
            wire::EntityState shown = wire::predictEntity(held.state, latestTimeUs_ - held.stampUs);
            shown.vxCms = shown.vyCms = shown.vzCms = 0;   // the picture is a position
            shown.ballistic = false;
            out[id] = shown;
        }
        return out;
    }
    std::uint32_t latest() const { return latest_; }

private:
    struct Held {
        wire::EntityState state;
        std::int64_t stampUs = 0;
    };
    using HeldMap = std::map<std::uint32_t, Held>;
    std::map<std::uint32_t, std::map<std::uint16_t, HeldMap>> states_;
    std::deque<std::uint32_t> order_;
    std::uint32_t latest_ = 0;
    std::int64_t latestTimeUs_ = 0;
    bool seen_ = false;
};

// Every entity of `truth` is shown within `cm` of it (a rest-state upsert is exact, so this is the rest tolerance).
bool sameEntities(const EntityMap& shown, const EntityMap& truth, int cm) {
    if (shown.size() != truth.size()) return false;
    for (const auto& [id, e] : truth) {
        const auto it = shown.find(id);
        if (it == shown.end()) return false;
        const int dx = it->second.xCm - e.xCm, dy = it->second.yCm - e.yCm, dz = it->second.zCm - e.zCm;
        if (dx * dx + dy * dy + dz * dz > cm * cm) return false;
    }
    return true;
}

// A session with `count` fuel pieces (stable ids 1..count) that a test moves around.
struct FuelSession : FakeSession {
    explicit FuelSession(std::size_t count) {
        EntityGroupSample g;
        g.group = "fuel";
        for (std::size_t i = 0; i < count; ++i) {
            g.ids.push_back(static_cast<std::uint32_t>(i + 1));
            g.positions.push_back({static_cast<float>(i % 40) * 0.2f, static_cast<float>(i / 40) * 0.2f, 0.1f});
        }
        groups.push_back(std::move(g));
        hostBody.pose.position = {2.f, 4.f, 0.2f};
    }
    EntityGroupSample& fuel() { return groups[0]; }
    void move(std::size_t index, float dx) { fuel().positions[index].x += dx; }
    std::map<std::uint32_t, wire::EntityState> truth() const {
        std::map<std::uint32_t, wire::EntityState> out;
        const EntityGroupSample& g = groups[0];
        for (std::size_t i = 0; i < g.ids.size(); ++i)
            out[g.ids[i]] = {g.ids[i], wire::quantizeCentiI16(g.positions[i].x), wire::quantizeCentiI16(g.positions[i].y),
                             wire::quantizeCentiI16(g.positions[i].z)};
        return out;
    }
};

// Plays ticks, delivers snapshots to `dc`, sends acknowledgements when `ack` holds.
struct DeltaRun {
    FuelSession& s;
    Harness& h;
    TestClient& c;
    DeltaClient dc;
    std::size_t consumed = 0;
    std::vector<wire::MatchSnapshot> received;

    void tick(bool ack = true) {
        s.advance();
        h.clock.advance(5ms);
        h.tickEnd();
        REQUIRE(h.host->waitOutboundIdle(3s));
        c.drain();
        while (consumed < c.log.size()) {
            const proto::Message& m = c.log[consumed++];
            const auto* snap = std::get_if<wire::MatchSnapshot>(&m);
            if (snap == nullptr) continue;
            received.push_back(*snap);
            if (dc.apply(*snap) && ack) {
                wire::MatchSnapshotAck a;
                a.sessionId = snap->sessionId;
                a.runEpoch = snap->runEpoch;
                a.snapshotSeq = snap->snapshotSeq;
                REQUIRE(c.send(a).ok());
            }
        }
        h.poll();
    }
};

std::uint64_t fnv(std::uint64_t h, std::span<const std::byte> bytes) {
    for (const std::byte b : bytes) h = (h ^ std::to_integer<std::uint64_t>(b)) * 1099511628211ull;
    return h;
}

}  // namespace

TEST_CASE("match host snapshots: robots are quantized and listed host first") {
    FakeSession s;
    s.hostBody.pose.position = {1.234f, 2.5f, 0.1f};
    s.hostBody.pose.rotation = Quat::fromYaw(0.5f);
    s.hostBody.linearVelocity = {1.5f, -0.25f, 0.f};
    s.hostBody.angularVelocity = {0.f, 0.f, 2.f};
    WheelSample w;
    w.steerAngleRad = 0.25;
    w.spinAngleRad = 1.0;
    s.hostWheels = {w, w};
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    s.advance(5);
    h.tickEnd();
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchSnapshot>() > 0; }));
    const wire::MatchSnapshot snap = *a.last<wire::MatchSnapshot>();
    REQUIRE(snap.robots.size() == 2);
    const wire::RobotWireState& host = snap.robots[0];
    CHECK(host.participantId == wire::kHostParticipantId);
    CHECK(host.xMm == 1234);
    CHECK(host.yMm == 2500);
    CHECK(host.yawQ == wire::quantizeAngle(0.5));
    CHECK(host.vxMmps == 1500);
    CHECK(host.vyMmps == -250);
    CHECK(host.omegaMradps == 2000);
    REQUIRE(host.wheels.size() == 2);
    CHECK(host.wheels[0].steerAngleQ == wire::quantizeAngle(0.25));
    CHECK(host.wheels[0].spinAngleQ == wire::quantizeAngle(1.0));
    CHECK(snap.robots[1].participantId == 1);
    CHECK(snap.robots[1].wheels.size() == 4);
    CHECK(snap.tick == 5);
    CHECK(snap.simTimeUs == 25000);
    CHECK(snap.snapshotSeq == 1);
}

TEST_CASE("match host snapshots: HUD values go through the key table, text is not carried") {
    FakeSession s;
    HudWidgetDescriptor meter;
    meter.id = "fake.meter";
    meter.valueKey = "fake.fuel";
    meter.maxKey = "fake.cap";
    HudWidgetDescriptor board;
    board.id = "fake.board";
    board.kind = HudWidgetKind::Scoreboard;
    board.sides = {{"RED", "fake.red", "fake.red_status", ""}, {"BLUE", "fake.blue", "", ""}};
    s.descriptors = {meter, board};
    s.hud = {{"fake.fuel", Value(std::int64_t{7})}, {"fake.cap", Value(50.0)}, {"fake.red", Value(true)},
             {"fake.red_status", Value(std::string("Active"))}, {"fake.unlisted", Value(1.0)}};
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    s.advance(5);
    h.tickEnd();
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchSnapshot>() > 0; }));
    const HudKeyTable table = HudKeyTable::fromDescriptors(s.descriptors);
    const wire::MatchSnapshot snap = *a.last<wire::MatchSnapshot>();
    std::map<std::string, float> got;
    for (const wire::HudValue& v : snap.gameHud) got[std::string(*table.keyOf(v.keyId))] = v.value;
    CHECK(got == std::map<std::string, float>{{"fake.cap", 50.f}, {"fake.fuel", 7.f}, {"fake.red", 1.f}});
    CHECK(h.host->stats().hudKeysUnknown == 1);   // "fake.unlisted"
}

TEST_CASE("match host snapshots: the pacing is on the sim-time lattice and follows the link quality") {
    FakeSession s;
    MatchHostConfig cfg;
    Harness h(s, cfg);
    ScriptedClient sc(h);
    TestClient& a = *sc.client;
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    sc.transport->hasStats = true;
    sc.transport->rttMs = 10;

    const auto snapshotsOver = [&](int ticks) {
        const std::uint64_t before = h.host->stats().snapshotsSent;
        for (int i = 0; i < ticks; ++i) {
            s.advance();
            h.clock.advance(5ms);
            h.tickEnd();
        }
        return h.host->stats().snapshotsSent - before;
    };
    const auto second = [&] { return snapshotsOver(200); };

    SUBCASE("a LAN link gets 60 Hz") {
        CHECK(second() >= 59);
        CHECK(second() <= 61);
        CHECK(h.host->participants()[0].snapshotRateHz == 60);
    }
    SUBCASE("a bad link falls to 30 Hz after two samples and returns after six good ones") {
        second();
        sc.transport->rttMs = 150;
        sc.transport->lossPermille = 60;
        second();   // two samples at 500 ms
        second();
        CHECK(h.host->participants()[0].snapshotRateHz == 30);
        const std::uint64_t wan = second();
        CHECK(wan >= 29);
        CHECK(wan <= 31);

        sc.transport->rttMs = 20;
        sc.transport->lossPermille = 0;
        second();
        second();
        CHECK(h.host->participants()[0].snapshotRateHz == 30);   // hysteresis: six good samples (3 s) are needed
        second();
        second();
        CHECK(h.host->participants()[0].snapshotRateHz == 60);
        CHECK(second() >= 59);
    }
    SUBCASE("a link in the dead band does not flap") {
        sc.transport->rttMs = 50;   // between the LAN and WAN thresholds
        for (int i = 0; i < 6; ++i) second();
        CHECK(h.host->participants()[0].snapshotRateHz == 60);
    }
    SUBCASE("a transport that cannot measure its link keeps the LAN rate") {
        sc.transport->hasStats = false;
        for (int i = 0; i < 4; ++i) second();
        CHECK(h.host->participants()[0].snapshotRateHz == 60);
    }
}

TEST_CASE("match host snapshots: snapshots are unreliable, setup and lifecycle are reliable") {
    FakeSession s;
    Harness h(s);
    ScriptedClient sc(h);
    TestClient& a = *sc.client;
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    h.flush();
    const int reliable = sc.transport->sends.load() - sc.transport->unreliableSends.load();
    CHECK(reliable >= 1);   // JoinAccept, Roster
    CHECK(sc.transport->unreliableSends.load() == 0);
    for (int i = 0; i < 10; ++i) {
        s.advance();
        h.clock.advance(5ms);
        h.tickEnd();
    }
    h.flush();
    CHECK(sc.transport->unreliableSends.load() >= 2);
}

TEST_CASE("match host snapshots: no snapshots while ticking is on pause, idle snapshots keep clients informed") {
    FakeSession s;
    Harness h(s);
    TestClient a(h);
    REQUIRE(a.join(a.joinMessage("A", swerveSpec(), s)));
    CHECK(h.host->stats().snapshotsSent == 0);   // Running: onPoll never sends snapshots, only ticks do

    h.obs.matchState = wire::MatchState::Paused;
    h.poll();
    const std::uint64_t first = h.host->stats().snapshotsSent;
    CHECK(first >= 1);
    h.poll();
    CHECK(h.host->stats().snapshotsSent == first);   // 100 ms of wall time have not passed
    h.clock.advance(120ms);
    h.poll();
    CHECK(h.host->stats().snapshotsSent == first + 1);
    REQUIRE(a.waitFor([&] { return a.count<wire::MatchSnapshot>() >= 2; }));
    CHECK(a.last<wire::MatchSnapshot>()->info.state == wire::MatchState::Paused);
}

TEST_CASE("match host deltas: full state first, then only what changed since the acknowledged snapshot") {
    FuelSession s(60);
    Harness h(s);
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("A", swerveSpec(), s)));
    DeltaRun run{s, h, c, {}, 0, {}};

    run.tick();
    REQUIRE(run.received.size() == 1);
    REQUIRE(run.received[0].entityDeltas.size() == 1);
    const wire::EntityDelta& full = run.received[0].entityDeltas[0];
    CHECK(full.baselineId == wire::kNoBaseline);
    CHECK(full.upserted.size() == 60);
    CHECK(full.removed.empty());
    CHECK(run.dc.view(0) == s.truth());

    REQUIRE(h.pumpUntil([&] { return h.host->stats().acks == 1; }));
    CHECK(h.host->participants()[0].baselineId == 1);

    s.move(3, 0.5f);
    s.move(17, -0.3f);
    s.fuel().ids.erase(s.fuel().ids.begin() + 40);       // id 41 leaves the field
    s.fuel().positions.erase(s.fuel().positions.begin() + 40);
    while (run.received.size() < 2) run.tick(false);   // no acknowledgement: the next delta is still against snapshot 1
    const wire::MatchSnapshot& delta = run.received[1];
    REQUIRE(delta.entityDeltas.size() == 1);
    CHECK(delta.entityDeltas[0].baselineId == 1);
    CHECK(delta.entityDeltas[0].upserted.size() == 2);
    CHECK(delta.entityDeltas[0].removed == std::vector<std::uint32_t>{41});
    CHECK(run.dc.view(0) == s.truth());
}

TEST_CASE("match host deltas: an ack older than the history falls back to full state") {
    FuelSession s(40);
    Harness h(s);
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("A", swerveSpec(), s)));
    DeltaRun run{s, h, c, {}, 0, {}};
    run.tick();   // snapshot 1, acknowledged
    REQUIRE(h.pumpUntil([&] { return h.host->stats().acks == 1; }));
    while (run.received.size() < 40) run.tick(false);   // nothing acknowledged for 39 snapshots: snapshot 1 left the history
    const wire::MatchSnapshot& last = run.received.back();
    REQUIRE(last.entityDeltas.size() == 1);
    CHECK(last.entityDeltas[0].baselineId == wire::kNoBaseline);
    CHECK(last.entityDeltas[0].upserted.size() == 40);
    CHECK(h.host->participants()[0].baselineId == wire::kNoBaseline);

    wire::MatchSnapshotAck stale;
    stale.sessionId = last.sessionId;
    stale.runEpoch = last.runEpoch;
    stale.snapshotSeq = 1;   // far too old
    REQUIRE(c.send(stale).ok());
    REQUIRE(h.pumpUntil([&] { return h.host->stats().acksIgnored >= 1; }));
    CHECK(h.host->participants()[0].baselineId == wire::kNoBaseline);
}

TEST_CASE("match host deltas: a client converges on the truth under 30 percent loss in both directions") {
    FuelSession s(120);
    Harness h(s);
    testing::ImpairmentProfile lossy;
    lossy.lossProbability = 0.3;
    lossy.latency = 0us;
    TestClient c(h, lossy, 99);
    REQUIRE(c.join(c.joinMessage("A", swerveSpec(), s)));
    DeltaRun run{s, h, c, {}, 0, {}};
    std::uint32_t lcg = 12345;
    const auto jiggle = [&] {
        for (int k = 0; k < 20; ++k) {
            lcg = lcg * 1664525u + 1013904223u;
            s.move(lcg % 120, static_cast<float>((lcg >> 8) % 7) * 0.02f - 0.06f);
        }
    };
    for (int i = 0; i < 400; ++i) {
        if (i % 3 == 0) jiggle();
        run.tick();
    }
    for (int i = 0; i < 400; ++i) run.tick();   // the fuel rests: the client must settle on the truth
    CHECK(sameEntities(run.dc.view(0), s.truth(), 1));   // within the 1 cm an entity at rest is held to
    CHECK(run.received.size() > 120);   // 240 sent at 60 Hz, 70 percent arrive
    CHECK(h.host->stats().acksIgnored + h.host->stats().acks > 50);
}

TEST_CASE("match host deltas: robots and info always fit the budget, entity changes are trimmed and spread over later snapshots") {
    FuelSession s(800);
    Harness h(s);
    std::vector<std::unique_ptr<TestClient>> clients;
    std::vector<std::unique_ptr<DeltaRun>> runs;
    for (int i = 0; i < 5; ++i) {
        clients.push_back(std::make_unique<TestClient>(h));
        REQUIRE(clients.back()->join(clients.back()->joinMessage("P", swerveSpec(), s)));
    }
    for (int i = 0; i < 5; ++i) runs.push_back(std::make_unique<DeltaRun>(DeltaRun{s, h, *clients[static_cast<std::size_t>(i)], {}, 0, {}}));
    s.hud = {};
    std::uint32_t lcg = 777;
    std::size_t largest = 0, snapshots = 0;
    for (int tick = 0; tick < 240; ++tick) {
        for (std::size_t i = 0; i < 800; ++i) {   // every fuel piece moves every tick
            lcg = lcg * 1664525u + 1013904223u;
            s.move(i, static_cast<float>((lcg >> 16) % 9) * 0.02f - 0.08f);
        }
        s.advance();
        h.clock.advance(5ms);
        h.tickEnd();
        REQUIRE(h.host->waitOutboundIdle(3s));
        for (std::size_t i = 0; i < clients.size(); ++i) {
            DeltaRun& run = *runs[i];
            clients[i]->drain();
            while (run.consumed < clients[i]->log.size()) {
                const proto::Message m = clients[i]->log[run.consumed++];
                const auto* snap = std::get_if<wire::MatchSnapshot>(&m);
                if (snap == nullptr) continue;
                ++snapshots;
                CHECK(snap->robots.size() == 6);          // host + five remote robots are never trimmed
                CHECK(snap->info.state == wire::MatchState::Running);
                std::vector<std::byte> bytes;
                REQUIRE(h.codec->encode(*snap, bytes).ok());
                CHECK(bytes.size() <= wire::kSnapshotBudgetBytes);
                largest = std::max(largest, bytes.size());
                if (run.dc.apply(*snap)) {
                    wire::MatchSnapshotAck ack;
                    ack.sessionId = snap->sessionId;
                    ack.runEpoch = snap->runEpoch;
                    ack.snapshotSeq = snap->snapshotSeq;
                    REQUIRE(clients[i]->send(ack).ok());
                }
            }
        }
        h.poll();
    }
    CHECK(snapshots >= 5 * 60);
    CHECK(h.host->stats().snapshotsTruncated > 0);
    CHECK(h.host->stats().entityChangesDeferred > 0);
    CHECK(h.host->stats().snapshotsOverBudget == 0);
    CHECK(h.host->stats().maxSnapshotBytes <= wire::kSnapshotBudgetBytes);
    CHECK(largest <= wire::kSnapshotBudgetBytes);
    CHECK(largest > wire::kSnapshotBudgetBytes * 3 / 4);   // the budget is actually used
    std::printf("[match host] 6 robots + 800 moving fuel: largest snapshot %zu bytes of %zu, truncated %llu of %llu snapshots\n", largest,
                wire::kSnapshotBudgetBytes, static_cast<unsigned long long>(h.host->stats().snapshotsTruncated),
                static_cast<unsigned long long>(h.host->stats().snapshotsSent));

    // The fuel rests: every client catches up completely, a few entities per snapshot.
    for (int i = 0; i < 400; ++i)
        for (DeltaRun* run : {runs[0].get(), runs[3].get()}) run->tick();
    CHECK(sameEntities(runs[0]->dc.view(0), s.truth(), 1));
}

TEST_CASE("match host snapshots: the worst case without entities fits the budget") {
    FakeSession s;
    s.descriptors.clear();
    std::vector<std::string> keys;
    for (int i = 0; i < 40; ++i) keys.push_back("fake.key" + std::to_string(i));
    for (const std::string& k : keys) {
        HudWidgetDescriptor d;
        d.id = k;
        d.valueKey = k;
        s.descriptors.push_back(d);
        s.hud.push_back({k, Value(1234.5678)});
    }
    RobotConfig big = swerveSpec();
    for (int i = 0; i < 4; ++i) big.drivetrain.spec.wheels.push_back(makeWheel(("extra" + std::to_string(i)).c_str(), 0.1f * static_cast<float>(i), 0.f));
    REQUIRE(big.drivetrain.spec.wheels.size() == 8);
    s.hostWheels.assign(8, WheelSample{});
    for (int i = 0; i < 8; ++i) s.robotHud.push_back({keys[static_cast<std::size_t>(i)], Value(1234.5678)});
    Harness h(s);
    std::vector<std::unique_ptr<TestClient>> clients;
    for (std::size_t i = 0; i < rt::kMaxRemoteRobots; ++i) {
        clients.push_back(std::make_unique<TestClient>(h));
        REQUIRE(clients.back()->join(clients.back()->joinMessage("P", big, s)));
    }
    s.hostBody.pose.position = {-300.f, 300.f, 0.f};   // far positions use the long encodings
    for (auto& [id, r] : s.robots) r.pose.position = {200.f * static_cast<float>(id), -250.f, 0.f};
    s.advance(100000);
    h.tickEnd();
    REQUIRE(clients[0]->waitFor([&] { return clients[0]->count<wire::MatchSnapshot>() > 0; }));
    const wire::MatchSnapshot snap = *clients[0]->last<wire::MatchSnapshot>();
    CHECK(snap.robots.size() == 6);
    CHECK(snap.gameHud.size() == wire::kMaxGameHudValues);
    for (const wire::RobotWireState& r : snap.robots) CHECK(r.wheels.size() == 8);
    CHECK(snap.robots[1].hud.size() == wire::kMaxHudValuesPerRobot);
    std::vector<std::byte> bytes;
    REQUIRE(h.codec->encode(snap, bytes).ok());
    CHECK(bytes.size() <= wire::kSnapshotBudgetBytes);
    CHECK(h.host->stats().snapshotsOverBudget == 0);
}

TEST_CASE("match host snapshots: the encoded snapshots of the same run are identical") {
    const auto run = [](std::uint64_t seed) {
        FuelSession s(300);
        Harness h(s, {}, 5);
        TestClient c(h);
        REQUIRE(c.join(c.joinMessage("A", swerveSpec(), s)));
        DeltaRun dr{s, h, c, {}, 0, {}};
        std::uint32_t lcg = static_cast<std::uint32_t>(seed);
        std::uint64_t hash = 1469598103934665603ull;
        for (int tick = 0; tick < 150; ++tick) {
            for (std::size_t i = 0; i < 300; ++i) {
                lcg = lcg * 1664525u + 1013904223u;
                if ((lcg >> 24) % 3 == 0) s.move(i, static_cast<float>((lcg >> 16) % 5) * 0.03f);
            }
            dr.tick();
            REQUIRE(h.pumpUntil([&] { return h.host->stats().acks + h.host->stats().acksIgnored >= dr.received.size(); }));
        }
        for (const wire::MatchSnapshot& snap : dr.received) {
            std::vector<std::byte> bytes;
            REQUIRE(h.codec->encode(snap, bytes).ok());
            hash = fnv(hash, bytes);
        }
        return std::pair{hash, dr.received.size()};
    };
    const auto first = run(42);
    const auto second = run(42);
    CHECK(first.second > 20);
    CHECK(first == second);
    CHECK(run(43).first != first.first);
}
