// Entity stress scenario of a full practice match: the host robot and five remote robots all fire the 2026 fuel shooter at
// 20 balls/s (one shooter zone each, one ball per volley, 8 m/s) through the REAL plugin, SimulationSession and MatchHost over
// impaired in-memory links, with a model client per remote robot. It answers whether the packet budget (kSnapshotBudgetBytes
// per snapshot, one snapshot per 1/60 s on a LAN and 1/30 s on a WAN) keeps the fuel the clients hold fresh, and what the host
// costs the relay. The numbers are the baseline of entity prediction.
//
//   default run   a short variant of both network profiles (always runs, a few seconds)
//   WPIEMU_STRESS=1         the full 20 s of fire, both aims, both profiles
//   WPIEMU_STRESS_REPORT=1  prints the measured table on stdout (the numbers are in the doctest output either way)
//
// The model client counts the AGE of an entity update as the sim time between two updates of the same entity that moved in
// between (the staleness a client would extrapolate or interpolate over).
#include <cinttypes>
#include <sstream>

#include "../runtime/remote_robot_rig.h"
#include "match_stress_support.h"

using namespace match_stress;
namespace g26 = wpiemu::plugins::game_2026;
using wpiemu::sdk::match::kSnapshotBudgetBytes;

namespace {

constexpr double kStepS = 0.005;
constexpr std::size_t kRobots = 6;           // the host and five remote robots
constexpr std::size_t kRemotes = kRobots - 1;
constexpr double kFireRateHz = 20.0;
constexpr double kExitSpeedMps = 8.0;
constexpr int kFuelCap = 800;
constexpr int kMeterFuel = 500;              // meter capacity and preload: 25 s of fire per robot
// The relay's token buckets (relay/room_table.h): the host's rate is maxPeersPerRoom times the per-peer rate, the room's twice that.
constexpr double kRelayPeerBytesPerS = wpiemu::sdk::match::relay::kRelayPeerBytesPerSecond;
constexpr double kRelayHostBytesPerS = kRelayPeerBytesPerS * static_cast<double>(wpiemu::sdk::match::relay::kRelayMaxPeersPerRoom);
constexpr double kRelayRoomBytesPerS = 2.0 * kRelayHostBytesPerS;
// What the relay path adds to a game frame: the 2-byte peer envelope and the 24-byte SecureTransport overhead.
constexpr double kRelayFrameOverheadBytes = wpiemu::sdk::match::relay::kRelayEnvelopeBytes + 24;

struct NetProfile {
    std::string name;
    wpiemu::sdk::testing::ImpairmentProfile impairment;
    std::uint32_t rttMs = 0;          // what the transport reports as its link stats (drives the snapshot rate)
    std::uint32_t lossPermille = 0;
    std::uint64_t leadTicks = 6;      // how far ahead of the newest host tick a client schedules its inputs
};

NetProfile lanProfile() {
    NetProfile p;
    p.name = "LAN 60 Hz";
    p.impairment.latency = std::chrono::microseconds(1000);
    p.impairment.jitter = std::chrono::microseconds(500);
    p.rttMs = 2;
    p.leadTicks = 6;
    return p;
}

NetProfile wanProfile() {
    NetProfile p;
    p.name = "WAN 30 Hz, 60 ms RTT, 3 % loss";
    p.impairment.latency = std::chrono::microseconds(27000);
    p.impairment.jitter = std::chrono::microseconds(6000);
    p.impairment.lossProbability = 0.03;
    p.rttMs = 60;
    p.lossPermille = 30;
    p.leadTicks = 22;
    return p;
}

// Where the robots shoot.
//   Exchange   three pairs of robots face each other across the neutral zone and shoot at each other's intake: the balls are
//              taken back in, so the fire is sustained and about a hundred balls are in the air at any time.
//   OffField   outward over the side lines: the fuel leaves the field and is lost, so the fire is sustained but short-lived.
//   IntoField  from both sides toward the middle: the fuel piles up and the 800 body cap is reached (a burst, then quiet).
enum class Aim { Exchange, OffField, IntoField };

const char* aimName(Aim aim) {
    switch (aim) {
        case Aim::Exchange: return "robots exchange fire";
        case Aim::OffField: return "off-field fire";
        case Aim::IntoField: return "fire into the field";
    }
    return "";
}

struct Scenario {
    NetProfile net;
    Aim aim = Aim::Exchange;
    double warmupS = 1.5;
    double measureS = 2.0;
    double settleS = 40.0;   // the longest wait, after the remote shooters stop, for the clients to converge on everything at rest
};

struct Placement {
    float x;
    float y;
    float yaw;
};

constexpr float kPi = 3.14159265f;

std::array<Placement, kRobots> placements(Aim aim) {
    // Index 0 is the host robot; 1..5 the remote robots in join order.
    if (aim == Aim::Exchange) {
        // Pairs 5 m apart (muzzle to intake 4.3 m), between the bumps and clear of the staged fuel; the lanes of a pair are 25 cm apart.
        return {{{5.9f, 1.0f, 0.f}, {10.9f, 1.25f, kPi}, {5.9f, 2.6f, 0.f}, {10.9f, 2.85f, kPi}, {5.9f, 6.4f, 0.f}, {10.9f, 6.65f, kPi}}};
    }
    const float south = -kPi / 2;
    const float north = kPi / 2;
    const float nearYaw = aim == Aim::OffField ? south : north;
    const float farYaw = aim == Aim::OffField ? north : south;
    return {{{6.0f, 0.9f, nearYaw}, {8.0f, 0.9f, nearYaw}, {10.5f, 0.9f, nearYaw},
             {6.0f, 7.2f, farYaw}, {8.0f, 7.2f, farYaw}, {10.5f, 7.2f, farYaw}}};
}

double hoodOf(Aim aim) { return aim == Aim::Exchange ? 1.222 : 0.7; }   // 1.222 rad: a 1.5 s lob that lands an 8 m/s shot 4.3 m away at intake height

// bit 0 shooter, bit 1 pickup (Exchange only)
std::uint32_t fireFlags(Aim aim) { return aim == Aim::Exchange ? 0b11u : 0b01u; }

wpiemu::sdk::ZoneConfig shooterZone(Aim aim) {
    wpiemu::sdk::ZoneConfig z;
    z.id = "shooter";
    z.kind = g26::kShooterKind;
    z.local.position = {0.25f, 0.f, 0.55f};
    z.properties = wpiemu::sdk::Value::makeObject();
    z.properties["fireRateHz"] = kFireRateHz;
    z.properties["parallelBalls"] = 1;
    z.properties["fixedExitSpeedMps"] = kExitSpeedMps;
    z.properties["hoodAngleRad"] = hoodOf(aim);
    return z;
}

wpiemu::sdk::ZoneConfig pickupZone() {
    wpiemu::sdk::ZoneConfig z;
    z.id = "intake";
    z.kind = g26::kPickupKind;
    z.local.position = {0.45f, 0.f, 0.2f};
    z.sizeM = {0.50f, 0.80f, 0.50f};
    z.properties = wpiemu::sdk::Value::makeObject();
    z.properties["intakeRateHz"] = 0.0;
    return z;
}

std::vector<wpiemu::sdk::ZoneConfig> zonesOf(Aim aim) {
    std::vector<wpiemu::sdk::ZoneConfig> zones{shooterZone(aim)};
    if (aim == Aim::Exchange) zones.push_back(pickupZone());
    return zones;
}

wpiemu::sdk::RobotConfig shooterRobot(const char* name, Aim aim) {
    wpiemu::sdk::RobotConfig spec = remote_test::swerveRobot(name);
    spec.gamePluginId = std::string(g26::pluginId());
    spec.zones = zonesOf(aim);
    return spec;
}
struct Result {
    std::string name;
    double measureS = 0;
    double snapshotHz = 0;                     // received per client
    std::uint16_t hostRateHz = 0;
    Stat moving, fuel, upserts, removals, bytes, ageMs, ageSnapshots, firstUpdateMs, behind;
    Stat simTickMs, hostTickEndMs;
    std::size_t starvedIds = 0;
    std::uint64_t starvedEvents = 0;
    double maxStaleMs = 0;
    std::size_t maxStuck = 0;
    std::size_t finalStuck = 0;
    std::size_t finalBehind = 0;
    double convergedAfterS = -1;               // seconds after the remote shooters stopped until all rested fuel was right; -1 = not within the limit
    double hostBytesPerS = 0;                  // application bytes toward all clients
    double hostRelayBytesPerS = 0;             // plus the relay envelope and encryption overhead per frame
    double clientsToHostBytesPerS = 0;
    double maxClientToHostRelayBytesPerS = 0;
    double roomRelayBytesPerS = 0;
    core::match::MatchHostStats host;
    ClientMetrics totals;                      // counters summed over the clients
    std::uint64_t shots = 0;
    bool pluginIds = false;
    std::size_t idMismatches = 0;
    std::size_t firstSnapshotsIncomplete = 0;
};

using namespace wpiemu;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

double msBetween(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); }

void append(std::vector<double>& to, const std::vector<double>& from) { to.insert(to.end(), from.begin(), from.end()); }

Result runScenario(const Scenario& sc) {
    g26session::Options options;
    options.pluginInts = {{"capacity", kMeterFuel}, {"preloadFuel", kMeterFuel}, {"maxFuelOnField", kFuelCap}};
    options.hostZones = zonesOf(sc.aim);
    g26session::Rig rig(options);
    TruthTap tap(*rig.session);
    mt::Harness h(tap);
    REQUIRE(g26::registerPlugin(h.kit.plugins).ok());

    std::vector<std::unique_ptr<ModelClient>> clients;
    for (std::size_t i = 0; i < kRemotes; ++i) {
        clients.push_back(std::make_unique<ModelClient>(h, sc.net.impairment, 100 + i, tap));
        mt::ScriptedTransport& end = clients.back()->hostEnd();
        end.hasStats = true;
        end.rttMs = sc.net.rttMs;
        end.lossPermille = sc.net.lossPermille;
        wire::Join join;
        join.displayName = "Shooter" + std::to_string(i + 1);
        join.gamePluginId = tap.gamePluginId();
        join.wpilibYear = tap.wpilibYear();
        join.robotSpecJson = mt::specJson(shooterRobot("Shooter", sc.aim));
        REQUIRE(clients.back()->client().send(join).ok());
    }

    // The sim stands still while everyone joins; the manual clock moves so the impaired links deliver.
    const auto joined = [&] {
        for (const auto& c : clients) {
            if (c->client().count<wire::JoinReject>() > 0) {
                const auto reject = c->client().last<wire::JoinReject>();
                FAIL("join rejected: " << reject->detail);
            }
            if (c->client().count<wire::JoinAccept>() == 0) return false;
        }
        return true;
    };
    for (int i = 0; i < 4000 && !joined(); ++i) {
        h.clock.advance(5ms);
        h.poll();
        for (const auto& c : clients) c->client().drain();
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE(joined());
    const std::array<Placement, kRobots> where = placements(sc.aim);
    rig.teleport(wire::kHostParticipantId, {where[0].x, where[0].y, 0.04f}, where[0].yaw);
    for (std::size_t i = 0; i < kRemotes; ++i) {
        mt::TestClient& c = clients[i]->client();
        c.accept = c.last<wire::JoinAccept>();
        c.log.clear();
        clients[i]->seedClock(c.accept->serverTick);
        rig.teleport(c.accept->participantId, {where[i + 1].x, where[i + 1].y, 0.04f}, where[i + 1].yaw);
    }

    // ---- the run ----
    std::vector<double> simTickMs, hostTickEndMs;
    std::uint64_t tickNo = 0;
    const auto step = [&](bool firing, bool measuring) {
        h.clock.advance(5ms);
        if (tickNo % 4 == 0)
            for (const auto& c : clients) c->sendInput(sc.net.leadTicks, firing ? fireFlags(sc.aim) : 0u, 4);
        h.poll();
        const Clock::time_point t0 = Clock::now();
        rig.session->tick(rig.frame, false);
        const Clock::time_point t1 = Clock::now();
        h.tickEnd();
        const Clock::time_point t2 = Clock::now();
        REQUIRE(h.host->waitOutboundIdle(3s));
        const double tapMs = std::chrono::duration<double, std::milli>(tap.takeTapTime()).count();
        for (const auto& c : clients) c->pump(measuring, kRobots);
        if (measuring) {
            simTickMs.push_back(msBetween(t0, t1));
            hostTickEndMs.push_back(std::max(0.0, msBetween(t1, t2) - tapMs));
        }
        ++tickNo;
    };
    const auto ticksOf = [](double seconds) { return static_cast<int>(std::lround(seconds / kStepS)); };

    for (int i = 0; i < ticksOf(sc.warmupS); ++i) step(true, false);

    struct Counters {
        std::uint64_t hostBytes = 0, hostFrames = 0, upBytes = 0, upFrames = 0;
        std::vector<std::uint64_t> clientUpBytes, clientUpFrames;
    };
    const auto counters = [&] {
        Counters c;
        for (const auto& cl : clients) {
            c.hostBytes += cl->hostEnd().bytesSent.load();
            c.hostFrames += static_cast<std::uint64_t>(cl->hostEnd().sends.load());
            c.upBytes += cl->hostEnd().bytesReceived.load();
            c.upFrames += cl->hostEnd().framesReceived.load();
            c.clientUpBytes.push_back(cl->hostEnd().bytesReceived.load());
            c.clientUpFrames.push_back(cl->hostEnd().framesReceived.load());
        }
        return c;
    };
    const Counters before = counters();
    for (auto& c : clients) (void)c->takeMetrics();
    for (int i = 0; i < ticksOf(sc.measureS); ++i) step(true, true);
    const Counters after = counters();
    std::vector<ClientMetrics> perClient;
    for (auto& c : clients) perClient.push_back(c->takeMetrics());

    // The remote shooters stop (the host's keeps firing); settled when nothing that rests is wrong on any client for 1 s.
    double convergedAfterS = -1;
    int calm = 0;
    for (int i = 0; i < ticksOf(sc.settleS); ++i) {
        step(false, false);
        bool stuck = false;
        for (const auto& c : clients) stuck = stuck || c->stuckNow() > 0;
        calm = stuck ? 0 : calm + 1;
        if (calm >= ticksOf(1.0)) {
            convergedAfterS = static_cast<double>(i + 1 - calm) * kStepS;
            break;
        }
    }

    // ---- results ----
    Result r;
    r.name = sc.net.name + (std::string(", ") + aimName(sc.aim));
    r.measureS = sc.measureS;
    r.convergedAfterS = convergedAfterS;
    std::vector<double> moving, fuel, upserts, removals, bytes, ageMs, ageSnaps, firstMs, behind;
    for (const ClientMetrics& m : perClient) {
        append(moving, m.moving);
        append(fuel, m.fuelOnField);
        append(upserts, m.upserts);
        append(removals, m.removals);
        append(bytes, m.entityBytes);
        append(ageMs, m.ageMs);
        append(ageSnaps, m.ageSnapshots);
        append(firstMs, m.firstUpdateMs);
        append(behind, m.behind);
        r.starvedEvents += m.starvedEvents;
        r.maxStaleMs = std::max(r.maxStaleMs, m.maxStaleMs);
        r.maxStuck = std::max(r.maxStuck, m.maxStuck);
        r.totals.received += m.received;
        r.totals.staleDropped += m.staleDropped;
        r.totals.unappliable += m.unappliable;
        r.totals.redundantUpserts += m.redundantUpserts;
        r.totals.fullStates += m.fullStates;
        r.totals.incomplete += m.incomplete;
        r.totals.noTruth += m.noTruth;
    }
    std::set<std::uint32_t> starved;
    for (const ClientMetrics& m : perClient) starved.insert(m.starvedIds.begin(), m.starvedIds.end());
    r.starvedIds = starved.size();
    r.moving = statOf(moving);
    r.fuel = statOf(fuel);
    r.upserts = statOf(upserts);
    r.removals = statOf(removals);
    r.bytes = statOf(bytes);
    r.ageMs = statOf(ageMs);
    r.ageSnapshots = statOf(ageSnaps);
    r.firstUpdateMs = statOf(firstMs);
    r.behind = statOf(behind);
    r.simTickMs = statOf(simTickMs);
    r.hostTickEndMs = statOf(hostTickEndMs);
    r.snapshotHz = static_cast<double>(r.totals.received) / static_cast<double>(kRemotes) / sc.measureS;
    for (const auto& p : h.host->participants()) r.hostRateHz = std::max(r.hostRateHz, p.snapshotRateHz);
    for (const auto& c : clients) {
        r.finalStuck = std::max(r.finalStuck, c->stuckNow());
        r.finalBehind = std::max(r.finalBehind, c->laggingNow());
    }
    const double hostBytes = static_cast<double>(after.hostBytes - before.hostBytes);
    const double hostFrames = static_cast<double>(after.hostFrames - before.hostFrames);
    r.hostBytesPerS = hostBytes / sc.measureS;
    r.hostRelayBytesPerS = (hostBytes + hostFrames * kRelayFrameOverheadBytes) / sc.measureS;
    r.clientsToHostBytesPerS = static_cast<double>(after.upBytes - before.upBytes) / sc.measureS;
    const double upRelay = (static_cast<double>(after.upBytes - before.upBytes) +
                            static_cast<double>(after.upFrames - before.upFrames) * kRelayFrameOverheadBytes) / sc.measureS;
    for (std::size_t i = 0; i < kRemotes; ++i) {
        const double bytesUp = static_cast<double>(after.clientUpBytes[i] - before.clientUpBytes[i]);
        const double framesUp = static_cast<double>(after.clientUpFrames[i] - before.clientUpFrames[i]);
        r.maxClientToHostRelayBytesPerS =
            std::max(r.maxClientToHostRelayBytesPerS, (bytesUp + framesUp * kRelayFrameOverheadBytes) / sc.measureS);
    }
    r.roomRelayBytesPerS = r.hostRelayBytesPerS + upRelay;
    r.host = h.host->stats();
    r.pluginIds = tap.pluginPublishesIds();
    r.idMismatches = tap.idMismatches();
    if (const auto last = tap.lastSnapshot()) {
        if (const sdk::Value* v = last->hudValue(g26::hud::kShots)) r.shots += static_cast<std::uint64_t>(v->asInt());
        for (const sdk::RobotSample& robot : last->remoteRobots())
            for (const sdk::HudValue& hv : robot.hud)
                if (hv.key == g26::hud::kShots) r.shots += static_cast<std::uint64_t>(hv.value.asInt());
    }
    return r;
}

std::string line3(const char* label, const Stat& s, const char* unit, const char* extra = "") {
    char buf[256];
    std::snprintf(buf, sizeof buf, "  %-34s p50 %8.1f  p95 %8.1f  max %8.1f %s %s\n", label, s.p50, s.p95, s.max, unit, extra);
    return buf;
}

std::string format(const Result& r, double expectedShots) {
    std::ostringstream out;
    char buf[320];
    out << "== " << r.name << " (" << r.measureS << " s measured, " << kRobots << " robots, 20 balls/s each) ==\n";
    std::snprintf(buf, sizeof buf, "  snapshots/s per client %.1f (host rate %u Hz), plugin publishes entity ids: %s\n", r.snapshotHz,
                  r.hostRateHz, r.pluginIds ? "YES" : "NO");
    out << buf;
    out << line3("fuel bodies on field", r.fuel, "");
    out << line3("moving entities / snapshot", r.moving, "(truth: changed since previous snapshot)");
    out << line3("upserts / snapshot", r.upserts, "");
    out << line3("removals / snapshot", r.removals, "");
    out << line3("encoded snapshot bytes", r.bytes, "B", "(budget 1200)");
    std::snprintf(buf, sizeof buf, "  truncated snapshots %" PRIu64 " of %" PRIu64 " sent, changes deferred %" PRIu64 ", over budget %" PRIu64 "\n",
                  r.host.snapshotsTruncated, r.host.snapshotsSent, r.host.entityChangesDeferred, r.host.snapshotsOverBudget);
    out << buf;
    out << line3("update age of a moving entity", r.ageMs, "ms");
    out << line3("  the same in snapshots", r.ageSnapshots, "snapshots");
    out << line3("spawn to first update", r.firstUpdateMs, "ms");
    out << line3("entities behind the truth / snapshot", r.behind, "");
    std::snprintf(buf, sizeof buf, "  starved (moving, not updated > 500 ms): %zu distinct entities, %" PRIu64 " (snapshot, entity) events, max staleness %.0f ms,"
                  " most at rest for 1.5 s and still wrong %zu\n",
                  r.starvedIds, r.starvedEvents, r.maxStaleMs, r.maxStuck);
    out << buf;
    if (r.convergedAfterS >= 0)
        std::snprintf(buf, sizeof buf, "  after the remote shooters stopped every client was right about everything at rest within %.1f s\n", r.convergedAfterS);
    else
        std::snprintf(buf, sizeof buf, "  NOT converged after the remote shooters stopped: %zu entities at rest still wrong, %zu behind\n", r.finalStuck, r.finalBehind);
    out << buf;
    std::snprintf(buf, sizeof buf, "  host -> clients %.1f KB/s app, %.1f KB/s with relay framing (host bucket 384 KB/s: %.0f %%)\n",
                  r.hostBytesPerS / 1024.0, r.hostRelayBytesPerS / 1024.0, 100.0 * r.hostRelayBytesPerS / kRelayHostBytesPerS);
    out << buf;
    std::snprintf(buf, sizeof buf, "  clients -> host %.1f KB/s total, %.1f KB/s the busiest client (peer bucket 64 KB/s); room both ways %.1f KB/s (limit 768 KB/s: %.0f %%)\n",
                  r.clientsToHostBytesPerS / 1024.0, r.maxClientToHostRelayBytesPerS / 1024.0, r.roomRelayBytesPerS / 1024.0,
                  100.0 * r.roomRelayBytesPerS / kRelayRoomBytesPerS);
    out << buf;
    out << line3("host sim tick (800 fuel)", r.simTickMs, "ms", "(budget 5 ms)");
    out << line3("host onTickEnd (snapshots)", r.hostTickEndMs, "ms");
    std::snprintf(buf, sizeof buf, "  shots fired %" PRIu64 " (about %.0f expected over the whole run), redundant upserts %" PRIu64 " (resent until acknowledged), snapshots unappliable %" PRIu64 ", stale dropped %" PRIu64 "\n",
                  r.shots, expectedShots, r.totals.redundantUpserts, r.totals.unappliable, r.totals.staleDropped);
    out << buf;
    return out.str();
}

void gate(const Result& r, bool wan, Aim aim) {
    // The budget and what always travels.
    CHECK(r.host.snapshotsOverBudget == 0);
    CHECK(r.host.maxSnapshotBytes <= kSnapshotBudgetBytes);
    CHECK(r.bytes.max <= static_cast<double>(kSnapshotBudgetBytes));
    CHECK(r.totals.received > 20);
    CHECK(r.totals.incomplete == 0);   // every snapshot carries all six robots and a Running info
    CHECK(r.totals.noTruth == 0);
    CHECK(r.pluginIds);
    CHECK(r.idMismatches == 0);
    CHECK(r.totals.unappliable == 0);
    // Nothing starves forever: once the shooters stop, everything at rest ends up right on every client. (During sustained fire
    // the baseline does starve entities for seconds: r.starvedIds and r.maxStaleMs are reported, not gated.)
    CHECK(r.convergedAfterS >= 0.0);
    // The fuel really flew (the scenario measures what it says).
    if (aim == Aim::IntoField) CHECK(r.moving.max > 100.0);   // a burst while the pile grows, then quiet
    else CHECK(r.moving.p50 > 20.0);
    CHECK(r.fuel.max > 400.0);
    CHECK(r.fuel.max <= static_cast<double>(kFuelCap));
    // The relay's buckets hold at the WAN rate.
    if (wan) {
        CHECK(r.hostRelayBytesPerS < kRelayHostBytesPerS);
        CHECK(r.roomRelayBytesPerS < kRelayRoomBytesPerS);
        CHECK(r.maxClientToHostRelayBytesPerS < kRelayPeerBytesPerS);
        CHECK(r.hostRateHz == wpiemu::sdk::match::kSnapshotRateWanHz);
    } else {
        CHECK(r.hostRateHz == wpiemu::sdk::match::kSnapshotRateLanHz);
    }
    // Generous: a loaded CI machine must not fail it, a runaway host would.
    CHECK(r.simTickMs.p95 < 100.0);
    CHECK(r.hostTickEndMs.p95 < 100.0);
}

void runAndReport(const Scenario& sc, bool wan) {
    const Result r = runScenario(sc);
    const double totalS = sc.warmupS + sc.measureS;
    const double expected = kRobots * kFireRateHz * totalS;
    const std::string table = format(r, expected);
    MESSAGE("\n" << table);
    if (reportRequested()) std::fputs(table.c_str(), stdout);
    gate(r, wan, sc.aim);
}

}  // namespace

TEST_CASE("entity stress: six shooters on a LAN (short)") {
    runAndReport({lanProfile(), Aim::Exchange, 1.5, 2.0, 40.0}, false);
}

TEST_CASE("entity stress: six shooters on a WAN (short)") {
    runAndReport({wanProfile(), Aim::Exchange, 2.0, 2.0, 40.0}, true);
}

TEST_CASE("entity stress: 20 s of exchanged fire on a LAN") {
    if (!fullRunRequested()) return;
    runAndReport({lanProfile(), Aim::Exchange, 2.0, 20.0, 40.0}, false);
}

TEST_CASE("entity stress: 20 s of exchanged fire on a WAN") {
    if (!fullRunRequested()) return;
    runAndReport({wanProfile(), Aim::Exchange, 2.0, 20.0, 40.0}, true);
}

TEST_CASE("entity stress: 20 s of off-field fire on a LAN") {
    if (!fullRunRequested()) return;
    runAndReport({lanProfile(), Aim::OffField, 2.0, 20.0, 40.0}, false);
}

TEST_CASE("entity stress: 20 s of off-field fire on a WAN") {
    if (!fullRunRequested()) return;
    runAndReport({wanProfile(), Aim::OffField, 2.0, 20.0, 40.0}, true);
}

TEST_CASE("entity stress: fire into the field up to the 800 body cap on a LAN") {
    if (!fullRunRequested()) return;
    runAndReport({lanProfile(), Aim::IntoField, 1.0, 20.0, 40.0}, false);
}

TEST_CASE("entity stress: fire into the field up to the 800 body cap on a WAN") {
    if (!fullRunRequested()) return;
    runAndReport({wanProfile(), Aim::IntoField, 1.0, 20.0, 40.0}, true);
}