#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <thread>
#include <vector>

#include "../match/match_stress_support.h"
#include "multirobot_session_rig.h"
#include "test_support.h"

// The 2026 plugin publishes one stable entity id per fuel body (EntityGroupSample::ids), which is what lets the match host
// delta-encode the fuel to remote players.

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26session::Options;
using g26session::Rig;
using sdk::proto::AllianceStation;
namespace wire = wpiemu::sdk::match;

namespace {

constexpr float kMidX = dim::kFieldLengthM * 0.5f;
constexpr float kMidY = dim::kFieldWidthM * 0.5f;

const sdk::EntityGroupSample* fuelGroup(const sdk::ISimSnapshot& snap) {
    for (const sdk::EntityGroupSample& g : snap.entityGroups())
        if (g.group == "fuel") return &g;
    return nullptr;
}

// A pickup robot and a shooter robot in the neutral zone, driven by their zone flags: fuel is collected and re-launched.
void setUpFuelFlow(Rig& rig) {
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.add(2, AllianceStation::Blue2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX - 1.2f, kMidY, 0.04f});
    rig.teleport(2, {kMidX + 1.2f, kMidY - 1.5f, 0.04f}, 3.14159f);
}

struct Observed {
    std::uint64_t digest = 0;   // over every (tick, id, quantized position) in publication order
    std::size_t ticks = 0;
    std::size_t removedIds = 0;
    std::size_t addedIds = 0;
};

// Runs the fuel flow and checks the id contract on every tick's snapshot.
Observed observeFuelFlow(int workerCount, bool check) {
    Options options;
    options.workerCount = workerCount;
    Rig rig(options);
    setUpFuelFlow(rig);

    Observed out;
    std::vector<std::uint32_t> previousIds;
    std::vector<sdk::Vec3> previousPositions;
    std::set<std::uint32_t> retired;   // ids that have been published and then disappeared
    std::uint32_t maxSeen = 0;
    std::uint64_t digest = 1469598103934665603ULL;
    const auto mix = [&](std::uint64_t v) { digest = (digest ^ v) * 1099511628211ULL; };

    for (int tick = 0; tick < 300; ++tick) {
        rig.run(1, {{1, 0b01}, {2, 0b10}});
        const std::shared_ptr<const sdk::ISimSnapshot> snap = rig.snapshot();
        const sdk::EntityGroupSample* g = fuelGroup(*snap);
        REQUIRE(g != nullptr);
        if (check) {
            REQUIRE(g->ids.size() == g->positions.size());
            REQUIRE(sdk::entityIdsConsistent(*g));
            const std::set<std::uint32_t> unique(g->ids.begin(), g->ids.end());
            REQUIRE(unique.size() == g->ids.size());
            for (const std::uint32_t id : g->ids) {
                CHECK(id != 0);
                CHECK_MESSAGE(retired.count(id) == 0, "id " << id << " was published for a fuel that is gone");
            }
            const std::set<std::uint32_t> before(previousIds.begin(), previousIds.end());
            for (const std::uint32_t id : previousIds)
                if (unique.count(id) == 0) {
                    retired.insert(id);
                    ++out.removedIds;
                }
            for (const std::uint32_t id : g->ids)
                if (before.count(id) == 0 && !previousIds.empty()) {
                    ++out.addedIds;
                    CHECK_MESSAGE(id > maxSeen, "a new fuel reused id " << id);
                }
            // an id that persists names the same body: it cannot jump between ticks
            std::map<std::uint32_t, sdk::Vec3> was;
            for (std::size_t i = 0; i < previousIds.size(); ++i) was[previousIds[i]] = previousPositions[i];
            for (std::size_t i = 0; i < g->ids.size(); ++i) {
                const auto it = was.find(g->ids[i]);
                if (it != was.end()) CHECK(sdk::length(g->positions[i] - it->second) < 0.5f);
            }
        }
        for (std::size_t i = 0; i < g->ids.size(); ++i) {
            maxSeen = std::max(maxSeen, g->ids[i]);
            mix(g->ids[i]);
            mix(static_cast<std::uint64_t>(wire::quantizeCentiI16(g->positions[i].x)));
            mix(static_cast<std::uint64_t>(wire::quantizeCentiI16(g->positions[i].y)));
            mix(static_cast<std::uint64_t>(wire::quantizeCentiI16(g->positions[i].z)));
        }
        previousIds = g->ids;
        previousPositions = g->positions;
        ++out.ticks;
    }
    out.digest = digest;
    return out;
}

}  // namespace

TEST_CASE("fuel entity ids: every fuel gets a unique id that a recycled pool slot does not reuse") {
    g26test::Session s;
    g26test::emptyField(s);
    s.start();
    REQUIRE(s.game != nullptr);
    FuelPool& pool = s.game->pool();

    const std::uint32_t a = pool.spawn({4.f, 2.f, 0.075f}, {});
    const std::uint32_t b = pool.spawn({5.f, 2.f, 0.075f}, {});
    const std::uint32_t c = pool.spawn({6.f, 2.f, 0.075f}, {});
    REQUIRE((a != 0 && b != 0 && c != 0));
    const std::set<std::uint32_t> first = {pool.entityId(a), pool.entityId(b), pool.entityId(c)};
    CHECK(first.size() == 3);
    CHECK(first.count(0) == 0);

    const std::uint32_t idA = pool.entityId(a);
    const std::uint32_t idC = pool.entityId(c);
    const sdk::BodyId bodyC = pool.bodyOf(c);
    const std::uint32_t retired = pool.entityId(b);
    pool.despawn(b);
    CHECK(pool.entityId(b) == 0);

    const std::uint32_t d = pool.spawn({7.f, 2.f, 0.075f}, {});
    CHECK(d == b);                                   // the slot is recycled
    CHECK(pool.entityId(d) != retired);              // the fuel is not
    CHECK(first.count(pool.entityId(d)) == 0);
    CHECK(pool.entityId(a) == idA);                  // the others keep theirs
    CHECK(pool.entityId(c) == idC);
    CHECK(pool.bodyOf(c) == bodyC);

    pool.clear();
    CHECK(pool.spawn({4.f, 2.f, 0.075f}, {}) != 0);  // a new match starts the ids over
    std::uint32_t only = 0;
    pool.forEach([&](std::uint32_t slot, const sdk::Vec3&) { only = pool.entityId(slot); });
    CHECK(only == 1);
}

TEST_CASE("fuel entity ids: the published group has one stable unique id per fuel, ids stay unique over collect and launch") {
    const Observed o = observeFuelFlow(1, true);
    CHECK(o.ticks == 300);
    CHECK(o.removedIds > 0);   // fuel was collected
    CHECK(o.addedIds > 0);     // and launched again
}

TEST_CASE("fuel entity ids: the initial staging publishes ids 1..N in pool order and a reset publishes them again") {
    Rig rig;
    rig.run(1, {});
    const std::shared_ptr<const sdk::ISimSnapshot> staged = rig.snapshot();
    const sdk::EntityGroupSample* g = fuelGroup(*staged);
    REQUIRE(g != nullptr);
    REQUIRE(g->ids.size() == 360 + 2 * 24);
    const std::vector<std::uint32_t> before = g->ids;
    CHECK(std::is_sorted(before.begin(), before.end()));
    CHECK(before.front() == 1);

    rig.session->reset();
    rig.run(1, {});
    const std::shared_ptr<const sdk::ISimSnapshot> restaged = rig.snapshot();
    const sdk::EntityGroupSample* again = fuelGroup(*restaged);
    REQUIRE(again != nullptr);
    CHECK(again->ids == before);
}

TEST_CASE("fuel entity ids: the same scripted inputs publish the same ids and positions for 1, 2 and 4 physics workers") {
    const Observed reference = observeFuelFlow(1, false);
    CHECK(observeFuelFlow(1, false).digest == reference.digest);
    CHECK(observeFuelFlow(2, false).digest == reference.digest);
    CHECK(observeFuelFlow(4, false).digest == reference.digest);
}

TEST_CASE("fuel entity ids: a match host over the real session and plugin sends the fuel to a client") {
    using namespace std::chrono_literals;
    namespace mt = match_test;
    namespace ms = match_stress;

    Rig rig;
    ms::TruthTap tap(*rig.session);
    mt::Harness h(tap);
    REQUIRE(registerPlugin(h.kit.plugins).ok());
    ms::ModelClient client(h, {}, 100, tap);

    wire::Join join;
    join.displayName = "Shooter";
    join.gamePluginId = tap.gamePluginId();
    join.wpilibYear = tap.wpilibYear();
    sdk::RobotConfig spec = Rig::fuelRobot(6.0);
    spec.gamePluginId = std::string(pluginId());
    join.robotSpecJson = mt::specJson(spec);
    REQUIRE(client.client().send(join).ok());
    const auto joined = [&] {
        client.client().drain();
        if (client.client().count<wire::JoinReject>() > 0) FAIL("join rejected: " << client.client().last<wire::JoinReject>()->detail);
        return client.client().count<wire::JoinAccept>() > 0;
    };
    for (int i = 0; i < 4000 && !joined(); ++i) {
        h.clock.advance(5ms);
        h.poll();
        std::this_thread::sleep_for(1ms);
    }
    REQUIRE(joined());
    mt::TestClient& c = client.client();
    c.accept = c.last<wire::JoinAccept>();
    c.log.clear();
    client.seedClock(c.accept->serverTick);
    rig.teleport(c.accept->participantId, {kMidX + 1.2f, kMidY - 1.5f, 0.04f}, 3.14159f);

    for (int tick = 0; tick < 500; ++tick) {
        h.clock.advance(5ms);
        if (tick % 4 == 0) client.sendInput(4, tick < 100 ? 0b10u : 0u, 4);
        h.poll();
        rig.session->tick(rig.frame, false);
        h.tickEnd();
        REQUIRE(h.host->waitOutboundIdle(3s));
        client.pump(true, 2);
    }

    CHECK(tap.pluginPublishesIds());
    CHECK(tap.idMismatches() == 0);
    const sdk::EntityGroupSample* fuel = fuelGroup(*tap.lastSnapshot());
    REQUIRE(fuel != nullptr);
    CHECK(client.metrics().received > 20);
    CHECK(client.metrics().noTruth == 0);
    CHECK(client.metrics().unappliable == 0);
    // After the shooting stopped the client holds every fuel that is on the field, and nothing else.
    CHECK(client.heldCount() == fuel->positions.size());
    CHECK(client.laggingNow() == 0);
}
