// Randomized synchronization of one entity group between an EntityDeltaEncoder (the host) and the reference client memory over
// a lossy, delayed, reordering link: whatever is lost, late or reordered, once the world rests the client shows exactly the
// entities the host has (no ghosts, none missing) within the rest tolerance, and while it moves the shown error stays bounded.
#include <algorithm>
#include <map>
#include <random>
#include <vector>

#include "entity_memory.h"
#include "match_test_support.h"
#include "wpiemu/core/match/entity_delta.h"
#include "wpiemu/sdk/match/entity_prediction.h"

using namespace match_test;
using wpiemu::core::match::EntityDeltaEncoder;
using wpiemu::core::match::EntityGroupFrame;
using wpiemu::core::match::EntityPlan;
using wpiemu::core::match::EntityPolicy;
using wpiemu::core::match::RobotAnchor;

namespace {

constexpr std::int64_t kFrameUs = 16'667;

struct LinkProfile {
    double loss = 0.0;            // each direction
    int delaySteps = 1;           // one-way, in snapshots
    int jitterSteps = 0;          // extra random delay (reorders)
    std::size_t budget = 1000;    // changes per snapshot
};

struct Outcome {
    bool converged = false;
    std::size_t ghosts = 0;
    std::size_t missing = 0;
    std::size_t sent = 0;
    double worstMovingErrorCm = 0;
    std::size_t applied = 0;
};

// Entities that fly in straight lines (and are updated by the host as the model says), appear and disappear at random.
Outcome simulate(std::uint32_t seed, const LinkProfile& link, const EntityPolicy& policy, int movingSteps, int restSteps) {
    std::mt19937 rng(seed);
    const auto chance = [&](double p) { return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < p; };

    std::map<std::uint32_t, wire::EntityState> world;
    std::uint32_t nextId = 1;
    const auto spawn = [&] {
        wire::EntityState e;
        e.id = nextId++;
        e.xCm = static_cast<std::int16_t>(rng() % 1500);
        e.yCm = static_cast<std::int16_t>(rng() % 800);
        e.zCm = 8;
        if (chance(0.5)) {   // thrown
            e.vxCms = static_cast<std::int16_t>(static_cast<int>(rng() % 800) - 400);
            e.vyCms = static_cast<std::int16_t>(static_cast<int>(rng() % 400) - 200);
            e.vzCms = static_cast<std::int16_t>(300 + rng() % 400);
            e.ballistic = true;
        }
        world[e.id] = e;
    };
    for (int i = 0; i < 80; ++i) spawn();

    // The truth advances by the model itself (so the host's predictions are right until it "collides": a random velocity change).
    std::map<std::uint32_t, std::pair<wire::EntityState, std::int64_t>> origin;   // entity as set at its last change, and when
    for (auto& [id, e] : world) origin[id] = {e, 0};

    EntityDeltaEncoder enc(policy);
    EntityMemory client;
    struct InFlight {
        int arrivesAt;
        wire::MatchSnapshot snapshot;
    };
    struct AckInFlight {
        int arrivesAt;
        std::uint32_t seq;
    };
    std::vector<InFlight> down;
    std::vector<AckInFlight> up;
    std::uint32_t seq = 0;
    Outcome out;

    const int total = movingSteps + restSteps;
    for (int step = 0; step < total; ++step) {
        const std::int64_t t = static_cast<std::int64_t>(step) * kFrameUs;
        const bool moving = step < movingSteps;
        // The world: everything follows its model; a few entities change course, leave, or appear.
        for (auto& [id, o] : origin) world[id] = wire::predictEntity(o.first, t - o.second);
        if (moving) {
            std::vector<std::uint32_t> ids;
            for (const auto& [id, e] : world) ids.push_back(id);
            for (const std::uint32_t id : ids) {
                if (chance(0.03)) {   // a collision: new velocity from here
                    wire::EntityState e = world[id];
                    e.vxCms = static_cast<std::int16_t>(static_cast<int>(rng() % 600) - 300);
                    e.vyCms = static_cast<std::int16_t>(static_cast<int>(rng() % 600) - 300);
                    e.vzCms = static_cast<std::int16_t>(rng() % 500);
                    e.ballistic = e.vzCms > 0 && chance(0.6);
                    origin[id] = {e, t};
                    world[id] = e;
                } else if (chance(0.01)) {
                    world.erase(id);
                    origin.erase(id);
                }
            }
            if (chance(0.3)) {
                spawn();
                origin[nextId - 1] = {world[nextId - 1], t};
            }
        } else {
            // Rest: everything stops where it is (the world is static from here on).
            for (auto& [id, o] : origin) {
                wire::EntityState e = world[id];
                e.vxCms = e.vyCms = e.vzCms = 0;
                e.ballistic = false;
                o = {e, t};
                world[id] = e;
            }
        }

        EntityGroupFrame frame;
        frame.groupId = 0;
        for (const auto& [id, e] : world) frame.entities.push_back(e);
        const std::vector<RobotAnchor> anchors{{500, 300}};
        const EntityPlan plan = enc.plan(std::vector<EntityGroupFrame>{frame}, anchors, t);
        const std::size_t count = std::min(plan.changes.size(), link.budget);
        wire::MatchSnapshot snapshot;
        snapshot.snapshotSeq = ++seq;
        snapshot.simTimeUs = t;
        snapshot.entityDeltas = EntityDeltaEncoder::deltas(plan, count);
        enc.commit(plan, count, snapshot.snapshotSeq, t);
        for (const wire::EntityDelta& d : snapshot.entityDeltas) out.sent += d.upserted.size();
        if (!chance(link.loss))
            down.push_back({step + link.delaySteps + static_cast<int>(rng() % static_cast<unsigned>(link.jitterSteps + 1)), snapshot});

        // Deliveries due now, in arrival order (a jittered one may overtake an earlier one).
        std::stable_sort(down.begin(), down.end(), [](const InFlight& a, const InFlight& b) { return a.arrivesAt < b.arrivesAt; });
        while (!down.empty() && down.front().arrivesAt <= step) {
            const wire::MatchSnapshot s = std::move(down.front().snapshot);
            down.erase(down.begin());
            if (client.apply(s) == EntityMemory::Result::Applied) {
                ++out.applied;
                if (!chance(link.loss))
                    up.push_back({step + link.delaySteps + static_cast<int>(rng() % static_cast<unsigned>(link.jitterSteps + 1)), s.snapshotSeq});
            }
        }
        std::stable_sort(up.begin(), up.end(), [](const AckInFlight& a, const AckInFlight& b) { return a.arrivesAt < b.arrivesAt; });
        while (!up.empty() && up.front().arrivesAt <= step) {
            enc.onAck(up.front().seq);
            up.erase(up.begin());
        }

        if (moving && step > 60 && client.seen()) {
            for (const auto& [id, held] : client.group(0)) {
                const auto truth = world.find(id);
                if (truth == world.end()) continue;
                const wire::EntityState shown = EntityMemory::shown(held, t);
                const double dx = shown.xCm - truth->second.xCm, dy = shown.yCm - truth->second.yCm, dz = shown.zCm - truth->second.zCm;
                out.worstMovingErrorCm = std::max(out.worstMovingErrorCm, std::sqrt(dx * dx + dy * dy + dz * dz));
            }
        }
    }

    const auto shown = client.picture(0);
    std::map<std::uint32_t, wire::EntityState> truth;
    for (const auto& [id, e] : world) {
        wire::EntityState p = e;
        p.vxCms = p.vyCms = p.vzCms = 0;
        p.ballistic = false;
        truth[id] = p;
    }
    for (const auto& [id, e] : shown)
        if (truth.find(id) == truth.end()) ++out.ghosts;
    for (const auto& [id, e] : truth)
        if (shown.find(id) == shown.end()) ++out.missing;
    out.converged = sameEntities(shown, truth, 1);
    return out;
}

}  // namespace

TEST_CASE("entity sync: a removal is repeated until a snapshot that carries it is acknowledged") {
    EntityDeltaEncoder enc;
    const auto frameOf = [](std::vector<wire::EntityState> entities) {
        EntityGroupFrame f;
        f.groupId = 0;
        f.entities = std::move(entities);
        return std::vector<EntityGroupFrame>{f};
    };
    wire::EntityState e;
    e.id = 5;
    e.xCm = 100;
    std::int64_t t = 0;
    std::uint32_t seq = 0;
    const auto send = [&](const std::vector<EntityGroupFrame>& frame) {
        const EntityPlan plan = enc.plan(frame, {}, t);
        enc.commit(plan, plan.changes.size(), ++seq, t);
        t += kFrameUs;
        return plan;
    };
    // seq 1 carries the entity and is acknowledged; seq 2 and 3 are built before any further acknowledgement.
    send(frameOf({e}));
    REQUIRE(enc.onAck(1));
    send(frameOf({e}));   // 2: nothing to say
    // The entity is updated in seq 3 (in flight) and gone from seq 4 on.
    wire::EntityState moved = e;
    moved.xCm = 300;
    const EntityPlan p3 = send(frameOf({moved}));
    REQUIRE(p3.changes.size() == 1);
    for (int k = 0; k < 4; ++k) {   // seq 4..7: the acknowledgement of 3 has not arrived, every snapshot tells the client
        const EntityPlan gone = send(frameOf({}));
        CAPTURE(k);
        REQUIRE(gone.changes.size() == 1);
        CHECK(gone.changes[0].removal);
        CHECK(gone.changes[0].state.id == 5);
    }
    REQUIRE(enc.onAck(5));   // a snapshot that carried the removal arrived
    const EntityPlan quiet = send(frameOf({}));
    CHECK(quiet.changes.empty());
}

TEST_CASE("entity sync: a removal of an update that only exists in flight is sent and repeated") {
    EntityDeltaEncoder enc;
    const auto frameOf = [](std::vector<wire::EntityState> entities) {
        EntityGroupFrame f;
        f.groupId = 0;
        f.entities = std::move(entities);
        return std::vector<EntityGroupFrame>{f};
    };
    wire::EntityState a, b;
    a.id = 1;
    b.id = 2;
    b.xCm = 50;
    std::int64_t t = 0;
    std::uint32_t seq = 0;
    const auto send = [&](const std::vector<EntityGroupFrame>& frame) {
        const EntityPlan plan = enc.plan(frame, {}, t);
        enc.commit(plan, plan.changes.size(), ++seq, t);
        t += kFrameUs;
        return plan;
    };
    send(frameOf({a}));          // 1
    REQUIRE(enc.onAck(1));
    send(frameOf({a, b}));       // 2: b is new and in flight (no acknowledgement yet)
    const EntityPlan p3 = send(frameOf({a}));   // 3: b is gone before the baseline ever held it
    REQUIRE(p3.changes.size() == 1);
    CHECK(p3.changes[0].removal);
    CHECK(p3.changes[0].state.id == 2);
    const EntityPlan p4 = send(frameOf({a}));   // 4: still unacknowledged, still told
    REQUIRE(p4.changes.size() == 1);
    CHECK(p4.changes[0].removal);
    REQUIRE(enc.onAck(4));
    CHECK(send(frameOf({a})).changes.empty());
}

TEST_CASE("entity sync: a lossless link follows the world with bounded error and ends up exact") {
    for (std::uint32_t seed = 1; seed <= 6; ++seed) {
        CAPTURE(seed);
        const Outcome o = simulate(seed, LinkProfile{0.0, 1, 0, 1000}, EntityPolicy{}, 400, 200);
        CHECK(o.converged);
        CHECK(o.ghosts == 0);
        CHECK(o.missing == 0);
        CHECK(o.worstMovingErrorCm < 40.0);
    }
}

TEST_CASE("entity sync: with loss, delay and reordering the client never keeps an entity the host no longer has") {
    for (const double loss : {0.1, 0.3, 0.5}) {
        for (const int delay : {1, 3, 6}) {
            for (std::uint32_t seed = 1; seed <= 6; ++seed) {
                CAPTURE(loss);
                CAPTURE(delay);
                CAPTURE(seed);
                const Outcome o = simulate(seed * 7919u, LinkProfile{loss, delay, 3, 1000}, EntityPolicy{}, 400, 600);
                CHECK(o.ghosts == 0);
                CHECK(o.missing == 0);
                CHECK(o.converged);
            }
        }
    }
}

TEST_CASE("entity sync: a small budget still converges and nothing is shown that does not exist") {
    for (const std::size_t budget : {4u, 10u, 25u}) {
        for (std::uint32_t seed = 1; seed <= 4; ++seed) {
            CAPTURE(budget);
            CAPTURE(seed);
            const Outcome o = simulate(seed * 104729u, LinkProfile{0.2, 3, 2, budget}, EntityPolicy{}, 300, 3000);
            CHECK(o.ghosts == 0);
            CHECK(o.converged);
        }
    }
}

TEST_CASE("entity sync: updates still in flight are not sent again, which saves most of the repeats on a slow link") {
    EntityPolicy noSuppression;
    noSuppression.suppressInFlight = false;   // every unacknowledged change is due again, as in a plain baseline delta
    std::size_t with = 0, without = 0;
    for (std::uint32_t seed = 1; seed <= 4; ++seed) {
        const Outcome a = simulate(seed, LinkProfile{0.0, 4, 0, 1000}, EntityPolicy{}, 300, 100);
        const Outcome b = simulate(seed, LinkProfile{0.0, 4, 0, 1000}, noSuppression, 300, 100);
        CHECK(a.converged);
        CHECK(b.converged);
        with += a.sent;
        without += b.sent;
    }
    CAPTURE(with);
    CAPTURE(without);
    CHECK(with * 10 < without * 8);   // at least 20 percent fewer updates sent
}
