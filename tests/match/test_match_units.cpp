#include <condition_variable>
#include <cmath>
#include <limits>
#include <mutex>

#include "match_test_support.h"
#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/core/match/entity_delta.h"
#include "wpiemu/core/match/hud_keys.h"
#include "wpiemu/core/match/peer_outbox.h"
#include "wpiemu/core/match/snapshot_budget.h"
#include "wpiemu/core/match/snapshot_rate.h"
#include "wpiemu/core/match/spec_intake.h"
#include "wpiemu/core/match/wire_frame.h"
#include "wpiemu/sdk/match/entity_prediction.h"

using namespace match_test;

namespace {

wire::EntityState ent(std::uint32_t id, int x, int y = 0, int z = 0) {
    return {id, static_cast<std::int16_t>(x), static_cast<std::int16_t>(y), static_cast<std::int16_t>(z)};
}

wire::EntityState entv(std::uint32_t id, int x, int y, int z, int vx, int vy, int vz, bool ballistic) {
    wire::EntityState e = ent(id, x, y, z);
    e.vxCms = static_cast<std::int16_t>(vx);
    e.vyCms = static_cast<std::int16_t>(vy);
    e.vzCms = static_cast<std::int16_t>(vz);
    e.ballistic = ballistic;
    return e;
}

EntityGroupFrame group(std::uint16_t id, std::vector<wire::EntityState> entities) {
    EntityGroupFrame g;
    g.groupId = id;
    g.entities = std::move(entities);
    return g;
}

}  // namespace

TEST_CASE("hud keys: the table is the sorted set of the descriptors' keys, ids start at 1") {
    HudWidgetDescriptor a;
    a.valueKey = "b.value";
    a.maxKey = "a.max";
    HudWidgetDescriptor b;
    b.valueKey = "a.max";   // duplicates collapse
    b.sides = {{"L", "c.score", "d.status", ""}, {"R", "e.score", "", ""}};
    const std::vector<std::string> extra{"z.extra", ""};
    const HudKeyTable table = HudKeyTable::fromDescriptors(std::vector<HudWidgetDescriptor>{a, b}, extra);
    CHECK(table.keys() == std::vector<std::string>{"a.max", "b.value", "c.score", "d.status", "e.score", "z.extra"});
    CHECK(table.idOf("a.max") == 1);
    CHECK(table.idOf("z.extra") == 6);
    CHECK_FALSE(table.idOf("nope").has_value());
    CHECK_FALSE(table.idOf("").has_value());
    CHECK(table.keyOf(3) == "c.score");
    CHECK_FALSE(table.keyOf(0).has_value());
    CHECK_FALSE(table.keyOf(7).has_value());
    // The same plugin gives the same table on any machine, in any descriptor order.
    CHECK(HudKeyTable::fromDescriptors(std::vector<HudWidgetDescriptor>{b, a}, extra).keys() == table.keys());
}

TEST_CASE("entity deltas: with no baseline the plan is the full state, nearest to a robot first") {
    EntityDeltaEncoder enc;
    const std::vector<EntityGroupFrame> current{group(0, {ent(1, 900), ent(2, 50), ent(3, 450), ent(4, 60)})};
    const std::vector<RobotAnchor> anchors{{0, 0}};
    const EntityPlan plan = enc.plan(current, anchors, 0);
    CHECK(plan.baselineId == wire::kNoBaseline);
    REQUIRE(plan.changes.size() == 4);
    // Bands are 2 m: ids 2 and 4 (0.5 m, 0.6 m) share band 0 and are ordered by id, then 3 (4.5 m), then 1 (9 m).
    CHECK(plan.changes[0].state.id == 2);
    CHECK(plan.changes[1].state.id == 4);
    CHECK(plan.changes[2].state.id == 3);
    CHECK(plan.changes[3].state.id == 1);
    for (const EntityChange& c : plan.changes) CHECK_FALSE(c.removal);
    CHECK(EntityDeltaEncoder::deltas(plan, 2)[0].upserted.size() == 2);
    CHECK(EntityDeltaEncoder::deltas(plan, 99)[0].upserted.size() == 4);
}

TEST_CASE("entity deltas: against a baseline only due changes are listed, removals first, then the worst error first") {
    EntityDeltaEncoder enc;
    const std::vector<RobotAnchor> anchors{{0, 0}};
    const std::vector<EntityGroupFrame> first{group(0, {ent(1, 10), ent(2, 20), ent(3, 30), ent(4, 40), ent(5, 50)})};
    EntityPlan p1 = enc.plan(first, anchors, 0);
    enc.commit(p1, p1.changes.size(), 1, 0);
    REQUIRE(enc.onAck(1));
    CHECK(enc.baselineId() == 1);

    // 100 ms later: 1 moved 1 cm (within the 1 cm tolerance at rest), 3 moved 50 cm, 4 is gone, 6 is new.
    const std::vector<EntityGroupFrame> second{group(0, {ent(1, 11), ent(2, 20), ent(3, 80), ent(5, 50), ent(6, 5)})};
    const EntityPlan p2 = enc.plan(second, anchors, 100'000);
    CHECK(p2.baselineId == 1);
    REQUIRE(p2.changes.size() == 3);
    CHECK(p2.changes[0].removal);
    CHECK(p2.changes[0].state.id == 4);
    CHECK(p2.changes[1].state.id == 6);   // a new entity ranks above any real error
    CHECK(p2.changes[2].state.id == 3);
    const auto deltas = EntityDeltaEncoder::deltas(p2, 3);
    CHECK(deltas[0].baselineId == 1);
    CHECK(deltas[0].removed == std::vector<std::uint32_t>{4});
    REQUIRE(deltas[0].upserted.size() == 2);
    CHECK(deltas[0].upserted[0].id == 3);   // the wire lists are ascending by id whatever the send order was
    CHECK(deltas[0].upserted[1].id == 6);
    CHECK(wire::entityDeltaIdsAscending(deltas[0]));
}

TEST_CASE("entity deltas: what was committed is the next baseline, truncated changes stay pending") {
    EntityDeltaEncoder enc;
    const std::vector<EntityGroupFrame> world{group(0, {ent(1, 10), ent(2, 20), ent(3, 30)})};
    EntityPlan p1 = enc.plan(world, {}, 0);
    enc.commit(p1, 2, 1, 0);   // only two of three fit
    REQUIRE(enc.onAck(1));
    const EntityPlan p2 = enc.plan(world, {}, 5'000);
    CHECK(p2.baselineId == 1);
    CHECK(p2.changes.size() == 1);   // the third is still missing at the client
    enc.commit(p2, 1, 2, 5'000);
    REQUIRE(enc.onAck(2));
    CHECK(enc.plan(world, {}, 10'000).changes.empty());
}

TEST_CASE("entity deltas: acknowledgements and the bounded history") {
    EntityDeltaEncoder enc;
    const std::vector<EntityGroupFrame> world{group(0, {ent(1, 10)})};
    CHECK_FALSE(enc.onAck(1));   // nothing sent yet
    for (std::uint32_t seq = 1; seq <= 40; ++seq) {
        EntityPlan p = enc.plan(world, {}, seq * 1000);
        enc.commit(p, p.changes.size(), seq, seq * 1000);
        if (seq == 1) CHECK(enc.onAck(1));
    }
    CHECK(enc.historySize() == EntityDeltaEncoder::kHistory);
    CHECK(enc.baselineId() == wire::kNoBaseline);   // snapshot 1 left the history: full state again
    CHECK_FALSE(enc.onAck(1));
    CHECK(enc.onAck(39));
    CHECK_FALSE(enc.onAck(38));   // older than the baseline
    CHECK(enc.baselineId() == 39);
    enc.reset();
    CHECK(enc.historySize() == 0);
    CHECK(enc.baselineId() == wire::kNoBaseline);
}

TEST_CASE("entity deltas: a group that vanished is removed entity by entity, and lists never exceed the wire cap") {
    EntityDeltaEncoder enc;
    std::vector<wire::EntityState> many;
    for (std::uint32_t i = 1; i <= 1500; ++i) many.push_back(ent(i, static_cast<int>(i % 300)));
    const std::vector<EntityGroupFrame> world{group(0, many), group(3, {ent(1, 1)})};
    EntityPlan p1 = enc.plan(world, {}, 0);
    CHECK(p1.omittedByCap == 1500 - wire::kMaxEntityDeltaEntries);
    CHECK(p1.changes.size() == wire::kMaxEntityDeltaEntries + 1);
    for (const wire::EntityDelta& d : EntityDeltaEncoder::deltas(p1, p1.changes.size())) CHECK(d.upserted.size() <= wire::kMaxEntityDeltaEntries);
    enc.commit(p1, p1.changes.size(), 1, 0);
    REQUIRE(enc.onAck(1));

    const std::vector<EntityGroupFrame> later{group(0, many)};   // group 3 is gone
    const EntityPlan p2 = enc.plan(later, {}, 16'000);
    CHECK(p2.groupIds == std::vector<std::uint16_t>{0, 3});
    bool removedGroup3 = false;
    for (const EntityChange& c : p2.changes) removedGroup3 = removedGroup3 || (c.removal && c.groupId == 3 && c.state.id == 1);
    CHECK(removedGroup3);
}

namespace {

// One entity under the policy: sent at stamp 0 in snapshot 1 (acknowledged), planned again `laterUs` later with `truth`.
EntityPlan planAgainst(const wire::EntityState& sent, const wire::EntityState& truth, std::int64_t laterUs, const EntityPolicy& policy = {}) {
    EntityDeltaEncoder enc(policy);
    EntityPlan p1 = enc.plan(std::vector<EntityGroupFrame>{group(0, {sent})}, {}, 0);
    enc.commit(p1, p1.changes.size(), 1, 0);
    REQUIRE(enc.onAck(1));
    return enc.plan(std::vector<EntityGroupFrame>{group(0, {truth})}, {}, laterUs);
}

}  // namespace

TEST_CASE("entity deltas: an entity that moves as predicted is not resent, one that deviates is") {
    // A ball in flight: 400 cm/s toward +x, 600 cm/s up, from z = 100 cm.
    const wire::EntityState sent = entv(7, 0, 0, 100, 400, 0, 600, true);
    const auto truthAfter = [&](std::int64_t us) { return wire::predictEntity(sent, us); };
    for (const std::int64_t us : {16'000, 100'000, 400'000, 900'000}) {
        CAPTURE(us);
        wire::EntityState truth = truthAfter(us);
        CHECK(planAgainst(sent, truth, us).changes.empty());
        truth.xCm = static_cast<std::int16_t>(truth.xCm + 4);   // within the 5 cm threshold
        CHECK(planAgainst(sent, truth, us).changes.empty());
        truth.xCm = static_cast<std::int16_t>(truth.xCm + 2);   // 6 cm off
        CHECK(planAgainst(sent, truth, us).changes.size() == 1);
    }
    // Without the model (velocity 0, not ballistic) the same ball would need an update at every snapshot.
    wire::EntityState blind = sent;
    blind.vxCms = blind.vyCms = blind.vzCms = 0;
    blind.ballistic = false;
    CHECK(planAgainst(blind, truthAfter(100'000), 100'000).changes.size() == 1);
}

TEST_CASE("entity deltas: keyframes re-confirm entities, moving ones sooner than the ones at rest") {
    const wire::EntityState rest = ent(1, 100, 200, 7);
    const wire::EntityState moving = entv(2, 100, 200, 7, 50, 0, 0, false);
    EntityPolicy policy;
    CHECK(planAgainst(rest, rest, policy.keyframeRestUs - 1).changes.empty());
    const EntityPlan restDue = planAgainst(rest, rest, policy.keyframeRestUs);
    REQUIRE(restDue.changes.size() == 1);
    CHECK(restDue.changes[0].reason == EntityChangeReason::Keyframe);
    // A rolling ball predicted exactly is quiet until its keyframe (1 s).
    CHECK(planAgainst(moving, wire::predictEntity(moving, 900'000), 900'000).changes.empty());
    const EntityPlan movingDue = planAgainst(moving, wire::predictEntity(moving, policy.keyframeMovingUs), policy.keyframeMovingUs);
    REQUIRE(movingDue.changes.size() == 1);
    CHECK(movingDue.changes[0].reason == EntityChangeReason::Keyframe);
    // Custom policy values are honoured.
    policy.keyframeRestUs = 500'000;
    policy.errorThresholdCm = 0;
    policy.restErrorThresholdCm = 0;
    CHECK(planAgainst(rest, rest, 499'000, policy).changes.empty());
    CHECK(planAgainst(rest, rest, 500'000, policy).changes.size() == 1);
    wire::EntityState nudged = rest;
    nudged.xCm += 1;
    CHECK(planAgainst(rest, nudged, 16'000, policy).changes.size() == 1);
    // At rest the default tolerance is one centimetre, so an entity ends up within a centimetre of where it really is.
    CHECK(planAgainst(rest, nudged, 16'000).changes.empty());
    nudged.xCm += 1;
    CHECK(planAgainst(rest, nudged, 16'000).changes.size() == 1);
}

TEST_CASE("entity deltas: the priority is error times age over the distance band, so nothing starves") {
    EntityDeltaEncoder enc;
    const std::vector<RobotAnchor> anchors{{0, 0}};
    // id 1 and id 3 (the same, but 10 m from the robot) are confirmed at 3.4 s; id 2 was last confirmed at 0.
    const std::vector<EntityGroupFrame> start{group(0, {ent(1, 0), ent(2, 0), ent(3, 1000)})};
    EntityPlan p1 = enc.plan(start, anchors, 0);
    enc.commit(p1, p1.changes.size(), 1, 0);
    REQUIRE(enc.onAck(1));
    const std::vector<EntityGroupFrame> mid{group(0, {ent(1, 50), ent(2, 0), ent(3, 1050)})};
    EntityPlan p2 = enc.plan(mid, anchors, 3'400'000);
    REQUIRE(p2.changes.size() == 2);   // 1 and 3 moved; 2 did not and its keyframe is not due
    enc.commit(p2, p2.changes.size(), 2, 3'400'000);
    REQUIRE(enc.onAck(2));
    // At 3.5 s: id 1 and 3 are 100 cm off after 0.1 s, id 2 is only 8 cm off but has waited 3.5 s.
    const std::vector<EntityGroupFrame> late{group(0, {ent(1, 150), ent(2, 8), ent(3, 1150)})};
    const EntityPlan p3 = enc.plan(late, anchors, 3'500'000);
    REQUIRE(p3.changes.size() == 3);
    // priority = (error + 3) * (age + 0.25 s) / (1 + band): id 2 11 * 3.75 = 41.3, id 1 103 * 0.35 = 36.1, id 3 36.1 / 6 = 6.
    CHECK(p3.changes[0].state.id == 2);
    CHECK(p3.changes[1].state.id == 1);
    CHECK(p3.changes[2].state.id == 3);
}

TEST_CASE("entity deltas: velocity, flight and the stamp of an update survive the baseline round trip") {
    EntityDeltaEncoder enc;
    const wire::EntityState flying = entv(9, 10, 20, 150, -300, 40, 250, true);
    EntityPlan p1 = enc.plan(std::vector<EntityGroupFrame>{group(0, {flying})}, {}, 1'000'000);
    REQUIRE(EntityDeltaEncoder::deltas(p1, 1)[0].upserted.size() == 1);
    CHECK(EntityDeltaEncoder::deltas(p1, 1)[0].upserted[0] == flying);
    enc.commit(p1, 1, 1, 1'000'000);
    REQUIRE(enc.onAck(1));
    // The stamp is the sim time of the snapshot that carried it: 250 ms later the prediction still holds the ball...
    CHECK(enc.plan(std::vector<EntityGroupFrame>{group(0, {wire::predictEntity(flying, 250'000)})}, {}, 1'250'000).changes.empty());
    // ...and the same truth against a prediction that started at another stamp is off: the stamp matters.
    CHECK(enc.plan(std::vector<EntityGroupFrame>{group(0, {wire::predictEntity(flying, 250'000)})}, {}, 2'000'000).changes.size() == 1);
}

TEST_CASE("snapshot rate: hysteresis and sampling") {
    SnapshotRateSelector sel;
    auto t = std::chrono::steady_clock::time_point{} + 10s;
    proto::LinkStats good;
    good.rttMs = 10;
    proto::LinkStats bad;
    bad.rttMs = 200;
    CHECK(sel.update(std::nullopt, t) == 60);
    CHECK(sel.update(bad, t) == 60);   // one bad sample is not enough
    CHECK(sel.update(bad, t + 100ms) == 60);   // and the next one is too early to count
    CHECK(sel.update(good, t + 500ms) == 60);   // a good one resets the run
    CHECK(sel.update(bad, t + 1000ms) == 60);
    CHECK(sel.update(bad, t + 1500ms) == 30);
    for (int i = 0; i < 5; ++i) CHECK(sel.update(good, t + 2000ms + 500ms * i) == 30);
    CHECK(sel.update(good, t + 4500ms) == 60);
}

namespace {

// A link whose send blocks until released.
class BlockingLink final : public rt::IMessageLink {
public:
    Status send(const proto::Message&) override {
        std::unique_lock lock(m_);
        ++started_;
        cv_.wait(lock, [&] { return !blocked_ || closed_; });
        if (closed_) return makeError(ErrorCode::Disconnected, "closed");
        ++sent_;
        return okStatus();
    }
    void close() override {
        std::lock_guard lock(m_);
        closed_ = true;
        cv_.notify_all();
    }
    bool connected() const override { return !closed_; }
    std::string describe() const override { return "blocking"; }
    void release() {
        std::lock_guard lock(m_);
        blocked_ = false;
        cv_.notify_all();
    }
    int started() const {
        std::lock_guard lock(m_);
        return started_;
    }
    int sentCount() const {
        std::lock_guard lock(m_);
        return sent_;
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool blocked_ = true;
    bool closed_ = false;
    int started_ = 0;
    int sent_ = 0;
};

}  // namespace

TEST_CASE("peer outbox: a stuck peer costs the caller nothing and is reported") {
    testing::ManualWallClock clock;
    auto link = std::make_shared<BlockingLink>();
    OutboxLimits limits;
    limits.reliableCapacity = 4;
    limits.unreliableCapacity = 2;
    limits.stallTimeout = 1s;
    PeerOutbox outbox(link, clock, limits);

    REQUIRE(outbox.enqueue(wire::Roster{}) == PeerOutbox::Result::Queued);
    REQUIRE(testing::waitUntil([&] { return link->started() == 1; }, 2s));   // the sender is now stuck in send()
    CHECK_FALSE(outbox.stalled());
    clock.advance(1500ms);
    CHECK(outbox.stalled());

    for (int i = 0; i < 4; ++i) CHECK(outbox.enqueue(wire::Roster{}) == PeerOutbox::Result::Queued);
    CHECK(outbox.enqueue(wire::Roster{}) == PeerOutbox::Result::Overflow);   // reliable backlog full
    CHECK(outbox.enqueue(wire::MatchSnapshot{}) == PeerOutbox::Result::Queued);
    CHECK(outbox.enqueue(wire::MatchSnapshot{}) == PeerOutbox::Result::Queued);
    CHECK(outbox.enqueue(wire::MatchSnapshot{}) == PeerOutbox::Result::QueuedDroppedOldest);   // latest wins
    CHECK(outbox.droppedUnreliable() == 1);
    CHECK(outbox.pending() == 1 + 4 + 2);

    link->release();
    CHECK(outbox.waitIdle(2s));
    CHECK(link->sentCount() == 1 + 4 + 2);
    CHECK_FALSE(outbox.stalled());
    CHECK(outbox.sent() == 7);
}

TEST_CASE("peer outbox: closeAfter closes the link once the message is out, and a dead link fails the outbox") {
    testing::ManualWallClock clock;
    auto link = std::make_shared<BlockingLink>();
    link->release();
    PeerOutbox outbox(link, clock, {});
    CHECK(outbox.enqueue(wire::MatchControl{}, true) == PeerOutbox::Result::Queued);
    CHECK(outbox.waitIdle(2s));
    CHECK(testing::waitUntil([&] { return !link->connected(); }, 2s));
    CHECK(outbox.enqueue(wire::Roster{}) == PeerOutbox::Result::Queued);
    CHECK(testing::waitUntil([&] { return outbox.failed(); }, 2s));
    CHECK(outbox.enqueue(wire::Roster{}) == PeerOutbox::Result::Closed);
}

TEST_CASE("spec intake: the real pipeline decodes the spec and the fairness limits clamp it") {
    PipelineKit kit;
    SUBCASE("a plausible robot passes unchanged") {
        const auto spec = acceptRemoteSpec(specJson(swerveSpec()), kit.pipeline);
        REQUIRE(spec.ok());
        CHECK(spec.value().clamped.empty());
        CHECK(spec.value().config.drivetrain.spec.wheels.size() == 4);
        CHECK(spec.value().config.chassis.massKg == doctest::Approx(50.f));
    }
    SUBCASE("an out of range robot is clamped and each correction is reported") {
        RobotConfig heavy = swerveSpec();
        heavy.chassis.massKg = 900.f;
        heavy.drivetrain.spec.wheels[0].radiusM = 1.0f;
        const auto spec = acceptRemoteSpec(specJson(heavy), kit.pipeline);
        REQUIRE(spec.ok());
        CHECK(spec.value().config.chassis.massKg == doctest::Approx(kRemoteSpecLimits.massKg.max));
        CHECK(spec.value().config.drivetrain.spec.wheels[0].radiusM == doctest::Approx(kRemoteSpecLimits.wheelRadiusM.max));
        CHECK(spec.value().clamped.size() >= 2);
        CHECK(checkRemoteSpec(spec.value().config.chassis, spec.value().config.drivetrain.spec).empty());
    }
    SUBCASE("a wheel count outside the limits is refused, not clamped") {
        RobotConfig one = swerveSpec();
        one.drivetrain.spec.wheels.resize(1);
        const auto spec = acceptRemoteSpec(specJson(one), kit.pipeline);
        REQUIRE_FALSE(spec.ok());
        CHECK_FALSE(spec.error().message.empty());
        CHECK(spec.error().message.size() <= wire::kMaxDetailBytes);
    }
    SUBCASE("garbage, empty and oversized descriptions are refused") {
        CHECK_FALSE(acceptRemoteSpec("", kit.pipeline).ok());
        CHECK_FALSE(acceptRemoteSpec("[1,2,3]", kit.pipeline).ok());
        CHECK_FALSE(acceptRemoteSpec("{\"drivetrain\": 5}", kit.pipeline).ok());
        CHECK_FALSE(acceptRemoteSpec(std::string(wire::kMaxRobotSpecBytes + 1, ' '), kit.pipeline).ok());
        std::string deep(300, '[');
        deep.append(300, ']');
        CHECK_FALSE(acceptRemoteSpec(deep, kit.pipeline).ok());   // JsonValueCodec::kMaxDepth bounds hostile nesting
    }
}

TEST_CASE("wire frame: groups without stable ids are skipped, non-finite values are sanitized") {
    SimSnapshotData d;
    EntityGroupSample fuel;
    fuel.ids = {7, 3};
    fuel.positions = {{1.f, 2.f, 3.f}, {std::nanf(""), 0.f, 400.f}};
    EntityGroupSample anonymous;
    anonymous.positions = {{1.f, 1.f, 1.f}};
    d.entityGroups = {fuel, anonymous};
    d.chassis.pose.position = {std::numeric_limits<float>::infinity(), 1.f, 0.f};
    const SimSnapshot snap(std::move(d));
    const WireFrame frame = buildWireFrame(snap, HudKeyTable{}, {});
    REQUIRE(frame.groups.size() == 1);
    CHECK(frame.groupsWithoutIds == 1);
    CHECK(frame.groups[0].groupId == 0);
    CHECK(frame.groups[0].entities[0].id == 3);   // sorted by id
    CHECK(frame.groups[0].entities[0].xCm == 0);   // NaN -> 0
    CHECK(frame.groups[0].entities[0].zCm == 32767);   // saturates
    CHECK(frame.groups[0].entities[1] == ent(7, 100, 200, 300));
    CHECK(frame.robots[0].xMm == 0);   // +inf is not a position: 0
    CHECK(hudWireValue(Value(std::string("text"))) == std::nullopt);
    CHECK(hudWireValue(Value(true)) == 1.f);
}

namespace {

wire::MatchSnapshot coreSnapshot(std::size_t robots, std::size_t hudPerRobot) {
    wire::MatchSnapshot s;
    s.sessionId = 1;
    s.runEpoch = 1;
    s.snapshotSeq = 1;
    for (std::size_t r = 0; r < robots; ++r) {
        wire::RobotWireState robot;
        robot.participantId = static_cast<wire::ParticipantId>(r);
        robot.wheels.assign(4, wire::WheelWireState{1234, -4321});
        for (std::size_t h = 0; h < hudPerRobot; ++h) robot.hud.push_back({static_cast<std::uint16_t>(h + 1), 1.5f});
        s.robots.push_back(std::move(robot));
    }
    for (std::uint16_t h = 1; h <= 4; ++h) s.gameHud.push_back({h, 2.5f});
    return s;
}

EntityPlan manyChanges(std::size_t count) {
    EntityDeltaEncoder enc;
    std::vector<wire::EntityState> entities;
    for (std::size_t i = 0; i < count; ++i) entities.push_back(ent(static_cast<std::uint32_t>(i + 1), static_cast<int>(i % 200) * 7, static_cast<int>(i / 200) * 9));
    return enc.plan(std::vector<EntityGroupFrame>{group(0, std::move(entities))}, {}, 0);
}

}  // namespace

TEST_CASE("snapshot budget: entity changes fill what the core part leaves and never exceed the budget") {
    const auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::vector<std::byte> scratch;
    const SnapshotSizer sizeOf = [&](const wire::MatchSnapshot& s) {
        scratch.clear();
        REQUIRE(codec->encode(s, scratch).ok());
        return scratch.size();
    };
    const EntityPlan plan = manyChanges(600);
    for (const std::size_t budget : {800u, 1200u, 1800u}) {
        wire::MatchSnapshot s = coreSnapshot(6, 4);
        EntitySizeEstimate estimate;
        const SnapshotFit fit = fitSnapshot(s, plan, budget, sizeOf, estimate);
        CAPTURE(budget);
        CHECK_FALSE(fit.coreOverBudget);
        CHECK(fit.encodedBytes <= budget);
        CHECK(fit.encodedBytes == sizeOf(s));
        CHECK(fit.entityChanges > 0);
        CHECK(fit.entityChanges < plan.changes.size());
        CHECK(s.robots.size() == 6);
        // The prefix is maximal up to the estimate's resolution: ten more changes do not fit.
        wire::MatchSnapshot more = s;
        more.entityDeltas = EntityDeltaEncoder::deltas(plan, fit.entityChanges + 10);
        CHECK(sizeOf(more) > budget);
    }
    wire::MatchSnapshot all = coreSnapshot(2, 0);
    EntitySizeEstimate estimate;
    CHECK(fitSnapshot(all, manyChanges(5), 1200, sizeOf, estimate).entityChanges == 5);
}

TEST_CASE("snapshot budget: an oversized core part drops only the HUD, never robots or info") {
    const auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::vector<std::byte> scratch;
    const SnapshotSizer sizeOf = [&](const wire::MatchSnapshot& s) {
        scratch.clear();
        REQUIRE(codec->encode(s, scratch).ok());
        return scratch.size();
    };
    wire::MatchSnapshot s = coreSnapshot(6, 8);
    s.info.state = wire::MatchState::Running;
    s.info.matchTimeS = 77.f;
    EntitySizeEstimate estimate;
    const SnapshotFit fit = fitSnapshot(s, manyChanges(50), 300, sizeOf, estimate);
    CHECK(fit.coreOverBudget);
    CHECK(fit.hudDropped);
    CHECK(fit.entityChanges == 0);
    CHECK(s.robots.size() == 6);
    CHECK(s.info.matchTimeS == 77.f);
    for (const wire::RobotWireState& r : s.robots) {
        CHECK(r.hud.empty());
        CHECK(r.wheels.size() == 4);
    }
    CHECK(fit.encodedBytes == sizeOf(s));
}

TEST_CASE("snapshot budget: the size is the encoded size, so fields added to a robot are accounted for") {
    const auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::vector<std::byte> scratch;
    const EntityPlan plan = manyChanges(400);
    const auto run = [&](std::size_t extraBytesPerRobot) {
        const SnapshotSizer sizeOf = [&](const wire::MatchSnapshot& s) {
            scratch.clear();
            REQUIRE(codec->encode(s, scratch).ok());
            return scratch.size() + extraBytesPerRobot * s.robots.size();   // what a new fixed-size field would add
        };
        wire::MatchSnapshot s = coreSnapshot(6, 4);
        EntitySizeEstimate estimate;
        const SnapshotFit fit = fitSnapshot(s, plan, 1200, sizeOf, estimate);
        CHECK(fit.encodedBytes <= 1200);
        return fit.entityChanges;
    };
    const std::size_t without = run(0);
    const std::size_t with = run(4 * 13);   // four zone poses of about 13 bytes
    CHECK(with < without);
}