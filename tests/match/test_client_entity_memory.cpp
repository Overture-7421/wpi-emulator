#include <doctest/doctest.h>

#include "wpiemu/core/match/entity_memory.h"

using wpiemu::core::match::EntityMemory;
namespace wire = wpiemu::sdk::match;

namespace {

wire::MatchSnapshot snapshot(std::uint32_t seq, std::int64_t simUs, std::uint32_t baseline, std::vector<wire::EntityState> upserts,
                             std::vector<std::uint32_t> removed = {}) {
    wire::MatchSnapshot s;
    s.snapshotSeq = seq;
    s.simTimeUs = simUs;
    wire::EntityDelta d;
    d.groupId = 3;
    d.baselineId = baseline;
    d.upserted = std::move(upserts);
    d.removed = std::move(removed);
    s.entityDeltas.push_back(std::move(d));
    return s;
}

wire::EntityState entity(std::uint32_t id, int xCm) {
    wire::EntityState e;
    e.id = id;
    e.xCm = static_cast<std::int16_t>(xCm);
    return e;
}

}  // namespace

TEST_CASE("client entity memory: the picture of every recent snapshot is kept and found by its sim time") {
    EntityMemory memory;
    REQUIRE(memory.apply(snapshot(1, 100'000, wire::kNoBaseline, {entity(1, 10), entity(2, 20)})) == EntityMemory::Result::Applied);
    REQUIRE(memory.apply(snapshot(2, 200'000, 1, {entity(1, 15)}, {2})) == EntityMemory::Result::Applied);
    REQUIRE(memory.apply(snapshot(3, 300'000, 2, {entity(3, 30)})) == EntityMemory::Result::Applied);

    CHECK(memory.group(3).size() == 2);           // the newest picture: entities 1 and 3 (2 was removed)
    CHECK(memory.groupOf(1, 3).size() == 2);      // the first one still has 1 and 2
    CHECK(memory.groupIds(2) == std::vector<std::uint16_t>{3});
    CHECK(memory.groupOf(2, 3).at(1).state.xCm == 15);

    CHECK(*memory.newestAtOrBefore(250'000) == 2);
    CHECK(*memory.newestAtOrBefore(300'000) == 3);
    CHECK(*memory.newestAtOrBefore(10'000'000) == 3);
    CHECK(*memory.newestAtOrBefore(0) == 1);      // all are newer: the oldest held
    CHECK(*memory.previousOf(3) == 2);
    CHECK(*memory.previousOf(2) == 1);
    CHECK_FALSE(memory.previousOf(1).has_value());
    CHECK(memory.timeOf(2) == 200'000);
}

TEST_CASE("client entity memory: stale snapshots and unknown baselines change nothing, clear forgets everything") {
    EntityMemory memory;
    REQUIRE(memory.apply(snapshot(5, 500'000, wire::kNoBaseline, {entity(1, 10)})) == EntityMemory::Result::Applied);
    CHECK(memory.apply(snapshot(5, 500'000, wire::kNoBaseline, {entity(1, 99)})) == EntityMemory::Result::Stale);
    CHECK(memory.apply(snapshot(4, 400'000, wire::kNoBaseline, {entity(1, 99)})) == EntityMemory::Result::Stale);
    CHECK(memory.apply(snapshot(6, 600'000, 77, {entity(1, 99)})) == EntityMemory::Result::UnknownBaseline);
    CHECK(memory.latest() == 5);
    CHECK(memory.group(3).at(1).state.xCm == 10);

    memory.clear();
    CHECK_FALSE(memory.seen());
    CHECK(memory.group(3).empty());
    CHECK_FALSE(memory.newestAtOrBefore(0).has_value());
    // After a clear the same sequence numbers apply again.
    CHECK(memory.apply(snapshot(5, 500'000, wire::kNoBaseline, {entity(1, 10)})) == EntityMemory::Result::Applied);
}

TEST_CASE("client entity memory: only the newest kHistory snapshots are held") {
    EntityMemory memory;
    for (std::uint32_t seq = 1; seq <= EntityMemory::kHistory + 5; ++seq)
        REQUIRE(memory.apply(snapshot(seq, static_cast<std::int64_t>(seq) * 10'000, wire::kNoBaseline, {entity(1, 1)})) == EntityMemory::Result::Applied);
    CHECK(memory.groupOf(1, 3).empty());
    CHECK(memory.groupOf(6, 3).size() == 1);
    CHECK(*memory.newestAtOrBefore(0) == 6);   // the oldest one still held
}
