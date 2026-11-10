#include <doctest/doctest.h>

#include <cstdio>
#include <vector>

#include "match_samples.h"
#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::match;
using namespace wpiemu::sdk::proto;

// Snapshot size against kSnapshotBudgetBytes. The budget is the payload the host aims for per client datagram; the wire
// limits (caps) are far above it, so the host trims entity deltas to fit. These tests measure the codec, they do not
// raise the budget.
namespace {

std::size_t encodedSize(const MatchSnapshot& s) {
    const BinaryMessageCodec codec;
    std::vector<std::byte> out;
    const Status status = codec.encode(Message{s}, out);
    REQUIRE_MESSAGE(status.ok(), (status.ok() ? "" : status.error().message));
    const Result<Message> back = codec.decode(out);
    REQUIRE(back.ok());
    CHECK(std::get<MatchSnapshot>(back.value()) == s);
    return out.size();
}

MatchSnapshot snapshotOf(std::size_t robots, std::size_t wheels, std::size_t hud, std::size_t groups,
                         std::size_t removedPerGroup, std::size_t upsertedPerGroup, std::size_t gameHud,
                         std::size_t phaseKeyChars, std::size_t movedZones = 0, std::size_t firstRobotWithMovedZones = 0) {
    MatchSnapshot s = test_ipc::sampleMatchSnapshot();
    s.info.phaseKey.assign(phaseKeyChars, 'p');
    s.robots.clear();
    for (std::size_t i = 0; i < robots; ++i) {
        RobotWireState r;
        r.participantId = static_cast<ParticipantId>(i);
        r.xMm = 10000 + static_cast<std::int32_t>(i);
        r.yMm = -5000;
        r.yawQ = 1000;
        r.vxMmps = 1500;
        r.vyMmps = -700;
        r.omegaMradps = 300;
        r.wheels.assign(wheels, WheelWireState{100, -200});
        r.zoneFlags = 0x3;
        r.hud.assign(hud, HudValue{1, 12.f});
        if (i >= firstRobotWithMovedZones)
            for (std::size_t z = 0; z < movedZones; ++z)
                r.zonePoses.push_back({static_cast<std::uint8_t>(z), 700, -350, 900, 8192, -4096, 1000});
        s.robots.push_back(std::move(r));
    }
    s.entityDeltas.clear();
    for (std::size_t g = 0; g < groups; ++g) {
        EntityDelta d;
        d.groupId = static_cast<std::uint16_t>(g);
        d.baselineId = 4320;
        d.removed.assign(removedPerGroup, 7u);
        d.upserted.assign(upsertedPerGroup, EntityState{1, 100, -200, 5});
        s.entityDeltas.push_back(std::move(d));
    }
    s.gameHud.assign(gameHud, HudValue{3, 1.5f});
    return s;
}

}  // namespace

TEST_CASE("match snapshot size: the typical snapshot fits the budget") {
    // 6 robots x 4 wheels, 4 HUD values per robot, 40 entity upserts in one group, nothing else.
    const MatchSnapshot minimal = snapshotOf(6, 4, 4, 1, 0, 40, 0, 0);
    // The same with a phase key and a few game HUD values, as a real match sends.
    const MatchSnapshot typical = snapshotOf(6, 4, 4, 1, 0, 40, 4, 12);
    const std::size_t minimalBytes = encodedSize(minimal);
    const std::size_t typicalBytes = encodedSize(typical);
    std::printf("[snapshot size] 6x4 wheels, 4 hud, 40 upserts: %zu bytes (with 4 game hud and a phase key: %zu), budget %zu\n",
                minimalBytes, typicalBytes, kSnapshotBudgetBytes);
    CHECK(minimalBytes <= kSnapshotBudgetBytes);
    CHECK(typicalBytes <= kSnapshotBudgetBytes);
    CHECK(fitsLimits(typical));
}

TEST_CASE("match snapshot size: the worst case encodes and round-trips, far above the budget the host trims to") {
    const MatchSnapshot worst = snapshotOf(kMaxRobotsPerMatch, kMaxWheelsPerRobot, kMaxHudValuesPerRobot, kMaxEntityGroups,
                                           kMaxEntityDeltaEntries, kMaxEntityDeltaEntries, kMaxGameHudValues, kMaxNameBytes);
    REQUIRE(fitsLimits(worst));
    const std::size_t bytes = encodedSize(worst);
    std::printf("[snapshot size] worst case (6x8 wheels, 8 hud, 8 groups x 1024+1024 entries): %zu bytes\n", bytes);
    CHECK(bytes > kSnapshotBudgetBytes);
    CHECK(bytes <= kDefaultMaxFrameBytes);

    // Without entity deltas the robots alone (6 x 8 wheels, 8 hud values) still fit the budget.
    const MatchSnapshot robotsOnly = snapshotOf(kMaxRobotsPerMatch, kMaxWheelsPerRobot, kMaxHudValuesPerRobot, 0, 0, 0,
                                                kMaxGameHudValues, kMaxNameBytes);
    const std::size_t robotsOnlyBytes = encodedSize(robotsOnly);
    std::printf("[snapshot size] worst robots, no entities: %zu bytes\n", robotsOnlyBytes);
    CHECK(robotsOnlyBytes <= kSnapshotBudgetBytes);

    // The same with the most moved zones every robot may report: the host trims entity deltas, never robots or poses.
    const MatchSnapshot withPoses = snapshotOf(kMaxRobotsPerMatch, kMaxWheelsPerRobot, kMaxHudValuesPerRobot, 0, 0, 0,
                                               kMaxGameHudValues, kMaxNameBytes, kMaxSnapshotZonePosesPerRobot);
    REQUIRE(fitsLimits(withPoses));
    const std::size_t withPosesBytes = encodedSize(withPoses);
    std::printf("[snapshot size] worst robots + %zu moved zones each, no entities: %zu bytes (budget %zu)\n",
                kMaxSnapshotZonePosesPerRobot, withPosesBytes, kSnapshotBudgetBytes);
    CHECK(withPosesBytes <= kSnapshotBudgetBytes);
    CHECK(withPosesBytes == robotsOnlyBytes + kMaxRobotsPerMatch * kMaxSnapshotZonePosesPerRobot * kWireZonePoseBytes);
}

TEST_CASE("match snapshot size: moved zones of the remote robots keep the typical snapshot within the budget") {
    // 6 robots x 4 wheels, 4 HUD values, 40 entity upserts (the typical snapshot) plus 2 moved zones on each of the 5 remote
    // robots (the host robot is robot 0).
    const MatchSnapshot plain = snapshotOf(6, 4, 4, 1, 0, 40, 4, 12);
    const MatchSnapshot moved = snapshotOf(6, 4, 4, 1, 0, 40, 4, 12, 2, 1);
    const MatchSnapshot movedMinimal = snapshotOf(6, 4, 4, 1, 0, 40, 0, 0, 2, 1);
    const std::size_t plainBytes = encodedSize(plain);
    const std::size_t movedBytes = encodedSize(moved);
    std::printf("[snapshot size] typical %zu bytes; with 2 moved zones on each of 5 remote robots %zu bytes (+%zu; minimal "
                "%zu), budget %zu\n",
                plainBytes, movedBytes, movedBytes - plainBytes, encodedSize(movedMinimal), kSnapshotBudgetBytes);
    CHECK(movedBytes == plainBytes + 5 * 2 * kWireZonePoseBytes);
    CHECK(movedBytes <= kSnapshotBudgetBytes);
}

TEST_CASE("match snapshot size: a zone pose costs 13 bytes and an empty list one byte per robot") {
    CHECK(kWireZonePoseBytes == 13);
    const std::size_t none = encodedSize(snapshotOf(3, 4, 2, 1, 0, 5, 0, 0));
    CHECK(encodedSize(snapshotOf(3, 4, 2, 1, 0, 5, 0, 0, 1)) == none + 3 * 13);
    CHECK(encodedSize(snapshotOf(3, 4, 2, 1, 0, 5, 0, 0, 4)) == none + 3 * 4 * 13);
}

TEST_CASE("match snapshot size: each entity upsert costs 10 bytes and each wheel 4, so the host can budget by counting") {
    const std::size_t base = encodedSize(snapshotOf(2, 4, 2, 1, 0, 0, 0, 0));
    CHECK(encodedSize(snapshotOf(2, 4, 2, 1, 0, 10, 0, 0)) == base + 10 * 10);
    CHECK(encodedSize(snapshotOf(2, 6, 2, 1, 0, 0, 0, 0)) == base + 2 * 2 * 4);
    CHECK(encodedSize(snapshotOf(2, 4, 2, 1, 5, 0, 0, 0)) == base + 5 * 4);
}
