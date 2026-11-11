#include <cstdio>

#include "match_test_support.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/match/robot_appearance.h"
#include "wpiemu/sdk/testing/fake_game_module.h"

using namespace match_test;

namespace {

ZoneConfig zoneAt(std::size_t index) {
    ZoneConfig z;
    z.id = "zone" + std::to_string(index);
    z.kind = "fake_game.fake_zone";
    z.local = Pose3{Vec3{0.1f + 0.05f * static_cast<float>(index), -0.1f * static_cast<float>(index), 0.1f}, Quat::fromYaw(0.1f * static_cast<float>(index))};
    z.sizeM = {0.2f, 0.3f, 0.1f};
    return z;
}

std::vector<ZoneConfig> zonesOf(std::size_t count) {
    std::vector<ZoneConfig> zones;
    for (std::size_t i = 0; i < count; ++i) zones.push_back(zoneAt(i));
    return zones;
}

RobotConfig zonedSpec(std::size_t zones, const char* name = "Zoned") {
    RobotConfig c = swerveSpec(name);
    c.gamePluginId = "fake_game";
    c.zones = zonesOf(zones);
    return c;
}

void addFakeGame(Harness& h) { REQUIRE(h.kit.plugins.add(std::make_unique<testing::FakeGameModule>("fake_game", 2999)).ok()); }

// Where a client moved zone `index` to: well away from any rest pose.
Pose3 movedPose(std::size_t index) {
    return Pose3{Vec3{0.3f + 0.01f * static_cast<float>(index), 0.2f, 0.4f}, Quat::fromYaw(0.5f + 0.1f * static_cast<float>(index))};
}

ZoneState zoneState(std::size_t index, bool active, std::optional<Pose3> pose) {
    ZoneState z;
    z.id = "zone" + std::to_string(index);
    z.active = active;
    z.poseInChassis = pose;
    return z;
}

// The zones of one robot: zone i is moved when `moved` has bit i; every zone is active.
std::vector<ZoneState> zoneStates(const std::vector<Pose3>& rest, std::uint32_t moved, bool setRestPose) {
    std::vector<ZoneState> out;
    for (std::size_t i = 0; i < rest.size(); ++i) {
        std::optional<Pose3> pose;
        if (((moved >> i) & 1u) != 0) pose = movedPose(i);
        else if (setRestPose) pose = rest[i];
        out.push_back(zoneState(i, true, pose));
    }
    return out;
}

// Plays ticks and collects the snapshots a client received.
struct Rig {
    FakeSession& s;
    Harness& h;
    TestClient& c;
    std::size_t consumed = 0;

    std::vector<wire::MatchSnapshot> run(int ticks) {
        std::vector<wire::MatchSnapshot> out;
        for (int i = 0; i < ticks; ++i) {
            s.advance();
            h.clock.advance(5ms);
            h.tickEnd();
            REQUIRE(h.host->waitOutboundIdle(3s));
            c.drain();
            h.poll();
        }
        c.drain();
        for (; consumed < c.log.size(); ++consumed)
            if (const auto* snap = std::get_if<wire::MatchSnapshot>(&c.log[consumed])) out.push_back(*snap);
        return out;
    }
};

const wire::RobotWireState* robotOf(const wire::MatchSnapshot& s, wire::ParticipantId id) {
    for (const wire::RobotWireState& r : s.robots)
        if (r.participantId == id) return &r;
    return nullptr;
}

std::size_t encodedSize(Harness& h, const wire::MatchSnapshot& snap) {
    std::vector<std::byte> bytes;
    REQUIRE(h.codec->encode(snap, bytes).ok());
    return bytes.size();
}

testing::ImpairmentProfile impaired() {
    testing::ImpairmentProfile p;   // no fixed delay: the join handshake runs without the manual clock advancing
    p.lossProbability = 0.25;
    p.reorderProbability = 0.2;
    p.duplicateProbability = 0.1;
    return p;
}

}  // namespace

TEST_CASE("match zone poses: no moved zone leaves the tails empty and the snapshot size unchanged") {
    FakeSession s;
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, zonesOf(2));
    Harness h(s);
    addFakeGame(h);
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("Zoned", zonedSpec(3), s)));
    const std::vector<Pose3> remoteRest = wire::appearanceZoneRestPoses(c.accept->roster[1].appearance);
    const std::vector<Pose3> hostRest = wire::appearanceZoneRestPoses(s.hostLook);
    REQUIRE(remoteRest.size() == 3);
    REQUIRE(hostRest.size() == 2);
    Rig rig{s, h, c};

    // Poses unset (fall back to rest), then the rest poses themselves: both are "not moved".
    std::vector<std::size_t> sizes;
    for (const bool setRest : {false, true}) {
        s.robots.at(1).zones = zoneStates(remoteRest, 0, setRest);
        s.hostZones = zoneStates(hostRest, 0, setRest);
        const std::vector<wire::MatchSnapshot> snaps = rig.run(40);
        REQUIRE_FALSE(snaps.empty());
        for (const wire::MatchSnapshot& snap : snaps) {
            REQUIRE(snap.robots.size() == 2);
            for (const wire::RobotWireState& r : snap.robots) CHECK(r.zonePoses.empty());
        }
        wire::MatchSnapshot probe = snaps.back();
        probe.snapshotSeq = 1;
        probe.tick = 1;
        sizes.push_back(encodedSize(h, probe));
    }

    // The same session without any zone poses (a runtime that does not report them) encodes to the same size.
    s.robots.at(1).zones = zoneStates(remoteRest, 0, false);
    s.hostZones = zoneStates(hostRest, 0, false);
    CHECK(sizes[0] == sizes[1]);
    const std::vector<wire::MatchSnapshot> again = rig.run(40);
    REQUIRE_FALSE(again.empty());
    wire::MatchSnapshot probe = again.back();
    probe.snapshotSeq = 1;
    probe.tick = 1;
    CHECK(encodedSize(h, probe) == sizes[0]);
}

TEST_CASE("match zone poses: moved zones of a remote robot and of the host reach a client intact over a lossy link") {
    FakeSession s;
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, zonesOf(3));
    Harness h(s);
    addFakeGame(h);
    TestClient c(h, impaired(), 11);
    REQUIRE(c.join(c.joinMessage("Zoned", zonedSpec(3), s)));
    const std::vector<Pose3> remoteRest = wire::appearanceZoneRestPoses(c.accept->roster[1].appearance);
    const std::vector<Pose3> hostRest = wire::appearanceZoneRestPoses(s.hostLook);
    Rig rig{s, h, c};

    // Remote zones 0 and 2 are moved; the host's zone 1.
    s.robots.at(1).zones = zoneStates(remoteRest, 0b101, true);
    s.hostZones = zoneStates(hostRest, 0b010, true);
    s.hostZones[1].active = false;
    const std::vector<wire::MatchSnapshot> moved = rig.run(80);
    REQUIRE(moved.size() >= 3);
    for (const wire::MatchSnapshot& snap : moved) {
        const wire::RobotWireState* remote = robotOf(snap, 1);
        const wire::RobotWireState* host = robotOf(snap, wire::kHostParticipantId);
        REQUIRE(remote != nullptr);
        REQUIRE(host != nullptr);
        REQUIRE(remote->zonePoses.size() == 2);
        CHECK(remote->zonePoses[0] == wire::makeZonePoseInput(0, movedPose(0)));
        CHECK(remote->zonePoses[1] == wire::makeZonePoseInput(2, movedPose(2)));
        REQUIRE(host->zonePoses.size() == 1);
        CHECK(host->zonePoses[0] == wire::makeZonePoseInput(1, movedPose(1)));
        // What the client rebuilds is the commanded pose to the wire resolution.
        CHECK(length(wire::toPose3(remote->zonePoses[1]).position - movedPose(2).position) < 0.001f);
        CHECK(remote->zoneFlags == 0b111);
        CHECK(host->zoneFlags == 0b101);
    }

    // Zone 0 goes back to rest, zone 1 starts moving: the list follows, nothing stale remains.
    s.robots.at(1).zones = zoneStates(remoteRest, 0b110, true);
    rig.run(20);   // anything still in flight from before
    const std::vector<wire::MatchSnapshot> later = rig.run(80);
    REQUIRE_FALSE(later.empty());
    for (const wire::MatchSnapshot& snap : later) {
        const wire::RobotWireState* remote = robotOf(snap, 1);
        REQUIRE(remote != nullptr);
        REQUIRE(remote->zonePoses.size() == 2);
        CHECK(remote->zonePoses[0].zoneIndex == 1);
        CHECK(remote->zonePoses[1].zoneIndex == 2);
    }

    // All back at rest.
    s.robots.at(1).zones = zoneStates(remoteRest, 0, true);
    s.hostZones = zoneStates(hostRest, 0, true);
    rig.run(20);
    const std::vector<wire::MatchSnapshot> rest = rig.run(80);
    REQUIRE_FALSE(rest.empty());
    for (const wire::MatchSnapshot& snap : rest)
        for (const wire::RobotWireState& r : snap.robots) CHECK(r.zonePoses.empty());
}

TEST_CASE("match zone poses: rest poses are the roster appearance poses, so a zone on a mechanism is moved only when it moves") {
    FakeSession s;
    ZoneConfig onMechanism = zoneAt(0);
    onMechanism.attachment.parentMechanismId = "arm";
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, std::vector<ZoneConfig>{onMechanism, zoneAt(1)});
    const std::vector<Pose3> rest = wire::appearanceZoneRestPoses(s.hostLook);
    REQUIRE(rest.size() == 2);
    CHECK(wire::makeZonePoseInput(0, rest[0]) == wire::makeZonePoseInput(0, remoteZoneSpecPose(onMechanism)));
    Harness h(s);
    TestClient c(h, impaired(), 5);
    REQUIRE(c.join(c.joinMessage("A", swerveSpec(), s)));
    Rig rig{s, h, c};

    s.hostZones = zoneStates(rest, 0, true);
    for (const wire::MatchSnapshot& snap : rig.run(60)) CHECK(robotOf(snap, wire::kHostParticipantId)->zonePoses.empty());

    s.hostZones[0].poseInChassis = Pose3{rest[0].position + Vec3{0.f, 0.f, 0.25f}, rest[0].rotation};
    const std::vector<wire::MatchSnapshot> snaps = rig.run(60);
    REQUIRE(snaps.size() > 2);
    const wire::RobotWireState* host = robotOf(snaps.back(), wire::kHostParticipantId);
    REQUIRE(host->zonePoses.size() == 1);
    CHECK(host->zonePoses[0].zoneIndex == 0);
    CHECK(host->zonePoses[0].zMm == wire::makeZonePoseInput(0, rest[0]).zMm + 250);
}

TEST_CASE("match zone poses: more than four moved zones keep the active ones first, lowest index next") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h);
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("Many", zonedSpec(8), s)));
    const std::vector<Pose3> rest = wire::appearanceZoneRestPoses(c.accept->roster[1].appearance);
    REQUIRE(rest.size() == 8);
    s.robots.at(1).zones = zoneStates(rest, 0xFF, true);
    for (std::size_t i = 0; i < 8; ++i) s.robots.at(1).zones[i].active = (i == 5 || i == 6 || i == 2);   // 3 active, 5 inactive
    Rig rig{s, h, c};
    const std::vector<wire::MatchSnapshot> snaps = rig.run(40);
    REQUIRE_FALSE(snaps.empty());
    const wire::RobotWireState* r = robotOf(snaps.back(), 1);
    REQUIRE(r->zonePoses.size() == wire::kMaxSnapshotZonePosesPerRobot);
    std::vector<int> indices;
    for (const wire::ZonePoseInput& z : r->zonePoses) indices.push_back(z.zoneIndex);
    CHECK(indices == std::vector<int>{0, 2, 5, 6});   // the three active zones and the lowest inactive one
}

TEST_CASE("match zone poses: a typical match with moved zones stays within the snapshot budget") {
    FakeSession s;
    std::vector<std::string> keys;
    for (int i = 0; i < 4; ++i) keys.push_back("fake.key" + std::to_string(i));
    for (const std::string& k : keys) {
        HudWidgetDescriptor d;
        d.id = k;
        d.valueKey = k;
        s.descriptors.push_back(d);
        s.robotHud.push_back({k, Value(1234.5678)});
    }
    EntityGroupSample g;
    g.group = "fuel";
    for (std::uint32_t i = 0; i < 40; ++i) {
        g.ids.push_back(i + 1);
        g.positions.push_back({0.2f * static_cast<float>(i), 1.f, 0.1f});
    }
    s.groups.push_back(g);
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, zonesOf(2));
    Harness h(s);
    addFakeGame(h);
    std::vector<std::unique_ptr<TestClient>> clients;
    for (std::size_t i = 0; i + 1 < wire::kMaxRobotsPerMatch; ++i) {
        clients.push_back(std::make_unique<TestClient>(h));
        REQUIRE(clients.back()->join(clients.back()->joinMessage("P", zonedSpec(2), s)));
    }
    const std::vector<Pose3> rest = wire::appearanceZoneRestPoses(clients[0]->accept->roster[1].appearance);
    for (auto& [id, r] : s.robots) r.zones = zoneStates(rest, 0b11, true);
    s.hostZones = zoneStates(wire::appearanceZoneRestPoses(s.hostLook), 0b11, true);
    Rig rig{s, h, *clients[0]};
    const std::vector<wire::MatchSnapshot> snaps = rig.run(40);
    REQUIRE_FALSE(snaps.empty());
    const wire::MatchSnapshot& first = snaps.front();
    REQUIRE(first.robots.size() == 6);
    for (const wire::RobotWireState& r : first.robots) CHECK(r.zonePoses.size() == 2);
    std::size_t upserts = 0;
    for (const wire::EntityDelta& d : first.entityDeltas) upserts += d.upserted.size();
    CHECK(upserts == 40);   // the full entity state still fits next to 12 moved zone poses
    const std::size_t bytes = encodedSize(h, first);
    CHECK(bytes <= wire::kSnapshotBudgetBytes);
    CHECK(h.host->stats().snapshotsTruncated == 0);
    std::printf("[match zone poses] typical: 6 robots x 4 wheels, 4 HUD values, 40 entities, 2 moved zones per robot: %zu of %zu bytes\n", bytes,
                wire::kSnapshotBudgetBytes);
}

TEST_CASE("match zone poses: the worst case keeps robots info and zone poses, entity deltas are trimmed first") {
    FakeSession s;
    std::vector<std::string> keys;
    for (int i = 0; i < 40; ++i) keys.push_back("fake.key" + std::to_string(i));
    for (const std::string& k : keys) {
        HudWidgetDescriptor d;
        d.id = k;
        d.valueKey = k;
        s.descriptors.push_back(d);
        s.hud.push_back({k, Value(1234.5678)});
    }
    for (int i = 0; i < 8; ++i) s.robotHud.push_back({keys[static_cast<std::size_t>(i)], Value(1234.5678)});
    EntityGroupSample g;
    g.group = "fuel";
    for (std::uint32_t i = 0; i < 800; ++i) {
        g.ids.push_back(i + 1);
        g.positions.push_back({0.1f * static_cast<float>(i % 50), 0.1f * static_cast<float>(i / 50), 0.1f});
    }
    s.groups.push_back(g);
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, zonesOf(4));
    RobotConfig big = zonedSpec(4);
    for (int i = 0; i < 4; ++i) big.drivetrain.spec.wheels.push_back(makeWheel(("extra" + std::to_string(i)).c_str(), 0.1f * static_cast<float>(i), 0.f));
    s.hostWheels.assign(8, WheelSample{});
    Harness h(s);
    addFakeGame(h);
    std::vector<std::unique_ptr<TestClient>> clients;
    for (std::size_t i = 0; i + 1 < wire::kMaxRobotsPerMatch; ++i) {
        clients.push_back(std::make_unique<TestClient>(h));
        REQUIRE(clients.back()->join(clients.back()->joinMessage("P", big, s)));
    }
    s.hostBody.pose.position = {-300.f, 300.f, 0.f};
    const std::vector<Pose3> rest = wire::appearanceZoneRestPoses(clients[0]->accept->roster[1].appearance);
    for (auto& [id, r] : s.robots) {
        r.pose.position = {200.f * static_cast<float>(id), -250.f, 0.f};
        r.zones = zoneStates(rest, 0b1111, true);
    }
    s.hostZones = zoneStates(wire::appearanceZoneRestPoses(s.hostLook), 0b1111, true);
    Rig rig{s, h, *clients[0]};
    const std::vector<wire::MatchSnapshot> snaps = rig.run(40);
    REQUIRE_FALSE(snaps.empty());
    for (const wire::MatchSnapshot& snap : snaps) {
        REQUIRE(snap.robots.size() == 6);
        for (const wire::RobotWireState& r : snap.robots) {
            CHECK(r.zonePoses.size() == wire::kMaxSnapshotZonePosesPerRobot);
            CHECK(r.wheels.size() == (r.participantId == wire::kHostParticipantId ? 8u : big.drivetrain.spec.wheels.size()));
        }
        CHECK(snap.robots[1].hud.size() == wire::kMaxHudValuesPerRobot);
        CHECK(snap.gameHud.size() == wire::kMaxGameHudValues);
        CHECK(encodedSize(h, snap) <= wire::kSnapshotBudgetBytes);
    }
    CHECK(h.host->stats().snapshotsOverBudget == 0);
    CHECK(h.host->stats().snapshotsTruncated > 0);
    CHECK(h.host->stats().entityChangesDeferred > 0);
    CHECK(h.host->stats().maxSnapshotBytes <= wire::kSnapshotBudgetBytes);
    std::printf("[match zone poses] worst case: 6 robots x 4 zone poses, full HUD, 800 entities: largest %zu of %zu bytes, truncated %llu of %llu\n",
                h.host->stats().maxSnapshotBytes, wire::kSnapshotBudgetBytes, static_cast<unsigned long long>(h.host->stats().snapshotsTruncated),
                static_cast<unsigned long long>(h.host->stats().snapshotsSent));
}

TEST_CASE("match zone poses: the encoded snapshots of the same run are identical") {
    const auto run = [] {
        FakeSession s;
        s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, {}, {}, zonesOf(2));
        Harness h(s, {}, 3);
        addFakeGame(h);
        TestClient c(h, impaired(), 9);
        REQUIRE(c.join(c.joinMessage("Zoned", zonedSpec(3), s)));
        const std::vector<Pose3> rest = wire::appearanceZoneRestPoses(c.accept->roster[1].appearance);
        s.robots.at(1).zones = zoneStates(rest, 0b110, true);
        s.hostZones = zoneStates(wire::appearanceZoneRestPoses(s.hostLook), 0b01, true);
        Rig rig{s, h, c};
        std::uint64_t hash = 1469598103934665603ull;
        std::size_t count = 0;
        for (const wire::MatchSnapshot& snap : rig.run(120)) {
            std::vector<std::byte> bytes;
            REQUIRE(h.codec->encode(snap, bytes).ok());
            for (const std::byte b : bytes) hash = (hash ^ std::to_integer<std::uint64_t>(b)) * 1099511628211ull;
            ++count;
        }
        return std::pair{hash, count};
    };
    const auto first = run();
    CHECK(first.second > 5);
    CHECK(first == run());
}
