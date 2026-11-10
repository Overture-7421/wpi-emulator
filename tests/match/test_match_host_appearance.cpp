#include "match_test_support.h"

#include "wpiemu/core/match/robot_appearance.h"
#include "wpiemu/core/match/spec_intake.h"
#include "wpiemu/sdk/game/zone_pose.h"
#include "wpiemu/sdk/match/robot_appearance.h"
#include "wpiemu/sdk/testing/fake_game_module.h"

using namespace match_test;

namespace {

ZoneConfig zoneAt(const char* id, Vec3 position, Vec3 size) {
    ZoneConfig z;
    z.id = id;
    z.kind = "fake_game.fake_zone";
    z.local = Pose3{position, Quat::fromEuler(0.f, 0.2f, 0.3f)};
    z.sizeM = size;
    return z;
}

RobotConfig specWithZones() {
    RobotConfig c = swerveSpec("Zoned");
    c.gamePluginId = "fake_game";
    c.zones = {zoneAt("intake", {0.4f, 0.f, 0.1f}, {0.3f, 0.6f, 0.15f}), zoneAt("far", {9.f, -9.f, 9.f}, {0.2f, 0.2f, 0.2f})};
    return c;
}

const wire::RosterEntry* entryOf(const std::vector<wire::RosterEntry>& entries, wire::ParticipantId id) {
    for (const wire::RosterEntry& e : entries)
        if (e.participantId == id) return &e;
    return nullptr;
}

// The pipeline knows the "fake_game" plugin, whose zone kind is "fake_game.fake_zone" (a spec with zones needs a plugin).
void addFakeGame(PipelineKit& kit) { REQUIRE(kit.plugins.add(std::make_unique<testing::FakeGameModule>("fake_game", 2999)).ok()); }

}  // namespace

TEST_CASE("match host: JoinAccept describes every robot, built from the accepted spec") {
    FakeSession s;
    s.hostLook = wire::makeRobotAppearance(ChassisSpec{}, std::vector<WheelSpec>{WheelSpec{}}, std::vector<Vec3>{Vec3{0.2f, 0.2f, -0.1f}}, {});
    Harness h(s);
    TestClient alice(h);
    REQUIRE(alice.join(alice.joinMessage("Alice", swerveSpec(), s)));
    const wire::JoinAccept& acc = *alice.accept;
    REQUIRE(acc.roster.size() == 2);

    // The host entry is what the session reports.
    CHECK(acc.roster[0].appearance == s.hostLook);

    // The remote entry: the chassis frame box, four swerve wheels at their hubs, no zones.
    const wire::RobotAppearance& a = acc.roster[1].appearance;
    CHECK(appearanceWellFormed(a));
    CHECK(a.sizeXMm == 700);
    CHECK(a.sizeYMm == 700);
    CHECK(a.sizeZMm == 120);
    CHECK(a.comHeightMm == 200);
    CHECK(a.centreZMm == -110);  // clearance 0.03 + half of 0.12 - COM height 0.2: below the COM, above the floor
    REQUIRE(a.wheels.size() == 4);
    CHECK(a.wheels[0].kind == WheelKind::Swerve);
    CHECK(a.wheels[0].steered);
    CHECK(a.wheels[0].xMm == 250);
    CHECK(a.wheels[0].yMm == 250);
    CHECK(a.wheels[3].xMm == -250);
    CHECK(a.wheels[3].yMm == -250);
    const float radius = static_cast<float>(kDefaultWheelRadiusM);
    CHECK(a.wheels[0].radiusMm == static_cast<std::uint16_t>(std::lround(radius * 1000.f)));
    CHECK(a.wheels[0].zMm == static_cast<std::int16_t>(std::lround((radius - 0.2f) * 1000.f)));  // hub: one radius above the floor
    CHECK(a.zones.empty());
    // The same appearance reaches the other clients and later rosters.
    REQUIRE(alice.waitFor([&] { return alice.count<wire::Roster>() > 0; }));
    const wire::Roster roster = *alice.last<wire::Roster>();
    REQUIRE(entryOf(roster.entries, 1) != nullptr);
    CHECK(entryOf(roster.entries, 1)->appearance == a);
    REQUIRE(entryOf(roster.entries, wire::kHostParticipantId) != nullptr);
    CHECK(entryOf(roster.entries, wire::kHostParticipantId)->appearance == s.hostLook);
}

TEST_CASE("match host: the appearance is the clamped spec, never the raw client description") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h.kit);
    RobotConfig spec = specWithZones();
    spec.chassis.sizeM = {6.f, 0.7f, 0.12f};       // above the 1.5 m footprint limit
    spec.chassis.massKg = 5000.f;
    spec.drivetrain.spec.wheels[0].radiusM = 3.f;  // above the 0.15 m wheel radius limit
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("Big", spec, s)));
    REQUIRE(c.accept->roster.size() == 2);
    const wire::RobotAppearance& a = c.accept->roster[1].appearance;
    CHECK(a.sizeXMm == 1500);
    CHECK(a.wheels[0].radiusMm == 150);

    // The host simulates exactly what the roster says: the appearance equals the one built from the config the session got.
    REQUIRE(s.robots.count(1) == 1);
    CHECK(a == core::match::appearanceOf(s.robots.at(1).request.spec));
}

TEST_CASE("match host: zone volumes, kinds and rest poses come from the accepted zones, inside the zone envelope") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h.kit);
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("Zoned", specWithZones(), s)));
    REQUIRE(c.accept->roster.size() == 2);
    const wire::RobotAppearance& a = c.accept->roster[1].appearance;
    REQUIRE(a.zones.size() == 2);
    CHECK(a.zones[0].kind == "fake_game.fake_zone");
    CHECK(a.zones[0].pose.zoneIndex == 0);
    CHECK(a.zones[1].pose.zoneIndex == 1);
    CHECK(a.zones[0].sizeXMm == 300);
    CHECK(a.zones[0].sizeYMm == 600);
    CHECK(a.zones[0].sizeZMm == 150);

    // The robot the session simulates and the appearance agree on every zone's rest pose, to the wire resolution; the zone
    // that was far outside the envelope was moved in (by the intake, once) rather than saturated on the wire.
    REQUIRE(s.robots.count(1) == 1);
    const std::vector<ZoneConfig>& zones = s.robots.at(1).request.spec.zones;
    REQUIRE(zones.size() == 2);
    const std::vector<Pose3> clientRest = wire::appearanceZoneRestPoses(a);
    const ZoneEnvelope box = remoteZoneEnvelope(s.robots.at(1).request.spec.chassis);
    for (std::size_t i = 0; i < zones.size(); ++i) {
        CAPTURE(i);
        const Pose3 hostRest = remoteZoneSpecPose(zones[i]);
        CHECK(length(clientRest[i].position - hostRest.position) < 0.001f);
        CHECK(wire::makeZonePoseInput(i, clientRest[i]) == wire::makeZonePoseInput(i, hostRest));
        CHECK(hostRest.position.x <= box.max.x);
        CHECK(hostRest.position.y >= box.min.y);
        CHECK(hostRest.position.z <= box.max.z);
    }
    CHECK(zones[0].local.position.x == doctest::Approx(0.4f));  // inside: untouched
    CHECK(zones[1].local.position.x < 9.f);
    CHECK(wire::selectSnapshotZonePoses(clientRest, std::vector<Pose3>{remoteZoneSpecPose(zones[0]), remoteZoneSpecPose(zones[1])}, 0).empty());
}

TEST_CASE("match host: spec intake moves out-of-envelope zones in and reports them") {
    PipelineKit kit;
    addFakeGame(kit);
    const sdk::Result<AcceptedSpec> accepted = acceptRemoteSpec(specJson(specWithZones()), kit.pipeline);
    REQUIRE(accepted.ok());
    const AcceptedSpec& spec = accepted.value();
    const ZoneEnvelope box = remoteZoneEnvelope(spec.config.chassis);
    REQUIRE(spec.config.zones.size() == 2);
    CHECK(spec.config.zones[1].local.position.x == doctest::Approx(box.max.x));
    CHECK(spec.config.zones[1].local.position.y == doctest::Approx(box.min.y));
    CHECK(spec.config.zones[1].local.position.z == doctest::Approx(box.max.z));
    bool reported = false;
    for (const std::string& line : spec.clamped) reported = reported || line.find("zones[1].local") == 0;
    CHECK(reported);
    for (const std::string& line : spec.clamped) CHECK(line.find("zones[0]") == std::string::npos);

    RobotConfig inside = swerveSpec("In");
    inside.gamePluginId = "fake_game";
    inside.zones = {zoneAt("intake", {0.4f, 0.f, 0.1f}, {0.3f, 0.6f, 0.15f})};
    const sdk::Result<AcceptedSpec> ok = acceptRemoteSpec(specJson(inside), kit.pipeline);
    REQUIRE(ok.ok());
    CHECK(ok.value().clamped.empty());
    CHECK(ok.value().config.zones[0].local.position.x == doctest::Approx(0.4f));
}

TEST_CASE("match host: spec intake clamps zone sizes, reports them and leaves sizes in range alone") {
    PipelineKit kit;
    addFakeGame(kit);
    RobotConfig big = swerveSpec("Big");
    big.gamePluginId = "fake_game";
    big.zones = {zoneAt("huge", {0.4f, 0.f, 0.1f}, {80.f, 0.3f, 0.001f}), zoneAt("fine", {0.4f, 0.f, 0.1f}, {0.3f, 1.5f, 0.02f})};
    const sdk::Result<AcceptedSpec> accepted = acceptRemoteSpec(specJson(big), kit.pipeline);
    REQUIRE(accepted.ok());
    const AcceptedSpec& spec = accepted.value();
    REQUIRE(spec.config.zones.size() == 2);
    CHECK(spec.config.zones[0].sizeM.x == doctest::Approx(1.5f));
    CHECK(spec.config.zones[0].sizeM.y == doctest::Approx(0.3f));
    CHECK(spec.config.zones[0].sizeM.z == doctest::Approx(0.02f));
    CHECK(spec.config.zones[1].sizeM.x == doctest::Approx(0.3f));
    CHECK(spec.config.zones[1].sizeM.y == doctest::Approx(1.5f));
    CHECK(spec.config.zones[1].sizeM.z == doctest::Approx(0.02f));
    int sizeLines = 0;
    for (const std::string& line : spec.clamped) {
        if (line.find("zones[0].sizeM") == 0) ++sizeLines;
        CHECK(line.find("zones[1]") == std::string::npos);
        CHECK(line.find(".local") == std::string::npos);
    }
    CHECK(sizeLines == 1);
}

TEST_CASE("match host: the roster appearance carries the clamped zone size the session simulates") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h.kit);
    RobotConfig spec = swerveSpec("Huge");
    spec.gamePluginId = "fake_game";
    spec.zones = {zoneAt("intake", {0.4f, 0.f, 0.1f}, {30.f, 0.6f, 12.f})};
    TestClient c(h);
    REQUIRE(c.join(c.joinMessage("Huge", spec, s)));
    REQUIRE(c.accept->roster.size() == 2);
    const wire::RobotAppearance& a = c.accept->roster[1].appearance;
    REQUIRE(a.zones.size() == 1);
    CHECK(a.zones[0].sizeXMm == 1500);
    CHECK(a.zones[0].sizeYMm == 600);
    CHECK(a.zones[0].sizeZMm == 1500);

    REQUIRE(s.robots.count(1) == 1);
    const std::vector<ZoneConfig>& zones = s.robots.at(1).request.spec.zones;
    REQUIRE(zones.size() == 1);
    CHECK(zones[0].sizeM.x == doctest::Approx(1.5f));
    CHECK(zones[0].sizeM.z == doctest::Approx(1.5f));
    CHECK(a == core::match::appearanceOf(s.robots.at(1).request.spec));
    const ZoneConfig back = wire::toZoneConfig(a.zones[0]);
    CHECK(back.sizeM.x == doctest::Approx(zones[0].sizeM.x));
    CHECK(back.sizeM.z == doctest::Approx(zones[0].sizeM.z));
}

TEST_CASE("match host: every roster after a join or a leave carries every appearance") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h.kit);
    TestClient alice(h);
    REQUIRE(alice.join(alice.joinMessage("Alice", specWithZones(), s)));
    TestClient bob(h);
    RobotConfig bobSpec = swerveSpec("Bob");
    bobSpec.chassis.sizeM = {0.9f, 0.5f, 0.15f};
    REQUIRE(bob.join(bob.joinMessage("Bob", bobSpec, s)));

    // Alice learns about Bob from a roster that carries Bob's box and her own, unchanged.
    REQUIRE(alice.waitFor([&] {
        const auto roster = alice.last<wire::Roster>();
        return roster && roster->entries.size() == 3;
    }));
    const wire::Roster roster = *alice.last<wire::Roster>();
    const wire::RosterEntry* bobEntry = entryOf(roster.entries, 2);
    REQUIRE(bobEntry != nullptr);
    CHECK(bobEntry->appearance.sizeXMm == 900);
    CHECK(bobEntry->appearance.sizeYMm == 500);
    CHECK(bobEntry->appearance == bob.accept->roster.back().appearance);
    REQUIRE(entryOf(roster.entries, 1) != nullptr);
    CHECK(entryOf(roster.entries, 1)->appearance == alice.accept->roster[1].appearance);
    CHECK(entryOf(roster.entries, 1)->appearance.zones.size() == 2);

    // Bob leaves: the next roster drops him and keeps Alice's appearance.
    wire::Leave leave;
    leave.detail = "bye";
    REQUIRE(bob.send(leave).ok());
    REQUIRE(alice.waitFor([&] {
        const auto r = alice.last<wire::Roster>();
        return r && r->entries.size() == 2;
    }));
    const wire::Roster after = *alice.last<wire::Roster>();
    CHECK(entryOf(after.entries, 2) == nullptr);
    REQUIRE(entryOf(after.entries, 1) != nullptr);
    CHECK(entryOf(after.entries, 1)->appearance == alice.accept->roster[1].appearance);
}

TEST_CASE("match host: a client that rejoins with its resume token gets the same appearances") {
    FakeSession s;
    Harness h(s);
    addFakeGame(h.kit);
    TestClient first(h);
    REQUIRE(first.join(first.joinMessage("A", specWithZones(), s)));
    const wire::JoinAccept before = *first.accept;
    first.close();
    REQUIRE(h.pumpUntil([&] { return !h.host->participants().empty() && !h.host->participants()[0].connected; }));

    TestClient again(h);
    wire::Join j = again.joinMessage("A", swerveSpec(), s);   // a different spec: the accepted robot stays as it was
    j.resumeToken = before.resumeToken;
    REQUIRE(again.join(j));
    REQUIRE(again.accept->roster.size() == 2);
    CHECK(again.accept->roster[1].appearance == before.roster[1].appearance);
    CHECK(again.accept->roster[1].appearance.zones.size() == 2);
}
