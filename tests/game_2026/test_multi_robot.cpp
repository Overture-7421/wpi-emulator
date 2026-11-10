#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "multirobot_support.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/game_values.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::above;
using g26test::emptyField;
using g26test::makeZone;
using g26test::publishAll;
using g26test::remotePickup;
using g26test::remoteShooter;
using g26test::robotHud;
using g26test::Roster;
using g26test::ScriptedMechanism;
using g26test::Session;
using sdk::proto::AllianceStation;

namespace {

constexpr float kFuelZ = 0.075f;

// Where a chassis at (x, y) facing +X has its default pickup volume.
sdk::Vec3 ahead(float x, float y) { return {x + 0.45f, y, kFuelZ}; }

sdk::ZoneConfig hostPickup(const std::string& roller) {
    sdk::Value p = sdk::Value::makeObject();
    p["intakeRateHz"] = 0.0;
    return makeZone("hostIntake", kPickupKind, p, {above(roller, 20.0)});
}

std::vector<sdk::Vec3> fuelPositions(Session& s) {
    std::vector<sdk::Vec3> out;
    s.game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) { out.push_back(p); });
    return out;
}

sdk::Vec3 onlyFuelVelocity(Session& s) {
    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    return s.env.physics.linearVelocity(s.game->pool().bodyOf(id));
}

sdk::Vec3 onlyFuelPosition(Session& s) {
    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    return s.env.physics.pose(s.game->pool().bodyOf(id)).position;
}

}  // namespace

TEST_CASE("multi robot: every robot starts with its own preloaded meter and they stay independent") {
    Session s;
    emptyField(s, 30, 5);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.add(2, AllianceStation::Blue1, {remotePickup("p")}, {10.f, 6.f, 0.f});

    for (const sdk::ParticipantId id : std::vector<sdk::ParticipantId>{0, 1, 2}) {
        REQUIRE(s.game->robot(id) != nullptr);
        CHECK(s.game->robot(id)->meter == 5);
        CHECK(s.game->robot(id)->shots == 0);
    }
    CHECK(s.game->robot(3) == nullptr);

    s.game->pool().spawn(ahead(2.f, 2.f), {});
    roster.setFlags(1, 1);
    s.tick(2);
    CHECK(s.game->robot(1)->meter == 6);
    CHECK(s.game->robot(2)->meter == 5);
    CHECK(s.game->fuelInRobot() == 5);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("multi robot: a capacity below the preload limits every robot") {
    Session s;
    emptyField(s, 4, 8);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {}, {2.f, 2.f, 0.f});
    CHECK(s.game->fuelInRobot() == 4);
    CHECK(s.game->robot(1)->meter == 4);
}

TEST_CASE("multi robot: a remote pickup absorbs only inside its own zone while its flag is set and the robot is enabled") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    // zone 0 belongs to another plugin: the pickup is zone 1 and therefore flag bit 1
    roster.add(1, AllianceStation::Red2, {makeZone("other", "other.kind"), remotePickup("p")}, {2.f, 2.f, 0.f});

    const sdk::Vec3 inside = ahead(2.f, 2.f);
    const sdk::Vec3 outside{2.45f, 3.5f, kFuelZ};
    const sdk::Vec3 behind{1.2f, 2.f, kFuelZ};
    s.game->pool().spawn(inside, {});
    s.game->pool().spawn(outside, {});
    s.game->pool().spawn(behind, {});

    s.tick(3);  // no flag
    CHECK(s.game->robot(1)->meter == 8);
    roster.setFlags(1, 1u << 0);  // the other plugin's bit
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 8);
    CHECK_FALSE(s.game->robot(1)->zones.front()->active());

    roster.setEnabled(1, false);
    roster.setFlags(1, 1u << 1);
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 8);  // flagged but disabled
    CHECK(s.game->fuelOnField() == 3);

    roster.setEnabled(1, true);
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 9);
    CHECK(s.game->robot(1)->zones.front()->active());
    CHECK(s.game->fuelOnField() == 2);
    CHECK(s.fuelNear(outside, 0.2f) == 1);
    CHECK(s.fuelNear(behind, 0.2f) == 1);
    CHECK(s.game->fuelInRobot() == 8);

    roster.setFlags(1, 0);
    s.game->pool().spawn(inside, {});
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 9);
    CHECK_FALSE(s.game->robot(1)->zones.front()->active());
}

TEST_CASE("multi robot: a remote pickup follows its robot and stops at capacity") {
    Session s;
    emptyField(s, 10, 9);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {6.f, 2.f, 0.f}, 1.5707963f);  // facing +Y
    roster.setFlags(1, 1);
    s.game->pool().spawn({6.f, 2.45f, kFuelZ}, {});
    s.game->pool().spawn({6.f, 2.5f, kFuelZ}, {});
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 10);
    CHECK(s.game->fuelOnField() == 1);

    roster.place(1, {8.f, 2.f, 0.f}, 0.f);
    s.game->pool().spawn(ahead(8.f, 2.f), {});
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 10);  // full: leaves the fuel
    CHECK(s.game->fuelOnField() == 2);
}

TEST_CASE("multi robot: a remote zone attached to a mechanism link is treated as chassis-attached at its local pose") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    sdk::ZoneConfig zone = remotePickup("p");
    zone.attachment.parentMechanismId = "arm";
    zone.attachment.parentLink = "tip";
    REQUIRE_FALSE(zone.attachment.onChassis());
    roster.add(1, AllianceStation::Red2, {zone}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    CHECK(s.game->robot(1)->meter == 9);
    CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 0);
}

TEST_CASE("multi robot: a remote shooter fires at its configured exit speed and aim and spends only its own fuel") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["fixedExitSpeedMps"] = 6.0;
    props["hoodAngleRad"] = 0.5;
    props["turretYawOffsetRad"] = 0.3;
    props["inheritRobotVelocity"] = false;
    const float yaw = 0.7f;
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", props)}, {4.f, 3.f, 0.f}, yaw);
    roster.add(2, AllianceStation::Blue1, {remoteShooter("shooter", props)}, {12.f, 3.f, 0.f});

    s.tick(3);  // not flagged
    CHECK(s.game->fuelOnField() == 0);

    roster.setFlags(1, 1);
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);
    CHECK(s.game->robot(1)->meter == 7);
    CHECK(s.game->robot(1)->shots == 1);
    CHECK(s.game->robot(2)->meter == 8);
    CHECK(s.game->robot(2)->shots == 0);
    CHECK(s.game->fuelInRobot() == 8);
    CHECK(s.game->shotsFired() == 0);

    const double heading = yaw + 0.3;
    const double ce = std::cos(0.5), se = std::sin(0.5);
    const sdk::Vec3 v = onlyFuelVelocity(s);
    CHECK(v.x == doctest::Approx(6.0 * ce * std::cos(heading)).epsilon(0.01));
    CHECK(v.y == doctest::Approx(6.0 * ce * std::sin(heading)).epsilon(0.01));
    CHECK(v.z == doctest::Approx(6.0 * se).epsilon(0.02));
    const sdk::Vec3 p = onlyFuelPosition(s);  // zone pose (0.25, 0, 0.55) rotated by the chassis yaw
    CHECK(p.x == doctest::Approx(4.0 + 0.25 * std::cos(yaw)).epsilon(0.01));
    CHECK(p.y == doctest::Approx(3.0 + 0.25 * std::sin(yaw)).epsilon(0.01));
    CHECK(p.z == doctest::Approx(0.55).epsilon(0.02));

    const auto* shooter = dynamic_cast<const ShooterZoneRuntime*>(s.game->robot(1)->zones.front().get());
    REQUIRE(shooter != nullptr);
    CHECK(shooter->exitSpeedMps() == doctest::Approx(6.0));
    CHECK(shooter->elevationRad() == doctest::Approx(0.5));
    CHECK(shooter->yawRad() == doctest::Approx(0.3));
}

TEST_CASE("multi robot: a remote shooter without a fixed exit speed uses the documented default") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["hoodAngleRad"] = 0.0;
    props["inheritRobotVelocity"] = false;
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", props)}, {4.f, 3.f, 0.f});
    roster.setFlags(1, 1);
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);
    const sdk::Vec3 v = onlyFuelVelocity(s);
    CHECK(v.x == doctest::Approx(kRemoteDefaultExitSpeedMps).epsilon(0.01));
    CHECK(v.y == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("multi robot: remote shooters ignore mechanism references without warnings, host references still count") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["flywheelMechanism"] = "flywheel";
    props["hoodMechanism"] = "hood";
    props["turretMechanism"] = "turret";
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", props)}, {4.f, 3.f, 0.f});
    roster.setFlags(1, 1);
    s.tick(2);
    CHECK(s.game->robot(1)->shots >= 1);
    CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 0);
}

TEST_CASE("multi robot: remote shooters fire at their own rate and stop when their meter is empty") {
    Session s;
    emptyField(s, 30, 3);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["fireRateHz"] = 20.0;
    props["fixedExitSpeedMps"] = 5.0;
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", props)}, {4.f, 3.f, 0.f});
    roster.add(2, AllianceStation::Blue1, {}, {12.f, 3.f, 0.f});
    roster.setFlags(1, 1);
    s.tick(100);  // 0.5 s: far more than the meter
    CHECK(s.game->robot(1)->meter == 0);
    CHECK(s.game->robot(1)->shots == 3);
    CHECK(s.game->robot(2)->meter == 3);
    CHECK(s.game->fuelInRobot() == 3);
}

TEST_CASE("multi robot: a remote shooter inherits the velocity of its own body") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["fixedExitSpeedMps"] = 5.0;
    props["hoodAngleRad"] = 0.0;
    roster.add(1, AllianceStation::Red2, {remoteShooter("shooter", props)}, {4.f, 3.f, 0.f});
    s.env.physics.setLinearVelocity(roster.view(1).body, {2.f, 0.f, 0.f});
    s.env.physics.setLinearVelocity(s.chassis, {-3.f, 0.f, 0.f});  // the host's motion must not leak in
    roster.setFlags(1, 1);
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);
    CHECK(onlyFuelVelocity(s).x == doctest::Approx(5.0 + 2.0).epsilon(0.01));
}

TEST_CASE("multi robot: fuel absorbed by one robot is gone for the others in the same tick, host first then by id") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(hostPickup("roller"));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    Roster roster(s);
    // robots 5 and 3 stand where the host stands: three robots would take the same fuel
    roster.add(5, AllianceStation::Blue2, {remotePickup("p", 1.0)}, {2.f, 2.f, 0.f});
    roster.add(3, AllianceStation::Blue1, {remotePickup("p", 1.0)}, {2.f, 2.f, 0.f});
    CHECK(s.game->robotIds() == std::vector<sdk::ParticipantId>{0u, 3u, 5u});

    roster.setFlags(5, 1);
    roster.setFlags(3, 1);
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    // robot 3 (lowest remote id) wins the only fuel; nobody counts it twice
    CHECK(s.game->robot(3)->meter == 9);
    CHECK(s.game->robot(5)->meter == 8);
    CHECK(s.game->fuelInRobot() == 8);
    CHECK(s.game->fuelOnField() == 0);

    roller.set("velocity", 100.0, "rad/s");  // the host joins: it acts before the remote robots
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    CHECK(s.game->fuelInRobot() == 9);
    CHECK(s.game->robot(3)->meter == 9);
    CHECK(s.game->robot(5)->meter == 8);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("multi robot: rate limited intakes sharing fuel take the nearest first in robot order and conserve fuel") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    roster.add(2, AllianceStation::Red2, {remotePickup("p", 1.0)}, {2.f, 2.f, 0.f});
    roster.add(1, AllianceStation::Red1, {remotePickup("p", 1.0)}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    const sdk::Vec3 nearCentre = ahead(2.f, 2.f);
    const sdk::Vec3 farther{2.55f, 2.1f, kFuelZ};
    s.game->pool().spawn(farther, {});
    s.game->pool().spawn(nearCentre, {});
    s.tick(1);
    CHECK(s.game->robot(1)->meter == 9);
    REQUIRE(s.game->fuelOnField() == 1);
    CHECK(s.fuelNear(farther, 0.05f) == 1);  // the one next to the zone centre went first

    roster.setFlags(2, 1);
    s.tick(1);
    CHECK(s.game->robot(2)->meter == 9);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("multi robot: remote flags never change what the host robot does") {
    const auto run = [](bool withRemote) {
        Session s;
        emptyField(s);
        s.section()["capacity"] = 30;
        ScriptedMechanism& roller = s.addScripted("roller");
        ScriptedMechanism& flywheel = s.addScripted("flywheel");
        ScriptedMechanism& indexer = s.addScripted("indexer");
        sdk::Value props = sdk::Value::makeObject();
        props["flywheelMechanism"] = "flywheel";
        props["flywheelRadiusM"] = 0.05;
        props["efficiency"] = 0.5;
        props["exitSpeedJitter"] = 0.1;
        props["aimJitterRad"] = 0.05;
        props["fireRateHz"] = 20.0;
        s.addZone(hostPickup("roller"));
        s.addZone(makeZone("hostShooter", kShooterKind, props, {above("indexer", 5.0)}));
        s.start();
        s.setChassis({4.f, 3.f, 0.f});
        flywheel.set("velocity", 400.0, "rad/s");
        for (int i = 0; i < 6; ++i) s.game->pool().spawn({4.45f + 0.02f * static_cast<float>(i), 3.f, kFuelZ}, {});
        std::unique_ptr<Roster> roster;
        if (withRemote) {
            roster = std::make_unique<Roster>(s);
            roster->add(1, AllianceStation::Blue3, {remotePickup("p"), remoteShooter("s")}, {14.f, 6.f, 0.f});
            roster->setFlags(1, 0b11);
        }
        for (int i = 0; i < 60; ++i) {
            roller.set("velocity", i < 20 ? 100.0 : 0.0, "rad/s");
            indexer.set("velocity", i >= 25 ? 20.0 : 0.0, "rad/s");
            s.tick();
        }
        // the remote robot shot from far away; only fuel near the host matters (pool order may differ)
        std::vector<sdk::Vec3> nearHost;
        for (const sdk::Vec3& p : fuelPositions(s))
            if (p.x < 10.f) nearHost.push_back(p);
        std::sort(nearHost.begin(), nearHost.end(), [](const sdk::Vec3& a, const sdk::Vec3& b) {
            return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
        });
        return std::make_tuple(s.game->fuelInRobot(), s.game->shotsFired(), nearHost, s.game->zones().size());
    };
    const auto alone = run(false);
    const auto crowded = run(true);
    CHECK(std::get<0>(alone) == std::get<0>(crowded));
    CHECK(std::get<1>(alone) == std::get<1>(crowded));
    CHECK(std::get<3>(alone) == std::get<3>(crowded));
    REQUIRE(std::get<2>(alone).size() == std::get<2>(crowded).size());
    for (std::size_t i = 0; i < std::get<2>(alone).size(); ++i) {
        CHECK(std::get<2>(alone)[i].x == std::get<2>(crowded)[i].x);
        CHECK(std::get<2>(alone)[i].y == std::get<2>(crowded)[i].y);
        CHECK(std::get<2>(alone)[i].z == std::get<2>(crowded)[i].z);
    }
}

TEST_CASE("multi robot: gameValue and the published zones stay the host robot's") {
    Session s;
    emptyField(s, 30, 8);
    s.addZone(hostPickup("roller"));
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    REQUIRE(s.game->robot(1)->meter == 9);
    CHECK(s.game->gameValue(game_value::kFuelCount).value() == doctest::Approx(8.0));
    sdk::SnapshotBuilder out;
    s.stateOwner->publish(out);
    const auto snap = out.build();
    REQUIRE(snap->zones().size() == 1);
    CHECK(snap->zones()[0].id == "hostIntake");
    CHECK(snap->hudValue(hud::kFuelCount)->asInt() == 8);
}

TEST_CASE("multi robot: the per-robot HUD carries that robot's own values under the host's keys") {
    Session s;
    emptyField(s, 12, 4);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.add(2, AllianceStation::Blue1, {remoteShooter("s")}, {12.f, 3.f, 0.f});
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    roster.setFlags(1, 1);
    roster.setFlags(2, 1);
    s.tick(2);
    const auto snap = publishAll(s, roster);
    REQUIRE(snap->remoteRobots().size() == 2);
    CHECK(robotHud(*snap, 1, hud::kFuelCount)->asInt() == 5);
    CHECK(robotHud(*snap, 1, hud::kFuelCapacity)->asInt() == 12);
    CHECK(robotHud(*snap, 1, hud::kShots)->asInt() == 0);
    CHECK(robotHud(*snap, 2, hud::kFuelCount)->asInt() < 4);
    CHECK(robotHud(*snap, 2, hud::kShots)->asInt() >= 1);
    CHECK(robotHud(*snap, 2, hud::kFuelCapacity)->asInt() == 12);
    // untimed match: no per-robot hub status, and shared values stay in the game's own HUD
    CHECK(robotHud(*snap, 1, hud::kHubOwnActive) == nullptr);
    CHECK(robotHud(*snap, 1, hud::kFuelOnField) == nullptr);
    CHECK(snap->hudValue(hud::kFuelCount)->asInt() == 4);

    // publishRobot for the host or an unknown robot adds nothing
    sdk::SnapshotBuilder out;
    sdk::RobotSample sample;
    sample.participantId = 7;
    out.addRobot(sample);
    sdk::RobotView ghost;
    ghost.participantId = 7;
    s.stateOwner->publishRobot(out, ghost);
    s.stateOwner->publishRobot(out, roster.view(0));
    CHECK(out.build()->remoteRobots()[0].hud.empty());
}

TEST_CASE("multi robot: a remote robot's alliance decides its own-hub status in a timed match") {
    Session s;
    emptyField(s);
    s.section()["firstInactiveHub"] = "red";
    s.env.driverStation.station = AllianceStation::Blue1;
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {}, {2.f, 2.f, 0.f});
    roster.add(2, AllianceStation::Blue3, {}, {12.f, 3.f, 0.f});
    s.env.driverStation.enabled = true;
    s.env.driverStation.mode = sdk::proto::RobotMode::Teleop;
    s.env.driverStation.matchTimeS = 140.0 - 15.0;  // shift 1: the red hub is inactive
    s.tick(2);
    REQUIRE(s.game->match().phase == MatchPhase::Shift1);
    REQUIRE_FALSE(s.game->hubActive(Alliance::Red));
    REQUIRE(s.game->hubActive(Alliance::Blue));

    const auto snap = publishAll(s, roster);
    CHECK(robotHud(*snap, 1, hud::kHubOwnActive)->asBool() == false);
    CHECK(robotHud(*snap, 2, hud::kHubOwnActive)->asBool() == true);
    CHECK(snap->hudValue(hud::kHubOwnActive)->asBool() == true);  // the host is Blue1
    // the host's view and the DS cannot diverge
    CHECK(roster.view(0).alliance() == sdk::proto::allianceOf(s.env.driverStation.station));
}

TEST_CASE("multi robot: removing a robot drops its state and a re-added robot starts fresh") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    REQUIRE(s.game->robot(1)->meter == 9);

    roster.remove(1);
    CHECK(s.game->robot(1) == nullptr);
    CHECK(s.game->robotIds() == std::vector<sdk::ParticipantId>{0u});
    s.tick(2);  // nothing refers to the removed robot
    CHECK(s.game->fuelInRobot() == 8);

    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    CHECK(s.game->robot(1)->meter == 8);
    CHECK(s.game->robot(1)->shots == 0);
    CHECK_FALSE(s.game->robot(1)->zones.front()->active());

    // the host cannot be removed and unknown ids are ignored
    s.game->onRobotRemoved(s.env.ctx, 0);
    s.game->onRobotRemoved(s.env.ctx, 99);
    CHECK(s.game->robot(0) != nullptr);

    // announcing a known robot again replaces its state
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    REQUIRE(s.game->robot(1)->meter == 9);
    s.game->onRobotAdded(s.env.ctx, roster.view(1));
    CHECK(s.game->robot(1)->meter == 8);
}

TEST_CASE("multi robot: announcing the host does not disturb its state") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(hostPickup("roller"));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    roller.set("velocity", 100.0, "rad/s");
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(2);
    REQUIRE(s.game->fuelInRobot() == 9);
    Roster roster(s);
    s.game->onRobotAdded(s.env.ctx, roster.view(0));
    CHECK(s.game->fuelInRobot() == 9);
    CHECK(s.game->zones().size() == 1);
    CHECK(s.game->robotIds() == std::vector<sdk::ParticipantId>{0u});
}

TEST_CASE("multi robot: reset keeps the roster and restores every robot") {
    Session s;
    emptyField(s, 30, 6);
    s.start();
    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["fixedExitSpeedMps"] = 5.0;
    roster.add(1, AllianceStation::Red2, {remoteShooter("s", props), remotePickup("p")}, {4.f, 3.f, 0.f});
    roster.add(2, AllianceStation::Blue1, {remotePickup("p")}, {10.f, 3.f, 0.f});
    roster.setFlags(1, 0b01);
    s.game->pool().spawn(ahead(10.f, 3.f), {});
    roster.setFlags(2, 1);
    s.tick(10);
    REQUIRE(s.game->robot(1)->shots > 0);
    REQUIRE(s.game->robot(2)->meter == 7);
    REQUIRE(s.game->robot(1)->zones.front()->active());

    roster.setFlags(1, 0);
    roster.setFlags(2, 0);
    s.stateOwner->reset(s.env.ctx);
    CHECK(s.game->robotIds() == std::vector<sdk::ParticipantId>{0u, 1u, 2u});
    CHECK(s.game->robot(1)->meter == 6);
    CHECK(s.game->robot(1)->shots == 0);
    CHECK(s.game->robot(2)->meter == 6);
    CHECK(s.game->fuelOnField() == 0);
    CHECK_FALSE(s.game->robot(1)->zones.front()->active());
    REQUIRE(s.game->robot(1)->zones.size() == 2);
    CHECK(s.game->robot(1)->zoneBits == std::vector<std::size_t>{0u, 1u});

    // zone runtimes were rebuilt: the shooter works again from a primed budget
    roster.setFlags(1, 0b01);
    s.tick(1);
    CHECK(s.game->robot(1)->shots == 1);
}

TEST_CASE("multi robot: robots announced before the first reset are part of it") {
    Session s;
    emptyField(s, 30, 7);
    s.start();
    Roster roster(s);
    std::unique_ptr<sdk::IGameState> fresh = s.module().createGameState(s.env.ctx).value();
    auto* state = dynamic_cast<Game2026State*>(fresh.get());
    REQUIRE(state != nullptr);

    // the runtime's order: createGameState, onRobotAdded for the host and the existing robots, then reset
    s.game->onRobotAdded(s.env.ctx, roster.view(0));
    sdk::RobotView view;
    view.participantId = 4;
    view.station = AllianceStation::Blue2;
    std::vector<sdk::ZoneConfig> zones{remotePickup("p")};
    view.zones = &zones;
    view.body = s.env.physics.createBody(sdk::BodyDesc{});
    state->onRobotAdded(s.env.ctx, roster.view(0));
    state->onRobotAdded(s.env.ctx, view);
    CHECK(state->robot(4) != nullptr);
    state->reset(s.env.ctx);
    CHECK(state->robot(4) != nullptr);
    CHECK(state->robot(4)->meter == 7);
    CHECK(state->fuelInRobot() == 7);
}

TEST_CASE("multi robot: a robot missing from the directory does nothing and does not crash") {
    Session s;
    emptyField(s);
    s.start();
    Roster roster(s);
    roster.add(1, AllianceStation::Red2, {remotePickup("p")}, {2.f, 2.f, 0.f});
    roster.setFlags(1, 1);
    roster.dir.remove(1);  // the directory forgot it before the game was told
    s.game->pool().spawn(ahead(2.f, 2.f), {});
    s.tick(3);
    CHECK(s.game->robot(1)->meter == 8);
    CHECK(s.game->fuelOnField() == 1);
}

namespace {

struct Outcome {
    std::uint64_t hash = 0;
    std::vector<sdk::Vec3> fuel;
    std::vector<int> meters;
    bool operator==(const Outcome& o) const {
        if (hash != o.hash || meters != o.meters || fuel.size() != o.fuel.size()) return false;
        for (std::size_t i = 0; i < fuel.size(); ++i)
            if (fuel[i].x != o.fuel[i].x || fuel[i].y != o.fuel[i].y || fuel[i].z != o.fuel[i].z) return false;
        return true;
    }
};

// Host and three remote robots with jittered shooters and contended intakes; `addOrder` permutes the join order.
Outcome scriptedMatch(unsigned seed, const std::vector<sdk::ParticipantId>& addOrder) {
    Session s;
    emptyField(s, 30, 8);
    s.env.config.sim.seed = seed;
    s.env.rng.seed(seed);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    sdk::Value hostProps = sdk::Value::makeObject();
    hostProps["flywheelMechanism"] = "flywheel";
    hostProps["flywheelRadiusM"] = 0.05;
    hostProps["efficiency"] = 0.5;
    hostProps["aimJitterRad"] = 0.1;
    hostProps["exitSpeedJitter"] = 0.1;
    hostProps["fireRateHz"] = 20.0;
    s.addZone(hostPickup("roller"));
    s.addZone(makeZone("hostShooter", kShooterKind, hostProps, {above("indexer", 5.0)}));
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    roller.set("velocity", 100.0, "rad/s");

    Roster roster(s);
    sdk::Value props = sdk::Value::makeObject();
    props["fixedExitSpeedMps"] = 7.0;
    props["aimJitterRad"] = 0.1;
    props["exitSpeedJitter"] = 0.2;
    props["fireRateHz"] = 15.0;
    props["parallelBalls"] = 2;
    for (const sdk::ParticipantId id : addOrder) {
        roster.add(id, static_cast<AllianceStation>(id), {remotePickup("p", 5.0), remoteShooter("s", props)},
                   {4.f + 0.3f * static_cast<float>(id), 3.f + 0.2f * static_cast<float>(id), 0.f}, 0.4f * static_cast<float>(id));
    }
    for (int i = 0; i < 40; ++i)
        s.game->pool().spawn({4.4f + 0.04f * static_cast<float>(i % 8), 3.f + 0.05f * static_cast<float>(i / 8), kFuelZ}, {});
    for (int tick = 0; tick < 120; ++tick) {
        for (const sdk::ParticipantId id : addOrder) roster.setFlags(id, ((tick / 10 + id) % 3 == 0) ? 0b10u : 0b01u);
        s.tick();
    }
    Outcome out;
    out.hash = s.env.physics.stateHash();
    out.fuel = fuelPositions(s);
    for (const sdk::ParticipantId id : s.game->robotIds()) out.meters.push_back(s.game->robot(id)->meter);
    for (const sdk::ParticipantId id : s.game->robotIds()) out.meters.push_back(s.game->robot(id)->shots);
    return out;
}

}  // namespace

TEST_CASE("multi robot: identical scripted inputs give identical fuel state, whatever the join order") {
    const Outcome reference = scriptedMatch(7, {1, 2, 3});
    int shots = 0;
    for (std::size_t i = reference.meters.size() / 2; i < reference.meters.size(); ++i) shots += reference.meters[i];
    CHECK(shots > 0);  // the scenario really fires
    CHECK(scriptedMatch(7, {1, 2, 3}) == reference);
    Outcome permuted = scriptedMatch(7, {3, 1, 2});
    permuted.hash = reference.hash;  // body handles follow the creation order; the game state must not
    CHECK(permuted == reference);
    CHECK_FALSE(scriptedMatch(8, {1, 2, 3}) == reference);
}
