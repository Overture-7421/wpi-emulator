#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/fuel_pool.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::above;
using g26test::makeZone;
using g26test::ScriptedMechanism;
using g26test::Session;
using g26test::within;

namespace {

// Session with no staged fuel on the field, so tests only see the fuel they create.
void emptyField(Session& s, int capacity = 30, int preload = 8) {
    s.section()["neutralFuel"] = 0;
    s.section()["depotFuel"] = 0;
    s.section()["capacity"] = capacity;
    s.section()["preloadFuel"] = preload;
}

// Pickup zone (0.30 x 0.60 x 0.40 m, centre 0.45 m ahead of the chassis) active while `roller` spins.
sdk::ZoneConfig pickupZone(const std::string& id = "intake", const char* roller = "roller") {
    sdk::Value p = sdk::Value::makeObject();
    p["intakeRateHz"] = 0.0;  // unlimited unless a test limits it
    std::vector<sdk::ZoneCondition> when;
    if (*roller != '\0') when.push_back(above(roller, 20.0));
    return makeZone(id, kPickupKind, p, when);
}

sdk::Value shooterProps() {
    sdk::Value p = sdk::Value::makeObject();
    p["flywheelMechanism"] = "flywheel";
    p["fireRateHz"] = 10.0;
    p["hoodAngleRad"] = 1.0471975512;  // 60 degrees
    p["flywheelRadiusM"] = 0.05;
    p["efficiency"] = 0.5;
    return p;
}

// Shooter zone that fires while the indexer spins faster than 5 rad/s.
sdk::ZoneConfig shooterZone(const std::string& id = "shooter", sdk::Value props = shooterProps()) {
    return makeZone(id, kShooterKind, std::move(props), {above("indexer", 5.0)});
}

constexpr float kYawQuarterTurn = 1.5707963f;

}  // namespace

TEST_CASE("initial staging follows the manual: neutral zone block, two depots, 8 preloaded") {
    Session s;
    s.start();
    REQUIRE(s.game != nullptr);
    CHECK(s.game->fuelOnField() == 360 + 2 * 24);
    CHECK(s.game->fuelInRobot() == 8);
    CHECK(s.game->outpostStock(Alliance::Red) == 24);
    CHECK(s.game->outpostStock(Alliance::Blue) == 24);

    // neutral block: 12 columns (1.83 m deep) around the field centre, within the 5.23 m wide box
    int neutral = 0;
    s.game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) {
        if (p.x > 6.0f && p.x < 10.5f) {
            ++neutral;
            CHECK(std::fabs(p.x - dim::kFieldLengthM / 2) < 1.0f);
            CHECK(std::fabs(p.y - dim::kFieldWidthM / 2) < 2.7f);
            CHECK(p.z == doctest::Approx(dim::kFuelRadiusM));
        }
    });
    CHECK(neutral == 360);

    // each depot holds 24 fuel inside its 27 x 42 in footprint
    const FieldLayout layout = buildFieldLayout();
    for (const Alliance a : kAlliances) {
        const DepotSpec& d = layout.depot(a);
        int inside = 0;
        s.game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) {
            if (std::fabs(p.x - d.center.x) < dim::kDepotDepthM / 2 && std::fabs(p.y - d.center.y) < dim::kDepotWidthM / 2)
                ++inside;
        });
        CHECK(inside == 24);
    }
}

TEST_CASE("staged fuel never overlaps: neighbours are at least one diameter apart") {
    Session s;
    s.start();
    std::vector<sdk::Vec3> all;
    s.game->pool().forEach([&](std::uint32_t, const sdk::Vec3& p) { all.push_back(p); });
    float minDist = 1e9f;
    for (std::size_t i = 0; i < all.size(); ++i)
        for (std::size_t j = i + 1; j < all.size(); ++j) minDist = std::min(minDist, sdk::length(all[i] - all[j]));
    CHECK(minDist >= 2 * dim::kFuelRadiusM - 1e-4f);
}

TEST_CASE("staging honours the config, capacity limits the preload and the body cap") {
    SUBCASE("custom counts") {
        Session s;
        s.section()["neutralFuel"] = 100;
        s.section()["depotFuel"] = 0;
        s.section()["capacity"] = 4;
        s.section()["preloadFuel"] = 8;
        s.start();
        CHECK(s.game->fuelOnField() == 100);
        CHECK(s.game->fuelInRobot() == 4);
    }
    SUBCASE("max bodies") {
        Session s;
        s.section()["maxFuelOnField"] = 50;
        s.start();
        CHECK(s.game->fuelOnField() == 50);
    }
    SUBCASE("a world that refuses bodies leaves an empty but working game state") {
        g26test::ForwardingWorld world;
        sdk::testing::FakeClock clock;
        sdk::testing::FakeDeviceHub hub;
        sdk::testing::CapturingLogger logger;
        sdk::RobotConfig config;
        sdk::Rng rng;
        sdk::proto::DsState ds;
        sdk::SimContext ctx(sdk::SimContextServices{world, clock, hub, logger, config, rng, ds, sdk::BodyId{}, nullptr, nullptr});
        world.refuseBodies = true;
        Game2026State state(ctx, GameConfig{}, buildFieldLayout());
        state.reset(ctx);
        CHECK(state.fuelOnField() == 0);
        CHECK(state.fuelInRobot() == 8);
        state.step(ctx, 0.005);
        sdk::SnapshotBuilder out;
        state.publish(out);
        CHECK(out.data().entityGroups.front().positions.empty());
    }
}

TEST_CASE("pickup zone absorbs fuel in its volume only while the roller spins") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(pickupZone());
    s.start();
    s.setChassis({2.f, 2.f, 0.f});

    const sdk::Vec3 inFront{2.45f, 2.f, 0.075f};
    const sdk::Vec3 farAway{2.45f, 3.5f, 0.075f};
    s.game->pool().spawn(inFront, {});
    s.game->pool().spawn(farAway, {});
    CHECK(s.game->fuelOnField() == 2);

    s.tick(20);  // roller stopped
    CHECK(s.game->fuelOnField() == 2);
    CHECK(s.game->fuelInRobot() == 8);

    roller.set("velocity", 100.0, "rad/s");
    s.tick(2);
    CHECK(s.game->fuelOnField() == 1);   // only the fuel inside the zone
    CHECK(s.game->fuelInRobot() == 9);
    CHECK(s.fuelNear(farAway, 0.3f) == 1);

    roller.set("velocity", 10.0, "rad/s");  // below threshold again
    s.game->pool().spawn(inFront, {});
    s.tick(10);
    CHECK(s.game->fuelInRobot() == 9);

    roller.set("velocity", -100.0, "rad/s");  // reverse also counts (|v|) - roller spits... but still spins
    s.tick(2);
    CHECK(s.game->fuelInRobot() == 10);

    auto* intake = dynamic_cast<PickupZoneRuntime*>(s.game->zones().front().get());
    REQUIRE(intake != nullptr);
    CHECK(intake->active());
    CHECK(intake->id() == "intake");
}

TEST_CASE("roller speed units are converted from the published unit") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(pickupZone());
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});

    roller.set("velocity", 60.0, "rpm");  // 6.28 rad/s < 20 rad/s threshold
    s.tick(5);
    CHECK(s.game->fuelOnField() == 1);
    roller.set("velocity", 600.0, "rpm");  // 62.8 rad/s
    s.tick(2);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("a pickup stops at capacity and a full robot leaves fuel on the field") {
    Session s;
    emptyField(s, /*capacity=*/10, /*preload=*/8);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(pickupZone());
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    roller.set("velocity", 100.0, "rad/s");
    for (int i = 0; i < 5; ++i) s.game->pool().spawn({2.35f + 0.1f * static_cast<float>(i % 3), 2.f, 0.075f}, {});

    s.tick(10);
    CHECK(s.game->fuelInRobot() == 10);
    CHECK(s.game->fuelOnField() == 3);
    s.tick(20);
    CHECK(s.game->fuelInRobot() == 10);
    CHECK(s.game->fuelOnField() == 3);
}

TEST_CASE("pickup rate limit and nearest-first order") {
    Session s;
    emptyField(s, 100, 0);
    ScriptedMechanism& roller = s.addScripted("roller");
    sdk::ZoneConfig zone = pickupZone();
    zone.properties["intakeRateHz"] = 10.0;
    s.addZone(zone);
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    roller.set("velocity", 100.0, "rad/s");
    for (int i = 0; i < 12; ++i) s.game->pool().spawn({2.32f + 0.02f * static_cast<float>(i), 2.f, 0.075f}, {});

    s.tick(100);  // 0.5 s at 10 Hz: the primed first fuel plus one every 0.1 s
    CHECK(s.game->fuelInRobot() >= 5);
    CHECK(s.game->fuelInRobot() <= 6);
    // the zone centre is at x = 2.45: the farthest fuel (x = 2.32 and 2.54) goes last
    CHECK(s.fuelNear({2.32f, 2.f, 0.f}, 0.05f) + s.fuelNear({2.54f, 2.f, 0.f}, 0.05f) >= 1);
}

TEST_CASE("the pickup zone follows the robot pose") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addZone(pickupZone());
    s.start();
    roller.set("velocity", 100.0, "rad/s");

    s.setChassis({2.f, 2.f, 0.f}, kYawQuarterTurn);  // facing +Y: zone centre at (2, 2.45)
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});   // where the zone WOULD be at yaw 0
    s.tick(5);
    CHECK(s.game->fuelOnField() == 1);
    s.game->pool().spawn({2.f, 2.45f, 0.075f}, {});
    s.tick(2);
    CHECK(s.game->fuelOnField() == 1);
    CHECK(s.game->fuelInRobot() == 9);
}

TEST_CASE("a zone attached to a mechanism link follows that link") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    ScriptedMechanism& arm = s.addScripted("arm");
    arm.links.push_back({"tip", sdk::Pose3{{0.f, 1.f, 0.f}, sdk::Quat{}}});
    roller.set("velocity", 100.0, "rad/s");
    sdk::MechanismConfig armConfig;  // the directory resolves attachment chains from the config
    armConfig.id = "arm";
    armConfig.type = "test.scripted";
    s.env.config.mechanisms.push_back(armConfig);
    sdk::ZoneConfig zone = pickupZone();
    zone.attachment.parentMechanismId = "arm";
    zone.attachment.parentLink = "tip";
    s.addZone(zone);
    s.start();
    s.setChassis({2.f, 2.f, 0.f});

    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});  // chassis-mounted position: not where the zone is
    s.tick(5);
    CHECK(s.game->fuelOnField() == 1);
    s.game->pool().spawn({2.45f, 3.f, 0.075f}, {});  // 1 m to the left, ahead of the tip
    s.tick(2);
    CHECK(s.game->fuelOnField() == 1);
    CHECK(s.game->fuelInRobot() == 9);

    arm.links.front().pose.position = {0.f, -1.f, 0.f};  // the link moves, the zone with it
    s.game->pool().spawn({2.45f, 1.f, 0.075f}, {});
    s.tick(2);
    CHECK(s.game->fuelInRobot() == 10);
}

TEST_CASE("a zone attached to an unresolvable mechanism falls back to the chassis frame") {
    Session s;
    emptyField(s);
    s.addScripted("roller").set("velocity", 100.0, "rad/s");
    sdk::ZoneConfig zone = pickupZone();
    zone.attachment.parentMechanismId = "ghost_arm";
    s.addZone(zone);
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});
    s.tick(2);
    CHECK(s.game->fuelInRobot() == 9);
}

TEST_CASE("pickup activation: single condition, AND, OR and always") {
    Session s;
    emptyField(s, 100, 0);
    ScriptedMechanism& roller = s.addScripted("roller");
    ScriptedMechanism& arm = s.addScripted("arm");
    const sdk::ZoneCondition deployed = within("arm", 1.5, 0.1);
    sdk::ZoneConfig deployedOnly = pickupZone("deployedOnly", "");
    deployedOnly.activation.conditions = {deployed};
    sdk::ZoneConfig both = pickupZone("both");
    both.activation.conditions.push_back(deployed);
    both.activation.mode = sdk::ActivationMode::All;
    sdk::ZoneConfig either = pickupZone("either");
    either.activation.conditions.push_back(deployed);
    either.activation.mode = sdk::ActivationMode::Any;
    s.addZone(deployedOnly);
    s.addZone(both);
    s.addZone(either);
    s.addZone(pickupZone("always", ""));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});

    auto active = [&](std::size_t i) { return s.game->zones()[i]->active(); };
    arm.set("position", 0.0, "rad");
    roller.set("velocity", 0.0, "rad/s");
    s.tick(1);
    CHECK_FALSE(active(0));
    CHECK_FALSE(active(1));
    CHECK_FALSE(active(2));
    CHECK(active(3));

    arm.set("position", 1.45, "rad");  // within tolerance of 1.5
    s.tick(1);
    CHECK(active(0));
    CHECK_FALSE(active(1));
    CHECK(active(2));

    roller.set("velocity", 100.0, "rad/s");
    s.tick(1);
    CHECK(active(1));

    arm.set("position", 0.0, "rad");  // retracted, roller still spinning
    s.tick(1);
    CHECK_FALSE(active(0));
    CHECK_FALSE(active(1));
    CHECK(active(2));
}

TEST_CASE("a condition on a mechanism that does not exist never holds and warns once") {
    Session s;
    emptyField(s);
    s.addZone(pickupZone("intake", "ghost_roller"));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});
    s.tick(50);
    CHECK(s.game->fuelOnField() == 1);
    CHECK_FALSE(s.game->zones().front()->active());
    CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 1);
}

TEST_CASE("a pickup with no activation conditions is always on") {
    Session s;
    emptyField(s);
    s.addZone(pickupZone("passive", ""));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});
    s.tick(2);
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->fuelInRobot() == 9);
}

TEST_CASE("zones of other plugins and unknown game_2026 kinds are ignored, owned zones are published in the snapshot") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    ScriptedMechanism& indexer = s.addScripted("indexer");
    s.addScripted("flywheel");
    s.addZone(pickupZone("intake"));
    s.addZone(makeZone("foreign", "game_9999.pickup"));
    s.addZone(makeZone("mystery", "game_2026.mystery"));
    s.addZone(shooterZone("shooter"));
    s.start();
    REQUIRE(s.game->zones().size() == 2);

    auto publish = [&] {
        sdk::SnapshotBuilder out;
        s.game->publish(out);
        return out.data().zones;
    };
    s.tick(1);
    auto zones = publish();
    REQUIRE(zones.size() == 2);
    CHECK(zones[0].id == "intake");
    CHECK_FALSE(zones[0].active);
    CHECK(zones[1].id == "shooter");
    CHECK_FALSE(zones[1].active);

    roller.set("velocity", 100.0, "rad/s");
    s.tick(1);
    zones = publish();
    CHECK(zones[0].active);
    CHECK_FALSE(zones[1].active);

    indexer.set("velocity", 20.0, "rad/s");  // the shooter's activation holds (it need not be able to fire to be active)
    s.tick(1);
    zones = publish();
    CHECK(zones[1].active);
}

TEST_CASE("shooter launches real fuel bodies with flywheel speed x efficiency along the hood angle") {
    Session s;
    emptyField(s);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    ScriptedMechanism& indexer = s.addScripted("indexer");
    s.addZone(shooterZone());
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    flywheel.set("velocity", 400.0, "rad/s");  // 20 m/s surface, 10 m/s exit
    indexer.set("velocity", 0.0, "rad/s");

    s.tick(5);
    CHECK(s.game->shotsFired() == 0);        // activation (indexer) does not hold
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->fuelInRobot() == 8);
    CHECK_FALSE(s.game->zones().front()->active());

    indexer.set("velocity", 20.0, "rad/s");
    s.tick(1);
    REQUIRE(s.game->shotsFired() == 1);
    CHECK(s.game->fuelInRobot() == 7);
    REQUIRE(s.game->fuelOnField() == 1);

    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    const sdk::BodyId body = s.game->pool().bodyOf(id);
    const sdk::Vec3 p = s.env.physics.pose(body).position;
    CHECK(p.x == doctest::Approx(4.25f).epsilon(0.01));   // zone pose (0.25, 0, 0.55)
    CHECK(p.y == doctest::Approx(3.f).epsilon(0.01));
    CHECK(p.z == doctest::Approx(0.55f).epsilon(0.02));
    const sdk::Vec3 v = s.env.physics.linearVelocity(body);
    CHECK(v.x == doctest::Approx(10.f * std::cos(1.0471975f)).epsilon(0.01));
    CHECK(v.y == doctest::Approx(0.f).epsilon(0.01));
    CHECK(v.z == doctest::Approx(10.f * std::sin(1.0471975f)).epsilon(0.01));

    auto* shooter = dynamic_cast<ShooterZoneRuntime*>(s.game->zones().front().get());
    REQUIRE(shooter != nullptr);
    CHECK(shooter->active());
    CHECK(shooter->feeding());
    CHECK(shooter->exitSpeedMps() == doctest::Approx(10.0));
    CHECK(shooter->elevationRad() == doctest::Approx(1.0471975));
}

TEST_CASE("shots follow the robot heading and inherit its velocity") {
    Session s;
    emptyField(s);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    ScriptedMechanism& indexer = s.addScripted("indexer");
    flywheel.set("velocity", 400.0, "rad/s");
    indexer.set("velocity", 20.0, "rad/s");
    s.addZone(shooterZone());
    s.start();
    s.setChassis({4.f, 3.f, 0.f}, kYawQuarterTurn);  // facing +Y
    s.env.physics.setLinearVelocity(s.chassis, {0.f, 0.f, 0.f});
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);

    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    sdk::Vec3 v = s.env.physics.linearVelocity(s.game->pool().bodyOf(id));
    CHECK(v.x == doctest::Approx(0.f).epsilon(0.01));
    CHECK(v.y == doctest::Approx(5.f).epsilon(0.01));

    // moving robot: +2 m/s in X is added to the launch velocity
    Session m;
    emptyField(m);
    m.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    m.addScripted("indexer").set("velocity", 20.0, "rad/s");
    m.addZone(shooterZone());
    m.start();
    m.setChassis({4.f, 3.f, 0.f});
    m.env.physics.setLinearVelocity(m.chassis, {2.f, 0.f, 0.f});
    m.tick(1);
    REQUIRE(m.game->fuelOnField() == 1);
    m.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    v = m.env.physics.linearVelocity(m.game->pool().bodyOf(id));
    CHECK(v.x == doctest::Approx(5.f + 2.f).epsilon(0.01));
}

TEST_CASE("hood and turret mechanisms steer the shot") {
    Session s;
    emptyField(s);
    s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    ScriptedMechanism& hood = s.addScripted("hood");
    ScriptedMechanism& turret = s.addScripted("turret");
    sdk::Value p = shooterProps();
    p["hoodMechanism"] = "hood";
    p["hoodAngleRad"] = 0.0;
    p["turretMechanism"] = "turret";
    s.addZone(shooterZone("shooter", p));
    s.start();
    s.setChassis({4.f, 3.f, 0.f});

    hood.set("position", 30.0, "deg");     // converted to 0.5236 rad
    turret.set("position", 90.0, "deg");   // facing +Y
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);
    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    const sdk::Vec3 v = s.env.physics.linearVelocity(s.game->pool().bodyOf(id));
    CHECK(v.x == doctest::Approx(0.f).epsilon(0.02));
    CHECK(v.y == doctest::Approx(10.f * std::cos(0.5235988f)).epsilon(0.01));
    CHECK(v.z == doctest::Approx(10.f * std::sin(0.5235988f)).epsilon(0.01));

    auto* shooter = dynamic_cast<ShooterZoneRuntime*>(s.game->zones().front().get());
    REQUIRE(shooter != nullptr);
    CHECK(shooter->yawRad() == doctest::Approx(1.5707963));
}

TEST_CASE("a shooter attached to a turret link and rotated by its own pose aims along the zone's +X") {
    Session s;
    emptyField(s);
    s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    ScriptedMechanism& turret = s.addScripted("turret");
    turret.links.push_back({"head", sdk::Pose3{{0.f, 0.f, 0.3f}, sdk::Quat::fromYaw(kYawQuarterTurn)}});  // turned to +Y
    sdk::MechanismConfig turretConfig;
    turretConfig.id = "turret";
    turretConfig.type = "test.scripted";
    s.env.config.mechanisms.push_back(turretConfig);
    sdk::ZoneConfig zone = shooterZone();
    zone.attachment.parentMechanismId = "turret";
    zone.attachment.parentLink = "head";
    zone.properties["hoodAngleRad"] = 0.0;
    zone.local.position = {0.2f, 0.f, 0.1f};
    s.addZone(zone);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    REQUIRE(s.game->fuelOnField() == 1);

    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    const sdk::BodyId body = s.game->pool().bodyOf(id);
    const sdk::Vec3 pos = s.env.physics.pose(body).position;
    CHECK(pos.x == doctest::Approx(4.f).epsilon(0.01));
    CHECK(pos.y == doctest::Approx(3.2f).epsilon(0.01));
    CHECK(pos.z == doctest::Approx(0.4f).epsilon(0.02));
    const sdk::Vec3 v = s.env.physics.linearVelocity(body);
    CHECK(v.x == doctest::Approx(0.f).epsilon(0.02));
    CHECK(v.y == doctest::Approx(10.f).epsilon(0.01));
}

TEST_CASE("fire rate, empty meter and slow flywheels") {
    Session s;
    emptyField(s, 30, 3);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    s.addZone(shooterZone());
    s.start();
    s.setChassis({4.f, 3.f, 0.f});

    flywheel.set("velocity", 20.0, "rad/s");  // 0.5 m/s: below the 2 m/s minimum
    s.tick(50);
    CHECK(s.game->shotsFired() == 0);

    flywheel.set("velocity", 400.0, "rad/s");
    s.tick(1);
    CHECK(s.game->shotsFired() == 1);          // primed: first shot immediately
    s.tick(5);
    CHECK(s.game->shotsFired() == 1);          // 10 Hz = one shot per 20 ticks
    s.tick(19);
    CHECK(s.game->shotsFired() == 2);
    s.tick(400);
    CHECK(s.game->shotsFired() == 3);          // only 3 fuel were loaded
    CHECK(s.game->fuelInRobot() == 0);
    CHECK(s.game->fuelOnField() == 3);
}

TEST_CASE("missing named shooter references inhibit firing with either speed source") {
    for (const char* reference : {"hoodMechanism", "turretMechanism", "activation"}) {
        for (const bool fixedSpeed : {false, true}) {
            INFO(reference, fixedSpeed);
            Session s;
            emptyField(s);
            s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
            s.addScripted("indexer").set("velocity", 20.0, "rad/s");
            auto p = shooterProps();
            if (fixedSpeed) p["fixedExitSpeedMps"] = 10.0;
            sdk::ZoneConfig zone = shooterZone("shooter", p);
            if (std::string(reference) == "activation") zone.activation.conditions.push_back(above("missing_aim", 1.0));
            else zone.properties[reference] = "missing_aim";
            s.addZone(zone);
            s.start();
            s.tick(50);
            CHECK(s.game->shotsFired() == 0);
            CHECK(s.game->fuelInRobot() == 8);
            CHECK(s.game->fuelOnField() == 0);
            CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 1);

            ScriptedMechanism& recovered = s.addScripted("missing_aim");
            recovered.set("position", 0.0, "rad");
            recovered.set("velocity", 5.0, "rad/s");
            s.directory->add(&recovered);
            s.tick();
            CHECK(s.game->shotsFired() == 1);
            CHECK(s.game->fuelInRobot() == 7);
            CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 1);
        }
    }
}

TEST_CASE("an empty activation, empty aim references and a bypassed fixed-speed flywheel remain valid") {
    for (const bool fixedSpeed : {false, true}) {
        Session s;
        emptyField(s);
        auto p = shooterProps();
        p["hoodMechanism"] = "";
        p["turretMechanism"] = "";
        if (fixedSpeed) {
            p["fixedExitSpeedMps"] = 10.0;
            p["flywheelMechanism"] = "intentionally_bypassed_missing_flywheel";
        } else {
            s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
        }
        s.addZone(makeZone("shooter", kShooterKind, p));  // no activation conditions: fires whenever the flywheel is up
        s.start();
        s.tick();
        CHECK(s.game->shotsFired() == 1);
        CHECK(s.game->fuelInRobot() == 7);
        CHECK(s.game->zones().front()->active());
        CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 0);
    }
}

namespace {

// Spawned fuel positions, sorted by (x, y) so lanes come out in a stable order.
std::vector<sdk::Vec3> fuelPositions(Session& s) {
    std::vector<sdk::Vec3> out;
    s.game->pool().forEach([&](std::uint32_t id, const sdk::Vec3&) {
        out.push_back(s.env.physics.pose(s.game->pool().bodyOf(id)).position);
    });
    std::sort(out.begin(), out.end(), [](const sdk::Vec3& a, const sdk::Vec3& b) {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    return out;
}

Session& parallelShooter(Session& s, int lanes, int preload = 8) {
    emptyField(s, 30, preload);
    s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    auto p = shooterProps();
    p["hoodAngleRad"] = 0.0;
    p["parallelBalls"] = lanes;
    s.addZone(shooterZone("shooter", p));
    return s;
}

}  // namespace

TEST_CASE("parallel shooter launches a volley side by side across the barrel") {
    Session s;
    parallelShooter(s, 3);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->shotsFired() == 3);
    CHECK(s.game->fuelInRobot() == 5);
    const std::vector<sdk::Vec3> p = fuelPositions(s);
    REQUIRE(p.size() == 3);
    CHECK(p[0].x == doctest::Approx(4.25f).epsilon(0.01));
    CHECK(p[0].y == doctest::Approx(3.f - 0.16f).epsilon(0.01));
    CHECK(p[1].y == doctest::Approx(3.f).epsilon(0.01));
    CHECK(p[2].y == doctest::Approx(3.f + 0.16f).epsilon(0.01));
}

TEST_CASE("parallel lanes follow the robot heading") {
    Session s;
    parallelShooter(s, 3);
    s.start();
    s.setChassis({4.f, 3.f, 0.f}, kYawQuarterTurn);  // facing +Y: lanes spread along X
    s.tick(1);
    const std::vector<sdk::Vec3> p = fuelPositions(s);
    REQUIRE(p.size() == 3);
    CHECK(p[0].x == doctest::Approx(4.f - 0.16f).epsilon(0.01));
    CHECK(p[2].x == doctest::Approx(4.f + 0.16f).epsilon(0.01));
    for (const sdk::Vec3& v : p) CHECK(v.y == doctest::Approx(3.25f).epsilon(0.01));
}

TEST_CASE("parallel volleys keep the total fire rate and only fire whole volleys") {
    Session s;
    parallelShooter(s, 2);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->shotsFired() == 2);
    s.tick(30);  // 10 Hz total: the next pair is due 0.2 s after the first
    CHECK(s.game->shotsFired() == 2);
    s.tick(20);
    CHECK(s.game->shotsFired() == 4);
}

TEST_CASE("a partial volley fires what the meter holds, centred") {
    Session s;
    parallelShooter(s, 3, 2);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->shotsFired() == 2);
    CHECK(s.game->fuelInRobot() == 0);
    const std::vector<sdk::Vec3> p = fuelPositions(s);
    REQUIRE(p.size() == 2);
    CHECK(p[0].y == doctest::Approx(3.f - 0.08f).epsilon(0.01));
    CHECK(p[1].y == doctest::Approx(3.f + 0.08f).epsilon(0.01));
}

TEST_CASE("each parallel lane inherits the velocity of its own spawn point") {
    Session s;
    parallelShooter(s, 2);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.env.physics.setAngularVelocity(s.chassis, {0.f, 0.f, 2.f});
    s.tick(1);
    std::vector<float> vx;
    s.game->pool().forEach([&](std::uint32_t id, const sdk::Vec3&) {
        vx.push_back(s.env.physics.linearVelocity(s.game->pool().bodyOf(id)).x);
    });
    REQUIRE(vx.size() == 2);
    CHECK(std::fabs(vx[0] - vx[1]) == doctest::Approx(2.f * 0.16f).epsilon(0.05));
}

TEST_CASE("parallelBalls is clamped to 1..8") {
    auto parse = [](double v) {
        sdk::Value p = sdk::Value::makeObject();
        p["parallelBalls"] = v;
        return parseShooterParams(p).parallelBalls;
    };
    CHECK(parseShooterParams(sdk::Value::makeObject()).parallelBalls == 1);
    CHECK(parse(0.0) == 1);
    CHECK(parse(99.0) == 8);
    CHECK(parse(3.0) == 3);
}

TEST_CASE("shooter without a flywheel reference can use a fixed exit speed, otherwise it stays quiet") {
    Session s;
    emptyField(s);
    sdk::Value p = sdk::Value::makeObject();
    p["fixedExitSpeedMps"] = 8.0;
    p["hoodAngleRad"] = 0.0;
    s.addZone(makeZone("fixed", kShooterKind, p));
    s.addZone(makeZone("silent", kShooterKind));
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->shotsFired() == 1);  // the fixed shooter (empty activation => fires when up to speed)
    auto* silent = dynamic_cast<ShooterZoneRuntime*>(s.game->zones()[1].get());
    REQUIRE(silent != nullptr);
    CHECK_FALSE(silent->feeding());
}

TEST_CASE("shot dispersion is seeded and reproducible") {
    auto run = [] {
        Session s;
        emptyField(s);
        s.env.rng.seed(1234);
        s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
        s.addScripted("indexer").set("velocity", 20.0, "rad/s");
        sdk::Value p = shooterProps();
        p["aimJitterRad"] = 0.05;
        p["exitSpeedJitter"] = 0.1;
        s.addZone(shooterZone("shooter", p));
        s.start();
        s.setChassis({4.f, 3.f, 0.f});
        s.tick(200);
        return std::make_pair(s.env.physics.stateHash(), s.game->shotsFired());
    };
    const auto a = run();
    const auto b = run();
    CHECK(a.second >= 7);
    CHECK(a == b);
}

TEST_CASE("fuel the robot shot can be collected again") {
    Session s;
    emptyField(s, 30, 1);
    s.addScripted("flywheel").set("velocity", 400.0, "rad/s");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    ScriptedMechanism& roller = s.addScripted("roller");
    sdk::Value shooter = shooterProps();
    shooter["fixedExitSpeedMps"] = 0.1;
    shooter["minExitSpeedMps"] = 0.0;
    shooter["hoodAngleRad"] = 1.5;  // nearly straight up
    s.addZone(shooterZone("shooter", shooter));
    sdk::ZoneConfig intake = pickupZone();
    intake.local.position = {0.25f, 0.f, 0.5f};  // includes the muzzle
    intake.sizeM = {0.6f, 0.6f, 1.2f};
    s.addZone(intake);
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->fuelInRobot() == 0);
    CHECK(s.game->shotsFired() == 1);
    roller.set("velocity", 100.0, "rad/s");
    s.tick(5);
    CHECK(s.game->fuelInRobot() == 1);
    CHECK(s.game->fuelOnField() == 0);
}

TEST_CASE("fuel pool: spawn, despawn, id reuse, cap, ghosts and RAII cleanup") {
    g26test::ForwardingWorld world;
    {
        FuelPool pool(world, FuelPhysicsConfig{}, 3);
        const std::uint32_t a = pool.spawn({0.f, 0.f, 1.f}, {});
        const std::uint32_t b = pool.spawn({1.f, 0.f, 1.f}, {});
        const std::uint32_t c = pool.spawn({2.f, 0.f, 1.f}, {});
        CHECK(a != 0);
        CHECK(b != 0);
        CHECK(c != 0);
        CHECK(pool.count() == 3);
        CHECK(pool.spawn({3.f, 0.f, 1.f}, {}) == 0);  // cap reached
        CHECK(world.stats().bodies == 3);

        pool.despawn(b);
        pool.despawn(b);  // double despawn is harmless
        pool.despawn(999);
        CHECK(pool.count() == 2);
        CHECK_FALSE(pool.alive(b));
        CHECK(pool.position(b) == sdk::Vec3{});
        const std::uint32_t d = pool.spawn({4.f, 0.f, 1.f}, {});
        CHECK(d == b);  // slot reused
        CHECK(pool.position(d).x == doctest::Approx(4.f));

        // the sphere is a real fuel: radius, density from the configured mass, sensor events enabled
        const sdk::ShapeDesc& sd = world.shapeLog.front();
        CHECK(std::get<sdk::SphereShape>(sd.geometry).radius == doctest::Approx(0.075f));
        const float volume = 4.f / 3.f * 3.14159265f * 0.075f * 0.075f * 0.075f;
        CHECK(sd.density * volume == doctest::Approx(0.215f).epsilon(0.001));
        CHECK(sd.sensorEvents);
        CHECK(tagKind(sd.userTag) == EntityKind::Fuel);
        CHECK((sd.mask & sdk::layer::Sensor) != 0);  // docs/reference/physics.md: fuel must include the sensor bit
        CHECK(sd.category == sdk::layer::Fuel);
        CHECK((sd.mask & kShotFuelLayer) != 0);  // settled fuel collides with ghosts

        // ghost fuel ignores the robot and other ghosts until its time has come
        FuelPool::SpawnOptions ghost;
        ghost.ghostUntilUs = 1000;
        pool.despawn(a);
        const std::uint32_t g = pool.spawn({0.f, 0.f, 2.f}, ghost);
        REQUIRE(g != 0);
        const sdk::ShapeDesc& gd = world.shapeLog.back();
        CHECK(gd.category == kShotFuelLayer);
        CHECK((gd.mask & sdk::layer::Robot) == 0);
        CHECK((gd.mask & kShotFuelLayer) == 0);
        CHECK((gd.mask & sdk::layer::Fuel) != 0);
        CHECK((gd.mask & sdk::layer::Field) != 0);
        CHECK((gd.mask & sdk::layer::Sensor) != 0);
        pool.updateGhosts(999);
        CHECK(world.filterCalls == 0);
        pool.updateGhosts(1000);
        CHECK(world.filterCalls == 1);
        CHECK((world.lastMask & sdk::layer::Robot) != 0);
        CHECK((world.lastMask & kShotFuelLayer) != 0);
        CHECK(world.lastCategory == sdk::layer::Fuel);
        pool.updateGhosts(2000);
        CHECK(world.filterCalls == 1);  // restored once
    }
    CHECK(world.stats().bodies == 0);  // destructor removed every body
}

TEST_CASE("fuel pool survives a world that refuses shapes") {
    g26test::ForwardingWorld world;
    world.shapeBudget = 0;
    FuelPool pool(world, FuelPhysicsConfig{}, 10);
    CHECK(pool.spawn({0.f, 0.f, 1.f}, {}) == 0);
    CHECK(pool.count() == 0);
    CHECK(world.stats().bodies == 0);  // the orphan body was removed again
}
