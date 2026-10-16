#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/fuel_intake.h"
#include "wpiemu/plugins/game_2026/fuel_pool.h"
#include "wpiemu/plugins/game_2026/fuel_shooter.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::boxValue;
using g26test::ScriptedMechanism;
using g26test::Session;

namespace {

// Session with no staged fuel on the field, so tests only see the fuel they create.
void emptyField(Session& s, int capacity = 30, int preload = 8) {
    s.section()["neutralFuel"] = 0;
    s.section()["depotFuel"] = 0;
    s.section()["capacity"] = capacity;
    s.section()["preloadFuel"] = preload;
}

sdk::Value intakeProps(const char* roller = "roller") {
    sdk::Value p = sdk::Value::makeObject();
    p["pickupZone"] = boxValue({0.45f, 0.f, 0.2f}, {0.30f, 0.60f, 0.40f});
    p["rollerMechanism"] = roller;
    p["rollerThreshold"] = 20.0;
    p["intakeRateHz"] = 0.0;  // unlimited unless a test limits it
    return p;
}

sdk::Value shooterProps() {
    sdk::Value p = sdk::Value::makeObject();
    p["flywheelMechanism"] = "flywheel";
    p["feedMechanism"] = "indexer";
    p["feedThreshold"] = 5.0;
    p["fireRateHz"] = 10.0;
    p["hoodAngleRad"] = 1.0471975512;  // 60 degrees
    p["flywheelRadiusM"] = 0.05;
    p["efficiency"] = 0.5;
    return p;
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

TEST_CASE("intake absorbs fuel in its zone only while the roller spins") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addMechanism("intake", kIntakeType, intakeProps());
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

    auto* intake = dynamic_cast<FuelIntakeMechanism*>(s.mechanisms.front().get());
    REQUIRE(intake != nullptr);
    CHECK(intake->state().active);
    CHECK(intake->state().values.front().name == "active");
}

TEST_CASE("roller speed units are converted from the published unit") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addMechanism("intake", kIntakeType, intakeProps());
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

TEST_CASE("intake stops at capacity and a full robot leaves fuel on the field") {
    Session s;
    emptyField(s, /*capacity=*/10, /*preload=*/8);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addMechanism("intake", kIntakeType, intakeProps());
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

TEST_CASE("intake rate limit and nearest-first order") {
    Session s;
    emptyField(s, 100, 0);
    ScriptedMechanism& roller = s.addScripted("roller");
    sdk::Value p = intakeProps();
    p["intakeRateHz"] = 10.0;
    s.addMechanism("intake", kIntakeType, p);
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

TEST_CASE("the pickup zone follows the robot pose and the mechanism mount") {
    Session s;
    emptyField(s);
    ScriptedMechanism& roller = s.addScripted("roller");
    s.addMechanism("intake", kIntakeType, intakeProps());
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

TEST_CASE("intake activation modes: deployed, roller AND deployed, OR, always") {
    Session s;
    emptyField(s, 100, 0);
    ScriptedMechanism& roller = s.addScripted("roller");
    ScriptedMechanism& arm = s.addScripted("arm");
    auto props = [&](const char* mode) {
        sdk::Value p = intakeProps();
        p["activation"] = mode;
        p["deployMechanism"] = "arm";
        p["deployedPosition"] = 1.5;
        p["deployedTolerance"] = 0.1;
        return p;
    };
    s.addMechanism("deployedOnly", kIntakeType, props("deployed"), sdk::Pose3{{0.f, 0.f, 0.f}, sdk::Quat{}});
    s.addMechanism("both", kIntakeType, props("roller_and_deployed"), sdk::Pose3{{0.f, 10.f, 0.f}, sdk::Quat{}});
    s.addMechanism("either", kIntakeType, props("roller_or_deployed"), sdk::Pose3{{0.f, 20.f, 0.f}, sdk::Quat{}});
    s.addMechanism("always", kIntakeType, props("always"), sdk::Pose3{{0.f, 30.f, 0.f}, sdk::Quat{}});
    s.start();
    s.setChassis({2.f, 2.f, 0.f});

    auto active = [&](std::size_t i) { return s.mechanisms[i]->state().active; };
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

TEST_CASE("a roller reference that does not exist never activates and warns once") {
    Session s;
    emptyField(s);
    s.addMechanism("intake", kIntakeType, intakeProps("ghost_roller"));
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});
    s.tick(50);
    CHECK(s.game->fuelOnField() == 1);
    CHECK_FALSE(s.mechanisms.front()->state().active);
    CHECK(s.env.logger.count(sdk::LogLevel::Warn) == 1);
}

TEST_CASE("an intake with no roller reference is always on") {
    Session s;
    emptyField(s);
    sdk::Value p = intakeProps("");
    s.addMechanism("passive", kIntakeType, p);
    s.start();
    s.setChassis({2.f, 2.f, 0.f});
    s.game->pool().spawn({2.45f, 2.f, 0.075f}, {});
    s.tick(2);
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->fuelInRobot() == 9);
}

TEST_CASE("shooter launches real fuel bodies with flywheel speed x efficiency along the hood angle") {
    Session s;
    emptyField(s);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    ScriptedMechanism& indexer = s.addScripted("indexer");
    s.addMechanism("shooter", kShooterType, shooterProps());
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    flywheel.set("velocity", 400.0, "rad/s");  // 20 m/s surface, 10 m/s exit
    indexer.set("velocity", 0.0, "rad/s");

    s.tick(5);
    CHECK(s.game->shotsFired() == 0);        // indexer not feeding
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->fuelInRobot() == 8);

    indexer.set("velocity", 20.0, "rad/s");
    s.tick(1);
    REQUIRE(s.game->shotsFired() == 1);
    CHECK(s.game->fuelInRobot() == 7);
    REQUIRE(s.game->fuelOnField() == 1);

    std::uint32_t id = 0;
    s.game->pool().forEach([&](std::uint32_t i, const sdk::Vec3&) { id = i; });
    const sdk::BodyId body = s.game->pool().bodyOf(id);
    const sdk::Vec3 p = s.env.physics.pose(body).position;
    CHECK(p.x == doctest::Approx(4.25f).epsilon(0.01));   // muzzle offset (0.25, 0, 0.55)
    CHECK(p.y == doctest::Approx(3.f).epsilon(0.01));
    CHECK(p.z == doctest::Approx(0.55f).epsilon(0.02));
    const sdk::Vec3 v = s.env.physics.linearVelocity(body);
    CHECK(v.x == doctest::Approx(10.f * std::cos(1.0471975f)).epsilon(0.01));
    CHECK(v.y == doctest::Approx(0.f).epsilon(0.01));
    CHECK(v.z == doctest::Approx(10.f * std::sin(1.0471975f)).epsilon(0.01));

    const sdk::MechanismState st = s.mechanisms.front()->state();
    CHECK(st.active);
    CHECK(st.values[0].name == "feeding");
    CHECK(st.values[1].name == "exitSpeed");
    CHECK(st.values[1].value == doctest::Approx(10.0));
}

TEST_CASE("shots follow the robot heading and inherit its velocity") {
    Session s;
    emptyField(s);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    ScriptedMechanism& indexer = s.addScripted("indexer");
    flywheel.set("velocity", 400.0, "rad/s");
    indexer.set("velocity", 20.0, "rad/s");
    s.addMechanism("shooter", kShooterType, shooterProps());
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
    m.addMechanism("shooter", kShooterType, shooterProps());
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
    s.addMechanism("shooter", kShooterType, p);
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

    const sdk::MechanismState st = s.mechanisms.front()->state();
    REQUIRE(st.links.size() == 1);
    CHECK(st.links[0].name == "muzzle");
}

TEST_CASE("fire rate, empty meter and slow flywheels") {
    Session s;
    emptyField(s, 30, 3);
    ScriptedMechanism& flywheel = s.addScripted("flywheel");
    s.addScripted("indexer").set("velocity", 20.0, "rad/s");
    s.addMechanism("shooter", kShooterType, shooterProps());
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

TEST_CASE("shooter without a flywheel reference can use a fixed exit speed, otherwise it stays quiet") {
    Session s;
    emptyField(s);
    sdk::Value p = sdk::Value::makeObject();
    p["fixedExitSpeedMps"] = 8.0;
    p["hoodAngleRad"] = 0.0;
    s.addMechanism("fixed", kShooterType, p);
    s.addMechanism("silent", kShooterType, sdk::Value::makeObject());
    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.tick(1);
    CHECK(s.game->shotsFired() == 1);  // the fixed shooter (feed reference empty => fires when up to speed)
    CHECK_FALSE(s.mechanisms[1]->state().active);
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
        s.addMechanism("shooter", kShooterType, p);
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
    s.addMechanism("shooter", kShooterType, shooter);
    sdk::Value intake = intakeProps();
    intake["pickupZone"] = boxValue({0.25f, 0.f, 0.5f}, {0.6f, 0.6f, 1.2f});  // includes the muzzle volume
    s.addMechanism("intake", kIntakeType, intake);
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
        CHECK((sd.mask & sdk::layer::Sensor) != 0);  // ADR 0003: fuel must include the sensor bit

        // ghost fuel ignores the robot until its time has come
        FuelPool::SpawnOptions ghost;
        ghost.ghostUntilUs = 1000;
        pool.despawn(a);
        const std::uint32_t g = pool.spawn({0.f, 0.f, 2.f}, ghost);
        REQUIRE(g != 0);
        CHECK((world.shapeLog.back().mask & sdk::layer::Robot) == 0);
        pool.updateGhosts(999);
        CHECK(world.filterCalls == 0);
        pool.updateGhosts(1000);
        CHECK(world.filterCalls == 1);
        CHECK((world.lastMask & sdk::layer::Robot) != 0);
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
