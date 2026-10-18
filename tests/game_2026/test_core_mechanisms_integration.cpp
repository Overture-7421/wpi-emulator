// The 2026 wrappers driven by the REAL generic core mechanisms (the single "mechanism" template as roller / flywheel / indexer):
// proves the named values the plugin reads (velocity, position) are the ones core actually publishes, end to end from a motor command to a fuel body leaving the muzzle.
#include <doctest/doctest.h>

#include <cmath>

#include "test_support.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/plugins/game_2026/fuel_intake.h"
#include "wpiemu/plugins/game_2026/fuel_shooter.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::boxValue;
using g26test::Session;

namespace {

void driveVolts(Session& s, const std::string& id, double volts) {
    sdk::MotorOutput o;
    o.voltageV = volts;
    s.env.hub.setMotorOutput({id, "motor"}, o);
}

void withCoreTemplates(Session& s) {
    s.extraTemplates = [](sdk::MechanismFactory& f) { REQUIRE(core::mechanisms::registerGenericMechanisms(f).ok()); };
}

double value(const sdk::MechanismState& st, const char* name) {
    for (const sdk::NamedValue& v : st.values)
        if (v.name == name) return v.value;
    return std::nan("");
}

}  // namespace

TEST_CASE("core roller + shooter + indexer drive the 2026 intake and shooter end to end") {
    Session s;
    withCoreTemplates(s);
    s.section()["neutralFuel"] = 0;
    s.section()["depotFuel"] = 0;
    for (const char* id : {"roller", "flywheel", "indexer"}) s.env.hub.autoBind({id, "motor"}, sdk::SlotType::Motor);

    sdk::Value flyProps = sdk::Value::makeObject();
    flyProps["gearRatio"] = 1.0;
    s.addMechanism("roller", "mechanism");
    s.addMechanism("flywheel", "mechanism", flyProps);
    s.addMechanism("indexer", "mechanism");
    sdk::Value intake = sdk::Value::makeObject();
    intake["rollerMechanism"] = "roller";
    intake["pickupZone"] = boxValue({0.45f, 0.f, 0.2f}, {0.30f, 0.60f, 0.40f});
    s.addMechanism("fuelIntake", kIntakeType, intake);
    sdk::Value shooter = sdk::Value::makeObject();
    shooter["flywheelMechanism"] = "flywheel";
    shooter["flywheelRadiusM"] = 0.05;
    shooter["feedMechanism"] = "indexer";
    s.addMechanism("fuelShooter", kShooterType, shooter);

    s.start();
    s.setChassis({4.f, 3.f, 0.f});
    s.game->pool().spawn({4.45f, 3.f, 0.075f}, {});

    // everything idle: nothing happens
    s.tick(100);
    CHECK(s.game->fuelOnField() == 1);
    CHECK(s.game->shotsFired() == 0);

    // spin the roller: the fuel in the pickup zone is taken
    driveVolts(s, "roller", 12.0);
    s.tick(200);
    CHECK(s.game->fuelOnField() == 0);
    CHECK(s.game->fuelInRobot() == 9);

    // spin the flywheel; the indexer is still off: no shot yet, but the speed shows up
    driveVolts(s, "flywheel", 12.0);
    s.tick(400);
    sdk::MechanismState fly;
    for (auto& m : s.mechanisms)
        if (m->id() == "flywheel") fly = m->state();
    CHECK(value(fly, "velocity") > 100.0);  // rad/s
    CHECK(s.game->shotsFired() == 0);

    // feed: shots leave at omega * r * efficiency
    driveVolts(s, "indexer", 12.0);
    s.tick(10);
    CHECK(s.game->shotsFired() >= 1);
    sdk::MechanismState shot;
    for (auto& m : s.mechanisms)
        if (m->id() == "fuelShooter") shot = m->state();
    CHECK(shot.active);
    CHECK(value(shot, "exitSpeed") == doctest::Approx(std::fabs(value(fly, "velocity")) * 0.05 * 0.6).epsilon(0.05));
    CHECK(value(shot, "exitSpeed") > 4.0);
}
