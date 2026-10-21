#include <doctest/doctest.h>

#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>

#include "wpiemu/core/mechanisms/linear_system.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/core/mechanisms/plants.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/testing/fake_mechanism_directory.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu::core::mechanisms;
namespace sdk = wpiemu::sdk;
using wpiemu::sdk::testing::FakeSimEnvironment;

namespace {

constexpr double kDt = 0.005;

struct Rig {
    sdk::MechanismFactory factory;
    FakeSimEnvironment env;

    Rig() { REQUIRE(registerGenericMechanisms(factory).ok()); }

    sdk::Result<std::unique_ptr<sdk::IMechanism>> create(const std::string& type, const std::string& id,
                                                         sdk::Value props = sdk::Value::makeObject()) {
        sdk::MechanismConfig cfg;
        cfg.id = id;
        cfg.type = type;
        cfg.properties = std::move(props);
        return factory.create(type, sdk::MechanismBuildContext{cfg, env.ctx});
    }

    std::unique_ptr<sdk::IMechanism> make(const std::string& type, const std::string& id,
                                          sdk::Value props = sdk::Value::makeObject()) {
        auto r = create(type, id, std::move(props));
        REQUIRE(r.ok());
        return std::move(r).value();
    }

    void bind(const std::string& id, const std::string& slot, sdk::SlotType type) {
        env.hub.autoBind({id, slot}, type);
    }
    void drive(const std::string& id, const std::string& slot, double volts) {
        sdk::MotorOutput o;
        o.voltageV = volts;
        env.hub.setMotorOutput({id, slot}, o);
    }
    void run(sdk::IMechanism& m, int ticks) {
        for (int i = 0; i < ticks; ++i) {
            m.tick(env.ctx, kDt);
            env.advance();
        }
    }
};

double value(const sdk::MechanismState& s, const std::string& name) {
    for (const sdk::NamedValue& v : s.values)
        if (v.name == name) return v.value;
    FAIL("missing value " << name);
    return 0.0;
}

const sdk::LinkPose* link(const sdk::MechanismState& s, const std::string& name) {
    for (const sdk::LinkPose& l : s.links)
        if (l.name == name) return &l;
    return nullptr;
}

sdk::Value props(std::initializer_list<std::pair<const char*, sdk::Value>> kv) {
    sdk::Value v = sdk::Value::makeObject();
    for (const auto& [k, x] : kv) v[k] = x;
    return v;
}


// Variants of the one template, as a user would configure them.
sdk::Value with(sdk::Value base, std::initializer_list<std::pair<const char*, sdk::Value>> kv) {
    for (const auto& [k, x] : kv) base[k] = x;
    return base;
}

sdk::Value elevatorProps(std::initializer_list<std::pair<const char*, sdk::Value>> extra = {}) {
    return with(props({{"motion", "linear"}, {"motorCount", 2}, {"gearRatio", 8.0}, {"massKg", 6.0}, {"drumRadiusM", 0.0225},
                       {"minPosition", 0.0}, {"maxPosition", 0.8}, {"simulateGravity", true}, {"viscousFrictionNmS", 0.0}}),
                extra);
}

sdk::Value pivotProps(std::initializer_list<std::pair<const char*, sdk::Value>> extra = {}) {
    return with(props({{"motion", "arm"}, {"gearRatio", 60.0}, {"momentOfInertiaKgM2", 0.0}, {"massKg", 2.0}, {"lengthM", 0.3},
                       {"minPosition", 0.0}, {"maxPosition", 1.5}, {"simulateGravity", true}, {"viscousFrictionNmS", 0.0}}),
                extra);
}

sdk::Value turretProps(std::initializer_list<std::pair<const char*, sdk::Value>> extra = {}) {
    return with(props({{"gearRatio", 40.0}, {"momentOfInertiaKgM2", 0.05}, {"minPosition", -3.3}, {"maxPosition", 3.3}}), extra);
}

constexpr const char* kType = "mechanism";

}  // namespace

TEST_CASE("the core registers exactly one template, with complete and consistent metadata") {
    Rig rig;
    CHECK(rig.factory.size() == 1);
    const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(kType);
    REQUIRE(info != nullptr);
    CHECK(info->type == kType);
    CHECK(info->origin == "core");
    CHECK_FALSE(info->displayName.empty());
    CHECK_FALSE(info->description.empty());
    CHECK_FALSE(info->defaultGeometry.empty());
    CHECK(sdk::templateLinkIssues(*info).empty());
    CHECK(info->links.size() == 1);
    CHECK(info->motionAxes.size() == 1);

    std::set<std::string> slotNames;
    bool hasRequiredMotor = false;
    for (const sdk::SlotSpec& s : info->slotSpecs) {
        CHECK(slotNames.insert(s.name).second);   // unique
        CHECK_FALSE(s.label.empty());
        CHECK(s.type != sdk::SlotType::BeamBreak);   // beam breaks belong to the game plugin
        if (s.required) {
            CHECK(s.type == sdk::SlotType::Motor);
            hasRequiredMotor = true;
        }
    }
    CHECK(hasRequiredMotor);

    std::set<std::string> keys;
    for (const sdk::PropertySpec& p : info->properties.properties) {
        CHECK(keys.insert(p.key).second);   // unique
        CHECK_FALSE(p.label.empty());
        CHECK_FALSE(p.group.empty());
        if (p.type == sdk::PropertyType::Enum) {
            bool found = false;
            for (const sdk::EnumOption& o : p.options) found = found || o.value == p.defaultValue.asString();
            CHECK(found);
        }
    }
    CHECK(keys.count("motion") == 1);
    // a new mechanism is a ready-to-run 10:1 Kraken that holds its position at neutral
    const sdk::Value defaults = sdk::defaultsOf(info->properties);
    CHECK(defaults.find("gearRatio")->asDouble() == 10.0);
    CHECK(defaults.find("neutralMode")->asString() == "brake");
    // its shape sits off the axis, so a rotation is visible
    REQUIRE(info->defaultGeometry.size() == 1);
    CHECK(info->defaultGeometry[0].name == "mover");
    CHECK(info->defaultGeometry[0].local.position.x > 0.f);
    for (const sdk::Issue& issue : sdk::validateProperties(info->properties, sdk::defaultsOf(info->properties)))
        CHECK_MESSAGE(issue.severity != sdk::Severity::Error, issue.path << ": " << issue.message);

    // registering twice is an error, not a silent override
    sdk::MechanismFactory again;
    REQUIRE(registerGenericMechanisms(again).ok());
    CHECK(registerGenericMechanisms(again).error().code == sdk::ErrorCode::AlreadyExists);
}

TEST_CASE("every motion variant builds from defaults and publishes its mover link") {
    Rig rig;
    const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(kType);
    for (const sdk::Value& variant : {sdk::Value::makeObject(), elevatorProps(), pivotProps(), turretProps()}) {
        auto m = rig.make(kType, "m", variant);
        CHECK(m->id() == "m");
        CHECK(m->type() == kType);
        for (int i = 0; i < 20; ++i) m->tick(rig.env.ctx, kDt);
        const sdk::MechanismState s = m->state();
        for (const sdk::LinkSpec& l : info->links) {
            CHECK(link(s, l.name) != nullptr);
            CHECK(m->linkPose(l.name).has_value());
        }
        CHECK(m->linkPose("root").has_value());
        CHECK_FALSE(m->linkPose("no-such-link").has_value());
        for (const sdk::NamedValue& v : s.values) CHECK(std::isfinite(v.value));
        CHECK(m->batteryCurrentA() >= 0.0);
    }
}

TEST_CASE("rotary spin-up matches the WPILib LinearSystemId solution and drives the sensors") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    rig.bind("r", "encoder", sdk::SlotType::PositionSensor);
    rig.bind("r", "velocitySensor", sdk::SlotType::VelocitySensor);
    auto m = rig.make(kType, "r", props({{"axis", "y"}, {"gearRatio", 3.0}, {"momentOfInertiaKgM2", 0.002}, {"viscousFrictionNmS", 0.0}}));
    rig.drive("r", "motor", 12.0);
    rig.run(*m, 100);

    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    const LinearSystem sys = dcMotorSystem(kraken, 1, 0.002, 3.0);
    const double a = sys.A(1, 1), b = sys.B(1, 0), t = 100 * kDt;
    const double vInf = b * 12.0 / -a;
    const double w = vInf * (1.0 - std::exp(a * t));
    const double x = vInf * (t - (std::exp(a * t) - 1.0) / a);

    const sdk::MechanismState s = m->state();
    CHECK(value(s, "velocity") == doctest::Approx(w).epsilon(1e-9));
    CHECK(value(s, "position") == doctest::Approx(x).epsilon(1e-9));
    CHECK(value(s, "voltage") == doctest::Approx(12.0));
    CHECK(s.active);

    // rotor side = after the 3:1 gearbox; encoder on the output (ratio 1)
    const auto fb = rig.env.hub.lastMotorFeedback({"r", "motor"});
    REQUIRE(fb.has_value());
    CHECK(fb->rotorPositionRot == doctest::Approx(3.0 * x / sdk::kTwoPi).epsilon(1e-9));
    CHECK(fb->rotorVelocityRps == doctest::Approx(3.0 * w / sdk::kTwoPi).epsilon(1e-9));
    CHECK(fb->rotorAccelerationRps2 > 0.0);   // still accelerating
    const auto enc = rig.env.hub.lastEncoder({"r", "encoder"});
    REQUIRE(enc.has_value());
    CHECK(enc->positionRot == doctest::Approx(x / sdk::kTwoPi).epsilon(1e-9));
    CHECK(enc->velocityRps == doctest::Approx(w / sdk::kTwoPi).epsilon(1e-9));
    const auto vel = rig.env.hub.lastEncoder({"r", "velocitySensor"});
    REQUIRE(vel.has_value());
    CHECK(vel->velocityRps == doctest::Approx(w / sdk::kTwoPi).epsilon(1e-9));

    // the mover link rotates about the selected axis (+Y) by the wrapped position
    const sdk::LinkPose* l = link(s, "mover");
    REQUIRE(l != nullptr);
    const sdk::Vec3 axisCheck = l->pose.rotation.rotate({0.f, 1.f, 0.f});
    CHECK(axisCheck.y == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(std::abs(2.0 * std::atan2(l->pose.rotation.y, l->pose.rotation.w) - std::remainder(x, sdk::kTwoPi)) < 1e-4);
}

TEST_CASE("sensor ratio, inversion and offset apply to position sensors only") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    rig.bind("r", "encoder", sdk::SlotType::PositionSensor);
    auto m = rig.make(kType, "r", props({{"sensorRatio", 2.0}, {"sensorInverted", true}, {"sensorOffsetRot", 0.5}}));
    rig.drive("r", "motor", 6.0);
    rig.run(*m, 40);
    const double pos = value(m->state(), "position");
    const auto enc = rig.env.hub.lastEncoder({"r", "encoder"});
    REQUIRE(enc.has_value());
    CHECK(enc->positionRot == doctest::Approx(-2.0 * pos / sdk::kTwoPi + 0.5).epsilon(1e-9));
    CHECK(enc->velocityRps < 0.0);
}

TEST_CASE("battery sag limits the applied voltage and the stator limit caps the current") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "r", props({{"statorCurrentLimitA", 40.0}}));
    rig.env.hub.setBatteryVoltage(8.0);
    rig.drive("r", "motor", 12.0);
    m->tick(rig.env.ctx, kDt);   // at stall: the limit, not the supply, decides
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    CHECK(value(m->state(), "current") == doctest::Approx(40.0).epsilon(1e-9));
    CHECK(value(m->state(), "voltage") == doctest::Approx(40.0 * kraken.resistanceOhm()).epsilon(1e-9));

    Rig sag;
    sag.bind("s", "motor", sdk::SlotType::Motor);
    auto unlimited = sag.make(kType, "s");
    sag.env.hub.setBatteryVoltage(8.0);
    sag.drive("s", "motor", 12.0);
    sag.run(*unlimited, 3);
    CHECK(value(unlimited->state(), "voltage") == doctest::Approx(8.0));
    CHECK(unlimited->batteryCurrentA() > 0.0);   // about V*I/Vbat = I at full duty
}

TEST_CASE("battery current at stall equals the stall current and is zero when idle") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "r", props({{"gearRatio", 100.0}, {"momentOfInertiaKgM2", 10.0}, {"neutralMode", "coast"}}));
    rig.drive("r", "motor", 12.0);
    m->tick(rig.env.ctx, kDt);
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    CHECK(m->batteryCurrentA() == doctest::Approx(kraken.stallCurrentA).epsilon(1e-6));
    rig.drive("r", "motor", 0.0);
    rig.run(*m, 5);
    CHECK(m->batteryCurrentA() == doctest::Approx(0.0).epsilon(1e-6));   // coast: no current
}

TEST_CASE("follower motors: mapped ones add their own voltage, unmapped ones mirror the primary") {
    {
        Rig rig;
        rig.bind("r", "motor", sdk::SlotType::Motor);
        auto m = rig.make(kType, "r", props({{"motorCount", 2}}));
        rig.drive("r", "motor", 10.0);
        m->tick(rig.env.ctx, kDt);
        CHECK(value(m->state(), "voltage") == doctest::Approx(10.0));   // mirrored
        CHECK(value(m->state(), "current") == doctest::Approx(2.0 * 10.0 / MotorCatalog::builtin().spec("kraken_x60")->resistanceOhm()));
    }
    {
        Rig rig;
        rig.bind("r", "motor", sdk::SlotType::Motor);
        rig.bind("r", "motor2", sdk::SlotType::Motor);
        auto m = rig.make(kType, "r", props({{"motorCount", 2}}));
        rig.drive("r", "motor", 10.0);
        rig.drive("r", "motor2", 4.0);
        m->tick(rig.env.ctx, kDt);
        CHECK(value(m->state(), "voltage") == doctest::Approx(7.0));    // mean over the two motors
        CHECK(rig.env.hub.lastMotorFeedback({"r", "motor2"}).has_value());
    }
}

TEST_CASE("opposed motors cancel their torque but both still draw current") {
    const double resistance = MotorCatalog::builtin().spec("kraken_x60")->resistanceOhm();
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    rig.bind("r", "motor2", sdk::SlotType::Motor);
    auto m = rig.make(kType, "r", props({{"motorCount", 2}}));
    rig.drive("r", "motor", 10.0);
    rig.drive("r", "motor2", -10.0);   // mechanism frame: a reversed motor driven forward by robot code
    rig.run(*m, 30);
    CHECK(value(m->state(), "voltage") == doctest::Approx(0.0));
    CHECK(value(m->state(), "velocity") == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(value(m->state(), "current") == doctest::Approx(2.0 * 10.0 / resistance));
    CHECK(m->batteryCurrentA() > 0.0);
}

TEST_CASE("five motors on one mechanism add up: mean voltage into the plant, summed current") {
    Rig rig;
    const double volts[5] = {12.0, 12.0, 6.0, 6.0, 0.0};
    const char* slots[5] = {"motor", "motor2", "motor3", "motor4", "motor5"};
    for (const char* slot : slots) rig.bind("s", slot, sdk::SlotType::Motor);
    auto m = rig.make(kType, "s", props({{"motorCount", 5}}));
    for (int i = 0; i < 5; ++i) rig.drive("s", slots[i], volts[i]);
    m->tick(rig.env.ctx, kDt);
    CHECK(value(m->state(), "voltage") == doctest::Approx(7.2));
    const double resistance = MotorCatalog::builtin().spec("kraken_x60")->resistanceOhm();
    CHECK(value(m->state(), "current") == doctest::Approx((12.0 + 12.0 + 6.0 + 6.0) / resistance));
    for (const char* slot : slots) CHECK(rig.env.hub.lastMotorFeedback({"s", slot}).has_value());
}

TEST_CASE("motor slots beyond the motor count are not read and carry a count marker") {
    Rig rig;
    const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(kType);
    REQUIRE(info != nullptr);
    int motors = 0;
    for (const sdk::SlotSpec& s : info->slotSpecs)
        if (s.type == sdk::SlotType::Motor) {
            ++motors;
            CHECK(s.countKey == "motorCount");
            CHECK(s.ordinal == motors);
        }
    CHECK(motors == sdk::kMaxMotorsPerAxis);

    rig.bind("s", "motor", sdk::SlotType::Motor);
    rig.bind("s", "motor3", sdk::SlotType::Motor);
    auto m = rig.make(kType, "s", props({{"motorCount", 2}}));
    rig.drive("s", "motor", 10.0);
    rig.drive("s", "motor3", -12.0);   // inactive
    m->tick(rig.env.ctx, kDt);
    CHECK(value(m->state(), "voltage") == doctest::Approx(10.0));   // motor 2 unmapped: mirrors motor 1
}

TEST_CASE("neutral mode: a brake decelerates faster than coast") {
    auto coastDown = [](const char* mode) {
        Rig rig;
        rig.bind("r", "motor", sdk::SlotType::Motor);
        auto m = rig.make(kType, "r", props({{"neutralMode", mode}}));
        rig.drive("r", "motor", 12.0);
        rig.run(*m, 100);
        rig.drive("r", "motor", 0.0);
        rig.run(*m, 40);
        return value(m->state(), "velocity");
    };
    const double coast = coastDown("coast"), brake = coastDown("brake");
    CHECK(coast > 0.0);
    CHECK(brake < coast);
}

TEST_CASE("a flywheel is a rotary mechanism: it spins up toward the free speed and reverses") {
    Rig rig;
    rig.bind("s", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "s", props({{"gearRatio", 1.0}, {"momentOfInertiaKgM2", 0.004}, {"viscousFrictionNmS", 0.0}}));
    CHECK_FALSE(m->state().active);
    rig.drive("s", "motor", 12.0);
    rig.run(*m, 400);
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    const LinearSystem sys = dcMotorSystem(kraken, 1, 0.004, 1.0);
    const sdk::MechanismState s = m->state();
    const double vInf = 12.0 * kraken.kvRadPerSecPerVolt();
    CHECK(value(s, "velocity") == doctest::Approx(vInf * (1.0 - std::exp(sys.A(1, 1) * 400 * kDt))).epsilon(1e-9));
    CHECK(value(s, "velocity") < vInf);
    CHECK(s.active);
    rig.drive("s", "motor", -12.0);
    rig.run(*m, 800);
    CHECK(value(m->state(), "velocity") < 0.0);   // reverses
}

TEST_CASE("linear: gravity, hard stops, limit switches and the mover link") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    rig.bind("e", "reverseLimit", sdk::SlotType::LimitSwitch);
    rig.bind("e", "forwardLimit", sdk::SlotType::LimitSwitch);
    rig.bind("e", "encoder", sdk::SlotType::PositionSensor);
    auto m = rig.make(kType, "e", elevatorProps());

    m->tick(rig.env.ctx, kDt);   // sits on the lower stop (brake, V = 0)
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == true);
    CHECK(rig.env.hub.lastDigital({"e", "forwardLimit"}).value() == false);

    rig.drive("e", "motor", 12.0);
    rig.run(*m, 400);
    sdk::MechanismState s = m->state();
    CHECK(value(s, "position") == 0.8);
    CHECK(value(s, "velocity") == 0.0);
    CHECK(value(s, "atUpperLimit") == 1.0);
    CHECK(rig.env.hub.lastDigital({"e", "forwardLimit"}).value() == true);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == false);
    const sdk::LinkPose* mover = link(s, "mover");
    REQUIRE(mover != nullptr);
    CHECK(mover->pose.position.z == doctest::Approx(0.8f));
    CHECK(mover->pose.position.x == 0.0f);
    // encoder on the drum: position / (2 pi r)
    CHECK(rig.env.hub.lastEncoder({"e", "encoder"})->positionRot == doctest::Approx(0.8 / (sdk::kTwoPi * 0.0225)));

    // stall current at the top
    CHECK(value(s, "current") == doctest::Approx(2.0 * 12.0 / MotorCatalog::builtin().spec("kraken_x60")->resistanceOhm()));
    rig.drive("e", "motor", -12.0);
    rig.run(*m, 600);
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == true);

    m->reset();
    CHECK(value(m->state(), "position") == 0.0);
}

TEST_CASE("linear without power sinks under gravity unless held by the feed-forward voltage") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "e", elevatorProps({{"startPosition", 0.4}, {"neutralMode", "coast"}}));
    rig.drive("e", "motor", 0.0);
    rig.run(*m, 60);
    CHECK(value(m->state(), "position") < 0.4);

    // feed-forward: kG = g / B (WPILib ElevatorFeedforward-style), computed from the same LinearSystemId
    auto holdTest = [&](double volts) {
        Rig r2;
        r2.bind("e", "motor", sdk::SlotType::Motor);
        auto e2 = r2.make(kType, "e", elevatorProps({{"startPosition", 0.4}}));
        r2.drive("e", "motor", volts);
        r2.run(*e2, 200);
        return value(e2->state(), "position");
    };
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    const LinearSystem sys = elevatorSystem(kraken, 2, 6.0, 0.0225, 8.0);
    const double kg = 9.8 / sys.B(1, 0);
    CHECK(holdTest(kg) == doctest::Approx(0.4).epsilon(1e-9));
    CHECK(holdTest(kg * 1.2) > 0.4);
    CHECK(holdTest(kg * 0.8) < 0.4);
}

TEST_CASE("linear travel axis moves the mover link, and only Z works against gravity") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "e", elevatorProps({{"axis", "x"}, {"startPosition", 0.3}}));
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* c = link(snapshot, "mover");
    REQUIRE(c != nullptr);
    CHECK(c->pose.position.x == doctest::Approx(0.3f));   // published before the first tick
    CHECK(c->pose.position.z == 0.0f);

    // along X nothing pulls it down
    rig.drive("e", "motor", 0.0);
    rig.run(*m, 60);
    CHECK(value(m->state(), "position") == doctest::Approx(0.3).epsilon(1e-9));
}

TEST_CASE("arm: angle from horizontal raises the +X end, gravity is held by the feed-forward") {
    Rig rig;
    rig.bind("p", "motor", sdk::SlotType::Motor);
    auto m = rig.make(kType, "p", pivotProps({{"startPosition", 0.5}, {"neutralMode", "coast"}}));
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* arm = link(snapshot, "mover");
    REQUIRE(arm != nullptr);
    const sdk::Vec3 tip = arm->pose.rotation.rotate({1.f, 0.f, 0.f});
    CHECK(tip.z == doctest::Approx(std::sin(0.5)).epsilon(1e-5));
    CHECK(tip.x == doctest::Approx(std::cos(0.5)).epsilon(1e-5));

    rig.run(*m, 20);   // unpowered, coasting: it falls
    CHECK(value(m->state(), "position") < 0.5);

    rig.drive("p", "motor", 12.0);
    rig.run(*m, 600);
    CHECK(value(m->state(), "position") == 1.5);
    CHECK(value(m->state(), "atUpperLimit") == 1.0);
    rig.drive("p", "motor", -12.0);
    rig.run(*m, 600);
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(value(m->state(), "atLowerLimit") == 1.0);
}

TEST_CASE("a rotary mechanism with stops is a turret and equal stops make it continuous") {
    Rig rig;
    rig.bind("t", "motor", sdk::SlotType::Motor);
    rig.bind("t", "forwardLimit", sdk::SlotType::LimitSwitch);
    auto m = rig.make(kType, "t", turretProps());
    rig.drive("t", "motor", 6.0);
    rig.run(*m, 1000);
    CHECK(value(m->state(), "position") == 3.3);
    CHECK(rig.env.hub.lastDigital({"t", "forwardLimit"}).value() == true);
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* l = link(snapshot, "mover");
    REQUIRE(l != nullptr);
    CHECK(l->pose.rotation.yaw() == doctest::Approx(3.3 - sdk::kTwoPi).epsilon(1e-4).scale(1.0));

    Rig rig2;
    rig2.bind("t", "motor", sdk::SlotType::Motor);
    auto cont = rig2.make(kType, "t", turretProps({{"minPosition", 0.0}, {"maxPosition", 0.0}}));
    rig2.drive("t", "motor", 6.0);
    rig2.run(*cont, 1000);
    CHECK(value(cont->state(), "position") > 10.0);   // no stops
}

TEST_CASE("limit switch polarity and margin properties") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    rig.bind("e", "reverseLimit", sdk::SlotType::LimitSwitch);
    auto m = rig.make(kType, "e", elevatorProps({{"limitPressedValue", false}}));
    m->tick(rig.env.ctx, kDt);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == false);   // pressed reads false
}

TEST_CASE("arm accepts the reviewed high-stiffness continuous configuration") {
    for (const double dt : {0.001, 0.002, 0.004, 0.005, 0.01, 0.02}) {
        Rig rig;
        rig.bind("p", "motor", sdk::SlotType::Motor);
        auto m = rig.make(kType, "p",
                          pivotProps({{"motorModel", "kraken_x60"},
                                      {"motorCount", 1},
                                      {"gearRatio", 300.0},
                                      {"momentOfInertiaKgM2", 0.01},
                                      {"comFraction", 0.5},
                                      {"efficiency", 1.0},
                                      {"minPosition", 0.0},
                                      {"maxPosition", 0.0},
                                      {"startPosition", 0.0}}));
        rig.drive("p", "motor", 6.0);
        m->tick(rig.env.ctx, dt);
        CHECK(std::isfinite(value(m->state(), "position")));
        CHECK(value(m->state(), "position") > 0.0);
        CHECK(value(m->state(), "velocity") > 0.0);
        CHECK(value(m->state(), "velocity") < 1.1);
        CHECK(value(m->state(), "atLowerLimit") == 0.0);
        CHECK(value(m->state(), "atUpperLimit") == 0.0);
    }
}

TEST_CASE("arm mode holds against gravity and rotary mode ignores it") {
    Rig rig;
    rig.bind("g", "motor", sdk::SlotType::Motor);
    auto arm = rig.make(kType, "g",
                        props({{"motion", "arm"}, {"simulateGravity", true}, {"massKg", 2.0}, {"lengthM", 0.4},
                               {"gearRatio", 80.0}, {"minPosition", -1.0}, {"maxPosition", 1.5}, {"neutralMode", "coast"}}));
    rig.run(*arm, 40);
    CHECK(value(arm->state(), "position") < 0.0);   // starts at 0 (horizontal) and sags

    Rig rot;
    rot.bind("g", "motor", sdk::SlotType::Motor);
    auto spinner = rot.make(kType, "g", props({{"simulateGravity", true}}));   // rotary: no mass/gravity model
    rot.run(*spinner, 40);
    CHECK(value(spinner->state(), "position") == 0.0);
}

TEST_CASE("invalid properties are rejected with a logged reason") {
    Rig rig;
    auto expectFail = [&](sdk::Value p, const std::string& needle) {
        INFO(needle);
        const size_t errorsBefore = rig.env.logger.count(sdk::LogLevel::Error);
        auto r = rig.create(kType, "bad", std::move(p));
        CHECK_FALSE(r.ok());
        REQUIRE(rig.env.logger.count(sdk::LogLevel::Error) == errorsBefore + 1);
        const auto entries = rig.env.logger.entries();
        CHECK(entries.back().message.find(needle) != std::string::npos);
    };
    expectFail(props({{"motorModel", "not_a_motor"}}), "motorModel");
    expectFail(props({{"gearRatio", -2.0}}), "gearRatio");
    expectFail(props({{"gearRatio", "fast"}}), "gearRatio");
    expectFail(props({{"axis", "w"}}), "axis");
    expectFail(props({{"motion", "teleport"}}), "motion");
    expectFail(elevatorProps({{"minPosition", 1.0}, {"maxPosition", 0.5}}), "maxPosition");
    expectFail(elevatorProps({{"startPosition", 5.0}}), "startPosition");
    expectFail(elevatorProps({{"drumRadiusM", 0.0}}), "drumRadiusM");
    expectFail(props({{"motorCount", 0}}), "motorCount");
}

TEST_CASE("unknown extra properties only warn, defaults fill missing ones") {
    Rig rig;
    auto m = rig.make(kType, "r", props({{"futureField", 1}}));
    CHECK(m->type() == kType);
    CHECK(rig.env.logger.count(sdk::LogLevel::Error) == 0);
}

TEST_CASE("mechanism ticks are deterministic") {
    auto run = [] {
        Rig rig;
        rig.bind("p", "motor", sdk::SlotType::Motor);
        auto m = rig.make(kType, "p", pivotProps());
        for (int i = 0; i < 500; ++i) {
            rig.drive("p", "motor", 6.0 * std::sin(i * 0.02));
            rig.run(*m, 1);
        }
        return std::make_pair(value(m->state(), "position"), value(m->state(), "velocity"));
    };
    CHECK(run() == run());
}

TEST_CASE("an unmapped required motor behaves neutrally and unmapped sensors are ignored") {
    Rig rig;
    auto m = rig.make(kType, "e", elevatorProps());   // nothing bound at all
    rig.run(*m, 100);
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(m->batteryCurrentA() == 0.0);
}

TEST_CASE("mechanism attachment: an arm mounted on the linear mover follows it") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto lift = rig.make(kType, "e", elevatorProps());
    auto arm = rig.make(kType, "p", pivotProps());
    rig.env.config.mechanisms.resize(2);
    rig.env.config.mechanisms[0].id = "e";
    rig.env.config.mechanisms[0].type = kType;
    rig.env.config.mechanisms[0].mount = sdk::Pose3{{0.2f, 0.f, 0.1f}, sdk::Quat{}};
    rig.env.config.mechanisms[1].id = "p";
    rig.env.config.mechanisms[1].type = kType;
    rig.env.config.mechanisms[1].attachTo("e", "mover");

    wpiemu::sdk::testing::FakeMechanismDirectory dir(rig.env.config, &rig.factory);
    dir.add(lift.get());
    dir.add(arm.get());
    rig.drive("e", "motor", 12.0);
    rig.run(*lift, 400);
    const auto pose = dir.linkPoseInChassis("p", "root");
    REQUIRE(pose.has_value());
    CHECK(pose->position.x == doctest::Approx(0.2f));
    CHECK(pose->position.z == doctest::Approx(0.1f + 0.8f).epsilon(1e-5));
    // and the arm's own moving link is further resolved through the chain
    const auto mover = dir.linkPoseInChassis("p", "mover");
    REQUIRE(mover.has_value());
    CHECK(mover->position.z == doctest::Approx(0.9f).epsilon(1e-5));
}

namespace {

constexpr const char* kPropertyTypeNames[] = {"Bool", "Int", "Double", "String", "Enum", "Vec3", "Pose3", "Box3", "Color"};
constexpr const char* kPropertyUnitNames[] = {"None",   "Meters",  "MetersPerSecond", "Radians", "RadiansPerSecond", "Kilograms",
                                              "Seconds", "Volts",   "Amps",            "Newtons", "NewtonMeters",     "KgM2",
                                              "Ratio",   "Percent", "Count",           "Hertz"};
constexpr const char* kSlotTypeNames[] = {"Motor",       "PositionSensor", "VelocitySensor", "LimitSwitch",    "BeamBreak",
                                          "Imu",         "AnalogSensor",   "Solenoid",       "DigitalOutput", "DistanceSensor"};

std::string scalar(const sdk::Value& v) {
    std::ostringstream o;
    o << std::setprecision(9);
    if (v.isBool()) o << (v.asBool() ? "true" : "false");
    else if (v.isInt()) o << v.asInt() << 'i';
    else if (v.isDouble()) o << v.asDouble() << 'd';
    else o << '"' << v.asString() << '"';
    return o.str();
}

std::string describe(const sdk::PropertySpec& p) {
    std::ostringstream o;
    o << std::setprecision(9) << p.key << '|' << p.label << '|' << p.group << '|' << kPropertyTypeNames[static_cast<int>(p.type)]
      << '|' << kPropertyUnitNames[static_cast<int>(p.unit)] << '|' << scalar(p.defaultValue) << '|';
    if (p.min) o << *p.min;
    else o << '-';
    o << '|';
    if (p.max) o << *p.max;
    else o << '-';
    o << '|' << (p.section == sdk::PropertySection::Drive ? "Drive" : "Configure") << '|' << p.axis;
    if (!p.visibleWhenKey.empty()) o << "|when " << p.visibleWhenKey << '=' << scalar(p.visibleWhenEquals);
    if (!p.options.empty()) {
        o << "|options";
        if (p.options.size() > 3) o << ' ' << p.options.size();
        else
            for (const sdk::EnumOption& option : p.options) o << ' ' << option.value;
    }
    o << '\n';
    return o.str();
}

std::string describe(const sdk::SlotSpec& s) {
    return s.name + "|" + s.label + "|" + kSlotTypeNames[static_cast<int>(s.type)] + "|" +
           (s.required ? "required" : "optional") + "|" + s.countKey + "|" + std::to_string(s.ordinal) + "\n";
}

}  // namespace

TEST_CASE("the template metadata is pinned: properties, slots, motion axis, link and geometry in order") {
    Rig rig;
    const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(kType);
    REQUIRE(info != nullptr);

    std::string properties;
    for (const sdk::PropertySpec& p : info->properties.properties) {
        properties += describe(p);
        CHECK_MESSAGE(!p.description.empty(), p.key);
    }
    CHECK(properties ==
          "motorModel|Motor model|Motor|Enum|None|\"kraken_x60\"|-|-|Drive|axis|options 19\n"
          "motorCount|Motor count|Motor|Int|Count|1i|1|8|Drive|axis\n"
          "gearRatio|Gear ratio|Motor|Double|Ratio|10d|0.01|10000|Drive|axis\n"
          "neutralMode|Neutral mode|Motor|Enum|None|\"brake\"|-|-|Drive|axis|options coast brake\n"
          "statorCurrentLimitA|Current limit|Motor|Double|Amps|0d|0|500|Drive|axis\n"
          "efficiency|Efficiency|Motor|Double|Ratio|1d|0.05|1|Drive|axis\n"
          "reportMotorLimits|Report limits on motor controller|Motor|Bool|None|false|-|-|Drive|axis\n"
          "motion|Type|Dynamics|Enum|None|\"rotary\"|-|-|Configure||options rotary arm linear\n"
          "momentOfInertiaKgM2|Moment of inertia|Dynamics|Double|KgM2|0.01d|0|100|Configure|\n"
          "massKg|Mass|Dynamics|Double|Kilograms|2d|0.001|2000|Configure|\n"
          "lengthM|Arm length|Dynamics|Double|Meters|0.3d|0.001|10|Configure||when motion=\"arm\"\n"
          "drumRadiusM|Drum radius|Dynamics|Double|Meters|0.02d|0.0001|1|Configure||when motion=\"linear\"\n"
          "simulateGravity|Simulate gravity|Dynamics|Bool|None|true|-|-|Configure|\n"
          "comFraction|Centre of mass|Dynamics|Double|Ratio|0.5d|0|1|Configure||when motion=\"arm\"\n"
          "viscousFrictionNmS|Viscous friction|Dynamics|Double|None|0.001d|0|1000|Configure|\n"
          "coulombFrictionNm|Coulomb friction|Dynamics|Double|None|0d|0|1000|Configure|\n"
          "minPosition|Minimum position|Limits|Double|None|0d|-10000|10000|Configure|\n"
          "maxPosition|Maximum position|Limits|Double|None|0d|-10000|10000|Configure|\n"
          "startPosition|Start position|Limits|Double|None|0d|-10000|10000|Configure|\n"
          "limitMargin|Limit switch margin|Limits|Double|None|1e-06d|0|1|Configure|\n"
          "limitPressedValue|Limit reads true when pressed|Limits|Bool|None|true|-|-|Configure|\n"
          "sensorRatio|Sensor ratio|Sensors|Double|Ratio|1d|0.001|10000|Configure|\n"
          "sensorInverted|Sensor inverted|Sensors|Bool|None|false|-|-|Configure|\n"
          "sensorOffsetRot|Sensor offset|Sensors|Double|None|0d|-1000000|1000000|Configure|\n"
          "axis|Axis (Rotary and Linear)|Motion|Enum|None|\"z\"|-|-|Configure||options x y z\n");

    std::string slots;
    for (const sdk::SlotSpec& s : info->slotSpecs) {
        slots += describe(s);
        CHECK_MESSAGE(!s.description.empty(), s.name);
    }
    CHECK(slots ==
          "motor|Motor|Motor|required|motorCount|1\n"
          "motor2|Motor 2|Motor|optional|motorCount|2\n"
          "motor3|Motor 3|Motor|optional|motorCount|3\n"
          "motor4|Motor 4|Motor|optional|motorCount|4\n"
          "motor5|Motor 5|Motor|optional|motorCount|5\n"
          "motor6|Motor 6|Motor|optional|motorCount|6\n"
          "motor7|Motor 7|Motor|optional|motorCount|7\n"
          "motor8|Motor 8|Motor|optional|motorCount|8\n"
          "encoder|Position sensor|PositionSensor|optional||0\n"
          "velocitySensor|Velocity sensor|VelocitySensor|optional||0\n"
          "reverseLimit|Reverse limit switch|LimitSwitch|optional||0\n"
          "forwardLimit|Forward limit switch|LimitSwitch|optional||0\n");

    REQUIRE(info->motionAxes.size() == 1);
    const sdk::MotionAxisSpec& axis = info->motionAxes[0];
    std::string motors;
    for (const std::string& m : axis.motorSlots) motors += (motors.empty() ? "" : ",") + m;
    CHECK(axis.name == "axis");
    CHECK(axis.kind == sdk::PlantKind::Angular);
    CHECK(motors == "motor,motor2,motor3,motor4,motor5,motor6,motor7,motor8");
    CHECK(axis.motionProperty == "motion");
    CHECK(axis.enabledProperty.empty());

    REQUIRE(info->links.size() == 1);
    CHECK(info->links[0].name == "mover");
    CHECK(info->links[0].label == "Moving part");
    CHECK(info->links[0].local == sdk::Pose3{});
    REQUIRE(info->defaultGeometry.size() == 1);
    const sdk::PrimitiveSpec& body = info->defaultGeometry[0];
    CHECK(body.name == "mover");
    CHECK(body.kind == sdk::PrimitiveKind::Box);
    CHECK(body.local.position == sdk::Vec3{0.10f, 0.f, 0.f});
    CHECK(body.sizeM == sdk::Vec3{0.20f, 0.05f, 0.05f});
    CHECK(body.color == "#9b59b6");
}