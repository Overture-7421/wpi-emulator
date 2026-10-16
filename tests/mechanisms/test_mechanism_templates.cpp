#include <doctest/doctest.h>

#include <cmath>
#include <set>

#include "wpiemu/core/mechanisms/linear_system.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
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

const std::vector<std::string> kAllTypes = {"roller",  "generic_1dof", "intake", "shooter",
                                            "pivot",   "turret",       "elevator", "indexer"};

}  // namespace

TEST_CASE("generic mechanism templates are registered with complete, consistent metadata") {
    Rig rig;
    CHECK(rig.factory.size() == kAllTypes.size());
    for (const std::string& type : kAllTypes) {
        INFO(type);
        const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(type);
        REQUIRE(info != nullptr);
        CHECK(info->type == type);
        CHECK(info->origin == "core");
        CHECK_FALSE(info->displayName.empty());
        CHECK_FALSE(info->category.empty());
        CHECK_FALSE(info->description.empty());
        CHECK_FALSE(info->defaultGeometry.empty());
        CHECK(sdk::templateLinkIssues(*info).empty());
        CHECK_FALSE(info->links.empty());

        std::set<std::string> slotNames;
        bool hasRequiredMotor = false;
        for (const sdk::SlotSpec& s : info->slotSpecs) {
            CHECK(slotNames.insert(s.name).second);   // unique
            CHECK_FALSE(s.label.empty());
            if (s.required) {
                CHECK(s.type == sdk::SlotType::Motor);
                hasRequiredMotor = true;
            }
        }
        if (type != "intake") CHECK(hasRequiredMotor);

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
        // defaults satisfy their own schema
        for (const sdk::Issue& issue : sdk::validateProperties(info->properties, sdk::defaultsOf(info->properties)))
            CHECK_MESSAGE(issue.severity != sdk::Severity::Error, issue.path << ": " << issue.message);
    }
    CHECK(rig.factory.size() == 8);
    // registering twice is an error, not a silent override
    sdk::MechanismFactory again;
    REQUIRE(registerGenericMechanisms(again).ok());
    CHECK(registerGenericMechanisms(again).error().code == sdk::ErrorCode::AlreadyExists);
}

TEST_CASE("every template builds from an empty property set and publishes its declared links") {
    Rig rig;
    for (const std::string& type : kAllTypes) {
        INFO(type);
        auto m = rig.make(type, "m");
        CHECK(m->id() == "m");
        CHECK(m->type() == type);
        const sdk::MechanismTemplateInfo* info = rig.factory.templateInfo(type);
        for (int i = 0; i < 20; ++i) m->tick(rig.env.ctx, kDt);
        const sdk::MechanismState s = m->state();
        CHECK(s.id == "m");
        CHECK(s.type == type);
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

TEST_CASE("roller spin-up matches the WPILib LinearSystemId solution and drives the sensors") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    rig.bind("r", "encoder", sdk::SlotType::PositionSensor);
    rig.bind("r", "velocitySensor", sdk::SlotType::VelocitySensor);
    auto m = rig.make("roller", "r",
                      props({{"gearRatio", 3.0}, {"momentOfInertiaKgM2", 0.002}, {"radiusM", 0.04}, {"viscousFrictionNmS", 0.0}}));
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
    CHECK(value(s, "surfaceSpeed") == doctest::Approx(w * 0.04).epsilon(1e-9));
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

    // spin axis link rotates about +Y by the wrapped position
    const sdk::LinkPose* l = link(s, "roller");
    REQUIRE(l != nullptr);
    const sdk::Vec3 axisCheck = l->pose.rotation.rotate({0.f, 1.f, 0.f});
    CHECK(axisCheck.y == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(std::abs(2.0 * std::atan2(l->pose.rotation.y, l->pose.rotation.w) -
                   std::remainder(x, sdk::kTwoPi)) < 1e-4);
}

TEST_CASE("sensor ratio, inversion and offset apply to position sensors only") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    rig.bind("r", "encoder", sdk::SlotType::PositionSensor);
    auto m = rig.make("roller", "r", props({{"sensorRatio", 2.0}, {"sensorInverted", true}, {"sensorOffsetRot", 0.5}}));
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
    auto m = rig.make("roller", "r", props({{"statorCurrentLimitA", 40.0}}));
    rig.env.hub.setBatteryVoltage(8.0);
    rig.drive("r", "motor", 12.0);
    m->tick(rig.env.ctx, kDt);   // at stall: the limit, not the supply, decides
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    CHECK(value(m->state(), "current") == doctest::Approx(40.0).epsilon(1e-9));
    CHECK(value(m->state(), "voltage") == doctest::Approx(40.0 * kraken.resistanceOhm()).epsilon(1e-9));

    Rig sag;
    sag.bind("s", "motor", sdk::SlotType::Motor);
    auto unlimited = sag.make("roller", "s");
    sag.env.hub.setBatteryVoltage(8.0);
    sag.drive("s", "motor", 12.0);
    sag.run(*unlimited, 3);
    CHECK(value(unlimited->state(), "voltage") == doctest::Approx(8.0));
    // battery draw is positive and about V*I/Vbat = I at full duty
    CHECK(unlimited->batteryCurrentA() > 0.0);
}

TEST_CASE("battery current at stall equals the stall current and is zero when idle") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    auto m = rig.make("roller", "r", props({{"gearRatio", 100.0}, {"momentOfInertiaKgM2", 10.0}}));
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
        auto m = rig.make("roller", "r", props({{"motorCount", 2}}));
        rig.drive("r", "motor", 10.0);
        m->tick(rig.env.ctx, kDt);
        CHECK(value(m->state(), "voltage") == doctest::Approx(10.0));   // mirrored
        CHECK(value(m->state(), "current") == doctest::Approx(2.0 * 10.0 / MotorCatalog::builtin().spec("kraken_x60")->resistanceOhm()));
    }
    {
        Rig rig;
        rig.bind("r", "motor", sdk::SlotType::Motor);
        rig.bind("r", "motor2", sdk::SlotType::Motor);
        auto m = rig.make("roller", "r", props({{"motorCount", 2}}));
        rig.drive("r", "motor", 10.0);
        rig.drive("r", "motor2", 4.0);
        m->tick(rig.env.ctx, kDt);
        CHECK(value(m->state(), "voltage") == doctest::Approx(7.0));    // mean over the two motors
        CHECK(rig.env.hub.lastMotorFeedback({"r", "motor2"}).has_value());
    }
}

TEST_CASE("an inverted motor reverses the mechanism but keeps the rotor sign convention") {
    Rig rig;
    rig.bind("r", "motor", sdk::SlotType::Motor);
    auto m = rig.make("roller", "r", props({{"motorInverted", true}}));
    rig.drive("r", "motor", 8.0);
    rig.run(*m, 30);
    CHECK(value(m->state(), "velocity") < 0.0);
    const auto fb = rig.env.hub.lastMotorFeedback({"r", "motor"});
    REQUIRE(fb.has_value());
    CHECK(fb->rotorVelocityRps > 0.0);   // the controller sees its positive direction moving
}

TEST_CASE("neutral mode: a brake decelerates faster than coast") {
    auto coastDown = [](const char* mode) {
        Rig rig;
        rig.bind("r", "motor", sdk::SlotType::Motor);
        auto m = rig.make("roller", "r", props({{"neutralMode", mode}}));
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

TEST_CASE("shooter publishes the flywheel surface speed and spins up toward the free speed") {
    Rig rig;
    rig.bind("s", "motor", sdk::SlotType::Motor);
    auto m = rig.make("shooter", "s",
                      props({{"radiusM", 0.05}, {"gearRatio", 1.0}, {"momentOfInertiaKgM2", 0.004}, {"viscousFrictionNmS", 0.0}}));
    CHECK_FALSE(m->state().active);
    rig.drive("s", "motor", 12.0);
    rig.run(*m, 400);
    const sdk::MotorSpec kraken = *MotorCatalog::builtin().spec("kraken_x60");
    const LinearSystem sys = dcMotorSystem(kraken, 1, 0.004, 1.0);
    const sdk::MechanismState s = m->state();
    const double vInf = 12.0 * kraken.kvRadPerSecPerVolt();
    CHECK(value(s, "velocity") == doctest::Approx(vInf * (1.0 - std::exp(sys.A(1, 1) * 400 * kDt))).epsilon(1e-9));
    CHECK(value(s, "velocity") < vInf);
    CHECK(value(s, "surfaceSpeed") == doctest::Approx(value(s, "velocity") * 0.05));
    CHECK(s.active);
    CHECK(link(s, "flywheel") != nullptr);
    rig.drive("s", "motor", -12.0);
    rig.run(*m, 800);
    CHECK(value(m->state(), "surfaceSpeed") < 0.0);   // reverses
}

TEST_CASE("elevator: gravity, hard stops, limit switches and the carriage link") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    rig.bind("e", "reverseLimit", sdk::SlotType::LimitSwitch);
    rig.bind("e", "forwardLimit", sdk::SlotType::LimitSwitch);
    rig.bind("e", "encoder", sdk::SlotType::PositionSensor);
    auto m = rig.make("elevator", "e");

    m->tick(rig.env.ctx, kDt);   // sits on the lower stop (brake, V = 0)
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == true);
    CHECK(rig.env.hub.lastDigital({"e", "forwardLimit"}).value() == false);

    rig.drive("e", "motor", 12.0);
    rig.run(*m, 400);
    sdk::MechanismState s = m->state();
    CHECK(value(s, "position") == 0.8);      // default travel
    CHECK(value(s, "velocity") == 0.0);
    CHECK(value(s, "atUpperLimit") == 1.0);
    CHECK(rig.env.hub.lastDigital({"e", "forwardLimit"}).value() == true);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == false);
    const sdk::LinkPose* carriage = link(s, "carriage");
    REQUIRE(carriage != nullptr);
    CHECK(carriage->pose.position.z == doctest::Approx(0.8f));
    CHECK(carriage->pose.position.x == 0.0f);
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

TEST_CASE("elevator without power sinks under gravity unless held by the feed-forward voltage") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto m = rig.make("elevator", "e", props({{"startPosition", 0.4}, {"neutralMode", "coast"}}));
    rig.drive("e", "motor", 0.0);
    rig.run(*m, 60);
    CHECK(value(m->state(), "position") < 0.4);

    // feed-forward: kG = g / B (WPILib ElevatorFeedforward-style), computed from the same LinearSystemId
    auto holdTest = [&](double volts) {
        Rig r2;
        r2.bind("e", "motor", sdk::SlotType::Motor);
        auto e2 = r2.make("elevator", "e", props({{"startPosition", 0.4}}));
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

TEST_CASE("elevator travel axis moves the carriage link") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto m = rig.make("elevator", "e", props({{"axis", "x"}, {"startPosition", 0.3}}));
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* c = link(snapshot, "carriage");
    REQUIRE(c != nullptr);
    CHECK(c->pose.position.x == doctest::Approx(0.3f));   // published before the first tick
    CHECK(c->pose.position.z == 0.0f);
}

TEST_CASE("pivot: angle from horizontal raises the +X end, gravity is held by the feed-forward") {
    Rig rig;
    rig.bind("p", "motor", sdk::SlotType::Motor);
    auto m = rig.make("pivot", "p", props({{"startPosition", 0.5}, {"neutralMode", "coast"}}));
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* arm = link(snapshot, "arm");
    REQUIRE(arm != nullptr);
    const sdk::Vec3 tip = arm->pose.rotation.rotate({1.f, 0.f, 0.f});
    CHECK(tip.z == doctest::Approx(std::sin(0.5)).epsilon(1e-5));
    CHECK(tip.x == doctest::Approx(std::cos(0.5)).epsilon(1e-5));

    rig.run(*m, 20);   // unpowered, coasting: it falls
    CHECK(value(m->state(), "position") < 0.5);

    // stops at [0, 1.5] by default
    rig.drive("p", "motor", 12.0);
    rig.run(*m, 600);
    CHECK(value(m->state(), "position") == 1.5);
    CHECK(value(m->state(), "atUpperLimit") == 1.0);
    rig.drive("p", "motor", -12.0);
    rig.run(*m, 600);
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(value(m->state(), "atLowerLimit") == 1.0);
}

TEST_CASE("turret: yaw link, stops and the continuous variant") {
    Rig rig;
    rig.bind("t", "motor", sdk::SlotType::Motor);
    rig.bind("t", "forwardLimit", sdk::SlotType::LimitSwitch);
    auto m = rig.make("turret", "t");
    rig.drive("t", "motor", 6.0);
    rig.run(*m, 1000);
    CHECK(value(m->state(), "position") == 3.3);
    CHECK(rig.env.hub.lastDigital({"t", "forwardLimit"}).value() == true);
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* l = link(snapshot, "turret");
    REQUIRE(l != nullptr);
    CHECK(l->pose.rotation.yaw() == doctest::Approx(3.3 - sdk::kTwoPi).epsilon(1e-4).scale(1.0));

    Rig rig2;
    rig2.bind("t", "motor", sdk::SlotType::Motor);
    auto cont = rig2.make("turret", "t", props({{"minPosition", 0.0}, {"maxPosition", 0.0}}));
    rig2.drive("t", "motor", 6.0);
    rig2.run(*cont, 1000);
    CHECK(value(cont->state(), "position") > 10.0);   // no stops
}

TEST_CASE("limit switch polarity and margin properties") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    rig.bind("e", "reverseLimit", sdk::SlotType::LimitSwitch);
    auto m = rig.make("elevator", "e", props({{"limitPressedValue", false}}));
    m->tick(rig.env.ctx, kDt);
    CHECK(rig.env.hub.lastDigital({"e", "reverseLimit"}).value() == false);   // pressed reads false
}

TEST_CASE("intake is active only while the roller spins AND the intake is deployed") {
    Rig rig;
    rig.bind("i", "rollerMotor", sdk::SlotType::Motor);
    rig.bind("i", "deployMotor", sdk::SlotType::Motor);
    rig.bind("i", "deployForwardLimit", sdk::SlotType::LimitSwitch);
    auto m = rig.make("intake", "i",
                      props({{"deployMinPosition", 0.0}, {"deployMaxPosition", 1.2}, {"deployedTolerance", 0.02},
                             {"rollerNeutralMode", "brake"}}));
    rig.drive("i", "rollerMotor", 12.0);
    rig.run(*m, 100);
    sdk::MechanismState s = m->state();
    CHECK(value(s, "rollerSurfaceSpeed") > 0.5);
    CHECK(value(s, "deployed") == 0.0);
    CHECK_FALSE(s.active);   // stowed

    rig.drive("i", "deployMotor", 12.0);
    rig.run(*m, 600);
    s = m->state();
    CHECK(value(s, "deployPosition") == 1.2);
    CHECK(value(s, "deployed") == 1.0);
    CHECK(s.active);
    CHECK(rig.env.hub.lastDigital({"i", "deployForwardLimit"}).value() == true);

    rig.drive("i", "rollerMotor", 0.0);
    rig.run(*m, 600);
    CHECK_FALSE(m->state().active);   // deployed but the roller stopped (brake mode: spins down fast)

    // the roller link is a child of the deploy link and spins about its own axis
    const sdk::MechanismState snapshot = m->state();
    const sdk::LinkPose* roller = link(snapshot, "roller");
    const sdk::LinkPose* deploy = link(snapshot, "deploy");
    REQUIRE(roller != nullptr);
    REQUIRE(deploy != nullptr);
    const sdk::Vec3 expected = deploy->pose.transformPoint({0.3f, 0.f, 0.f});
    CHECK(roller->pose.position.x == doctest::Approx(expected.x).epsilon(1e-5));
    CHECK(roller->pose.position.z == doctest::Approx(expected.z).epsilon(1e-5));
    CHECK(rig.factory.templateInfo("intake")->links.size() == 2);
}

TEST_CASE("intake without a deploy axis is always deployed") {
    Rig rig;
    rig.bind("i", "rollerMotor", sdk::SlotType::Motor);
    auto m = rig.make("intake", "i", props({{"deployEnabled", false}}));
    CHECK_FALSE(m->state().active);
    rig.drive("i", "rollerMotor", 12.0);
    rig.run(*m, 100);
    CHECK(m->state().active);
    CHECK(value(m->state(), "deployed") == 1.0);
    CHECK(link(m->state(), "deploy") == nullptr);
    CHECK(link(m->state(), "roller") != nullptr);
}

TEST_CASE("indexer never writes its beam breaks: the game logic owns them") {
    Rig rig;
    rig.bind("x", "motor", sdk::SlotType::Motor);
    rig.bind("x", "entryBeamBreak", sdk::SlotType::BeamBreak);
    rig.bind("x", "exitBeamBreak", sdk::SlotType::BeamBreak);
    auto m = rig.make("indexer", "x");
    rig.drive("x", "motor", 12.0);
    rig.run(*m, 50);
    CHECK_FALSE(rig.env.hub.lastDigital({"x", "entryBeamBreak"}).has_value());
    CHECK(m->state().active);

    // a game plugin writes through the hub; the mechanism does not overwrite it
    rig.env.hub.writeDigital({"x", "entryBeamBreak"}, true);
    rig.run(*m, 5);
    CHECK(rig.env.hub.lastDigital({"x", "entryBeamBreak"}).value() == true);
}

TEST_CASE("generic 1-DOF: motion types, position-window beam break and limit switches") {
    Rig rig;
    rig.bind("g", "motor", sdk::SlotType::Motor);
    rig.bind("g", "beamBreak", sdk::SlotType::BeamBreak);
    auto m = rig.make("generic_1dof", "g",
                      props({{"motion", "linear"}, {"minPosition", 0.0}, {"maxPosition", 1.0}, {"gearRatio", 20.0},
                             {"beamBreakSource", "window"}, {"beamBreakMin", 0.2}, {"beamBreakMax", 0.4}}));
    m->tick(rig.env.ctx, kDt);
    CHECK(rig.env.hub.lastDigital({"g", "beamBreak"}).value() == false);   // at 0: beam open
    rig.drive("g", "motor", 3.0);
    bool blocked = false, passed = false;
    for (int i = 0; i < 2000; ++i) {
        rig.run(*m, 1);
        const double p = value(m->state(), "position");
        const bool b = rig.env.hub.lastDigital({"g", "beamBreak"}).value();
        REQUIRE(b == (p >= 0.2 && p <= 0.4));
        blocked = blocked || b;
        passed = passed || (blocked && !b);
    }
    CHECK(blocked);
    CHECK(passed);   // it travelled through the window
    CHECK(value(m->state(), "position") == doctest::Approx(1.0));

    // external source: the template leaves the slot alone
    Rig ext;
    ext.bind("g", "beamBreak", sdk::SlotType::BeamBreak);
    ext.bind("g", "motor", sdk::SlotType::Motor);
    auto e = ext.make("generic_1dof", "g");
    ext.run(*e, 5);
    CHECK_FALSE(ext.env.hub.lastDigital({"g", "beamBreak"}).has_value());
}

TEST_CASE("generic 1-DOF arm mode holds against gravity and rotary mode ignores it") {
    Rig rig;
    rig.bind("g", "motor", sdk::SlotType::Motor);
    auto arm = rig.make("generic_1dof", "g",
                        props({{"motion", "arm"}, {"simulateGravity", true}, {"massKg", 2.0}, {"lengthM", 0.4},
                               {"gearRatio", 80.0}, {"minPosition", -1.0}, {"maxPosition", 1.5}, {"neutralMode", "coast"}}));
    rig.run(*arm, 40);
    CHECK(value(arm->state(), "position") < 0.0);   // starts at 0 (horizontal) and sags

    Rig rot;
    rot.bind("g", "motor", sdk::SlotType::Motor);
    auto spinner = rot.make("generic_1dof", "g", props({{"simulateGravity", true}}));   // rotary: no mass/gravity model
    rot.run(*spinner, 40);
    CHECK(value(spinner->state(), "position") == 0.0);
}

TEST_CASE("invalid properties are rejected with a logged reason") {
    Rig rig;
    auto expectFail = [&](const std::string& type, sdk::Value p, const std::string& needle) {
        INFO(needle);
        const size_t errorsBefore = rig.env.logger.count(sdk::LogLevel::Error);
        auto r = rig.create(type, "bad", std::move(p));
        CHECK_FALSE(r.ok());
        REQUIRE(rig.env.logger.count(sdk::LogLevel::Error) == errorsBefore + 1);
        const auto entries = rig.env.logger.entries();
        CHECK(entries.back().message.find(needle) != std::string::npos);
    };
    expectFail("roller", props({{"motorModel", "not_a_motor"}}), "motorModel");
    expectFail("roller", props({{"gearRatio", -2.0}}), "gearRatio");
    expectFail("roller", props({{"gearRatio", "fast"}}), "gearRatio");
    expectFail("roller", props({{"spinAxis", "w"}}), "spinAxis");
    expectFail("elevator", props({{"minPosition", 1.0}, {"maxPosition", 0.5}}), "maxPosition");
    expectFail("elevator", props({{"startPosition", 5.0}}), "startPosition");
    expectFail("elevator", props({{"drumRadiusM", 0.0}}), "drumRadiusM");
    expectFail("generic_1dof", props({{"beamBreakSource", "window"}, {"beamBreakMin", 2.0}, {"beamBreakMax", 1.0}}),
               "beamBreakMax");
    expectFail("intake", props({{"deployAxis", "q"}}), "deployAxis");
    expectFail("turret", props({{"motorCount", 0}}), "motorCount");
}

TEST_CASE("unknown extra properties only warn, defaults fill missing ones") {
    Rig rig;
    auto m = rig.make("roller", "r", props({{"futureField", 1}}));
    CHECK(m->type() == "roller");
    CHECK(rig.env.logger.count(sdk::LogLevel::Error) == 0);
}

TEST_CASE("mechanism ticks are deterministic") {
    auto run = [] {
        Rig rig;
        rig.bind("p", "motor", sdk::SlotType::Motor);
        auto m = rig.make("pivot", "p");
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
    auto m = rig.make("elevator", "e");   // nothing bound at all
    rig.run(*m, 100);
    CHECK(value(m->state(), "position") == 0.0);
    CHECK(m->batteryCurrentA() == 0.0);
}

TEST_CASE("mechanism attachment: a pivot mounted on the elevator carriage follows it") {
    Rig rig;
    rig.bind("e", "motor", sdk::SlotType::Motor);
    auto elevator = rig.make("elevator", "e");
    auto pivot = rig.make("pivot", "p");
    rig.env.config.mechanisms.resize(2);
    rig.env.config.mechanisms[0].id = "e";
    rig.env.config.mechanisms[0].type = "elevator";
    rig.env.config.mechanisms[0].mount = sdk::Pose3{{0.2f, 0.f, 0.1f}, sdk::Quat{}};
    rig.env.config.mechanisms[1].id = "p";
    rig.env.config.mechanisms[1].type = "pivot";
    rig.env.config.mechanisms[1].attachTo("e", "carriage");

    wpiemu::sdk::testing::FakeMechanismDirectory dir(rig.env.config, &rig.factory);
    dir.add(elevator.get());
    dir.add(pivot.get());
    rig.drive("e", "motor", 12.0);
    rig.run(*elevator, 400);
    const auto pose = dir.linkPoseInChassis("p", "root");
    REQUIRE(pose.has_value());
    CHECK(pose->position.x == doctest::Approx(0.2f));
    CHECK(pose->position.z == doctest::Approx(0.1f + 0.8f).epsilon(1e-5));
    // and the pivot's own moving link is further resolved through the chain
    const auto arm = dir.linkPoseInChassis("p", "arm");
    REQUIRE(arm.has_value());
    CHECK(arm->position.z == doctest::Approx(0.9f).epsilon(1e-5));
}
