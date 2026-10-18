#include <doctest/doctest.h>

#include <limits>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/game/zone_activation.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_mechanism_directory.h"

using namespace wpiemu::sdk;

namespace {

// A mechanism publishing exactly the values it is given.
class ScriptedMechanism final : public IMechanism {
public:
    explicit ScriptedMechanism(std::string id) : id_(std::move(id)) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return "scripted"; }
    void tick(SimContext&, double) override {}
    void reset() override {}
    MechanismState state() const override {
        MechanismState s;
        s.id = id_;
        s.type = "scripted";
        s.active = active;
        s.values = values;
        return s;
    }
    bool active = false;
    std::vector<NamedValue> values;

private:
    std::string id_;
};

ZoneCondition cond(std::string mech, ConditionOp op, double threshold, std::string value = "velocity") {
    ZoneCondition c;
    c.mechanismId = std::move(mech);
    c.op = op;
    c.threshold = threshold;
    c.value = std::move(value);
    return c;
}

struct Fixture {
    RobotConfig config;
    testing::FakeMechanismDirectory dir{config};
    ScriptedMechanism flywheel{"flywheel"};
    ScriptedMechanism hood{"hood"};
    Fixture() {
        dir.add(&flywheel);
        dir.add(&hood);
    }
};

}  // namespace

TEST_CASE("zones: defaults, equality and RobotConfig::findZone") {
    const ZoneConfig z;
    CHECK(z.attachment.onChassis());
    CHECK(z.activation.mode == ActivationMode::All);
    CHECK(z.activation.conditions.empty());
    CHECK(z.sizeM == Vec3{0.3f, 0.3f, 0.3f});
    const ZoneCondition c;
    CHECK(c.value == "velocity");
    CHECK(c.op == ConditionOp::Above);
    CHECK(c.tolerance == 0.1);
    CHECK(c.absolute);

    ZoneConfig a;
    a.id = "pickup";
    a.activation.conditions.push_back(cond("flywheel", ConditionOp::Above, 5.0));
    ZoneConfig b = a;
    CHECK(a == b);
    b.activation.conditions[0].threshold = 6.0;
    CHECK_FALSE(a == b);
    b = a;
    b.properties["rate"] = 2;
    CHECK_FALSE(a == b);

    RobotConfig config;
    config.zones.push_back(a);
    REQUIRE(config.findZone("pickup") != nullptr);
    CHECK(config.findZone("pickup")->id == "pickup");
    CHECK(config.findZone("nope") == nullptr);
}

TEST_CASE("zone activation: readMechanismValue converts units and reports presence") {
    Fixture f;
    f.flywheel.active = true;
    f.flywheel.values = {{"velocity", 60.0, "rpm"}, {"position", 90.0, "deg"}};

    const MechanismReading r = readMechanismValue(&f.dir, "flywheel", "velocity");
    CHECK(r.found);
    CHECK(r.hasValue);
    CHECK(r.active);
    CHECK(r.value == doctest::Approx(rpmToRadPerSec(60.0)));
    CHECK(readMechanismValue(&f.dir, "flywheel", "position").value == doctest::Approx(kPi / 2.0));

    const MechanismReading noValue = readMechanismValue(&f.dir, "flywheel", "current");
    CHECK(noValue.found);
    CHECK_FALSE(noValue.hasValue);
    CHECK_FALSE(readMechanismValue(&f.dir, "ghost", "velocity").found);
    CHECK_FALSE(readMechanismValue(nullptr, "flywheel", "velocity").found);
}

TEST_CASE("zone activation: Above, Below and Within with absolute and signed comparison") {
    Fixture f;
    f.flywheel.values = {{"velocity", -10.0, "rad/s"}};

    CHECK(evaluateCondition(cond("flywheel", ConditionOp::Above, 5.0), &f.dir));       // |-10| > 5
    CHECK_FALSE(evaluateCondition(cond("flywheel", ConditionOp::Below, 5.0), &f.dir));
    ZoneCondition signedAbove = cond("flywheel", ConditionOp::Above, 5.0);
    signedAbove.absolute = false;
    CHECK_FALSE(evaluateCondition(signedAbove, &f.dir));                                // -10 > 5 is false
    ZoneCondition signedBelow = cond("flywheel", ConditionOp::Below, -5.0);
    signedBelow.absolute = false;
    CHECK(evaluateCondition(signedBelow, &f.dir));

    ZoneCondition within = cond("flywheel", ConditionOp::Within, -10.05);
    within.tolerance = 0.1;
    CHECK(evaluateCondition(within, &f.dir));
    within.threshold = -9.0;
    CHECK_FALSE(evaluateCondition(within, &f.dir));
}

TEST_CASE("zone activation: an unpublished value falls back to the active flag") {
    Fixture f;
    f.hood.active = true;
    CHECK(evaluateCondition(cond("hood", ConditionOp::Above, 1.0, "angle"), &f.dir));
    CHECK(evaluateCondition(cond("hood", ConditionOp::Within, 1.0, "angle"), &f.dir));
    CHECK_FALSE(evaluateCondition(cond("hood", ConditionOp::Below, 1.0, "angle"), &f.dir));
    f.hood.active = false;
    CHECK_FALSE(evaluateCondition(cond("hood", ConditionOp::Above, 1.0, "angle"), &f.dir));
    CHECK(evaluateCondition(cond("hood", ConditionOp::Below, 1.0, "angle"), &f.dir));
}

TEST_CASE("zone activation: All/Any combination, empty list and missing mechanisms") {
    Fixture f;
    f.flywheel.values = {{"velocity", 10.0, "rad/s"}};

    ZoneActivation act;
    CHECK(evaluateActivation(act, &f.dir));  // empty: always active
    CHECK(evaluateActivation(act, nullptr));

    act.conditions = {cond("flywheel", ConditionOp::Above, 5.0), cond("flywheel", ConditionOp::Above, 50.0)};
    act.mode = ActivationMode::All;
    CHECK_FALSE(evaluateActivation(act, &f.dir));
    act.mode = ActivationMode::Any;
    CHECK(evaluateActivation(act, &f.dir));

    std::vector<std::string> missing;
    act.conditions = {cond("ghost", ConditionOp::Above, 0.0), cond("ghost", ConditionOp::Below, 0.0),
                      cond("flywheel", ConditionOp::Above, 5.0)};
    CHECK(evaluateActivation(act, &f.dir, &missing));  // Any: the flywheel holds
    CHECK(missing == std::vector<std::string>{"ghost"});
    act.mode = ActivationMode::All;
    CHECK_FALSE(evaluateActivation(act, &f.dir, &missing));
    CHECK(missing.size() == 1);
}

TEST_CASE("zone kinds: the fake game module contributes a kind it owns") {
    const testing::FakeGameModule module("fake_game", 2999);
    const std::vector<ZoneKindInfo> kinds = module.contributeZoneKinds();
    REQUIRE(kinds.size() == 1);
    CHECK(kinds[0].kind == "fake_game.fake_zone");
    CHECK(kinds[0].shape == ZoneShape::Volume);
    CHECK(kinds[0].properties.find("capacity") != nullptr);
}
