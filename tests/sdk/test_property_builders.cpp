#include <doctest/doctest.h>

#include <limits>
#include <string>

#include "wpiemu/sdk/config/config_bindings.h"
#include "wpiemu/sdk/config/property_builders.h"
#include "wpiemu/sdk/config/robot_spec_bindings.h"
#include "wpiemu/sdk/core/value.h"

using namespace wpiemu::sdk;

TEST_CASE("property builders set the fields a schema entry needs") {
    const PropertySpec n = props::integer("count", "Count", "Group", 3, 1, 8, "help");
    CHECK(n.key == "count");
    CHECK(n.label == "Count");
    CHECK(n.group == "Group");
    CHECK(n.description == "help");
    CHECK(n.type == PropertyType::Int);
    CHECK(n.unit == PropertyUnit::Count);
    CHECK(n.defaultValue == Value(3));
    CHECK(n.min == 1.0);
    CHECK(n.max == 8.0);
    CHECK(n.step == 0.0);

    const PropertySpec r = props::real("gain", "Gain", "Tuning", 2.5, 0.0, 10.0, PropertyUnit::Ratio, {}, 0.5);
    CHECK(r.type == PropertyType::Double);
    CHECK(r.unit == PropertyUnit::Ratio);
    CHECK(r.defaultValue == Value(2.5));
    CHECK(r.min == 0.0);
    CHECK(r.max == 10.0);
    CHECK(r.step == 0.5);
    CHECK(r.description.empty());

    const PropertySpec f = props::flag("on", "On", "Group", true);
    CHECK(f.type == PropertyType::Bool);
    CHECK(f.unit == PropertyUnit::None);
    CHECK(f.defaultValue == Value(true));

    const PropertySpec c = props::choice("mode", "Mode", "Group", "b", {{"a", "A"}, {"b", "B"}});
    CHECK(c.type == PropertyType::Enum);
    CHECK(c.defaultValue == Value("b"));
    REQUIRE(c.options.size() == 2);
    CHECK(c.options[1].label == "B");

    CHECK(props::text("name", "Name", "Group", "x").type == PropertyType::String);

    Box3 box;
    box.sizeM = {1.f, 2.f, 3.f};
    const PropertySpec b = props::box("zone", "Zone", "Group", box);
    CHECK(b.type == PropertyType::Box3);
    CHECK(b.unit == PropertyUnit::Meters);
    Box3 roundTrip;
    CHECK(fromValue(b.defaultValue, roundTrip));
    CHECK(roundTrip.sizeM == box.sizeM);

    const PropertySpec v = props::vec3("offset", "Offset", "Group", {1.f, 2.f, 3.f});
    CHECK(v.type == PropertyType::Vec3);
}

TEST_CASE("general property builder and withRange compose") {
    PropertySpec s = props::makeSpec("k", "K", PropertyType::Double, PropertyUnit::Volts, 12.0, "Battery", "desc");
    CHECK(s.group == "Battery");
    CHECK(s.description == "desc");
    CHECK(!s.min.has_value());
    props::withRange(s, 6.0, 14.0, 0.1);
    CHECK(s.min == 6.0);
    CHECK(s.max == 14.0);
    CHECK(s.step == 0.1);
}

TEST_CASE("Value readers fall back on missing keys, wrong types and non-finite numbers") {
    Value o = Value::makeObject();
    o["d"] = 2.5;
    o["i"] = 4;
    o["b"] = true;
    o["s"] = "text";
    o["nan"] = std::numeric_limits<double>::quiet_NaN();
    o["inf"] = std::numeric_limits<double>::infinity();

    CHECK(getDouble(o, "d", -1.0) == 2.5);
    CHECK(getDouble(o, "i", -1.0) == 4.0);
    CHECK(getDouble(o, "missing", -1.0) == -1.0);
    CHECK(getDouble(o, "s", -1.0) == -1.0);
    CHECK(getBool(o, "b", false));
    CHECK(!getBool(o, "d", false));
    CHECK(getString(o, "s", "fb") == "text");
    CHECK(getString(o, "d", "fb") == "fb");
    CHECK(getString(o, "missing") == "");

    CHECK(std::isnan(getDouble(o, "nan", 7.0)));
    CHECK(getFiniteDouble(o, "nan", 7.0) == 7.0);
    CHECK(getFiniteDouble(o, "inf", 7.0) == 7.0);
    CHECK(getFiniteDouble(o, "d", 7.0) == 2.5);
    CHECK(getDouble(Value{}, "anything", 3.0) == 3.0);
}

TEST_CASE("ConfigBindings supports strings, geometry, custom accessors and single-key apply") {
    ChassisSpec ch;
    const auto chassis = chassisEditorBindings();
    const Value encoded = chassis.encode(ch);
    CHECK(validateProperties(chassis.schema(), encoded).empty());
    CHECK(chassis.apply(ch, "massKg", Value(62.5)));
    CHECK(ch.massKg == doctest::Approx(62.5f));
    CHECK(chassis.apply(ch, "size", toValue(Vec3{0.8f, 0.7f, 0.15f})));
    CHECK(ch.sizeM == Vec3{0.8f, 0.7f, 0.15f});
    CHECK_FALSE(chassis.apply(ch, "unknownKey", Value(1.0)));

    WheelSpec w;
    const auto wheel = wheelEditorBindings(w);
    CHECK(wheel.apply(w, "driveMotorCount", Value(99), true));
    CHECK(w.driveMotorCount == kMaxMotorsPerAxis);
    CHECK(wheel.apply(w, "driveMotor", Value("NEO")));
    CHECK(w.driveMotor.name == "NEO");

    CHECK(chassisEditorFieldKey("sizeM") == "size");
    CHECK(chassisEditorFieldKey("groundClearanceM") == "groundClearance");
    CHECK(wheelEditorFieldKey("radiusM") == "radius");
    CHECK(wheelEditorFieldKey("tire.mu") == "mu");
    CHECK(wheelEditorFieldKey("driveMotor.stallTorqueNm") == "driveMotor");
}

