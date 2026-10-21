#include <doctest/doctest.h>

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/sdk/config/property_schema.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

namespace {

constexpr const char* kTypeNames[] = {"Bool", "Int", "Double", "String", "Enum", "Vec3", "Pose3", "Box3", "Color"};
constexpr const char* kUnitNames[] = {"None",   "Meters",  "MetersPerSecond", "Radians", "RadiansPerSecond", "Kilograms",
                                      "Seconds", "Volts",   "Amps",            "Newtons", "NewtonMeters",     "KgM2",
                                      "Ratio",   "Percent", "Count",           "Hertz"};

std::string scalar(const sdk::Value& v) {
    std::ostringstream o;
    o << std::setprecision(9);
    if (v.isBool()) o << (v.asBool() ? "true" : "false");
    else if (v.isInt()) o << v.asInt() << 'i';
    else if (v.isDouble()) o << v.asDouble() << 'd';
    else o << '"' << v.asString() << '"';
    return o.str();
}

// One key per line, in saved order; the suffix tells an Int from a Double.
std::string dump(const sdk::Value& object) {
    std::string out;
    for (std::size_t i = 0; i < object.object().size(); ++i)
        out += object.object().keyAt(i) + "=" + scalar(object.object().valueAt(i)) + "\n";
    return out;
}

std::string describe(const sdk::PropertySpec& p) {
    std::ostringstream o;
    o << p.key << '|' << p.label << '|' << p.group << '|' << kTypeNames[static_cast<int>(p.type)] << '|'
      << kUnitNames[static_cast<int>(p.unit)] << '|' << scalar(p.defaultValue) << '|';
    if (p.min) o << *p.min;
    else o << '-';
    o << '|';
    if (p.max) o << *p.max;
    else o << '-';
    for (const sdk::EnumOption& option : p.options) o << '|' << option.value << '=' << option.label;
    o << '\n';
    return o.str();
}

// Every typed field, independent of toValue. Enums print as their integer value.
std::string fields(const GameConfig& c) {
    std::ostringstream o;
    o << std::setprecision(9) << "capacity=" << c.capacity << " preload=" << c.preloadFuel << " neutral=" << c.neutralFuel
      << " depot=" << c.depotFuel << " chute=" << c.outpostChuteFuel << " maxField=" << c.maxFuelOnField
      << " human=" << static_cast<int>(c.humanPlayer) << " periodS=" << c.humanPlayerPeriodS << " mass=" << c.fuel.massKg
      << " friction=" << c.fuel.friction << " restitution=" << c.fuel.restitution
      << " rolling=" << c.fuel.rollingResistance << " sleeping=" << c.fuel.sleeping << " ghostS=" << c.shotGhostS
      << " hubShifts=" << c.enforceHubShifts << " firstInactive=" << static_cast<int>(c.firstInactive)
      << " hubDelayS=" << c.hubProcessingDelayS << " recycle=" << c.recycleScoredFuel;
    return o.str();
}

const char* const kDefaultFields =
    "capacity=30 preload=8 neutral=360 depot=24 chute=24 maxField=800 human=0 periodS=2 mass=0.215000004 "
    "friction=0.600000024 restitution=0.550000012 rolling=0.0199999996 sleeping=1 ghostS=0.150000006 hubShifts=1 "
    "firstInactive=0 hubDelayS=1 recycle=1";

}  // namespace

TEST_CASE("config schema: every key keeps its order, group, unit, default and range") {
    const sdk::PluginConfigSchema schema = configSchema();
    CHECK(schema.schemaVersion == kConfigSchemaVersion);
    CHECK(schema.groupOrder == std::vector<std::string>{"Fuel meter", "Field staging", "Fuel physics", "Match rules"});

    std::string actual;
    for (const sdk::PropertySpec& p : schema.properties.properties) {
        actual += describe(p);
        CHECK_MESSAGE(!p.description.empty(), p.key);
    }
    CHECK(actual ==
          "capacity|Fuel capacity|Fuel meter|Int|Count|30i|0|500\n"
          "preloadFuel|Preload|Fuel meter|Int|Count|8i|0|500\n"
          "neutralFuel|Neutral zone fuel|Field staging|Int|Count|360i|0|408\n"
          "depotFuel|Fuel per depot|Field staging|Int|Count|24i|0|24\n"
          "outpostChuteFuel|Fuel per outpost chute|Field staging|Int|Count|24i|0|100\n"
          "humanPlayer|Human player|Field staging|Enum|None|\"off\"|-|-|off=Off (chute fuel is never released)|"
          "timed=Timed (one fuel per period)\n"
          "humanPlayerPeriodS|Human player period|Field staging|Double|Seconds|2d|0.1|60\n"
          "maxFuelOnField|Max fuel bodies|Field staging|Int|Count|800i|0|5000\n"
          "fuelMassKg|Fuel mass|Fuel physics|Double|Kilograms|0.215000004d|0.05|1\n"
          "fuelFriction|Fuel friction|Fuel physics|Double|None|0.600000024d|0|2\n"
          "fuelRestitution|Fuel restitution|Fuel physics|Double|None|0.550000012d|0|1\n"
          "fuelRollingResistance|Fuel rolling resistance|Fuel physics|Double|None|0.0199999996d|0|1\n"
          "fuelSleeping|Fuel sleeping|Fuel physics|Bool|None|true|-|-\n"
          "shotGhostS|Shot ghost time|Fuel physics|Double|Seconds|0.15d|0|1\n"
          "enforceHubShifts|Enforce hub shifts|Match rules|Bool|None|true|-|-\n"
          "firstInactiveHub|First inactive hub|Match rules|Enum|None|\"auto\"|-|-|auto=Auto winner (FMS rule)|"
          "red=Red|blue=Blue\n"
          "hubProcessingDelayS|Hub processing delay|Match rules|Double|Seconds|1d|0|10\n"
          "recycleScoredFuel|Recycle scored fuel|Match rules|Bool|None|true|-|-\n");
}

TEST_CASE("config schema validation reports the same issues for the same section") {
    const sdk::PropertySchema& schema = configSchema().properties;
    CHECK(sdk::validateProperties(schema, sdk::defaultsOf(schema)).empty());

    sdk::Value section = sdk::defaultsOf(schema);
    section["capacity"] = 501;
    section["humanPlayer"] = "maybe";
    section["fuelSleeping"] = 1;
    section["extra"] = 1;
    std::string actual;
    for (const sdk::Issue& i : sdk::validateProperties(schema, section))
        actual += std::to_string(static_cast<int>(i.severity)) + " " + i.path + ": " + i.message + "\n";
    CHECK(actual ==
          "2 capacity: 'capacity' out of range\n"
          "2 humanPlayer: wrong type or invalid value for 'humanPlayer'\n"
          "2 fuelSleeping: wrong type or invalid value for 'fuelSleeping'\n"
          "1 extra: unknown property\n");
}

TEST_CASE("config: missing keys, a non-object section and bad enum names give the defaults") {
    CHECK(fields(GameConfig{}) == kDefaultFields);
    CHECK(fields(parseConfig(sdk::Value::makeObject())) == kDefaultFields);
    CHECK(fields(parseConfig(sdk::Value{})) == kDefaultFields);
    CHECK(fields(parseConfig(sdk::Value("text"))) == kDefaultFields);

    sdk::Value bad = sdk::Value::makeObject();
    bad["humanPlayer"] = "other";
    bad["firstInactiveHub"] = "green";
    CHECK(fields(parseConfig(bad)) == kDefaultFields);
    bad["humanPlayer"] = 5;
    bad["firstInactiveHub"] = 2;
    CHECK(fields(parseConfig(bad)) == kDefaultFields);
}

TEST_CASE("config: numbers clamp to the schema range, wrong types and NaN keep the default, unknown keys are ignored") {
    sdk::Value v = sdk::Value::makeObject();
    v["capacity"] = 20;
    v["preloadFuel"] = 50;           // clamped to the capacity afterwards
    v["neutralFuel"] = 999.7;        // a double truncates to an int, then clamps
    v["depotFuel"] = "x";
    v["outpostChuteFuel"] = 50.9;
    v["humanPlayer"] = "timed";
    v["humanPlayerPeriodS"] = 0;
    v["maxFuelOnField"] = true;
    v["fuelMassKg"] = std::numeric_limits<double>::quiet_NaN();
    v["fuelFriction"] = 5;
    v["fuelRestitution"] = -1;
    v["fuelRollingResistance"] = 0.5;
    v["fuelSleeping"] = false;
    v["shotGhostS"] = "0.3";
    v["enforceHubShifts"] = 1;
    v["firstInactiveHub"] = "blue";
    v["hubProcessingDelayS"] = 99;
    v["recycleScoredFuel"] = false;
    v["bogus"] = 1;
    CHECK(fields(parseConfig(v)) ==
          "capacity=20 preload=20 neutral=408 depot=24 chute=50 maxField=800 human=1 periodS=0.1 mass=0.215000004 "
          "friction=2 restitution=0 rolling=0.5 sleeping=0 ghostS=0.150000006 hubShifts=1 firstInactive=2 hubDelayS=10 "
          "recycle=0");

    sdk::Value low = sdk::Value::makeObject();
    low["capacity"] = -5;
    low["preloadFuel"] = 4;
    low["firstInactiveHub"] = "red";
    CHECK(fields(parseConfig(low)) ==
          "capacity=0 preload=0 neutral=360 depot=24 chute=24 maxField=800 human=0 periodS=2 mass=0.215000004 "
          "friction=0.600000024 restitution=0.550000012 rolling=0.0199999996 sleeping=1 ghostS=0.150000006 hubShifts=1 "
          "firstInactive=1 hubDelayS=1 recycle=1");
}

TEST_CASE("config: toValue writes every key in schema order and parseConfig reads it back") {
    const sdk::PropertySchema& schema = configSchema().properties;
    CHECK(dump(toValue(GameConfig{})) ==
          "capacity=30i\npreloadFuel=8i\nneutralFuel=360i\ndepotFuel=24i\noutpostChuteFuel=24i\nhumanPlayer=\"off\"\n"
          "humanPlayerPeriodS=2d\nmaxFuelOnField=800i\nfuelMassKg=0.215000004d\nfuelFriction=0.600000024d\n"
          "fuelRestitution=0.550000012d\nfuelRollingResistance=0.0199999996d\nfuelSleeping=true\n"
          "shotGhostS=0.150000006d\nenforceHubShifts=true\nfirstInactiveHub=\"auto\"\nhubProcessingDelayS=1d\n"
          "recycleScoredFuel=true\n");

    // The struct defaults and the schema defaults are two spellings of the same numbers.
    const sdk::Value encoded = toValue(GameConfig{});
    REQUIRE(encoded.object().size() == schema.properties.size());
    for (std::size_t i = 0; i < schema.properties.size(); ++i) {
        const sdk::PropertySpec& p = schema.properties[i];
        CHECK_MESSAGE(encoded.object().keyAt(i) == p.key, p.key);
        const sdk::Value* v = encoded.find(p.key);
        REQUIRE_MESSAGE(v != nullptr, p.key);
        if (p.defaultValue.isNumber()) CHECK_MESSAGE(v->asDouble() == doctest::Approx(p.defaultValue.asDouble()), p.key);
        else CHECK_MESSAGE(*v == p.defaultValue, p.key);
    }

    GameConfig c;
    c.capacity = 12;
    c.preloadFuel = 5;
    c.neutralFuel = 400;
    c.depotFuel = 10;
    c.outpostChuteFuel = 7;
    c.maxFuelOnField = 1234;
    c.humanPlayer = HumanPlayerMode::Timed;
    c.humanPlayerPeriodS = 3.5;
    c.fuel.massKg = 0.25f;
    c.fuel.friction = 0.5f;
    c.fuel.restitution = 0.75f;
    c.fuel.rollingResistance = 0.125f;
    c.fuel.sleeping = false;
    c.shotGhostS = 0.25f;
    c.enforceHubShifts = false;
    c.firstInactive = FirstInactiveMode::Blue;
    c.hubProcessingDelayS = 2.5;
    c.recycleScoredFuel = false;
    const sdk::Value saved = toValue(c);
    CHECK(dump(saved) ==
          "capacity=12i\npreloadFuel=5i\nneutralFuel=400i\ndepotFuel=10i\noutpostChuteFuel=7i\nhumanPlayer=\"timed\"\n"
          "humanPlayerPeriodS=3.5d\nmaxFuelOnField=1234i\nfuelMassKg=0.25d\nfuelFriction=0.5d\nfuelRestitution=0.75d\n"
          "fuelRollingResistance=0.125d\nfuelSleeping=false\nshotGhostS=0.25d\nenforceHubShifts=false\n"
          "firstInactiveHub=\"blue\"\nhubProcessingDelayS=2.5d\nrecycleScoredFuel=false\n");
    CHECK(fields(parseConfig(saved)) == fields(c));
    CHECK(dump(toValue(parseConfig(saved))) == dump(saved));
}
