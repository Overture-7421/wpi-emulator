#include <doctest/doctest.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/testing/fake_game_module.h"

using namespace wpiemu::sdk;

TEST_CASE("Result carries a value or an error") {
    Result<int> good = 42;
    CHECK(good.ok());
    CHECK(good.value() == 42);
    Result<int> bad = makeError(ErrorCode::NotFound, "nope");
    CHECK_FALSE(bad.ok());
    CHECK(bad.error().code == ErrorCode::NotFound);
    CHECK(bad.valueOr(7) == 7);

    Status s = okStatus();
    CHECK(s.ok());
    Status f = makeError(ErrorCode::IoError, "disk");
    CHECK_FALSE(f);
    CHECK(f.error().message == "disk");
}

TEST_CASE("strong ids are distinct, comparable and hashable") {
    static_assert(!std::is_convertible_v<BodyId, ShapeId>);
    BodyId none;
    CHECK_FALSE(none.valid());
    BodyId a{1}, b{2};
    CHECK(a < b);
    CHECK(a == BodyId{1});
    std::unordered_set<BodyId> set{a, b};
    CHECK(set.contains(BodyId{2}));
}

TEST_CASE("quaternion / pose math follows the Z-up convention") {
    const Quat yaw90 = Quat::fromYaw(static_cast<float>(kPi / 2));
    const Vec3 r = yaw90.rotate({1.f, 0.f, 0.f});
    CHECK(r.x == doctest::Approx(0.f).epsilon(1e-5));
    CHECK(r.y == doctest::Approx(1.f).epsilon(1e-5));  // +X rotates toward +Y (CCW from above)
    CHECK(yaw90.yaw() == doctest::Approx(static_cast<float>(kPi / 2)).epsilon(1e-5));

    const Pose3 a{{1.f, 2.f, 0.f}, yaw90};
    const Pose3 id = a * a.inverse();
    CHECK(id.position.x == doctest::Approx(0.f).epsilon(1e-5));
    CHECK(id.position.y == doctest::Approx(0.f).epsilon(1e-5));
    CHECK(id.rotation.w == doctest::Approx(1.f).epsilon(1e-5));
}

TEST_CASE("units convert to SI") {
    CHECK(inchesToMeters(4.0) == doctest::Approx(0.1016));
    CHECK(rpmToRadPerSec(60.0) == doctest::Approx(kTwoPi));
    CHECK(secondsToUs(0.005) == 5000);
    CHECK(usToSeconds(5000) == doctest::Approx(0.005));
}

TEST_CASE("DC motor model matches WPILib equations") {
    const DcMotorModel m(motors::neo());
    CHECK(m.torqueNm(12.0, 0.0) == doctest::Approx(2.6).epsilon(1e-9));          // stall torque
    CHECK(m.currentA(12.0, 0.0) == doctest::Approx(105.0).epsilon(1e-9));        // stall current
    CHECK(m.currentA(12.0, motors::neo().freeSpeedRadPerSec) == doctest::Approx(1.8).epsilon(1e-6));  // free current
}

TEST_CASE("motor table keys and names are unique, every row is physical and the SDK defaults come from it") {
    std::unordered_set<std::string> keys;
    std::unordered_set<std::string> names;
    for (const MotorPreset& p : motors::presets()) {
        INFO(p.key);
        CHECK(keys.insert(p.key).second);
        CHECK(names.insert(p.spec.name).second);   // saved wheel motors are matched to the table by name
        CHECK(p.spec.name == p.label);
        CHECK(isValidMotorSpec(p.spec));
    }
    REQUIRE(motors::findPreset("kraken_x44_foc") != nullptr);
    CHECK(motors::findPreset("nope") == nullptr);
    REQUIRE(motors::findPresetByName("Kraken X44") != nullptr);
    CHECK(motors::findPresetByName("Kraken X44")->key == "kraken_x44");
    CHECK(motors::findPresetByName("Homebrew") == nullptr);
    CHECK(motors::krakenX60().name == "Kraken X60");
    CHECK(motors::falcon500().name == "Falcon 500");
    CHECK(motors::neo().name == "NEO");
    CHECK(motors::neo550().name == "NEO 550");
}

TEST_CASE("motorSpecFaults names each non-physical field") {
    CHECK(motorSpecFaults(motors::neo()).empty());

    MotorSpec bad = motors::neo();
    bad.stallTorqueNm = 0.0;
    bad.freeSpeedRadPerSec = std::numeric_limits<double>::quiet_NaN();
    const auto faults = motorSpecFaults(bad);
    REQUIRE(faults.size() == 2);
    CHECK(std::string(faults[0].field) == "stallTorqueNm");
    CHECK(std::string(faults[1].field) == "freeSpeedRadPerSec");

    MotorSpec stalled = motors::neo();
    stalled.freeCurrentA = stalled.stallCurrentA;   // must stay below the stall current
    REQUIRE(motorSpecFaults(stalled).size() == 1);
    CHECK(std::string(motorSpecFaults(stalled)[0].field) == "freeCurrentA");
    stalled.freeCurrentA = -1.0;
    REQUIRE(motorSpecFaults(stalled).size() == 1);
    CHECK_FALSE(isValidMotorSpec(stalled));
}

TEST_CASE("Value: types, objects keep insertion order, deep equality") {
    Value v;
    v["b"] = 1;
    v["a"] = "text";
    v["nested"]["x"] = 2.5;
    v["list"].push_back(1);
    v["list"].push_back(true);

    CHECK(v.isObject());
    CHECK(v.object().keyAt(0) == "b");
    CHECK(v.object().keyAt(1) == "a");
    CHECK(v.find("b")->asInt() == 1);
    CHECK(v.findPath("nested.x")->asDouble() == 2.5);
    CHECK(v.findPath("nested.y") == nullptr);
    CHECK(v.get("a").asString() == "text");
    CHECK(v.get("missing", 9).asInt() == 9);
    CHECK(v.find("list")->array().size() == 2);

    Value w = v;
    CHECK(w == v);
    w["b"] = 2;
    CHECK_FALSE(w == v);
    CHECK(Value(1) == Value(1.0));
    CHECK(v.object().erase("a"));
    CHECK_FALSE(v.object().contains("a"));
}

TEST_CASE("value codec round-trips geometry types") {
    const Pose3 p{{1.f, 2.f, 3.f}, Quat::fromYaw(0.5f)};
    Pose3 back;
    CHECK(fromValue(toValue(p), back));
    CHECK(back.position == p.position);
    Box3 b{p, {1.f, 2.f, 3.f}};
    Box3 backBox;
    CHECK(fromValue(toValue(b), backBox));
    CHECK(backBox.sizeM == b.sizeM);
    Vec3 bad;
    CHECK_FALSE(fromValue(Value(3), bad));
}

TEST_CASE("Registry: ordered, rejects duplicates, creates by key") {
    struct Thing { virtual ~Thing() = default; virtual int n() const = 0; };
    struct One final : Thing { int n() const override { return 1; } };
    Registry<NoInfo, Thing> reg;
    CHECK(reg.add("one", {}, [] { return std::make_unique<One>(); }).ok());
    CHECK(reg.add("one", {}, [] { return std::make_unique<One>(); }).error().code == ErrorCode::AlreadyExists);
    CHECK(reg.add("", {}, [] { return std::make_unique<One>(); }).error().code == ErrorCode::InvalidArgument);
    CHECK(reg.create("one").value()->n() == 1);
    CHECK(reg.create("two").error().code == ErrorCode::NotFound);
    CHECK(reg.keys() == std::vector<std::string>{"one"});
}

TEST_CASE("Rng is deterministic for a seed") {
    Rng a(123), b(123);
    for (int i = 0; i < 10; ++i) CHECK(a.nextU64() == b.nextU64());
    Rng c(5);
    const double d = c.nextDouble();
    CHECK(d >= 0.0);
    CHECK(d < 1.0);
}

TEST_CASE("PropertySchema: defaults and validation") {
    PropertySchema schema;
    PropertySpec gear;
    gear.key = "gearRatio";
    gear.type = PropertyType::Double;
    gear.defaultValue = 5.0;
    gear.min = 0.1;
    gear.max = 100.0;
    PropertySpec mode;
    mode.key = "mode";
    mode.type = PropertyType::Enum;
    mode.options = {{"a", "A"}, {"b", "B"}};
    mode.defaultValue = "a";
    mode.required = true;
    schema.properties = {gear, mode};

    Value values = Value::makeObject();
    applyDefaults(schema, values);
    CHECK(values.find("gearRatio")->asDouble() == 5.0);
    CHECK(validateProperties(schema, values).empty());

    values["gearRatio"] = 1000.0;
    values["mode"] = "zzz";
    values["extra"] = 1;
    const auto issues = validateProperties(schema, values);
    CHECK(issues.size() == 3);
    CHECK(issues.back().severity == Severity::Warning);
    CHECK_FALSE(validateProperties(schema, Value(3)).empty());
}

