#include <doctest/doctest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/unit_display.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/byte_order.h"
#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/core/strings.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/testing/environment.h"
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

TEST_CASE("angles wrap to the half-open range around zero and convert in float") {
    CHECK(wrapAngleRad(0.5) == doctest::Approx(0.5));
    CHECK(wrapAngleRad(3.0 * kPi) == doctest::Approx(kPi));
    CHECK(wrapAngleRad(-kPi) == doctest::Approx(kPi));
    CHECK(wrapAngleRad(kPi + 0.1) == doctest::Approx(-kPi + 0.1));
    CHECK(wrapAngleDeg(540.0) == doctest::Approx(180.0));
    CHECK(wrapAngleDeg(-180.0) == doctest::Approx(180.0));
    CHECK(wrapAngleDeg(190.0) == doctest::Approx(-170.0));
    CHECK(wrapAngleDeg(-190.0) == doctest::Approx(170.0));
    CHECK(degToRadF(180.f) == doctest::Approx(kPiF));
    CHECK(radToDegF(kPiF) == doctest::Approx(180.f));
}

TEST_CASE("Euler extraction inverts fromEuler, ignores scale and stays finite at gimbal lock") {
    for (float roll : {-0.7f, 0.f, 0.3f})
        for (float pitch : {-0.9f, 0.f, 0.4f})
            for (float yaw : {-3.0f, 0.f, 2.0f}) {
                const Quat q = Quat::fromEuler(roll, pitch, yaw);
                const Euler e = q.toEuler();
                CHECK(e.rollRad == doctest::Approx(roll).epsilon(1e-4));
                CHECK(e.pitchRad == doctest::Approx(pitch).epsilon(1e-4));
                CHECK(e.yawRad == doctest::Approx(yaw).epsilon(1e-4));
                CHECK(Quat{q.x * 3.f, q.y * 3.f, q.z * 3.f, q.w * 3.f}.toEuler().yawRad == doctest::Approx(yaw).epsilon(1e-4));
            }
    CHECK(Quat::fromEuler(Euler{0.1f, -0.2f, 0.3f}) == Quat::fromEuler(0.1f, -0.2f, 0.3f));
    const Euler lock = Quat::fromEuler(0.1f, 0.5f * kPiF, 0.2f).toEuler();
    CHECK(finite(lock.pitchRad));
    CHECK(finite(lock.rollRad));
    CHECK(lock.pitchRad == doctest::Approx(0.5f * kPiF).epsilon(1e-3));
}

TEST_CASE("rotationFromX takes +X onto any non-zero direction and defines the degenerate cases") {
    const auto maps = [](const Vec3& direction, const Vec3& expected, float tolerance = 1e-5f) {
        const Vec3 r = rotationFromX(direction).rotate({1.f, 0.f, 0.f});
        return length(r - expected) < tolerance;
    };
    CHECK(maps({0.f, 1.f, 0.f}, {0.f, 1.f, 0.f}));
    CHECK(maps({0.f, 0.f, 5.f}, {0.f, 0.f, 1.f}));                    // length does not matter
    CHECK(maps(normalized({1.f, -2.f, 3.f}), normalized({1.f, -2.f, 3.f})));
    CHECK(maps({1e-4f, 1e-4f, 0.f}, normalized({1e-4f, 1e-4f, 0.f})));  // tiny but non-zero
    CHECK(rotationFromX({1.f, 0.f, 0.f}) == Quat{});
    CHECK(maps({-1.f, 0.f, 0.f}, {-1.f, 0.f, 0.f}, 1e-3f));             // antiparallel: half turn about +Z
    CHECK(maps({-1.f, 1e-6f, 0.f}, {-1.f, 0.f, 0.f}, 1e-3f));
    CHECK(rotationFromX({0.f, 0.f, 0.f}) == Quat{});
    CHECK(rotationFromX({std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f}) == Quat{});
    CHECK(rotationFromX({std::numeric_limits<float>::infinity(), 0.f, 0.f}) == Quat{});
}

TEST_CASE("finite rejects NaN and infinity in any component of a vector, quaternion or pose") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK(finite(1.f));
    CHECK_FALSE(finite(inf));
    CHECK(finite(Vec3{1.f, 2.f, 3.f}));
    CHECK_FALSE(finite(Vec3{1.f, nan, 3.f}));
    CHECK(finite(Quat{}));
    CHECK_FALSE(finite(Quat{0.f, 0.f, inf, 1.f}));
    CHECK_FALSE(finite(Quat{0.f, 0.f, 0.f, nan}));
    CHECK(finite(Pose3{}));
    CHECK_FALSE(finite(Pose3{{0.f, 0.f, inf}, Quat{}}));
    CHECK_FALSE(finite(Pose3{{}, Quat{nan, 0.f, 0.f, 1.f}}));
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

TEST_CASE("unit display: one table converts, round-trips and labels every property unit in both systems") {
    using U = PropertyUnit;
    CHECK(toDisplay(0.0254, U::Meters, UnitSystem::Imperial) == doctest::Approx(1.0));
    CHECK(toDisplay(0.01, U::Meters, UnitSystem::Metric) == doctest::Approx(1.0));
    CHECK(toDisplay(0.45359237, U::Kilograms, UnitSystem::Imperial) == doctest::Approx(1.0));
    CHECK(toDisplay(kPi, U::Radians, UnitSystem::Metric) == doctest::Approx(180.0));
    CHECK(toDisplay(rpmToRadPerSec(60.0), U::RadiansPerSecond, UnitSystem::Imperial) == doctest::Approx(60.0));
    CHECK(std::string(unitDisplay(U::Meters, UnitSystem::Imperial).suffix) == " in");
    CHECK(std::string(unitDisplay(U::Meters, UnitSystem::Metric).suffix) == " cm");

    for (const UnitSystem system : {UnitSystem::Imperial, UnitSystem::Metric}) {
        for (int i = 0; i <= static_cast<int>(U::Hertz); ++i) {   // Hertz is the last PropertyUnit
            const U unit = static_cast<U>(i);
            INFO(i);
            CHECK(fromDisplay(toDisplay(1.2345, unit, system), unit, system) == doctest::Approx(1.2345));
            CHECK(unitDisplay(unit, system).factor == doctest::Approx(toDisplay(1.0, unit, system)));
        }
    }
}

TEST_CASE("little-endian helpers write the least significant byte first and read it back") {
    std::vector<std::byte> out;
    appendLittleEndian(out, 0x0102030405060708ull, 8);
    REQUIRE(out.size() == 8);
    CHECK(static_cast<int>(out[0]) == 0x08);
    CHECK(static_cast<int>(out[7]) == 0x01);
    CHECK(readLittleEndian(out, 0, 8) == 0x0102030405060708ull);
    CHECK(readLittleEndian(out, 4, 4) == 0x01020304ull);

    std::vector<std::uint8_t> raw;
    appendLittleEndian(raw, 0xABCDull, 2);
    appendLittleEndian(raw, 0x1FFull, 1);   // only the low byte is written
    CHECK(raw == std::vector<std::uint8_t>{0xCD, 0xAB, 0xFF});
}

TEST_CASE("asciiLower lowers A-Z only") {
    CHECK(asciiLower("Blue LEFT 123-_") == "blue left 123-_");
    CHECK(asciiLower("\xC3\x84") == "\xC3\x84");   // non-ASCII bytes stay as they are, whatever the locale
    CHECK(asciiLower("").empty());
}

TEST_CASE("asciiTrim strips whitespace and NUL padding from both ends only") {
    CHECK(asciiTrim("  a b\t\r\n") == "a b");
    CHECK(asciiTrim(std::string_view("12\0\0", 4)) == "12");
    CHECK(asciiTrim(" \t\n").empty());
    CHECK(asciiTrim("").empty());
    CHECK(asciiTrim("\xC2\xA0x") == "\xC2\xA0x");   // a non-ASCII space is not whitespace here
}

TEST_CASE("parseBool and parseInteger accept only complete unambiguous text") {
    CHECK(parseBool(" Yes ") == std::optional<bool>(true));
    CHECK(parseBool("ON") == std::optional<bool>(true));
    CHECK(parseBool("0") == std::optional<bool>(false));
    CHECK(parseBool("False") == std::optional<bool>(false));
    CHECK_FALSE(parseBool("maybe").has_value());
    CHECK_FALSE(parseBool("").has_value());

    CHECK(parseInteger<int>("-42") == std::optional<int>(-42));
    CHECK(parseInteger<std::uint32_t>("4294967295") == std::optional<std::uint32_t>(4294967295u));
    CHECK_FALSE(parseInteger<std::uint32_t>("4294967296").has_value());   // overflow
    CHECK_FALSE(parseInteger<std::uint32_t>("-5").has_value());
    CHECK_FALSE(parseInteger<int>("12x").has_value());
    CHECK_FALSE(parseInteger<int>(" 12").has_value());
    CHECK_FALSE(parseInteger<int>("").has_value());
}

TEST_CASE("log levels print as console names and parse case-insensitively with warning accepted") {
    for (const LogLevel level : {LogLevel::Trace, LogLevel::Debug, LogLevel::Info, LogLevel::Warn, LogLevel::Error})
        CHECK(parseLogLevel(logLevelName(level)) == std::optional<LogLevel>(level));
    CHECK(logLevelName(LogLevel::Warn) == "WARN");
    CHECK(parseLogLevel(" Warning ") == std::optional<LogLevel>(LogLevel::Warn));
    CHECK_FALSE(parseLogLevel("loud").has_value());
}

TEST_CASE("the process environment reads unset and empty variables as absent and a fixed one is injectable") {
    CHECK_FALSE(readEnvironment("WPIEMU_SURELY_UNSET_VARIABLE").has_value());
    CHECK(readEnvironment("PATH").has_value());   // set on every machine that can run the tests

    const EnvLookup fixed = testing::envOf({{"A", "1"}});
    CHECK(fixed("A") == std::optional<std::string>("1"));
    CHECK_FALSE(fixed("B").has_value());
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

