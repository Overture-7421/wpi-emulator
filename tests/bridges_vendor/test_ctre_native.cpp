#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "wpiemu/bridges/vendor/ctre_native.h"
#include "wpiemu/bridges/vendor/symbol_resolver.h"
#include "wpiemu/sdk/core/result.h"

using namespace wpiemu::bridges::vendor;
using wpiemu::sdk::ErrorCode;

namespace {

// Fake CTRE exports with the exact C ABI of platform_c_style.h / DeviceEncoding_Interface.h / Configs_Interface.h.
std::string g_lastModel;
std::string g_lastBus;
int g_configCalls = 0;

int fakeEncodeDevice(int deviceId, const char* model, const char* canbus, std::uint32_t* encoding) {
    g_lastModel = model;
    g_lastBus = canbus;
    *encoding = static_cast<std::uint32_t>(0x80 + deviceId);
    return 0;
}

int fakeGetConfigsNoMalloc(int /*context*/, const char* /*network*/, int deviceHash, double /*timeout*/, char* str,
                           std::uint32_t strLen, bool /*useDid*/) {
    ++g_configCalls;
    if (deviceHash == 0x80 + 99) return -1;  // simulated failure for id 99
    const std::string text = "1901,s_0\n1918,s_1070945649\n987,s_1056964608\n";
    if (text.size() + 1 > strLen) return -2;
    std::memcpy(str, text.c_str(), text.size() + 1);
    return 0;
}

double g_physicsRotorVelocity = 0.0;

std::int32_t fakeGetPhysics(int type, int id, const char* key, double* value) {
    if (id == 63) return 1200;
    if (std::string(key) != "RotorVelocity") return 1201;
    *value = g_physicsRotorVelocity + type;
    return 0;
}

std::int32_t fakeSetPhysics(int, int, const char* key, double value) {
    if (std::string(key) != "RotorVelocity") return 1201;
    g_physicsRotorVelocity = value;
    return 0;
}

class FakeResolver final : public ISymbolResolver {
public:
    void* resolve(std::string_view symbol) const override {
        const auto it = symbols.find(std::string(symbol));
        return it == symbols.end() ? nullptr : it->second;
    }
    std::map<std::string, void*> symbols;

    static std::shared_ptr<FakeResolver> full() {
        auto r = std::make_shared<FakeResolver>();
        r->symbols["c_ctre_phoenix6_encode_device"] = reinterpret_cast<void*>(&fakeEncodeDevice);
        r->symbols["c_ctre_phoenix6_get_configs_no_malloc"] = reinterpret_cast<void*>(&fakeGetConfigsNoMalloc);
        r->symbols["c_ctre_phoenix6_platform_sim_get_physics_value"] = reinterpret_cast<void*>(&fakeGetPhysics);
        r->symbols["c_ctre_phoenix6_platform_sim_set_physics_input"] = reinterpret_cast<void*>(&fakeSetPhysics);
        return r;
    }
};

}  // namespace

TEST_CASE("vendor config snapshot: decodes the ADR 0002 example and typed accessors") {
    const auto snap = CtreConfigSnapshot::parse(
        "1901,s_1\n"
        "1918,s_1070945649\n"      // SensorToMechanismRatio 1.6667 (IEEE-754 float bits)
        "1919,s_1065353216\n"      // RotorToSensorRatio 1.0
        "1920,s_1\n"
        "1921,s_7\n"
        "987,f_-0.25\n"            // decimal form seen by hooks
        "1928,s_0\n"
        "9999,s_12345\n");
    CHECK(snap.size() == 8);
    CHECK(snap.clockwisePositive().value() == true);
    CHECK(snap.sensorToMechanismRatio().value() == doctest::Approx(1.66667).epsilon(1e-5));
    CHECK(snap.rotorToSensorRatio().value() == doctest::Approx(1.0));
    CHECK(snap.feedbackSensorSource().value() == 1);
    CHECK(snap.remoteSensorId().value() == 7);
    CHECK(snap.magnetOffsetRot().value() == doctest::Approx(-0.25));
    CHECK(snap.forwardLimitType().value() == 0);
    CHECK_FALSE(snap.reverseLimitType().has_value());
    CHECK(snap.raw().at(9999) == "s_12345");  // unknown SPNs stay available as raw text
    CHECK(snap.summary().find("sensorToMechanism=1.66667") != std::string::npos);
}

TEST_CASE("vendor config snapshot: tolerant of CRLF, junk lines and empty input") {
    CHECK(CtreConfigSnapshot::parse("").empty());
    const auto snap = CtreConfigSnapshot::parse("garbage\r\n,s_1\nabc,s_2\n1901,s_0\r\n1904\n");
    CHECK(snap.size() == 1);
    CHECK(snap.clockwisePositive().value() == false);
    CHECK_FALSE(snap.number(1918).has_value());
    CHECK(CtreConfigSnapshot::parse("1918,s_notanumber").sensorToMechanismRatio() == std::nullopt);
    CHECK(CtreConfigSnapshot::parse("828,s_1").clockwisePositive().value() == true);  // CANcoder SensorDirection
}

TEST_CASE(
    "vendor config snapshot: negative float bits may use signed or unsigned decimal integers") {
    CHECK(CtreConfigSnapshot::parse("987,s_-1105788928").magnetOffsetRot().value() ==
          doctest::Approx(-0.1474609375));
    CHECK(CtreConfigSnapshot::parse("987,s_3189178368").magnetOffsetRot().value() ==
          doctest::Approx(-0.1474609375));
    CHECK_FALSE(CtreConfigSnapshot::parse("987,s_-4294967296").magnetOffsetRot().has_value());
}

TEST_CASE("vendor native: reads configs through the exported C ABI") {
    g_configCalls = 0;
    CtreSimNative native(FakeResolver::full());
    REQUIRE(native.hasConfigReadback());
    REQUIRE(native.hasPhysicsApi());
    const auto text = native.readConfigs(CtreDeviceType::P6_TalonFX, 4, "canivore");
    REQUIRE(text.ok());
    CHECK(g_lastModel == "talon fx");
    CHECK(g_lastBus == "canivore");
    const auto snap = CtreConfigSnapshot::parse(text.value());
    CHECK(snap.sensorToMechanismRatio().value() == doctest::Approx(1.66667).epsilon(1e-5));
    CHECK(snap.magnetOffsetRot().value() == doctest::Approx(0.5));

    CHECK(native.readConfigs(CtreDeviceType::P6_CANcoder, 1, "").ok());
    CHECK(g_lastBus == "rio");  // empty bus falls back to "rio" (the sim is bus-blind)
    CHECK(g_lastModel == "cancoder");
}

TEST_CASE("vendor native: failures and unsupported device types are errors, not crashes") {
    CtreSimNative native(FakeResolver::full());
    const auto failed = native.readConfigs(CtreDeviceType::P6_TalonFX, 99, "rio");
    REQUIRE_FALSE(failed.ok());
    CHECK(failed.error().code == ErrorCode::IoError);
    const auto p5 = native.readConfigs(CtreDeviceType::TalonSRX, 1, "rio");
    REQUIRE_FALSE(p5.ok());
    CHECK(p5.error().code == ErrorCode::Unsupported);
}

TEST_CASE("vendor native: absent symbols are tolerated") {
    CtreSimNative none(std::make_shared<FakeResolver>());
    CHECK_FALSE(none.hasConfigReadback());
    CHECK_FALSE(none.hasPhysicsApi());
    const auto r = none.readConfigs(CtreDeviceType::P6_TalonFX, 4, "rio");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::Unsupported);
    double v = 0;
    CHECK(none.getPhysicsValue(CtreDeviceType::P6_TalonFX, 4, "RotorVelocity", &v) == kCtreNativeUnavailable);
    CHECK(none.setPhysicsInput(CtreDeviceType::P6_TalonFX, 4, "RotorVelocity", 1.0) == kCtreNativeUnavailable);

    CtreSimNative nullResolver(nullptr);
    CHECK_FALSE(nullResolver.hasConfigReadback());

    auto partial = std::make_shared<FakeResolver>();  // physics only (e.g. an older CTRE build)
    partial->symbols["c_ctre_phoenix6_platform_sim_get_physics_value"] = reinterpret_cast<void*>(&fakeGetPhysics);
    partial->symbols["c_ctre_phoenix6_platform_sim_set_physics_input"] = reinterpret_cast<void*>(&fakeSetPhysics);
    CtreSimNative physicsOnly(partial);
    CHECK_FALSE(physicsOnly.hasConfigReadback());
    CHECK(physicsOnly.hasPhysicsApi());

    CHECK(makeCtreSimNative(std::make_shared<FakeResolver>()) == nullptr);
    CHECK(makeCtreSimNative(FakeResolver::full()) != nullptr);
}

TEST_CASE("vendor native: physics get/set return CTRE status codes verbatim") {
    CtreSimNative native(FakeResolver::full());
    CHECK(native.setPhysicsInput(CtreDeviceType::P6_TalonFX, 4, "RotorVelocity", 12.5) == 0);
    double v = 0;
    CHECK(native.getPhysicsValue(CtreDeviceType::P6_CANcoder, 4, "RotorVelocity", &v) == 0);
    CHECK(v == doctest::Approx(12.5 + 5));
    CHECK(native.getPhysicsValue(CtreDeviceType::P6_TalonFX, 63, "RotorVelocity", &v) == kCtreSimDeviceNotFound);
    CHECK(native.getPhysicsValue(CtreDeviceType::P6_TalonFX, 4, "Nope", &v) == kCtreSimPhysicsTypeNotSupported);
}

TEST_CASE("vendor native: device type tables") {
    CHECK(ctreDeviceTypeFromDescriptorType("talonfx") == CtreDeviceType::P6_TalonFX);
    CHECK(ctreDeviceTypeFromDescriptorType("pigeon2") == CtreDeviceType::P6_Pigeon2);
    CHECK(ctreDeviceTypeFromDescriptorType("talonsrx") == CtreDeviceType::TalonSRX);
    CHECK_FALSE(ctreDeviceTypeFromDescriptorType("sparkmax").has_value());
    CHECK(ctreModelName(CtreDeviceType::P6_TalonFXS) == "talon fxs");
    CHECK(ctreModelName(CtreDeviceType::P6_CANrange) == "canrange");
    CHECK(ctreModelName(CtreDeviceType::VictorSPX).empty());
    CHECK(static_cast<int>(CtreDeviceType::P6_CANdle) == 10);  // ctre::phoenix::platform::DeviceType ABI
    CHECK(static_cast<int>(CtreDeviceType::P6_TalonFX) == 4);
}

TEST_CASE("vendor module resolver: attach-only never loads, names are per platform") {
    std::string error;
    CHECK(ModuleSymbolResolver::open("definitely_not_a_loaded_module_xyz", ModuleLoadMode::AttachOnly, &error) == nullptr);
    CHECK_FALSE(error.empty());
    CHECK(ctreSimModuleName().find("CTRE_PhoenixTools_Sim") != std::string::npos);
    CHECK(wpiHalModuleName().find("wpiHal") != std::string::npos);
    CHECK(makeCtreSimNative() == nullptr);  // no CTRE module in the unit-test process
#if defined(_WIN32)
    const auto kernel = ModuleSymbolResolver::open("kernel32.dll", ModuleLoadMode::AttachOnly);
    REQUIRE(kernel != nullptr);
    CHECK(kernel->resolve("GetTickCount64") != nullptr);
    CHECK(kernel->resolve("no_such_export_wpiemu") == nullptr);
#endif
}
