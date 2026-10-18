#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "hal_test_rig.h"
#include "wpiemu/bridges/hal/hal_units.h"

using namespace hal_test;

TEST_CASE("hal units: speed normalisation handles NaN, inf and out-of-range") {
    CHECK(hal::normalizeSpeed(0.25) == 0.25);
    CHECK(hal::normalizeSpeed(3.0) == 1.0);
    CHECK(hal::normalizeSpeed(-3.0) == -1.0);
    CHECK(hal::normalizeSpeed(std::numeric_limits<double>::quiet_NaN()) == 0.0);
    CHECK(hal::normalizeSpeed(std::numeric_limits<double>::infinity()) == 0.0);
}

TEST_CASE("hal units: encoder conversions") {
    CHECK(hal::rotationsToCounts(1.5, 2048.0) == 3072);
    CHECK(hal::rotationsToCounts(-0.5, 2048.0) == -1024);
    CHECK(hal::rotationsToCounts(1e30, 2048.0) == std::numeric_limits<int>::max());
    CHECK(hal::rotationsToCounts(-1e30, 2048.0) == std::numeric_limits<int>::min());
    CHECK(hal::rotationsToCounts(std::nan(""), 2048.0) == 0);
    CHECK(hal::countsToRotations(1024, 2048.0) == doctest::Approx(0.5));
    CHECK(hal::countsToRotations(1024, 0.0) == 0.0);
    CHECK(hal::velocityToPeriod(2.0, 1000.0) == doctest::Approx(1.0 / 2000.0));
    CHECK(hal::velocityToPeriod(-2.0, 1000.0) == doctest::Approx(-1.0 / 2000.0));
    CHECK(std::isinf(hal::velocityToPeriod(0.0, 1000.0)));
    CHECK(std::isinf(hal::velocityToPeriod(1.0, 0.0)));
}

TEST_CASE("hal units: duty ratio wraps once per revolution") {
    CHECK(hal::rotationsToDutyRatio(0.25) == doctest::Approx(0.25));
    CHECK(hal::rotationsToDutyRatio(1.25) == doctest::Approx(0.25));
    CHECK(hal::rotationsToDutyRatio(-0.25) == doctest::Approx(0.75));
    CHECK(hal::rotationsToDutyRatio(2.0) == 0.0);
    CHECK(hal::rotationsToDutyRatio(std::nan("")) == 0.0);
}

TEST_CASE("registerHalBridges registers one bridge per family in a fixed order") {
    Rig rig;
    const std::vector<std::string> expected{
        "hal.pwm",           "hal.dio",           "hal.analog_in",        "hal.analog_out",        "hal.encoder",
        "hal.duty_cycle",    "hal.relay",         "hal.ctre_pcm",         "hal.ctre_pcm.solenoid", "hal.rev_ph",
        "hal.rev_ph.solenoid", "hal.power_distribution", "hal.addressable_led", "hal.roborio",     "hal.accelerometer",
        "hal.can"};
    CHECK(rig.registry.keys() == expected);
    REQUIRE(rig.bridges.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) CHECK(rig.bridges[i]->name() == expected[i]);
    const auto* info = rig.registry.info("hal.pwm");
    REQUIRE(info != nullptr);
    CHECK(info->family == "hal");
}

TEST_CASE("registerHalBridges rejects a null api and duplicate registration") {
    sdk::DeviceBridgeRegistry registry;
    CHECK(hal::registerHalBridges(registry, nullptr).error().code == sdk::ErrorCode::InvalidArgument);
    CHECK(registry.empty());

    auto api = std::make_shared<hal::testing::FakeHalApi>();
    REQUIRE(hal::registerHalBridges(registry, api).ok());
    CHECK(hal::registerHalBridges(registry, api).error().code == sdk::ErrorCode::AlreadyExists);
}

TEST_CASE("bridges outlive the registering scope: the registry factories share the api") {
    sdk::DeviceBridgeRegistry registry;
    {
        auto api = std::make_shared<hal::testing::FakeHalApi>();
        api->pwmHal.init[1] = true;
        REQUIRE(hal::registerHalBridges(registry, api).ok());
    }
    auto bridge = registry.create("hal.pwm");
    REQUIRE(bridge.ok());
    RecordingHost host;
    REQUIRE(bridge.value()->start(host).ok());
    bridge.value()->poll();
    CHECK(host.announced.size() == 1);
}

TEST_CASE("an idle HAL announces only the always-present roboRIO") {
    Rig rig;
    rig.poll();
    REQUIRE(rig.host.announced.size() == 1);
    CHECK(rig.host.announced.begin()->second.descriptor.type == "roborio");
}

TEST_CASE("every announced device has a unique key and the hal family on the rio bus") {
    Rig rig;
    auto& a = *rig.api;
    a.pwmHal.init[0] = a.pwmHal.init[19] = true;
    a.dioHal.init[0] = a.dioHal.init[25] = true;
    a.analogInHal.init[7] = true;
    a.analogOutHal.init[1] = true;
    a.encoderHal.init[0] = true;
    a.dutyCycleHal.init[3] = true;
    a.relayHal.fwdInit[2] = a.relayHal.revInit[2] = true;
    a.pcmHal.init[0] = true;
    a.pcmHal.setSolenoid(0, 3, true);
    a.phHal.init[5] = true;
    a.phHal.setSolenoid(5, 15, true);
    a.pdHal.init[0] = true;
    a.ledHal.init[0] = true;
    a.accelHal.isActive = true;
    rig.poll();

    std::map<std::string, int> keys;
    for (const auto& [id, info] : rig.host.announced) {
        CHECK(info.descriptor.family == "hal");
        CHECK(info.descriptor.bus == "rio");
        CHECK(info.descriptor.vendor.empty() == false);
        ++keys[info.descriptor.key().toString()];
        CHECK_FALSE(info.specs.empty());
    }
    for (const auto& [k, n] : keys) CHECK_MESSAGE(n == 1, k);
    CHECK(rig.host.announced.size() == 18);  // incl. roboRIO
}
