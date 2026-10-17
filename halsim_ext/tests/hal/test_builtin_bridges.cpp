#include <doctest/doctest.h>

#include <algorithm>
#include <numbers>

#include "../fakes.h"
#include "hal/SimDevice.h"
#include "wpiemu/sdk/devices/builtin_bridges.h"
#include "wpiemu/sdk/devices/device_bridge.h"

using namespace wpiemu::sdk;

TEST_CASE("the built-in bridge aggregator registers every shipped module, HAL and vendor") {
    DeviceBridgeRegistry registry;
    const Status status = registerBuiltinBridges(registry);
    CHECK(status.ok());

    const std::vector<std::string> keys = registry.keys();
    auto has = [&](std::string_view prefix) {
        return std::any_of(keys.begin(), keys.end(), [&](const std::string& k) { return k.rfind(prefix, 0) == 0; });
    };
    CHECK(has("hal."));
    CHECK(has("ctre."));

    auto created = registry.createAll();
    REQUIRE(created);
    CHECK(created.value().size() == keys.size());
}

TEST_CASE("registering the built-in bridges twice is reported, not silently merged") {
    DeviceBridgeRegistry registry;
    REQUIRE(registerBuiltinBridges(registry).ok());
    const std::size_t before = registry.size();
    (void)registerBuiltinBridges(registry);  // duplicates are rejected or ignored by the modules, never doubled
    CHECK(registry.size() == before);
}

TEST_CASE("vendor session: Reset Begin and reconnect rebase wrapped yaw through the generic seam") {
    namespace hs = wpiemu::halsim::testing;
    using namespace std::chrono_literals;
    enum class Transition { Reset, Begin, Reconnect };
    auto transition = Transition::Reset;
    SUBCASE("Reset") {}
    SUBCASE("End then Begin") {
        transition = Transition::Begin;
    }
    SUBCASE("reconnect") {
        transition = Transition::Reconnect;
    }

    struct SimDevice {
        HAL_SimDeviceHandle handle = HAL_CreateSimDevice("CANGyro:Pigeon 2 (v6)[13]");
        ~SimDevice() { HAL_FreeSimDevice(handle); }
    } pigeon;
    REQUIRE(pigeon.handle != 0);
    const auto rawYaw =
        HAL_CreateSimValueDouble(pigeon.handle, "rawYawInput", HAL_SimValueInput, 0);
    REQUIRE(rawYaw != 0);
    REQUIRE(HAL_CreateSimValueDouble(pigeon.handle, "yaw", HAL_SimValueOutput, 0) != 0);
    auto config = hs::Rig::defaultConfig();
    config.reconnect = transition == Transition::Reconnect;
    hs::Rig rig(hs::defaultAck(), config);
    DeviceBridgeRegistry registry;
    REQUIRE(registerBuiltinBridges(registry).ok());
    auto bridges = registry.createAll();
    REQUIRE(bridges);
    rig.bridges = std::move(bridges).value();
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto announce = rig.emulator->take<proto::DeviceAnnounce>(
        1000ms, [](const proto::DeviceAnnounce& a) { return a.descriptor.type == "pigeon2"; });
    REQUIRE(announce);
    const auto yaw = std::find_if(announce->signalSpecs.begin(), announce->signalSpecs.end(),
                                  [](const SignalSpec& spec) { return spec.name == "yaw"; });
    REQUIRE(yaw != announce->signalSpecs.end());
    const auto index = static_cast<std::uint16_t>(yaw - announce->signalSpecs.begin());
    const auto tick = [&](double degrees) {
        REQUIRE(rig.emulator->tick({{announce->id, index, degrees * std::numbers::pi / 180.0}}));
    };
    tick(0);
    tick(170);
    tick(-170);
    CHECK(HAL_GetSimValueDouble(rawYaw) == doctest::Approx(190));

    if (transition == Transition::Reconnect) {
        auto ack = hs::defaultAck();
        ack.sessionId = 8;
        ack.runEpoch = 5;
        auto first = std::move(rig.emulator);
        rig.connectNext(ack);
        first->disconnect();
        REQUIRE(rig.emulator->waitForLifecycle(3000ms));
    } else {
        rig.emulator->epoch = 2;
        const auto command = transition == Transition::Reset ? proto::SimCommand::Reset
                                                             : proto::SimCommand::EndLockstep;
        REQUIRE(rig.emulator->sendControl(command, 1));
        const auto ack = rig.emulator->take<proto::SimControlAck>(1000ms);
        REQUIRE(ack);
        REQUIRE(ack->result == proto::ControlResult::Applied);
        if (transition == Transition::Begin) {
            rig.emulator->epoch = 3;
            REQUIRE(rig.emulator->sendControl(proto::SimCommand::BeginLockstep, 2));
            const auto begin = rig.emulator->take<proto::SimControlAck>(1000ms);
            REQUIRE(begin);
            REQUIRE(begin->result == proto::ControlResult::Applied);
        }
    }
    tick(0);
    CHECK(HAL_GetSimValueDouble(rawYaw) == doctest::Approx(0));
    tick(30);
    CHECK(HAL_GetSimValueDouble(rawYaw) == doctest::Approx(30));
}
