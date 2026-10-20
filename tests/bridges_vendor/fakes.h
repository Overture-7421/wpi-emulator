#pragma once

// Test doubles for the vendor bridge: the SDK's in-memory HAL SimDevice tree and recording bridge host, plus
// fixtures that build the Phoenix 6 26.3.0 wrapper classes' SimDevice trees (names, directions and value names).

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"
#include "wpiemu/sdk/testing/fake_bridge_host.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_sim_device_backend.h"

namespace wpiemu::vendor_tests {

using namespace wpiemu::sdk;

using FakeValueSpec = sdk::testing::FakeValueSpec;
using FakeSimBackend = sdk::testing::FakeSimDeviceBackend;

// A retired device leaves `announced`, so idOf() no longer finds it.
struct FakeHost : sdk::testing::FakeBridgeHost {
    FakeHost() { eraseOnRetire = true; }
};

// ---- Fixtures: SimDevice trees exactly as the Phoenix 6 26.3.0 wrapper classes create them ---------------------

inline constexpr auto kIn = SignalDirection::Input;
inline constexpr auto kOut = SignalDirection::Output;
inline constexpr auto kBidir = SignalDirection::Bidirectional;

struct TalonFxTree {
    SimDeviceHandle motor, rotor, fwd, rev;
};

inline TalonFxTree addTalonFx(FakeSimBackend& b, int id, const char* model = "Talon FX") {
    const std::string base = std::string(model) + " (v6)[" + std::to_string(id) + "]";
    TalonFxTree t;
    t.motor = b.addDevice("CANMotor:" + base,
                          {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                           {"dutyCycle", kOut},
                           {"motorVoltage", kOut},
                           {"torqueCurrent", kOut},
                           {"supplyCurrent", kOut}});
    t.rotor = b.addDevice("CANEncoder:" + base + "/Rotor Sensor",
                          {{"position", kOut}, {"rawPositionInput", kIn}, {"velocity", kIn}, {"acceleration", kIn}});
    t.fwd = b.addDevice("CANDIO:" + base + "/Fwd Limit",
                        {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kBidir, SignalType::Bool}});
    t.rev = b.addDevice("CANDIO:" + base + "/Rev Limit",
                        {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kBidir, SignalType::Bool}});
    return t;
}

inline SimDeviceHandle addCanCoder(FakeSimBackend& b, int id) {
    return b.addDevice("CANEncoder:CANcoder (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"position", kOut},
                        {"rawPositionInput", kIn},
                        {"velocity", kIn}});
}

inline SimDeviceHandle addPigeon2(FakeSimBackend& b, int id) {
    return b.addDevice("CANGyro:Pigeon 2 (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"yaw", kOut},
                        {"rawYawInput", kIn},
                        {"pitch", kIn},
                        {"roll", kIn},
                        {"angularVelX", kIn},
                        {"angularVelY", kIn},
                        {"angularVelZ", kIn}});
}

inline SimDeviceHandle addCanDle(FakeSimBackend& b, int id) {
    return b.addDevice("CAN:CANdle (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"fiveVRail", kIn, SignalType::Double, 5.0},
                        {"outputCurrent", kIn},
                        {"temperature", kIn, SignalType::Double, 25.0},
                        {"vbatModulation", kOut}});
}

inline SimDeviceHandle addCanRange(FakeSimBackend& b, int id) {
    return b.addDevice("CAN:CANrange (v6)[" + std::to_string(id) + "]",
                       {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                        {"distance", kIn},
                        {"isDetected", kOut, SignalType::Bool}});
}

}  // namespace wpiemu::vendor_tests
