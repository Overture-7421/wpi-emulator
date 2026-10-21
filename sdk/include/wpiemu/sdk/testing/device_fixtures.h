#pragma once

// Device announcements and descriptors that several suites build. Where suites need different content the
// difference is a parameter (capabilities, bus, unit): no variant is shared by silently picking one.

#include <algorithm>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "wpiemu/sdk/devices/device_descriptor.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/devices/signal.h"
#include "wpiemu/sdk/devices/signals.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk::testing {

inline SignalSpec doubleSignal(const char* name, SignalDirection direction, const char* unit = "") {
    return SignalSpec{name, direction, SignalType::Double, unit};
}

// What the Phoenix 6 rules announce for a TalonFX.
inline constexpr CapabilitySet kTalonFxCapabilities =
    Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor;

// A Phoenix 6 TalonFX. The bus stays unknown unless given: the SimDevice tree carries none.
inline DeviceDescriptor talonFxDescriptor(int canId, CapabilitySet capabilities = kTalonFxCapabilities,
                                          std::string bus = kBusUnknown) {
    DeviceDescriptor d;
    d.family = "phoenix6";
    d.type = "talonfx";
    d.vendor = "ctre";
    d.id = canId;
    d.bus = std::move(bus);
    d.capabilities = capabilities;
    return d;
}

// Signals 0 duty, 1 voltage, 2 torque (outputs); 3 rotor position, 4 velocity, 5 acceleration, 6 forward limit (inputs).
inline proto::DeviceAnnounce talonFxAnnounce(std::uint32_t runtimeId, int canId, std::string bus = kBusUnknown) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{runtimeId};
    a.descriptor = talonFxDescriptor(canId, kTalonFxCapabilities, std::move(bus));
    a.signalSpecs = {doubleSignal(signal::kDutyCycle, SignalDirection::Output),
                     doubleSignal(signal::kMotorVoltage, SignalDirection::Output),
                     doubleSignal(signal::kTorqueCurrent, SignalDirection::Output),
                     doubleSignal(signal::kRotorPosition, SignalDirection::Input),
                     doubleSignal(signal::kRotorVelocity, SignalDirection::Input),
                     doubleSignal(signal::kRotorAcceleration, SignalDirection::Input),
                     doubleSignal(signal::kForwardLimit, SignalDirection::Bidirectional)};
    return a;
}

// A Pigeon 2 on an unknown bus: the nine IMU signals (yaw, pitch, roll, angular velocity xyz, acceleration xyz), all
// inputs and all labelled `unit`, which suites choose differently.
inline proto::DeviceAnnounce pigeon2Announce(std::uint32_t runtimeId, int canId, const char* unit) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{runtimeId};
    a.descriptor.family = "phoenix6";
    a.descriptor.type = "pigeon2";
    a.descriptor.vendor = "ctre";
    a.descriptor.id = canId;
    a.descriptor.bus = kBusUnknown;
    a.descriptor.capabilities = CapabilitySet(Capability::Imu);
    for (const char* name : {signal::kYaw, signal::kPitch, signal::kRoll, signal::kAngularVelX, signal::kAngularVelY,
                             signal::kAngularVelZ, signal::kAccelX, signal::kAccelY, signal::kAccelZ})
        a.signalSpecs.push_back(doubleSignal(name, SignalDirection::Input, unit));
    return a;
}

inline bool hasWarning(std::span<const HubWarning> warnings, std::string_view code) {
    return std::any_of(warnings.begin(), warnings.end(), [&](const HubWarning& w) { return w.code == code; });
}

inline bool hasWarning(const IDeviceHub& hub, std::string_view code) { return hasWarning(hub.warnings(), code); }

}  // namespace wpiemu::sdk::testing
