#pragma once

// Slots: the typed "sockets" a model (mechanism, drivetrain module, robot IMU) exposes. The user
// maps a discovered device onto each required slot (editor / robot.json). The editor renders the
// slot list generically from SlotSpec; models read/write slots through IDeviceHub.

#include <string>
#include <vector>

#include "wpiemu/sdk/devices/device_descriptor.h"

namespace wpiemu::sdk {

enum class SlotType {
    Motor,           // consumes MotorOutput; may also receive rotor feedback (integrated encoder)
    PositionSensor,  // model writes position (rot) [+velocity]
    VelocitySensor,
    LimitSwitch,     // model writes a boolean
    BeamBreak,       // model writes a boolean
    Imu,             // model writes yaw/pitch/roll/rates
    AnalogSensor,
    Solenoid,        // consumes a boolean
    DigitalOutput,   // consumes a boolean
};

enum class SlotDirection {
    ActuatorInput,   // data flows robot -> model (we read device Output signals)
    SensorOutput,    // data flows model -> robot (we write device Input signals)
};

inline SlotDirection defaultDirection(SlotType t) {
    switch (t) {
        case SlotType::Motor:
        case SlotType::Solenoid:
        case SlotType::DigitalOutput:
            return SlotDirection::ActuatorInput;
        default:
            return SlotDirection::SensorOutput;
    }
}

// Capabilities a device must have to be mappable onto a slot of this type.
inline CapabilitySet requiredCapabilities(SlotType t) {
    switch (t) {
        case SlotType::Motor: return Capability::MotorOutput;
        case SlotType::PositionSensor: return Capability::PositionSensor;
        case SlotType::VelocitySensor: return Capability::VelocitySensor;
        case SlotType::LimitSwitch:
        case SlotType::BeamBreak: return Capability::DigitalInput;
        case SlotType::Imu: return Capability::Imu;
        case SlotType::AnalogSensor: return Capability::AnalogInput;
        case SlotType::Solenoid: return Capability::Solenoid;
        case SlotType::DigitalOutput: return Capability::DigitalOutput;
    }
    return {};
}

struct SlotSpec {
    std::string name;         // unique within the owner, e.g. "motor", "encoder", "forwardLimit"
    std::string label;        // UI text
    SlotType type = SlotType::Motor;
    bool required = true;     // unmapped required slot => validation error; optional => loopback/neutral
    std::string description;
};

// Where a slot lives. `owner` is a mechanism instance id, or one of the reserved owners below.
struct SlotAddress {
    std::string owner;
    std::string slot;

    auto operator<=>(const SlotAddress&) const = default;
    std::string toString() const { return owner + "." + slot; }
};

namespace slot_owner {
inline constexpr const char* kDrivetrain = "drivetrain";  // slots "<wheel>.drive", "<wheel>.steer"
inline constexpr const char* kRobot = "robot";            // robot-level slots, e.g. "imu"
}  // namespace slot_owner

struct SlotMapping {
    SlotAddress slot;
    DeviceKey device;

    bool operator==(const SlotMapping&) const = default;
};

// True when `device` has everything `spec` needs.
inline bool isCompatible(const SlotSpec& spec, const DeviceDescriptor& device) {
    return device.capabilities.containsAll(requiredCapabilities(spec.type));
}

}  // namespace wpiemu::sdk
