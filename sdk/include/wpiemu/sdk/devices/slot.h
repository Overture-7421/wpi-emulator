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
    LimitSwitch,  // model writes a boolean
    BeamBreak,    // model writes a boolean
    Imu,          // model writes yaw/pitch/roll/rates
    AnalogSensor,
    Solenoid,        // consumes a boolean
    DigitalOutput,   // consumes a boolean
    DistanceSensor,  // model writes distance (m)
};

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
        case SlotType::DistanceSensor:
            return Capability::Distance;
        case SlotType::Solenoid: return Capability::Solenoid;
        case SlotType::DigitalOutput: return Capability::DigitalOutput;
    }
    return {};
}

// Most motors one gearbox (a mechanism axis or a drivetrain wheel) can have.
inline constexpr int kMaxMotorsPerAxis = 8;

struct SlotSpec {
    std::string name;         // unique within the owner, e.g. "motor", "encoder", "forwardLimit"
    std::string label;        // UI text
    SlotType type = SlotType::Motor;
    bool required = true;     // unmapped required slot => validation error; optional => loopback/neutral
    std::string description;
    // Extra motors: the slot only exists while the owner's integer property `countKey` is >= `ordinal`
    // (1-based motor number). Empty `countKey` = always present.
    std::string countKey;
    int ordinal = 0;
};

// Where a slot lives. `owner` is a mechanism instance id, or one of the reserved owners below.
struct SlotAddress {
    std::string owner;
    std::string slot;

    auto operator<=>(const SlotAddress&) const = default;
    std::string toString() const { return owner + "." + slot; }
};

namespace slot_owner {
inline constexpr const char* kDrivetrain =
    "drivetrain";  // "<wheel>.drive[N]", ".steer", ".steerEncoder"
inline constexpr const char* kRobot = "robot";            // robot-level slots, e.g. "imu"
}  // namespace slot_owner

// Drive slot `index` (0-based) of a drivetrain wheel: "<wheel>.drive", "<wheel>.drive2", ...
inline std::string driveSlotName(const std::string& wheel, int index) {
    return wheel + ".drive" + (index > 0 ? std::to_string(index + 1) : std::string());
}

struct SlotMapping {
    SlotAddress slot;
    DeviceKey device;
    // Motor slots only: the controller spins against the mechanism's positive direction (it is mounted
    // mirrored, or robot code inverted it). The hub negates its command and the rotor feedback.
    bool reversed = false;

    bool operator==(const SlotMapping&) const = default;
};

// True when `device` has everything `spec` needs.
inline bool isCompatible(const SlotSpec& spec, const DeviceDescriptor& device) {
    return device.capabilities.containsAll(requiredCapabilities(spec.type));
}

// True when at least one slot type could take `device`. Infrastructure such as the roboRIO or the power
// distribution hub is detected but has nothing to map to, so it is never reported as unmapped.
inline bool isMappable(const DeviceDescriptor& device) {
    for (const SlotType t :
         {SlotType::Motor, SlotType::PositionSensor, SlotType::VelocitySensor,
          SlotType::LimitSwitch, SlotType::BeamBreak, SlotType::Imu, SlotType::AnalogSensor,
          SlotType::Solenoid, SlotType::DigitalOutput, SlotType::DistanceSensor})
        if (device.capabilities.containsAll(requiredCapabilities(t))) return true;
    return false;
}

}  // namespace wpiemu::sdk
