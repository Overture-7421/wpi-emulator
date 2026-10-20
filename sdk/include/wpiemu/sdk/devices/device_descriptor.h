#pragma once

#include <compare>
#include <cstdint>
#include <string>

namespace wpiemu::sdk {

// What a physical/simulated device can do. Used to validate slot<->device mappings in the
// editor and to decide how the hub interprets a device's signals.
enum class Capability : std::uint32_t {
    None = 0,
    MotorOutput = 1u << 0,      // robot commands it (duty/voltage/torque current)
    RotorFeedback = 1u << 1,    // sim can write rotor position/velocity (integrated sensor)
    PositionSensor = 1u << 2,   // encoder/absolute position
    VelocitySensor = 1u << 3,
    Imu = 1u << 4,
    DigitalInput = 1u << 5,     // sim -> robot boolean (limit switch, beam break)
    DigitalOutput = 1u << 6,    // robot -> sim boolean
    AnalogInput = 1u << 7,
    PwmOutput = 1u << 8,
    Solenoid = 1u << 9,
    LimitInputs = 1u << 10,     // device has integrated fwd/rev limit inputs
    SupplyVoltage = 1u << 11,   // sim can write supply/bus voltage
    Led = 1u << 12,
    Distance = 1u << 13,
};

struct CapabilitySet {
    std::uint32_t bits = 0;

    constexpr CapabilitySet() = default;
    constexpr explicit CapabilitySet(std::uint32_t b) : bits(b) {}
    constexpr CapabilitySet(Capability c) : bits(static_cast<std::uint32_t>(c)) {}  // NOLINT

    constexpr bool has(Capability c) const { return (bits & static_cast<std::uint32_t>(c)) != 0; }
    constexpr bool containsAll(CapabilitySet o) const { return (bits & o.bits) == o.bits; }
    constexpr CapabilitySet with(Capability c) const { return CapabilitySet{bits | static_cast<std::uint32_t>(c)}; }
    constexpr CapabilitySet operator|(CapabilitySet o) const { return CapabilitySet{bits | o.bits}; }
    constexpr bool operator==(const CapabilitySet&) const = default;
};

constexpr CapabilitySet operator|(Capability a, Capability b) { return CapabilitySet(a) | CapabilitySet(b); }

// Bus label meaning "not known": the bus is NOT derivable from what the bridge sees (e.g. Phoenix 6
// SimDevice names carry no bus). Only an explicit observation (L3 hook) may fill it in later.
inline constexpr const char* kBusUnknown = "";

// What the SIMULATOR sees as one device, independent of how the robot program addressed it. It
// deliberately has no bus: CTRE sim state is keyed by (type, id) only, so a TalonFX 4 constructed
// on "rio" and one on "canivore" are the SAME physical simulator device (docs/ARCHITECTURE.md).
struct PhysicalDeviceId {
    std::string family;
    std::string type;
    int id = -1;
    int channel = -1;

    auto operator<=>(const PhysicalDeviceId&) const = default;

    std::string toString() const {
        std::string s = family + "/" + type + "[" + std::to_string(id) + "]";
        if (channel >= 0) s += "#" + std::to_string(channel);
        return s;
    }
};

// The robot-DECLARED address of a device (what the robot program / robot.json says); this is what
// robot.json stores. Several declared addresses may resolve to one PhysicalDeviceId (aliases).
// Stable identity of a device across runs; this (not the runtime DeviceId) is what robot.json stores.
struct DeviceKey {
    std::string family;  // "hal" | "phoenix6" | "phoenix5" | ... (bridge-defined, lower-case)
    std::string type;    // "pwm", "dio", "talonfx", "cancoder", "pigeon2", "candle", "talonsrx", ...
    int id = -1;         // CAN id / port index / module index; -1 if not applicable
    std::string bus = "rio";  // "rio", or a CANivore label; label only (docs/reference/bridges.md). Declared, not verified.
    int channel = -1;    // sub-channel (PCM solenoid channel, relay fwd/rev, ...); -1 if n/a

    auto operator<=>(const DeviceKey&) const = default;

    // Simulator-side identity: this address with the bus dropped.
    PhysicalDeviceId physical() const { return PhysicalDeviceId{family, type, id, channel}; }

    std::string toString() const {
        std::string s = family + "/" + type + "[" + std::to_string(id) + "]@" + bus;
        if (channel >= 0) s += "#" + std::to_string(channel);
        return s;
    }
};

struct DeviceDescriptor {
    std::string family;
    std::string type;
    std::string vendor;       // "wpilib", "ctre", ... (display / grouping)
    int id = -1;
    // Bus as far as the bridge knows it: "rio" for rio-only families, kBusUnknown when the bridge
    // cannot tell (SimDeviceRule::busIsStatic == false). Never invented per device.
    std::string bus = "rio";
    int channel = -1;
    CapabilitySet capabilities;
    std::string nativeName;   // e.g. the HAL SimDevice name this was derived from (diagnostics)
    std::string displayName;  // optional friendly label; UI falls back to key().toString()

    DeviceKey key() const { return DeviceKey{family, type, id, bus, channel}; }
    PhysicalDeviceId physicalId() const { return PhysicalDeviceId{family, type, id, channel}; }
    bool operator==(const DeviceDescriptor&) const = default;
};

}  // namespace wpiemu::sdk
