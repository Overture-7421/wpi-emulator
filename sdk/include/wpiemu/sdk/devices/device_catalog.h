#pragma once

// Canonical catalog of built-in device families, types, display names and default capability sets.
// Shared across bridges, editor device directory, demo hosts and test fixtures so adding a device type
// updates one table in the SDK.

#include <array>
#include <span>
#include <string>
#include <string_view>

#include "wpiemu/sdk/devices/device_descriptor.h"

namespace wpiemu::sdk {

namespace device_family {
inline constexpr const char* kHal = "hal";
inline constexpr const char* kPhoenix6 = "phoenix6";
inline constexpr const char* kPhoenix5 = "phoenix5";
inline constexpr const char* kRev = "rev";
}  // namespace device_family

namespace device_type {
inline constexpr const char* kTalonFx = "talonfx";
inline constexpr const char* kTalonFxs = "talonfxs";
inline constexpr const char* kTalonSrx = "talonsrx";
inline constexpr const char* kVictorSpx = "victorspx";
inline constexpr const char* kCancoder = "cancoder";
inline constexpr const char* kPigeon2 = "pigeon2";
inline constexpr const char* kCandi = "candi";
inline constexpr const char* kCandle = "candle";
inline constexpr const char* kSparkMax = "sparkmax";
inline constexpr const char* kSparkFlex = "sparkflex";
inline constexpr const char* kRoboRio = "roborio";
inline constexpr const char* kPwm = "pwm";
inline constexpr const char* kDio = "dio";
inline constexpr const char* kAnalog = "analog";
inline constexpr const char* kAnalogIn = "analog_in";
inline constexpr const char* kEncoder = "encoder";
inline constexpr const char* kDutyCycleEncoder = "duty_cycle_encoder";
inline constexpr const char* kRelay = "relay";
inline constexpr const char* kSolenoid = "solenoid";
inline constexpr const char* kPneumatics = "pneumatics";
inline constexpr const char* kPowerDistribution = "power_distribution";
inline constexpr const char* kAddressableLed = "addressable_led";
inline constexpr const char* kCan = "can";
}  // namespace device_type

struct DeviceTypeInfo {
    std::string_view family;
    std::string_view type;
    std::string_view displayName;
    CapabilitySet capabilities;
};

namespace device_catalog_detail {

inline constexpr std::array<DeviceTypeInfo, 21> kBuiltinDeviceTypes{{
    {device_family::kPhoenix6, device_type::kTalonFx, "TalonFX",
     Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor},
    {device_family::kPhoenix6, device_type::kTalonFxs, "Talon FXS",
     Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor},
    {device_family::kPhoenix5, device_type::kTalonSrx, "Talon SRX",
     Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor},
    {device_family::kPhoenix5, device_type::kVictorSpx, "Victor SPX", CapabilitySet(Capability::MotorOutput)},
    {device_family::kPhoenix6, device_type::kCancoder, "CANcoder",
     Capability::PositionSensor | Capability::VelocitySensor},
    {device_family::kPhoenix6, device_type::kPigeon2, "Pigeon 2", CapabilitySet(Capability::Imu)},
    {device_family::kPhoenix6, device_type::kCandi, "CANdi",
     Capability::DigitalInput | Capability::PositionSensor | Capability::VelocitySensor},
    {device_family::kPhoenix6, device_type::kCandle, "CANdle", CapabilitySet(Capability::Led)},
    {device_family::kRev, device_type::kSparkMax, "SPARK MAX",
     Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor},
    {device_family::kRev, device_type::kSparkFlex, "SPARK Flex",
     Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor},
    {device_family::kHal, device_type::kRoboRio, "roboRIO", {}},
    {device_family::kHal, device_type::kPwm, "PWM", CapabilitySet(Capability::MotorOutput)},
    {device_family::kHal, device_type::kDio, "DIO", Capability::DigitalInput | Capability::DigitalOutput},
    {device_family::kHal, device_type::kAnalog, "Analog in",
     Capability::AnalogInput | Capability::Distance | Capability::PositionSensor},
    {device_family::kHal, device_type::kAnalogIn, "Analog in",
     Capability::AnalogInput | Capability::Distance | Capability::PositionSensor},
    {device_family::kHal, device_type::kEncoder, "Encoder",
     Capability::PositionSensor | Capability::VelocitySensor},
    {device_family::kHal, device_type::kDutyCycleEncoder, "Duty-Cycle Encoder",
     Capability::PositionSensor | Capability::VelocitySensor},
    {device_family::kHal, device_type::kRelay, "Relay", CapabilitySet(Capability::DigitalOutput)},
    {device_family::kHal, device_type::kSolenoid, "Solenoid", CapabilitySet(Capability::Solenoid)},
    {device_family::kHal, device_type::kPneumatics, "Pneumatics", CapabilitySet(Capability::Solenoid)},
    {device_family::kHal, device_type::kAddressableLed, "Addressable LED", CapabilitySet(Capability::Led)},
}};

}  // namespace device_catalog_detail

inline std::span<const DeviceTypeInfo> builtinDeviceTypes() { return device_catalog_detail::kBuiltinDeviceTypes; }

inline const DeviceTypeInfo* findDeviceType(std::string_view type) {
    for (const DeviceTypeInfo& info : device_catalog_detail::kBuiltinDeviceTypes)
        if (info.type == type) return &info;
    return nullptr;
}

// Human-readable label for a device `type` key ("talonfx" -> "TalonFX"); returns `type` when unknown.
inline std::string_view deviceTypeDisplayName(std::string_view type) {
    const DeviceTypeInfo* info = findDeviceType(type);
    return info != nullptr ? info->displayName : type;
}

// Default capability set of a known device `type` key; empty when unknown.
inline CapabilitySet deviceTypeCapabilities(std::string_view type) {
    const DeviceTypeInfo* info = findDeviceType(type);
    return info != nullptr ? info->capabilities : CapabilitySet{};
}

}  // namespace wpiemu::sdk
