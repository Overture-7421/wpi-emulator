#pragma once

// Canonical signal names. Bridges SHOULD normalise vendor signals to these names and units
// (via SimDeviceRule::scale or in code) so the hub can interpret any vendor's device identically.
// A bridge may add extra vendor-specific signals under its own names.
//
// Units (canonical): voltage V, current A, rotary position in ROTATIONS, rotary velocity in
// ROTATIONS/s (Phoenix convention; the hub converts to rad), IMU angles in RADIANS and rates in
// rad/s, linear distance in metres, duty/PWM speed in [-1, 1].

namespace wpiemu::sdk::signal {

// ---- robot -> sim (Output) --------------------------------------------------------------------
inline constexpr const char* kDutyCycle = "dutyCycle";          // [-1, 1]
inline constexpr const char* kMotorVoltage = "motorVoltage";    // V applied to the motor
inline constexpr const char* kTorqueCurrent = "torqueCurrent";  // A (TorqueCurrentFOC)
inline constexpr const char* kSupplyCurrentOut = "supplyCurrent";
inline constexpr const char* kDigitalOut = "digitalOut";        // bool
inline constexpr const char* kPwmSpeed = "pwmSpeed";            // [-1, 1]
inline constexpr const char* kSolenoid = "solenoid";            // bool

// ---- sim -> robot (Input) -----------------------------------------------------------------------
inline constexpr const char* kRotorPosition = "rotorPosition";  // rot
inline constexpr const char* kRotorVelocity = "rotorVelocity";  // rps
inline constexpr const char* kRotorAcceleration = "rotorAcceleration";  // rps^2
inline constexpr const char* kPosition = "position";            // rot (encoders / CANcoder)
inline constexpr const char* kVelocity = "velocity";            // rps
inline constexpr const char* kSupplyVoltage = "supplyVoltage";  // V
inline constexpr const char* kForwardLimit = "forwardLimit";    // bool (true = limit pressed)
inline constexpr const char* kReverseLimit = "reverseLimit";    // bool
inline constexpr const char* kMotorFamily = "motorFamily";      // sdk::MotorFamily as a number: the motor the plant simulates
inline constexpr const char* kDigitalIn = "digitalIn";          // bool
inline constexpr const char* kAnalogVoltage = "analogVoltage";  // V
inline constexpr const char* kDistance = "distance";            // m
inline constexpr const char* kDetected = "detected";            // bool

inline constexpr const char* kYaw = "yaw";                      // rad
inline constexpr const char* kPitch = "pitch";
inline constexpr const char* kRoll = "roll";
inline constexpr const char* kAngularVelX = "angularVelX";      // rad/s
inline constexpr const char* kAngularVelY = "angularVelY";
inline constexpr const char* kAngularVelZ = "angularVelZ";
inline constexpr const char* kAccelX = "accelX";                // m/s^2
inline constexpr const char* kAccelY = "accelY";
inline constexpr const char* kAccelZ = "accelZ";

}  // namespace wpiemu::sdk::signal
