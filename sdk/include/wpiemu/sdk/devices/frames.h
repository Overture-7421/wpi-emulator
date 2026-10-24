#pragma once

// Typed, vendor-neutral per-slot data frames exchanged between models (mechanisms, drivetrain)
// and the device hub. The hub translates between these and the raw per-device signals of the
// IPC frames (see devices/signals.h for canonical names/units).

#include <optional>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::sdk {

// What the robot commanded a motor controller to do this tick (robot -> sim).
//
// A controller that regulates current rather than voltage (Phoenix 6 FOC modes, TorqueCurrentFOC) sets
// `hasTorqueCurrent`: the plant then resolves `torqueCurrentA` against the motor it simulates
// (dynamics/motor_model.h voltageForTorqueCurrent) and ignores `voltageV`, which such a controller derives from
// its own motor model and the speed we fed it.
struct MotorOutput {
    bool valid = false;           // false: slot unmapped or no data yet; use neutral (0 V)
    double dutyCycle = 0.0;       // [-1, 1]
    double voltageV = 0.0;        // voltage across the motor; the hub derives duty * battery V if the device only reports duty
    double torqueCurrentA = 0.0;  // the controller's torque-current command, 0 when it regulates voltage
    bool hasTorqueCurrent = false;
};

// What the sim writes back for a motor's integrated sensor (sim -> robot). Rotor side, i.e.
// BEFORE the device's own SensorToMechanismRatio (docs/reference/bridges.md: we drive the rotor).
struct MotorFeedback {
    double rotorPositionRot = 0.0;
    double rotorVelocityRps = 0.0;
    double rotorAccelerationRps2 = 0.0;
    std::optional<bool> forwardLimit;
    std::optional<bool> reverseLimit;
    // The motor the plant simulates on this slot, for a firmware sim that can be told what is attached
    // (devices/signals.h kMotorFamily). Unknown: not sent.
    MotorFamily motorFamily = MotorFamily::Unknown;
};

struct EncoderFeedback {
    double positionRot = 0.0;
    double velocityRps = 0.0;
};

struct ImuFeedback {
    double yawRad = 0.0;   // CCW+ about +Z, absolute (the device's own zeroing is applied by the robot-side firmware sim)
    double pitchRad = 0.0;
    double rollRad = 0.0;
    Vec3 angularVelocityRadPerSec;
    Vec3 accelerationMps2;
};

}  // namespace wpiemu::sdk
