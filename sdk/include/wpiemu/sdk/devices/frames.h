#pragma once

// Typed, vendor-neutral per-slot data frames exchanged between models (mechanisms, drivetrain)
// and the device hub. The hub translates between these and the raw per-device signals of the
// IPC frames (see devices/signals.h for canonical names/units).

#include <optional>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

// What the robot commanded a motor controller to do this tick (robot -> sim).
struct MotorOutput {
    bool valid = false;           // false: slot unmapped or no data yet; use neutral (0 V)
    double dutyCycle = 0.0;       // [-1, 1]
    double voltageV = 0.0;        // voltage across the motor; the hub derives duty * battery V if the device only reports duty
    double torqueCurrentA = 0.0;  // TorqueCurrentFOC request, 0 otherwise
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
