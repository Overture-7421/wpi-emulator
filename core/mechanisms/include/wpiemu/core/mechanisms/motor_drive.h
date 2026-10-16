#pragma once

// Turns what the robot commanded a motor controller to do (MotorOutput) into the voltage the motor
// actually sees: battery-sag aware, torque-current requests, neutral mode and stator current limit.

#include <memory>

#include "wpiemu/core/mechanisms/motor_catalog.h"
#include "wpiemu/sdk/devices/frames.h"

namespace wpiemu::core::mechanisms {

enum class NeutralMode { Coast, Brake };

class MotorDrive {
public:
    // `statorCurrentLimitA <= 0` disables the limit.
    MotorDrive(sdk::MotorSpec motor, double statorCurrentLimitA, NeutralMode neutral);

    // Voltage across ONE motor. `motorOmegaRadPerSec` is the motor SHAFT speed in the direction of the
    // command, `batteryVoltageV` the present supply (the command saturates at +-battery).
    //  * !valid / no data: neutral (Brake = 0 V shorted, Coast = the back-EMF, i.e. no current)
    //  * hasTorqueCurrent: V = I*R + omega/Kv (a TorqueCurrentFOC request)
    //  * otherwise voltageV (or dutyCycle * battery if only duty is present)
    double voltage(const sdk::MotorOutput& out, double motorOmegaRadPerSec, double batteryVoltageV) const;

    // Current of ONE motor at that voltage/speed (after limiting).
    double currentA(double voltageV, double motorOmegaRadPerSec) const { return model_.currentA(voltageV, motorOmegaRadPerSec); }

    const sdk::MotorSpec& spec() const { return model_.spec(); }

private:
    CurrentLimitedMotorModel model_;
    NeutralMode neutral_;
};

}  // namespace wpiemu::core::mechanisms
