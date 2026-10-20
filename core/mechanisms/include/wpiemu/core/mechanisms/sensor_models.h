#pragma once

// Ideal (noise-free) sensor models used by the mechanism templates. Pure value types: they only map
// the plant state to the typed frames of devices/frames.h. All angles are ROTATIONS here (the unit
// of device signals); callers convert from the plant's radians / metres.

#include "wpiemu/sdk/devices/frames.h"

namespace wpiemu::core::mechanisms {

// Default trigger band of a limit switch, in position units (m or rad).
inline constexpr double kDefaultLimitMargin = 1e-6;

// Position/velocity sensor on a shaft that turns `sensorRatio` times per mechanism-output rotation
// (a CANcoder on the output = 1, an integrated encoder behind a gearbox = gear ratio).
struct EncoderModel {
    double sensorRatio = 1.0;
    bool inverted = false;
    double offsetRot = 0.0;   // added to the position (absolute-encoder magnet offset)

    sdk::EncoderFeedback sample(double outputRot, double outputRps) const {
        const double s = inverted ? -sensorRatio : sensorRatio;
        return {s * outputRot + offsetRot, s * outputRps};
    }
};

// Integrated motor sensor: the ROTOR side of the gearbox (docs/reference/bridges.md: the emulator drives the rotor,
// the device firmware applies its SensorToMechanismRatio). `inverted` mirrors the motor direction.
struct RotorSensorModel {
    double gearRatio = 1.0;   // motor : output
    bool inverted = false;

    sdk::MotorFeedback sample(double outputRot, double outputRps, double outputRps2) const {
        const double s = inverted ? -gearRatio : gearRatio;
        sdk::MotorFeedback f;
        f.rotorPositionRot = s * outputRot;
        f.rotorVelocityRps = s * outputRps;
        f.rotorAccelerationRps2 = s * outputRps2;
        return f;
    }
};

// A limit switch that closes when the mechanism reaches `position` from the given side. The raw
// reading uses `pressedValue` (true: reads true when pressed; false: normally-closed wiring).
struct LimitSwitchModel {
    enum class Side { AtOrAbove, AtOrBelow };

    double position = 0.0;
    Side side = Side::AtOrAbove;
    double margin = kDefaultLimitMargin;   // trigger band in position units (m or rad)
    bool pressedValue = true;

    bool pressed(double p) const {
        return side == Side::AtOrAbove ? p >= position - margin : p <= position + margin;
    }
    bool reading(double p) const { return pressed(p) == pressedValue; }
};

}  // namespace wpiemu::core::mechanisms
