#pragma once

// The wheel row of a snapshot, shared by the host robot and the remote robots so both render the same way.

#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::core::runtime {

inline sdk::WheelSample makeWheelSample(const sdk::WheelSpec& spec, const sdk::WheelState& state) {
    sdk::WheelSample w;
    w.name = spec.name;
    w.contactPoint = state.contactPoint;
    w.steerAngleRad = state.steerAngleRad;
    w.spinAngleRad = state.driveWheelAngleRad;
    w.normalLoadN = state.normalLoadN;
    w.slipRatio = state.slipRatio;
    w.slipAngleTan = state.slipAngleTan;
    w.airborne = state.airborne;
    return w;
}

}  // namespace wpiemu::core::runtime
