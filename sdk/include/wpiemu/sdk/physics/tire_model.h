#pragma once

#include <string>
#include <string_view>
#include <utility>
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/registry.h"

namespace wpiemu::sdk {

// Per-wheel traction parameters (docs/reference/physics.md, FRC defaults).
struct TireParams {
    float mu = 1.1f;               // peak friction coefficient (0.8 - 1.4)
    float slideRatio = 0.85f;      // sliding friction / peak
    float peakSlipLong = 0.15f;    // slip ratio at peak force
    float peakSlipLat = 0.20f;     // tan(slip angle) at peak force
    float vRefMps = 0.5f;          // creep velocity at standstill (static friction regularisation)
    float lateralScale = 1.0f;     // 1 = full friction circle; ~0.12-0.15 omni / mecanum rollers
    float tractionAngleRad = 0.f;  // 0 normal wheels, +-pi/4 mecanum
};

// Inputs for ONE wheel for ONE step. Contact-frame velocities are of the ground contact point
// relative to the ground (positive long = chassis moving along the wheel's traction axis).
struct TireInput {
    float normalLoadN = 0.f;        // <= 0: airborne, no force
    float vLongMps = 0.f;           // contact velocity along the traction axis
    float vLatMps = 0.f;            // contact velocity along the free axis
    float chassisShareMassKg = 1.f; // M / nWheels, used for the implicit lateral damping
    float wheelRadiusM = 0.0508f;
    float dt = 0.005f;
    bool prescribedMotion = false; // evaluate contact without integrating wheel spin
};

// The wheel-side drivetrain state handed to the tire solver (reflected to the wheel shaft).
struct WheelDrive {
    float omegaRadPerSec = 0.f;   // wheel spin rate at the start of the step
    float torqueNm = 0.f;         // motor torque at the wheel (after gearing, efficiency)
    float dampingNmS = 0.f;       // motor back-EMF damping at the wheel; 0 when current-limited
    float inertiaKgM2 = 0.f;      // reflected rotor + wheel inertia
};

struct TireOutput {
    float fLongN = 0.f;           // force along the traction axis, applied at the contact point
    float fLatN = 0.f;            // force along the free axis
    float omegaNext = 0.f;        // end-of-step wheel spin rate
    float slipRatio = 0.f;
    float slipAngleTan = 0.f;
    bool saturated = false;       // beyond peak (sliding)
};

// Strategy: how a wheel turns slip into force. Stateless and const so one instance serves all
// wheels and is trivially thread-safe. Implementations: brush (default), Pacejka-lite, ...
class ITireModel {
    WPIEMU_INTERFACE(ITireModel)
public:
    virtual std::string_view id() const = 0;
    virtual TireOutput solve(const TireParams& params, const TireInput& in, const WheelDrive& drive) const = 0;
};

// Open/closed registry of tire models ("brush", "pacejka_lite", ...), owned by the core runtime.
class TireModelRegistry : public Registry<NoInfo, ITireModel> {
public:
    Status registerModel(std::string id, Factory factory) { return add(std::move(id), NoInfo{}, std::move(factory)); }
};

}  // namespace wpiemu::sdk
