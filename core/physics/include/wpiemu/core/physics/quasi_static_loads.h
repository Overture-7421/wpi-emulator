#pragma once

// Quasi-static normal-load distribution for any wheel layout (docs/reference/physics.md): min-norm solution of
//   sum N = W,   sum N x = -h m ax,   sum N y = -h m ay
// (x, y relative to the centre of mass in the body frame; ax, ay = acceleration produced by the tire forces).

#include <span>

namespace wpiemu::core::physics {

struct WheelXY {
    float x = 0.f;  // m, chassis frame, relative to the centre of mass
    float y = 0.f;
};

// loads.size() must equal wheels.size(). W = massKg * gNormal where gNormal = g * cos(tilt).
// Collinear / fewer-than-three-wheel layouts fall back to an equal split. Lifted wheels get 0 N and the rest
// are renormalised so the sum stays W.
void quasiStaticLoads(std::span<const WheelXY> wheels, float massKg, float comHeightM, float ax, float ay,
                      float gNormal, std::span<float> loads);

}  // namespace wpiemu::core::physics
