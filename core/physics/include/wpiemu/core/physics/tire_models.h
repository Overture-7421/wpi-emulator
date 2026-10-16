#pragma once

// Slip-based tire models (ADR 0003 "Tire model"). Both are stateless and implement the semi-implicit
// wheel + chassis-share solve; they differ only in the shape of the slip->force curve.
//
//   "brush"         g(s) = 1 - (1-s)^3 up to the peak, then decays exponentially to the sliding level.
//   "pacejka_lite"  Magic-formula shape sin(C atan(B s)), peak 1 at s = 1, asymptote = slideRatio.
//
// s is the combined normalised slip (1 = peak force); the friction-circle coupling is the same for both.

#include <string_view>

#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::core::physics {

// Normalised friction curves, exposed for tests and tooling. s >= 0, result in [0, 1].
float brushCurve(float s, float slideRatio);
float pacejkaLiteCurve(float s, float slideRatio);

class BrushTireModel final : public sdk::ITireModel {
public:
    static constexpr std::string_view kId = "brush";
    std::string_view id() const override { return kId; }
    sdk::TireOutput solve(const sdk::TireParams& params, const sdk::TireInput& in,
                          const sdk::WheelDrive& drive) const override;
};

class PacejkaLiteTireModel final : public sdk::ITireModel {
public:
    static constexpr std::string_view kId = "pacejka_lite";
    std::string_view id() const override { return kId; }
    sdk::TireOutput solve(const sdk::TireParams& params, const sdk::TireInput& in,
                          const sdk::WheelDrive& drive) const override;
};

// Registers "brush" and "pacejka_lite". Fails with AlreadyExists if either key is taken.
sdk::Status registerTireModels(sdk::TireModelRegistry& registry);

}  // namespace wpiemu::core::physics
