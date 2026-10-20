#include "wpiemu/core/physics/tire_models.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::physics {
namespace {

using sdk::TireInput;
using sdk::TireOutput;
using sdk::TireParams;
using sdk::WheelDrive;

constexpr float kSlipEpsilonMps = 0.02f;  // finite-difference step for the slip-velocity stiffness
constexpr float kMinInertia = 1e-9f;

float clampSlide(float slideRatio) { return std::clamp(slideRatio, 0.3f, 0.98f); }

// Magic-formula constants such that g(1) = 1 and g(inf) = slideRatio.
struct PacejkaShape {
    float C;
    float B;
    explicit PacejkaShape(float slide) {
        C = 2.f - (2.f / sdk::kPiF) * std::asin(clampSlide(slide));
        B = std::tan(sdk::kPiF / (2.f * C));
    }
    float operator()(float s) const { return std::sin(C * std::atan(B * std::max(s, 0.f))); }
};

struct BrushCurve {
    float slide;
    float operator()(float s) const { return brushCurve(s, slide); }
};

struct FricForce {
    float fLong = 0.f;
    float fLat = 0.f;
    bool sliding = false;
};

template <class Curve>
FricForce frictionForce(const Curve& g, const TireParams& p, float normalLoad, float uLong, float uLat,
                        float vRefLong, float vRefLat) {
    const float a = uLong / (vRefLong * p.peakSlipLong);
    const float b = uLat / (vRefLat * p.peakSlipLat);
    const float k = p.lateralScale;
    const float sC = std::hypot(a, k * b);
    const float sP = std::hypot(b, k * a);
    const float fMax = p.mu * normalLoad;
    FricForce out;
    out.fLong = sC > 1e-9f ? fMax * g(sC) * a / sC : 0.f;
    out.fLat = sP > 1e-9f ? fMax * k * g(sP) * b / sP : 0.f;
    out.sliding = sC > 1.f || sP > 1.f;
    return out;
}

template <class Curve>
TireOutput solveImpl(const Curve& g, const TireParams& p, const TireInput& in, const WheelDrive& d) {
    TireOutput out;
    out.omegaNext = d.omegaRadPerSec;
    if (!(in.dt > 0.f)) return out;

    const float reff = in.wheelRadiusM * std::cos(p.tractionAngleRad);
    const float rim = d.omegaRadPerSec * reff;
    const float Ie = std::max(d.inertiaKgM2 + in.dt * d.dampingNmS, kMinInertia);

    if (!(in.normalLoadN > 0.f)) {  // airborne: the wheel spins freely under motor torque
        if (!in.prescribedMotion) out.omegaNext = d.omegaRadPerSec + in.dt * d.torqueNm / Ie;
        if (!std::isfinite(out.omegaNext)) out.omegaNext = d.omegaRadPerSec;
        return out;
    }

    const float vRefLong = std::max({p.vRefMps, std::fabs(in.vLongMps), std::fabs(rim)});
    const float vRefLat = std::max(p.vRefMps, std::hypot(in.vLongMps, in.vLatMps));
    const float uL = rim - in.vLongMps;
    const float uT = -in.vLatMps;
    const FricForce f0 = frictionForce(g, p, in.normalLoadN, uL, uT, vRefLong, vRefLat);
    const float kL = std::max(
        0.f, (frictionForce(g, p, in.normalLoadN, uL + kSlipEpsilonMps, uT, vRefLong, vRefLat).fLong - f0.fLong) /
                 kSlipEpsilonMps);
    const float kT = std::max(
        0.f, (frictionForce(g, p, in.normalLoadN, uL, uT + kSlipEpsilonMps, vRefLong, vRefLat).fLat - f0.fLat) /
                 kSlipEpsilonMps);
    const float invM = 1.f / std::max(in.chassisShareMassKg, 1e-3f);

    // Longitudinal: wheel and chassis share one slip velocity; solve u' = u + du with F' = F0 + kL du.
    const float c = (in.prescribedMotion ? 0.f : reff * reff / Ie) + invM;
    const float acceleration = in.prescribedMotion ? 0.f : reff * d.torqueNm / Ie;
    const float du = in.dt * (acceleration - c * f0.fLong) / (1.f + kL * in.dt * c);
    float fLong = f0.fLong + kL * du;
    // Lateral: chassis share only.
    float fLat = f0.fLat / (1.f + kT * in.dt * invM);

    const float fMaxL = p.mu * in.normalLoadN;
    const float fMaxT = fMaxL * p.lateralScale;
    fLong = std::clamp(fLong, -fMaxL, fMaxL);
    fLat = std::clamp(fLat, -fMaxT, fMaxT);

    out.fLongN = fLong;
    out.fLatN = fLat;
    if (!in.prescribedMotion)
        out.omegaNext = d.omegaRadPerSec + in.dt * (d.torqueNm - reff * fLong) / Ie;
    out.slipRatio = uL / vRefLong;
    out.slipAngleTan = uT / vRefLat;
    out.saturated = f0.sliding || std::fabs(fLong) >= 0.999f * fMaxL;
    if (!std::isfinite(out.fLongN) || !std::isfinite(out.fLatN) || !std::isfinite(out.omegaNext) || !std::isfinite(out.slipRatio) ||
        !std::isfinite(out.slipAngleTan)) {  // degenerate parameters or inputs: no force, the wheel keeps its spin
        TireOutput safe;
        safe.omegaNext = std::isfinite(d.omegaRadPerSec) ? d.omegaRadPerSec : 0.f;
        return safe;
    }
    return out;
}

}  // namespace

float brushCurve(float s, float slideRatio) {
    s = std::max(s, 0.f);
    if (s <= 1.f) {
        const float t = 1.f - s;
        return 1.f - t * t * t;  // slope 3 at 0, peak 1 at s = 1
    }
    return slideRatio + (1.f - slideRatio) * std::exp(-2.f * (s - 1.f));
}

float pacejkaLiteCurve(float s, float slideRatio) { return PacejkaShape(slideRatio)(s); }

TireOutput BrushTireModel::solve(const TireParams& params, const TireInput& in, const WheelDrive& drive) const {
    return solveImpl(BrushCurve{params.slideRatio}, params, in, drive);
}

TireOutput PacejkaLiteTireModel::solve(const TireParams& params, const TireInput& in, const WheelDrive& drive) const {
    return solveImpl(PacejkaShape(params.slideRatio), params, in, drive);
}

sdk::Status registerTireModels(sdk::TireModelRegistry& registry) {
    if (auto s = registry.registerModel(std::string(BrushTireModel::kId),
                                        [] { return std::make_unique<BrushTireModel>(); });
        !s.ok())
        return s;
    return registry.registerModel(std::string(PacejkaLiteTireModel::kId),
                                  [] { return std::make_unique<PacejkaLiteTireModel>(); });
}

}  // namespace wpiemu::core::physics
