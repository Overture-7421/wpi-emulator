#include "wpiemu/core/mechanisms/plants.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "wpiemu/core/mechanisms/linear_system.h"
#include "wpiemu/core/mechanisms/motor_catalog.h"

namespace wpiemu::core::mechanisms {

namespace {

constexpr double kStopEps = 1e-9;
constexpr double kMaxStepRate = 0.1;  // RK4 sub-step bound: |A| * h <= 0.1

struct Params {
    sdk::PlantSpec spec;
    LinearSystem system;
    double gamma = 1.0;    // motor shaft rad per output unit (G for angular, G / r for linear)
    double inertia = 1.0;  // J (angular) or carriage mass (linear)
    bool limited = false;
};

Params makeParams(sdk::PlantSpec spec, sdk::PlantKind kind, bool allowLimits) {
    spec.kind = kind;
    const bool linear = kind == sdk::PlantKind::Linear;
    Params p;
    p.gamma = linear ? spec.gearRatio / spec.drumRadiusM : spec.gearRatio;
    p.inertia = linear ? spec.massKg : spec.momentOfInertiaKgM2;
    p.system = linear ? elevatorSystem(spec.motor, spec.motorCount, spec.massKg, spec.drumRadiusM, spec.gearRatio)
                      : dcMotorSystem(spec.motor, spec.motorCount, spec.momentOfInertiaKgM2, spec.gearRatio);
    p.limited = allowLimits && spec.maxPosition > spec.minPosition;
    p.spec = std::move(spec);
    return p;
}

class Dc1DofPlant : public sdk::IPlant {
public:
    explicit Dc1DofPlant(Params p)
        : spec_(std::move(p.spec)), gamma_(p.gamma), inertia_(p.inertia), limited_(p.limited),
          motorRate_(p.system.A(1, 1)), inputGain_(p.system.B(1, 0)),
          kv_(spec_.motor.kvRadPerSecPerVolt()), resistance_(spec_.motor.resistanceOhm()),
          efficiency_(std::clamp(spec_.efficiency, 0.01, 1.0)) {
        viscousRate_ = std::max(spec_.viscousFrictionNmS, 0.0) / inertia_;
        coulombAccel_ = std::max(spec_.coulombFrictionNm, 0.0) / inertia_;
        state_.position = limited_ ? std::clamp(0.0, spec_.minPosition, spec_.maxPosition) : 0.0;
    }

    sdk::PlantKind kind() const override { return spec_.kind; }
    const sdk::PlantSpec& spec() const override { return spec_; }

    void reset(const sdk::PlantState& s) override {
        state_ = s;
        currentA_ = 0.0;
        clampToStops(state_);
    }
    sdk::PlantState state() const override { return state_; }

    sdk::PlantState step(const sdk::PlantInput& in) override {
        const double dt = in.dtSeconds;
        if (!(dt > 0.0) || !std::isfinite(dt)) return state_;
        const double volts = std::isfinite(in.voltageV) ? in.voltageV : 0.0;
        const double extAccel = std::isfinite(in.externalLoad) ? in.externalLoad / inertia_ : 0.0;

        const sdk::PlantState x0 = state_;
        currentA_ = static_cast<double>(spec_.motorCount) * (volts - gamma_ * x0.velocity / kv_) / resistance_;

        sdk::PlantState x1 = efficiency_ == 1.0 && !gravityDependsOnPosition()
                                 ? stepExact(x0, volts, gravityAccel(x0.position) + extAccel, dt)
                                 : stepNonlinear(x0, volts, extAccel, dt);
        applyCoulomb(x0, x1, volts, extAccel, dt);
        clampToStops(x1);
        state_ = x1;
        return state_;
    }

    double motorCurrentA() const override { return currentA_; }
    double motorShaftOmegaRadPerSec() const override { return gamma_ * state_.velocity; }
    bool atLowerLimit() const override { return limited_ && state_.position <= spec_.minPosition + kStopEps; }
    bool atUpperLimit() const override { return limited_ && state_.position >= spec_.maxPosition - kStopEps; }

protected:
    // Acceleration of the output coordinate caused by gravity at position q.
    virtual double gravityAccel(double /*q*/) const { return 0.0; }
    virtual bool gravityDependsOnPosition() const { return false; }

    const sdk::PlantSpec spec_;

private:
    // Acceleration without Coulomb friction.
    double freeAccel(double q, double v, double volts, double extAccel) const {
        double motor = motorRate_ * v + inputGain_ * volts;
        if (efficiency_ != 1.0) motor *= motor * v >= 0.0 ? efficiency_ : 1.0 / efficiency_;
        return motor - viscousRate_ * v + gravityAccel(q) + extAccel;
    }

    sdk::PlantState stepExact(const sdk::PlantState& x0, double volts, double disturbance, double dt) {
        if (dt != cachedDt_) {
            Matrix a(2, 2), b(2, 2);
            a(0, 1) = 1.0;
            a(1, 1) = motorRate_ - viscousRate_;
            b(1, 0) = inputGain_;
            b(1, 1) = 1.0;
            discrete_ = discretizeAB(a, b, dt);
            cachedDt_ = dt;
        }
        const double u1 = disturbance;
        return {discrete_.Ad(0, 0) * x0.position + discrete_.Ad(0, 1) * x0.velocity + discrete_.Bd(0, 0) * volts +
                    discrete_.Bd(0, 1) * u1,
                discrete_.Ad(1, 0) * x0.position + discrete_.Ad(1, 1) * x0.velocity + discrete_.Bd(1, 0) * volts +
                    discrete_.Bd(1, 1) * u1};
    }

    sdk::PlantState advanceFrozen(sdk::PlantState x, double volts, double disturbance,
                                  double h) const {
        const double drive = inputGain_ * volts;
        const double freeVelocity = -drive / motorRate_;
        double remaining = h;
        while (remaining > 0.0) {
            double factor = efficiency_;
            if (efficiency_ != 1.0) {
                if (x.velocity == 0.0) {
                    const double motoring = efficiency_ * drive + disturbance;
                    const double backdriving = drive / efficiency_ + disturbance;
                    if (drive == 0.0)
                        factor = 1.0 / efficiency_;
                    else if (drive > 0.0 ? motoring >= 0.0 : motoring <= 0.0)
                        factor = efficiency_;
                    else if (drive > 0.0 ? backdriving <= 0.0 : backdriving >= 0.0)
                        factor = 1.0 / efficiency_;
                    else
                        return x;  // The two quadrant accelerations point toward zero velocity.
                } else {
                    const double motor =
                        x.velocity == freeVelocity
                            ? motorRate_ * (disturbance - viscousRate_ * x.velocity)
                            : motorRate_ * x.velocity + drive;
                    factor = motor * x.velocity >= 0.0 ? efficiency_ : 1.0 / efficiency_;
                }
            }
            const double a = factor * motorRate_ - viscousRate_;
            const double equilibrium = -(factor * drive + disturbance) / a;
            const double endVelocity =
                equilibrium + (x.velocity - equilibrium) * std::exp(a * remaining);
            double duration = remaining;
            double boundaryVelocity = endVelocity;
            // With frozen gravity the scalar velocity is monotone. It can cross each
            // efficiency boundary at most once, so these exact events do not need a work cap.
            for (const double boundary : {0.0, freeVelocity}) {
                if (!((x.velocity < boundary && boundary < endVelocity) ||
                      (endVelocity < boundary && boundary < x.velocity)))
                    continue;
                const double crossing =
                    std::log1p((boundary - x.velocity) / (x.velocity - equilibrium)) / a;
                if (crossing < duration) {
                    duration = crossing;
                    boundaryVelocity = boundary;
                }
            }
            x.position +=
                equilibrium * duration + (x.velocity - equilibrium) * std::expm1(a * duration) / a;
            x.velocity = boundaryVelocity;
            remaining = duration == remaining ? 0.0 : remaining - duration;
        }
        return x;
    }

    sdk::PlantState stepNonlinear(const sdk::PlantState& x0, double volts, double extAccel,
                                  double dt) const {
        const double rate = std::abs(motorRate_) / std::min(efficiency_, 1.0) + viscousRate_;
        if (rate * dt > 10.0) {
            // Integrate electrical/viscous damping exactly. Only gravity is sampled: a half-step
            // predictor supplies its midpoint value, without an explicit stiff velocity mode.
            sdk::PlantState x = x0;
            double remaining = dt;
            while (remaining > 0.0) {
                // The time/angle bounds control nonlinear accuracy, not electrical stability.
                const double h =
                    std::min({remaining, 0.0001, 0.001 / (1.0 + std::abs(x.velocity))});
                const auto midpoint =
                    advanceFrozen(x, volts, gravityAccel(x.position) + extAccel, 0.5 * h);
                x = advanceFrozen(x, volts, gravityAccel(midpoint.position) + extAccel, h);
                remaining = h == remaining ? 0.0 : remaining - h;
            }
            return x;
        }
        // The stiff path above bounds this conversion to [1, 100], without capping h.
        const int n = static_cast<int>(std::max(1.0, std::ceil(rate * dt / kMaxStepRate)));
        const double h = dt / static_cast<double>(n);
        sdk::PlantState x = x0;
        const auto accel = [&](double q, double v) { return freeAccel(q, v, volts, extAccel); };
        for (int i = 0; i < n; ++i) {
            const double k1q = x.velocity, k1v = accel(x.position, x.velocity);
            const double k2q = x.velocity + 0.5 * h * k1v,
                         k2v = accel(x.position + 0.5 * h * k1q, k2q);
            const double k3q = x.velocity + 0.5 * h * k2v,
                         k3v = accel(x.position + 0.5 * h * k2q, k3q);
            const double k4q = x.velocity + h * k3v, k4v = accel(x.position + h * k3q, k4q);
            x.position += h / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
            x.velocity += h / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        }
        return x;
    }

    // Coulomb friction as an explicit post-step correction: sticks when the drive cannot overcome
    // it.
    void applyCoulomb(const sdk::PlantState& x0, sdk::PlantState& x1, double volts, double extAccel,
                      double dt) const {
        if (coulombAccel_ <= 0.0) return;
        if (std::abs(x0.velocity) <= kStopEps &&
            std::abs(freeAccel(x0.position, 0.0, volts, extAccel)) <= coulombAccel_) {
            x1 = {x0.position, 0.0};
            return;
        }
        const double dv = coulombAccel_ * dt;
        const double dir = std::abs(x0.velocity) > kStopEps ? (x0.velocity > 0.0 ? 1.0 : -1.0)
                                                            : (x1.velocity > 0.0 ? 1.0 : -1.0);
        const bool stops = x1.velocity * dir <= dv;
        if (stops) {
            const double tStop = std::min(dt, std::abs(x0.velocity) / coulombAccel_);
            x1 = {x0.position + 0.5 * x0.velocity * tStop, 0.0};
        } else {
            x1.velocity -= dir * dv;
            x1.position -= dir * 0.5 * coulombAccel_ * dt * dt;
        }
    }

    void clampToStops(sdk::PlantState& x) const {
        if (!limited_) return;
        if (x.position <= spec_.minPosition) x = {spec_.minPosition, 0.0};
        else if (x.position >= spec_.maxPosition) x = {spec_.maxPosition, 0.0};
    }

    double gamma_;
    double inertia_;
    bool limited_;
    double motorRate_;   // A(1,1) of the WPILib system (back-EMF damping), 1/s
    double inputGain_;   // B(1,0), (output unit/s^2)/V
    double kv_;
    double resistance_;
    double efficiency_;
    double viscousRate_ = 0.0;
    double coulombAccel_ = 0.0;
    sdk::PlantState state_;
    double currentA_ = 0.0;
    double cachedDt_ = -1.0;
    DiscreteSystem discrete_;
};

class DcMotorPlant final : public Dc1DofPlant {
public:
    DcMotorPlant(const sdk::PlantSpec& spec, sdk::PlantKind kind, bool allowLimits)
        : Dc1DofPlant(makeParams(spec, kind, allowLimits)) {}
};

class ElevatorPlant final : public Dc1DofPlant {
public:
    explicit ElevatorPlant(const sdk::PlantSpec& spec)
        : Dc1DofPlant(makeParams(spec, sdk::PlantKind::Linear, true)) {}

protected:
    double gravityAccel(double) const override { return spec_.simulateGravity ? -kPlantGravityMps2 : 0.0; }
};

class ArmPlant final : public Dc1DofPlant {
public:
    explicit ArmPlant(const sdk::PlantSpec& spec)
        : Dc1DofPlant(makeParams(spec, sdk::PlantKind::Angular, true)) {}

protected:
    double gravityAccel(double q) const override {
        if (!spec_.simulateGravity) return 0.0;
        return -spec_.massKg * kPlantGravityMps2 * spec_.armLengthM * std::cos(q - spec_.gravityAngleOffsetRad) /
               spec_.momentOfInertiaKgM2;
    }
    bool gravityDependsOnPosition() const override { return spec_.simulateGravity; }
};

sdk::Status invalid(const std::string& field, const std::string& why) {
    return sdk::makeError(sdk::ErrorCode::InvalidArgument, "plant spec: " + field + " " + why);
}

bool positive(double v) { return std::isfinite(v) && v > 0.0; }

template <class P, class... Args>
sdk::PlantFactory::Factory guarded(const char* model, Args... args) {
    return [model, args...](const sdk::PlantSpec& spec) -> std::unique_ptr<sdk::IPlant> {
        sdk::PlantSpec s = spec;
        s.model = model;
        if (!validatePlantSpec(s).ok()) return nullptr;
        return std::make_unique<P>(s, args...);
    };
}

}  // namespace

sdk::Status validatePlantSpec(const sdk::PlantSpec& spec) {
    const std::string& m = spec.model;
    const bool known = m == plant_model::kDcMotor || m == plant_model::kTurret || m == plant_model::kFlywheel ||
                       m == plant_model::kArm || m == plant_model::kElevator;
    if (!known) return sdk::makeError(sdk::ErrorCode::NotFound, "unknown plant model: " + m);
    if (!sdk::isValidMotorSpec(spec.motor)) return invalid("motor", "is not a physical motor");
    if (spec.motorCount < 1 || spec.motorCount > 64) return invalid("motorCount", "must be in [1, 64]");
    if (!positive(spec.gearRatio)) return invalid("gearRatio", "must be > 0");
    if (!(std::isfinite(spec.efficiency) && spec.efficiency > 0.0 && spec.efficiency <= 1.0))
        return invalid("efficiency", "must be in (0, 1]");
    if (!(std::isfinite(spec.viscousFrictionNmS) && spec.viscousFrictionNmS >= 0.0))
        return invalid("viscousFrictionNmS", "must be >= 0");
    if (!(std::isfinite(spec.coulombFrictionNm) && spec.coulombFrictionNm >= 0.0))
        return invalid("coulombFrictionNm", "must be >= 0");
    if (!std::isfinite(spec.minPosition) || !std::isfinite(spec.maxPosition) || spec.maxPosition < spec.minPosition)
        return invalid("minPosition/maxPosition", "must be finite with max >= min");

    const bool linear = m == plant_model::kElevator || (m == plant_model::kDcMotor && spec.kind == sdk::PlantKind::Linear);
    if (linear) {
        if (!positive(spec.massKg)) return invalid("massKg", "must be > 0");
        if (!positive(spec.drumRadiusM)) return invalid("drumRadiusM", "must be > 0");
    } else if (!positive(spec.momentOfInertiaKgM2)) {
        return invalid("momentOfInertiaKgM2", "must be > 0");
    }
    if (m == plant_model::kArm && spec.simulateGravity) {
        if (!positive(spec.massKg)) return invalid("massKg", "must be > 0 with gravity");
        if (!(std::isfinite(spec.armLengthM) && spec.armLengthM >= 0.0)) return invalid("armLengthM", "must be >= 0");
    }
    return sdk::okStatus();
}

sdk::Status registerBuiltinPlants(sdk::PlantFactory& factory) {
    struct Entry {
        const char* model;
        sdk::PlantFactory::Factory make;
    };
    const Entry entries[] = {
        {plant_model::kDcMotor,
         [](const sdk::PlantSpec& spec) -> std::unique_ptr<sdk::IPlant> {
             if (!validatePlantSpec(spec).ok()) return nullptr;
             return std::make_unique<DcMotorPlant>(spec, spec.kind, true);
         }},
        {plant_model::kTurret, guarded<DcMotorPlant>(plant_model::kTurret, sdk::PlantKind::Angular, true)},
        {plant_model::kFlywheel, guarded<DcMotorPlant>(plant_model::kFlywheel, sdk::PlantKind::Angular, false)},
        {plant_model::kArm, guarded<ArmPlant>(plant_model::kArm)},
        {plant_model::kElevator, guarded<ElevatorPlant>(plant_model::kElevator)},
    };
    for (const Entry& e : entries) {
        sdk::Status s = factory.registerPlant(e.model, e.make);
        if (!s.ok()) return s;
    }
    return sdk::okStatus();
}

}  // namespace wpiemu::core::mechanisms
