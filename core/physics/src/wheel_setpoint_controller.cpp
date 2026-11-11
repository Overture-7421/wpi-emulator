#include "wpiemu/core/physics/wheel_setpoint_controller.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::physics {
namespace {

// The explicit (one step late) damping term may remove at most this fraction of the module's speed per step.
constexpr double kMaxExplicitDampingPerStep = 0.5;
constexpr double kMinGear = 1e-6;

double finiteOrZero(double v) { return std::isfinite(v) ? v : 0.0; }

double clampVolts(double v, double limit) { return std::clamp(v, -limit, limit); }

}  // namespace

WheelSetpointController::WheelSetpointController(const sdk::DrivetrainSpec& spec, const Options& options) {
    const double dt = std::max(1e-6, static_cast<double>(options.designDtS));
    const double rotorInertia = options.plant.rotorInertiaKgM2;
    wheels_.reserve(spec.wheels.size());
    for (const sdk::WheelSpec& w : spec.wheels) {
        WheelGains g;

        // Drive: wheel inertia reflected as in TractionDrivetrain::driveFor (rotors through the gearbox plus the wheel).
        const sdk::MotorSpec& dm = w.driveMotor;
        const double gear = std::max(static_cast<double>(w.driveGearRatio), kMinGear);
        const double count = std::clamp(w.driveMotorCount, 1, sdk::kMaxMotorsPerAxis);
        const double eta = std::clamp(static_cast<double>(w.driveEfficiency), 0.01, 1.0);
        const double driveInertia = count * rotorInertia * gear * gear + w.inertiaKgM2;
        const double torquePerVolt = eta * gear * count * dm.ktNmPerAmp() / dm.resistanceOhm();
        g.driveVoltsPerRadPerSec = gear / dm.kvRadPerSecPerVolt();
        g.driveKp = options.driveLoopGain * driveInertia / (dt * torquePerVolt);
        g.driveLimitV = dm.nominalVoltageV;

        // Steer: J w' = Kt_s V - b w with the motor's back-EMF damping b; PD on top gives
        // J s^2 + (b + Kt_s Kd) s + Kt_s Kp.
        g.steered = sdk::isSteered(w);
        if (g.steered) {
            const sdk::MotorSpec& sm = w.steerMotor;
            const double sGear = std::max(static_cast<double>(w.steerGearRatio), kMinGear);
            const double sEta = std::clamp(static_cast<double>(options.plant.steerEfficiency), 0.01, 1.0);
            const double inertia = rotorInertia * sGear * sGear + w.steerInertiaKgM2;
            const double kt = sEta * sGear * sm.ktNmPerAmp() / sm.resistanceOhm();                    // Nm per volt
            const double b = sEta * sGear * sGear * sm.ktNmPerAmp() / (sm.resistanceOhm() * sm.kvRadPerSecPerVolt());
            const double zeta = std::max(0.1, static_cast<double>(options.steerDampingRatio));
            const double kdMax = kMaxExplicitDampingPerStep * inertia / (kt * dt);

            // Damping needed for the wanted natural frequency, then the cap: when the explicit term saturates, lower the
            // natural frequency so the damping ratio is kept instead of ringing. A motor whose back-EMF alone already
            // damps beyond the wanted ratio would creep (one slow real pole), so the natural frequency is raised until
            // the motor damping alone gives the wanted ratio, up to a limit that keeps the loop well sampled.
            const double wnMax = options.steerMaxNaturalFreqTimesDt / dt;
            double wn = std::max(1e-3, static_cast<double>(options.steerNaturalFreqRadPerSec));
            wn = std::min(std::max(wn, b / (2.0 * zeta * inertia)), std::max(wn, wnMax));
            double kd = std::max(0.0, (2.0 * zeta * wn * inertia - b) / kt);
            if (kd > kdMax) {
                kd = kdMax;
                wn = (b + kt * kd) / (2.0 * zeta * inertia);
            }
            g.steerKp = inertia * wn * wn / kt;
            g.steerKd = kd;
            g.steerLimitV = sm.nominalVoltageV;
            g.steerNaturalFreqRadPerSec = wn;
            g.steerDampingRatio = (b + kt * kd) / (2.0 * inertia * wn);
        }
        wheels_.push_back(g);
    }
    setpoints_.assign(wheels_.size(), sdk::WheelSetpoint{});
}

void WheelSetpointController::setSetpoints(std::span<const sdk::WheelSetpoint> setpoints) {
    if (setpoints.size() != setpoints_.size()) {
        ++rejected_;
        return;
    }
    for (std::size_t i = 0; i < setpoints.size(); ++i) {
        setpoints_[i].wheelSpeedRadPerSec = std::isfinite(setpoints[i].wheelSpeedRadPerSec) ? setpoints[i].wheelSpeedRadPerSec : 0.f;
        if (std::isfinite(setpoints[i].steerAngleRad)) setpoints_[i].steerAngleRad = setpoints[i].steerAngleRad;
    }
}

void WheelSetpointController::zeroSpeeds() {
    for (sdk::WheelSetpoint& s : setpoints_) s.wheelSpeedRadPerSec = 0.f;
}

void WheelSetpointController::reset() {
    setpoints_.assign(wheels_.size(), sdk::WheelSetpoint{});
    rejected_ = 0;
}

void WheelSetpointController::step(std::span<const sdk::WheelState> states, float /*dtSeconds*/,
                                   std::span<sdk::WheelCommand> out) {
    const std::size_t n = wheels_.size();
    if (states.size() != n || out.size() != n) {
        ++rejected_;
        for (sdk::WheelCommand& c : out) c = sdk::WheelCommand{};
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        sdk::WheelCommand& cmd = out[i];
        cmd = sdk::WheelCommand{};
        if (!enabled_) continue;

        const WheelGains& g = wheels_[i];
        const sdk::WheelState& st = states[i];
        const double target = setpoints_[i].wheelSpeedRadPerSec;
        const double omega = finiteOrZero(st.driveWheelOmegaRadPerSec);
        cmd.driveVoltageV = clampVolts(g.driveVoltsPerRadPerSec * target + g.driveKp * (target - omega), g.driveLimitV);

        if (g.steered) {
            const double error = sdk::wrapAngleRad(static_cast<double>(setpoints_[i].steerAngleRad) -
                                                   finiteOrZero(st.steerAngleRad));
            cmd.steerVoltageV =
                clampVolts(g.steerKp * error - g.steerKd * finiteOrZero(st.steerOmegaRadPerSec), g.steerLimitV);
        }
    }
}

}  // namespace wpiemu::core::physics
