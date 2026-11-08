#pragma once

// Turns commanded wheel setpoints (target wheel speed, target module azimuth) into the motor voltages a
// ChassisSimulator expects, so a robot that has no WPILib program behind it (a remote player in a match, a scripted bot)
// is driven through the same motor, current-limit, tire and steering-lag path as the local robot.
//
//   Drive  V = gear * setpoint / kv  +  Kp * (setpoint - measured wheel speed), clamped to the motor's nominal voltage.
//          The feed-forward is the back-EMF of the motor at the commanded speed; Kp is derived from the wheel's reflected
//          inertia so the discrete loop gain per step is `driveLoopGain` (< 1: no overshoot even unloaded, because the
//          chassis only adds inertia). Every drive motor of the wheel gets the same voltage.
//   Steer  V = Kp * wrap(setpoint - angle) - Kd * omega, clamped to the steer motor's nominal voltage. The error is
//          wrapped to (-pi, pi], so a module takes the short way round. The gains come from the steer motor, gearing and
//          reflected inertia as a second-order system of natural frequency `steerNaturalFreqRadPerSec` and damping ratio
//          `steerDampingRatio`; the motor's own back-EMF damping counts towards it and the explicit damping term is
//          capped so it stays stable at the design step (see the constructor).
//
// Pure and deterministic: no clock, no randomness, no allocation in step(). The controller keeps no loop state, so
// reset() only forgets the setpoints. Disabled (disable, e-stop, pause, input timeout) outputs 0 V, which brakes the
// drive motors through their back-EMF (docs/reference/physics.md) and lets the module motors go limp.
//
// Tuned for steps of at most Options::designDtS (5 ms); a larger step weakens the stability margin.

#include <cstddef>
#include <span>
#include <vector>

#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/drivetrain.h"

namespace wpiemu::core::physics {

class WheelSetpointController {
public:
    struct Options {
        float designDtS = 0.005f;                  // the largest step the gains are designed for
        float driveLoopGain = 0.8f;                // drive Kp * dt * torque-per-volt / inertia; keep below 1
        float steerNaturalFreqRadPerSec = 30.f;    // ~0.2 s to settle a large turn, 0.15 rad per 5 ms step
        float steerDampingRatio = 1.1f;            // above 1: no overshoot
        // Upper limit of the steer natural frequency in rad per design step; only a module whose motor back-EMF damps
        // it more than critically is raised towards it (so it does not creep).
        float steerMaxNaturalFreqTimesDt = 0.35f;
        // The plant assumptions of the drivetrain (rotor inertia, steer efficiency); pass the options given to the
        // drivetrain when they are not the defaults. Only those two fields are read.
        TractionOptions plant;
    };

    explicit WheelSetpointController(const sdk::DrivetrainSpec& spec, const Options& options);
    explicit WheelSetpointController(const sdk::DrivetrainSpec& spec) : WheelSetpointController(spec, Options{}) {}

    std::size_t wheelCount() const { return wheels_.size(); }

    // One setpoint per wheel, in DrivetrainSpec order. A span of another size is ignored (the previous setpoints stay)
    // and counted in rejectedInputs(). A non-finite wheel speed becomes 0 and a non-finite angle keeps the wheel's
    // previous target.
    void setSetpoints(std::span<const sdk::WheelSetpoint> setpoints);
    // Zero speed on every wheel, steer targets unchanged: the robot brakes without the modules snapping to 0.
    void zeroSpeeds();
    // Disabled outputs 0 V on every motor. Enabled by default; reset() does not change it.
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool enabled() const { return enabled_; }
    // Forgets the setpoints (zero speed, zero angle) and the rejection count.
    void reset();

    // Voltages for this tick from the wheel states of the previous one (IDrivetrain::wheelStates). `states` and `out` must
    // both hold wheelCount() entries; otherwise every output is zeroed and the call counts in rejectedInputs().
    // Non-finite measurements count as zero. Only driveVoltageV and steerVoltageV are written (driveMotors = 0).
    void step(std::span<const sdk::WheelState> states, float dtSeconds, std::span<sdk::WheelCommand> out);

    std::size_t rejectedInputs() const { return rejected_; }
    const std::vector<sdk::WheelSetpoint>& setpoints() const { return setpoints_; }

    // Gains of one wheel (for tests and diagnostics).
    struct WheelGains {
        double driveVoltsPerRadPerSec = 0.0;    // feed-forward: gear / kv
        double driveKp = 0.0;                   // V per rad/s of wheel speed error
        double driveLimitV = 0.0;
        bool steered = false;
        double steerKp = 0.0;                   // V per rad
        double steerKd = 0.0;                   // V per rad/s
        double steerLimitV = 0.0;
        double steerNaturalFreqRadPerSec = 0.0; // after the stability cap
        double steerDampingRatio = 0.0;         // including the motor's back-EMF damping
    };
    const WheelGains& gains(std::size_t wheel) const { return wheels_[wheel]; }

private:
    std::vector<WheelGains> wheels_;
    std::vector<sdk::WheelSetpoint> setpoints_;
    std::size_t rejected_ = 0;
    bool enabled_ = true;
};

}  // namespace wpiemu::core::physics
