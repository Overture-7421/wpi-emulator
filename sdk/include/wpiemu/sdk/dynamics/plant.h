#pragma once

// 1-DOF plant abstraction (WPILib LinearSystem / DCMotorSim / ElevatorSim / SingleJointedArmSim /
// FlywheelSim style) behind an interface so mechanisms are independent of how the plant is
// integrated (wpimath in core, a hand-rolled one in tests).
//
// Units are SI: Linear => metres, m/s, N; Angular => radians, rad/s, N*m. Position/velocity are
// of the MECHANISM output (after gearing).

#include <string>
#include <string_view>
#include <utility>
#include <memory>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::sdk {

enum class PlantKind { Linear, Angular };

struct PlantState {
    double position = 0.0;
    double velocity = 0.0;
};

struct PlantSpec {
    std::string model = "dc_motor";    // key into PlantFactory: "dc_motor", "elevator", "arm", "flywheel", "turret"
    PlantKind kind = PlantKind::Angular;
    MotorSpec motor = motors::krakenX60();
    int motorCount = 1;
    double gearRatio = 1.0;            // motor : mechanism (>1 reduces)
    double efficiency = 1.0;
    // Inertia / mass
    double momentOfInertiaKgM2 = 0.01; // angular models
    double massKg = 5.0;               // linear models (carriage) / arm mass
    double drumRadiusM = 0.02;         // linear: converts rotation to translation
    double armLengthM = 0.5;           // arm: pivot to centre of mass
    bool simulateGravity = false;
    double gravityAngleOffsetRad = 0.0;// arm: angle at which gravity torque is maximal reference (0 = horizontal)
    // Hard stops (set min == max to disable)
    double minPosition = 0.0;
    double maxPosition = 0.0;
    double viscousFrictionNmS = 0.0;
    double coulombFrictionNm = 0.0;
};

struct PlantInput {
    double voltageV = 0.0;     // across the motor(s)
    double externalLoad = 0.0; // N or N*m at the mechanism output (disturbances: fuel mass, contact)
    double dtSeconds = 0.005;
};

class IPlant {
    WPIEMU_INTERFACE(IPlant)
public:
    virtual PlantKind kind() const = 0;
    virtual const PlantSpec& spec() const = 0;
    virtual void reset(const PlantState& state) = 0;
    virtual PlantState state() const = 0;
    // Advances by input.dtSeconds. Hard stops clamp position and zero velocity into the stop.
    virtual PlantState step(const PlantInput& input) = 0;
    // Total motor current for the last step (battery model), amps.
    virtual double motorCurrentA() const = 0;
    // Motor shaft speed implied by the current state (for integrated-sensor feedback), rad/s.
    virtual double motorShaftOmegaRadPerSec() const = 0;
    virtual bool atLowerLimit() const = 0;
    virtual bool atUpperLimit() const = 0;
};

class PlantFactory : public Registry<NoInfo, IPlant, const PlantSpec&> {
public:
    Status registerPlant(std::string model, Factory factory) { return add(std::move(model), NoInfo{}, std::move(factory)); }
    Result<std::unique_ptr<IPlant>> createFor(const PlantSpec& spec) const { return create(spec.model, spec); }
};

}  // namespace wpiemu::sdk
