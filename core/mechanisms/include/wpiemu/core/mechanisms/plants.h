#pragma once

// IPlant implementations mirroring WPILib's LinearSystemId-based simulation models, plus the
// registration into a PlantFactory.
//
//   model key    kind     WPILib equivalent / notes
//   "dc_motor"   spec.kind  DCMotorSim (angular, J) or a linear carriage (massKg, drumRadiusM)
//   "turret"     Angular  DCMotorSim; optional hard stops (min == max => continuous)
//   "flywheel"   Angular  FlywheelSim; hard stops are ignored, position integrates (encoders)
//   "arm"        Angular  SingleJointedArmSim: gravity m*g*L_cm*cos(theta - offset)/J, hard stops
//   "elevator"   Linear   ElevatorSim: constant gravity acceleration, hard stops
//
// Dynamics (x = [position, velocity] of the MECHANISM output, N = spec.motorCount motors):
//   v' = (A v + B V)*f(eta) - (b/M) v + gravity + external/M          (A, B from linear_system.h)
// plus Coulomb friction (stiction handled explicitly) and clamping hard stops that zero the
// velocity (WPILib semantics). Linear cases (eta == 1, no angle-dependent gravity) are advanced with
// the exact matrix-exponential discretization; the others with RK4 (sub-stepped by the dynamics rate).
//
// All motor/gearbox maths follows WPILib's DCMotor aggregation (Kt, R/N, Kv).

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/dynamics/plant.h"

namespace wpiemu::core::mechanisms {

namespace plant_model {
inline constexpr const char* kDcMotor = "dc_motor";
inline constexpr const char* kTurret = "turret";
inline constexpr const char* kFlywheel = "flywheel";
inline constexpr const char* kArm = "arm";
inline constexpr const char* kElevator = "elevator";
}  // namespace plant_model

// Gravity used by the arm/elevator models. WPILib's ElevatorSim/SingleJointedArmSim hard-code 9.8.
inline constexpr double kPlantGravityMps2 = 9.8;

// Checks the fields the given `spec.model` needs (motor, counts, ratios, inertia/mass, limits...).
// InvalidArgument with a message naming the offending field; NotFound for an unknown model key.
sdk::Status validatePlantSpec(const sdk::PlantSpec& spec);

// Registers the five models above (AlreadyExists if any is registered). A factory callback cannot
// report details, so it returns null for an invalid spec: call validatePlantSpec first when you
// need the reason.
sdk::Status registerBuiltinPlants(sdk::PlantFactory& factory);

}  // namespace wpiemu::core::mechanisms
