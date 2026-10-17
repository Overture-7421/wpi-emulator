#pragma once

// Builds the chassis rigid body from ChassisSpec / DrivetrainSpec (ADR 0003 "Chassis model"):
// one body whose origin is the centre of mass, a collision hull (frame box + collidable primitives) lifted
// by the ground clearance, and one frictionless wheel-pad sphere per wheel that carries the weight.

#include <cstdint>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/physics/chassis_spec.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::core::physics {

// Categories the robot's hull collides with / is detected by. Sensor is included on purpose: sensor volumes
// only report a visitor whose mask contains the sensor's category (ADR 0003 pitfall).
inline constexpr std::uint64_t kRobotHullMask =
    sdk::layer::Field | sdk::layer::Fuel | sdk::layer::Robot | sdk::layer::Projectile | sdk::layer::Sensor;
inline constexpr std::uint64_t kWheelPadMask =
    sdk::layer::Field | sdk::layer::Fuel | sdk::layer::Robot | sdk::layer::Projectile;

// Effective free-axis scale of roller wheels (omni / mecanum): the spec may lower it, never raise it.
inline constexpr float kRollerLateralScaleMax = 0.15f;

struct ChassisPlacement {
    float xM = 0.f;
    float yM = 0.f;
    float yawRad = 0.f;
};

struct ChassisBuildOptions {
    std::uint64_t userTag = 0;  // stamped on every robot shape (returned in events and ray hits)
};

struct ChassisBody {
    sdk::BodyId body;
    sdk::ShapeId hull;
    std::vector<sdk::ShapeId> extraShapes;   // collidable PrimitiveSpecs
    std::vector<sdk::ShapeId> wheelPads;     // same order as DrivetrainSpec::wheels
    std::vector<sdk::Vec3> wheelHubLocal;    // hub centres in the body frame (origin = COM)
    sdk::MassProperties baseMass;            // chassis alone, before payload composition
    float restHeightM = 0.f;                 // COM height above the floor when all pads touch it
};

// Wheel hub in the body frame. WheelSpec::positionM.z is the hub height above the floor in the rest pose
// (<= 0 means "one radius"); x and y are already relative to the COM.
sdk::Vec3 wheelHubLocal(const sdk::ChassisSpec& chassis, const sdk::WheelSpec& wheel);

// Mass, COM (body origin) and principal inertia. A zero ChassisSpec::inertiaDiagKgM2 is derived from a solid
// box of sizeM placed at its real offset from the COM (parallel-axis theorem).
sdk::MassProperties chassisMassProperties(const sdk::ChassisSpec& chassis);

// InvalidArgument for non-positive mass / size / COM height, wheel radius or primitive dimensions.
// Failure leaves no newly created bodies/shapes in the world.
sdk::Result<ChassisBody> buildChassisBody(sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis,
                                          const sdk::DrivetrainSpec& drivetrain, const ChassisPlacement& placement,
                                          const ChassisBuildOptions& options = {});

// Pose that puts the chassis on the floor at `placement` (z = restHeightM + settle margin).
sdk::Pose3 chassisSpawnPose(const ChassisBody& body, const ChassisPlacement& placement);

}  // namespace wpiemu::core::physics
