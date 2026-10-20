#pragma once

// One-stop owner of a simulated chassis: body + wheel pads (chassis_builder), tire model and drivetrain from the
// SDK registries, headless mechanism coupling and IMU truth. The sim runtime can use it directly or compose the
// pieces itself.
//
// Tick order (docs/ARCHITECTURE.md section 4):
//   mechanisms submit to coupling()  ->  prepareStep(): drivetrain forces  ->  world.step()  ->  completeStep().
// step() does the last three for you (it steps the world).

#include <memory>
#include <span>

#include "wpiemu/core/physics/chassis_builder.h"
#include "wpiemu/core/physics/chassis_coupling.h"
#include "wpiemu/core/physics/chassis_imu.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::core::physics {

struct ChassisSimulatorArgs {
    sdk::IPhysicsWorld& world;                    // must outlive the simulator
    const sdk::ChassisSpec& chassis;              // copied
    const sdk::DrivetrainSpec& drivetrain;        // copied
    const sdk::DrivetrainRegistry& drivetrains;   // key: drivetrain.kind
    const sdk::TireModelRegistry& tireModels;     // key: drivetrain.tireModel
    ChassisPlacement placement;
    ChassisBuildOptions options;
    sdk::Vec3 gravity{0.f, 0.f, -sdk::kDefaultGravityMps2};   // for the IMU; matches PhysicsConfig::gravity's default
};

class ChassisSimulator {
public:
    // NotFound for an unknown drivetrain kind / tire model, InvalidArgument for bad specs.
    static sdk::Result<std::unique_ptr<ChassisSimulator>> create(const ChassisSimulatorArgs& args);
    ~ChassisSimulator();

    ChassisSimulator(const ChassisSimulator&) = delete;
    ChassisSimulator& operator=(const ChassisSimulator&) = delete;

    // Drivetrain traction forces for this tick (before the world step). Also flushes nothing else: wrenches and
    // payloads submitted through coupling() are already applied.
    void prepareStep(std::span<const sdk::WheelCommand> commands, float dt);
    // After the world step: constraint loads, IMU.
    void completeStep(float dt);
    // prepareStep + world.step(dt) + completeStep.
    void step(std::span<const sdk::WheelCommand> commands, float dt);

    // Teleport to `placement` at rest and clear drivetrain / coupling / IMU state.
    void reset(const ChassisPlacement& placement);

    sdk::BodyId body() const { return chassis_.body; }
    const ChassisBody& chassisBody() const { return chassis_; }
    sdk::Pose3 pose() const { return world_.pose(chassis_.body); }
    sdk::IDrivetrain& drivetrain() { return *drivetrain_; }
    const sdk::IDrivetrain& drivetrain() const { return *drivetrain_; }
    ChassisPhysicsCoupling& coupling() { return *coupling_; }
    const sdk::ImuFeedback& imu() const { return imu_.last(); }

private:
    ChassisSimulator(sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis, const sdk::DrivetrainSpec& drivetrain,
                     sdk::Vec3 gravity);

    sdk::IPhysicsWorld& world_;
    sdk::ChassisSpec chassisSpec_;
    sdk::DrivetrainSpec drivetrainSpec_;
    ChassisBody chassis_;
    std::unique_ptr<sdk::ITireModel> tire_;
    std::unique_ptr<sdk::IDrivetrain> drivetrain_;
    std::unique_ptr<ChassisPhysicsCoupling> coupling_;
    ChassisImuSensor imu_;
};

// Registers the shipped tire models and drivetrains ("brush", "pacejka_lite"; "swerve", "tank", "mecanum").
sdk::Status registerPhysicsComponents(sdk::DrivetrainRegistry& drivetrains, sdk::TireModelRegistry& tireModels);

}  // namespace wpiemu::core::physics
