#include "wpiemu/core/physics/chassis_simulator.h"

#include "wpiemu/core/physics/tire_models.h"

namespace wpiemu::core::physics {

ChassisSimulator::ChassisSimulator(sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis,
                                   const sdk::DrivetrainSpec& drivetrain, sdk::Vec3 gravity)
    : world_(world), chassisSpec_(chassis), drivetrainSpec_(drivetrain), imu_(gravity) {}

ChassisSimulator::~ChassisSimulator() {
    coupling_.reset();     // releases joints that reference the body
    drivetrain_.reset();
    if (chassis_.body.valid()) world_.destroyBody(chassis_.body);
}

sdk::Result<std::unique_ptr<ChassisSimulator>> ChassisSimulator::create(const ChassisSimulatorArgs& args) {
    std::unique_ptr<ChassisSimulator> sim(
        new ChassisSimulator(args.world, args.chassis, args.drivetrain, args.gravity));

    auto tire = args.tireModels.create(sim->drivetrainSpec_.tireModel);
    if (!tire.ok()) return tire.error();
    sim->tire_ = std::move(tire).value();

    auto body = buildChassisBody(sim->world_, sim->chassisSpec_, sim->drivetrainSpec_, args.placement, args.options);
    if (!body.ok()) return body.error();
    sim->chassis_ = std::move(body).value();

    const sdk::DrivetrainBuildArgs buildArgs{sim->drivetrainSpec_, sim->chassisSpec_, sim->world_, sim->chassis_.body,
                                             *sim->tire_};
    auto drivetrain = args.drivetrains.create(sim->drivetrainSpec_.kind, buildArgs);
    if (!drivetrain.ok()) return drivetrain.error();  // ~ChassisSimulator removes the body again
    sim->drivetrain_ = std::move(drivetrain).value();

    sim->coupling_ = std::make_unique<ChassisPhysicsCoupling>(sim->world_, sim->chassis_.body, sim->chassis_.baseMass);
    sim->coupling_->setMassListener(dynamic_cast<IChassisMassAware*>(sim->drivetrain_.get()));
    return sim;
}

void ChassisSimulator::prepareStep(std::span<const sdk::WheelCommand> commands, float dt) {
    drivetrain_->step(commands, dt);
}

void ChassisSimulator::completeStep(float dt) {
    coupling_->onStepCompleted();
    imu_.update(world_, chassis_.body, dt);
}

void ChassisSimulator::step(std::span<const sdk::WheelCommand> commands, float dt) {
    prepareStep(commands, dt);
    world_.step(dt);
    completeStep(dt);
}

void ChassisSimulator::reset(const ChassisPlacement& placement) {
    coupling_->reset();
    world_.setLinearVelocity(chassis_.body, {});
    world_.setAngularVelocity(chassis_.body, {});
    world_.setPose(chassis_.body, chassisSpawnPose(chassis_, placement));
    drivetrain_->reset();
    imu_.reset();
}

sdk::Status registerPhysicsComponents(sdk::DrivetrainRegistry& drivetrains, sdk::TireModelRegistry& tireModels) {
    if (auto s = registerTireModels(tireModels); !s.ok()) return s;
    return registerDrivetrains(drivetrains);
}

}  // namespace wpiemu::core::physics
