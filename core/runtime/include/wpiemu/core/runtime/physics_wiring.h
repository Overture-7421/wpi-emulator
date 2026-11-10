#pragma once

// Composition-root glue between the generic runtime and core/physics (header only: only code that links the whole core, i.e.
// the app and the integration tests, includes it; the runtime sources never do).
//
//   SessionParts parts;
//   parts.world = physics::createBox3dWorld(config.sim.physics);
//   parts.chassisBuilder = runtime::physicsChassisBuilder();
//   parts.coupling = std::make_unique<runtime::PhysicsCouplingDriver>();
//   parts.remoteChassisFactory = runtime::physicsRemoteChassisFactory(drivetrains, tireModels);   // practice matches
//   physics::registerPhysicsComponents(drivetrains, tireModels);

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include "wpiemu/core/physics/chassis_builder.h"
#include "wpiemu/core/physics/chassis_coupling.h"
#include "wpiemu/core/physics/chassis_simulator.h"
#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/core/physics/wheel_setpoint_controller.h"
#include "wpiemu/core/runtime/simulation_session.h"

namespace wpiemu::core::runtime {

// ChassisBuilder over physics::buildChassisBody (hull + wheel pads + mass), placed at the field origin. `userTag` is
// stamped on every shape of the host robot (0 = untagged; pass sdk::makeRobotTag(sdk::match::kHostParticipantId) in a
// match so contacts with remote robots name both robots).
inline ChassisBuilder physicsChassisBuilder(physics::ChassisPlacement placement = {}, std::uint64_t userTag = 0) {
    return [placement, userTag](sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis, const sdk::DrivetrainSpec& drivetrain) -> sdk::Result<ChassisInfo> {
        sdk::Result<physics::ChassisBody> body = physics::buildChassisBody(world, chassis, drivetrain, placement, physics::ChassisBuildOptions{userTag});
        if (!body) return body.error();
        ChassisInfo info;
        info.body = body.value().body;
        info.baseMass = body.value().baseMass;
        info.spawnPose = physics::chassisSpawnPose(body.value(), placement);
        return info;
    };
}

// IRemoteChassis over physics::ChassisSimulator (body, wheel pads, tire model, registered drivetrain) driven by a
// physics::WheelSetpointController. It only ever calls prepareStep / completeStep on the simulator, never step(): the
// session steps the shared world once for every robot.
class PhysicsRemoteChassis final : public IRemoteChassis {
public:
    static sdk::Result<std::unique_ptr<IRemoteChassis>> create(sdk::IPhysicsWorld& world, const sdk::DrivetrainRegistry& drivetrains,
                                                              const sdk::TireModelRegistry& tireModels, const sdk::RobotConfig& spec,
                                                              std::uint64_t userTag, const sdk::RobotStartPose& placement) {
        physics::ChassisSimulatorArgs args{world, spec.chassis, spec.drivetrain.spec, drivetrains, tireModels, toPlacement(placement), {userTag}};
        sdk::Result<std::unique_ptr<physics::ChassisSimulator>> sim = physics::ChassisSimulator::create(args);
        if (!sim) return sim.error();
        return std::unique_ptr<IRemoteChassis>(new PhysicsRemoteChassis(std::move(sim).value(), spec.drivetrain.spec));
    }

    std::size_t wheelCount() const override { return controller_.wheelCount(); }
    void setSetpoints(std::span<const sdk::WheelSetpoint> setpoints) override { controller_.setSetpoints(setpoints); }
    void brake() override { controller_.zeroSpeeds(); }
    void prepareStep(bool enabled, float dt) override {
        controller_.setEnabled(enabled);
        controller_.step(sim_->drivetrain().wheelStates(), dt, commands_);
        sim_->prepareStep(commands_, dt);
    }
    void completeStep(float dt) override { sim_->completeStep(dt); }
    void reset(const sdk::RobotStartPose& placement) override {
        sim_->reset(toPlacement(placement));
        controller_.reset();
    }
    sdk::BodyId body() const override { return sim_->body(); }
    std::span<const sdk::WheelState> wheelStates() const override { return sim_->drivetrain().wheelStates(); }
    bool restoreWheelKinematics(std::span<const sdk::WheelKinematics> wheels) override {
        return sim_->drivetrain().restoreWheelKinematics(wheels);
    }

private:
    PhysicsRemoteChassis(std::unique_ptr<physics::ChassisSimulator> sim, const sdk::DrivetrainSpec& drivetrain)
        : sim_(std::move(sim)), controller_(drivetrain), commands_(drivetrain.wheels.size()) {}

    static physics::ChassisPlacement toPlacement(const sdk::RobotStartPose& p) { return {p.xM, p.yM, p.headingRad}; }

    std::unique_ptr<physics::ChassisSimulator> sim_;
    physics::WheelSetpointController controller_;
    std::vector<sdk::WheelCommand> commands_;
};

// The RemoteChassisFactory of the real physics module. The registries must outlive every session built with it.
inline RemoteChassisFactory physicsRemoteChassisFactory(const sdk::DrivetrainRegistry& drivetrains, const sdk::TireModelRegistry& tireModels) {
    return [&drivetrains, &tireModels](sdk::IPhysicsWorld& world, const sdk::RobotConfig& spec, std::uint64_t userTag,
                                       const sdk::RobotStartPose& placement) {
        return PhysicsRemoteChassis::create(world, drivetrains, tireModels, spec, userTag, placement);
    };
}

// ICouplingDriver over physics::ChassisPhysicsCoupling (it applies wrenches and payload mass immediately, so beforeStep has
// nothing to flush). Also keeps the drivetrain's quasi-static load model informed about the composed mass.
class PhysicsCouplingDriver final : public ICouplingDriver, public sdk::IMechanismPhysicsCoupling {
public:
    void bind(sdk::IPhysicsWorld& world, sdk::BodyId chassis, const sdk::MassProperties& baseMass) override {
        coupling_ = std::make_unique<physics::ChassisPhysicsCoupling>(world, chassis, baseMass);
    }
    void attachDrivetrain(sdk::IDrivetrain* drivetrain) override {
        if (coupling_) coupling_->setMassListener(dynamic_cast<physics::IChassisMassAware*>(drivetrain));
    }
    sdk::IMechanismPhysicsCoupling& service() override { return *this; }
    void beforeStep(sdk::IPhysicsWorld&, sdk::BodyId, const sdk::MassProperties&) override {}
    void afterStep() override {
        if (coupling_) coupling_->onStepCompleted();
    }
    void reset() override {
        if (coupling_) coupling_->reset();
    }

    // ---- sdk::IMechanismPhysicsCoupling: delegation (neutral until bound) ----
    sdk::CouplingCapabilities capabilities() const override { return coupling_ ? coupling_->capabilities() : sdk::CouplingCapabilities{}; }
    void applyWrench(const sdk::Vec3& point, const sdk::Wrench& wrench) override {
        if (coupling_) coupling_->applyWrench(point, wrench);
    }
    sdk::ConstraintHandle addConstraint(const sdk::AnchorConstraintDesc& desc) override {
        return coupling_ ? coupling_->addConstraint(desc) : sdk::ConstraintHandle{};
    }
    void removeConstraint(sdk::ConstraintHandle handle) override {
        if (coupling_) coupling_->removeConstraint(handle);
    }
    sdk::ConstraintLoad constraintLoad(sdk::ConstraintHandle handle) const override {
        return coupling_ ? coupling_->constraintLoad(handle) : sdk::ConstraintLoad{};
    }
    void setPayload(std::string_view owner, const sdk::PayloadMass& payload) override {
        if (coupling_) coupling_->setPayload(owner, payload);
    }
    void clearPayload(std::string_view owner) override {
        if (coupling_) coupling_->clearPayload(owner);
    }

private:
    std::unique_ptr<physics::ChassisPhysicsCoupling> coupling_;
};

}  // namespace wpiemu::core::runtime
