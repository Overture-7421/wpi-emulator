#pragma once

// Composition-root glue between the generic runtime and core/physics (header only: only code that links the whole core, i.e.
// the app and the integration tests, includes it; the runtime sources never do).
//
//   SessionParts parts;
//   parts.world = physics::createBox3dWorld(config.sim.physics);
//   parts.chassisBuilder = runtime::physicsChassisBuilder();
//   parts.coupling = std::make_unique<runtime::PhysicsCouplingDriver>();
//   physics::registerPhysicsComponents(drivetrains, tireModels);

#include <memory>
#include <string_view>

#include "wpiemu/core/physics/chassis_builder.h"
#include "wpiemu/core/physics/chassis_coupling.h"
#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/core/runtime/simulation_session.h"

namespace wpiemu::core::runtime {

// ChassisBuilder over physics::buildChassisBody (hull + wheel pads + mass), placed at the field origin.
inline ChassisBuilder physicsChassisBuilder(physics::ChassisPlacement placement = {}) {
    return [placement](sdk::IPhysicsWorld& world, const sdk::ChassisSpec& chassis, const sdk::DrivetrainSpec& drivetrain) -> sdk::Result<ChassisInfo> {
        sdk::Result<physics::ChassisBody> body = physics::buildChassisBody(world, chassis, drivetrain, placement);
        if (!body) return body.error();
        ChassisInfo info;
        info.body = body.value().body;
        info.baseMass = body.value().baseMass;
        info.spawnPose = physics::chassisSpawnPose(body.value(), placement);
        return info;
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
