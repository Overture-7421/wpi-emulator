#pragma once

// Headless IMechanismPhysicsCoupling over a physics world (SDK 1.1, ADR 0006 / 0008).
//
//   applyWrench    supported: converted to a world force at the point plus a world torque (this step only)
//   payload        supported: composeMassProperties(chassis, payloads) pushed to the body immediately
//   constraints    supported when the world implements IPhysicsJointHost (the Box3D world does); capabilities()
//                  reports it honestly. Distance -> rigid distance joint, Prismatic -> prismatic joint with
//                  limits, ContactAnchor -> spherical (ball) joint that releases above maxForceN.
//
// The runtime calls onStepCompleted() right after IPhysicsWorld::step() so constraintLoad() reports the last
// completed step and over-loaded anchors let go; reset() drops every constraint and payload (session reset).
// ChassisSimulator does both. Sim thread only.

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/core/physics/drivetrains.h"
#include "wpiemu/core/physics/joint_host.h"
#include "wpiemu/sdk/mechanisms/physics_coupling.h"

namespace wpiemu::core::physics {

class ChassisPhysicsCoupling final : public sdk::IMechanismPhysicsCoupling {
public:
    // `world` and the body must outlive the coupling. `baseMass` is the chassis alone (ChassisBody::baseMass).
    ChassisPhysicsCoupling(sdk::IPhysicsWorld& world, sdk::BodyId chassis, const sdk::MassProperties& baseMass);
    ~ChassisPhysicsCoupling() override;

    ChassisPhysicsCoupling(const ChassisPhysicsCoupling&) = delete;
    ChassisPhysicsCoupling& operator=(const ChassisPhysicsCoupling&) = delete;

    // Notified whenever the composed mass changes (and once on registration). May be null.
    void setMassListener(IChassisMassAware* listener);

    // ---- IMechanismPhysicsCoupling -----------------------------------------------------
    sdk::CouplingCapabilities capabilities() const override;
    void applyWrench(const sdk::Vec3& chassisLocalPoint, const sdk::Wrench& wrench) override;
    sdk::ConstraintHandle addConstraint(const sdk::AnchorConstraintDesc& desc) override;
    void removeConstraint(sdk::ConstraintHandle handle) override;
    sdk::ConstraintLoad constraintLoad(sdk::ConstraintHandle handle) const override;
    void setPayload(std::string_view owner, const sdk::PayloadMass& payload) override;
    void clearPayload(std::string_view owner) override;

    // ---- runtime hooks -----------------------------------------------------------------
    void onStepCompleted();
    void reset();
    const sdk::MassProperties& composedMass() const { return composed_; }
    std::size_t constraintCount() const { return constraints_.size(); }

private:
    struct Constraint {
        sdk::AnchorConstraintDesc desc;
        JointId joint;
        sdk::ConstraintLoad load;
        bool engaged = true;
    };

    void recomposeMass();
    void dropConstraints();

    sdk::IPhysicsWorld& world_;
    IPhysicsJointHost* joints_;  // null when the world cannot create joints
    sdk::BodyId body_;
    sdk::MassProperties base_;
    sdk::MassProperties composed_;
    IChassisMassAware* listener_ = nullptr;
    std::map<std::string, sdk::PayloadMass, std::less<>> payloads_;
    std::map<std::uint64_t, Constraint> constraints_;
    std::uint64_t nextHandle_ = 1;
};

}  // namespace wpiemu::core::physics
