#pragma once

#include <cmath>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/mechanisms/physics_coupling.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk::testing {

class FakeMechanismPhysicsCoupling final : public IMechanismPhysicsCoupling {
public:
    struct PendingWrench {
        Vec3 point;
        Wrench wrench;
    };

    CouplingCapabilities capabilities() const override { return capabilities_; }

    void applyWrench(const Vec3& chassisLocalPoint, const Wrench& wrench) override {
        if (capabilities_.wrench) pending_.push_back({chassisLocalPoint, wrench});
    }

    ConstraintHandle addConstraint(const AnchorConstraintDesc& desc) override {
        if (!capabilities_.constraints || !finite(desc.chassisLocalPoint) || !finite(desc.fieldPoint) ||
            !finite(desc.axisWorld) || (desc.kind == AnchorKind::Prismatic && length(desc.axisWorld) <= 1e-6f))
            return {};
        const ConstraintHandle handle{nextConstraint_++};
        constraints_[handle] = {};
        return handle;
    }

    void removeConstraint(ConstraintHandle handle) override {
        constraints_.erase(handle);
        scriptedLoads_.erase(handle);
    }

    ConstraintLoad constraintLoad(ConstraintHandle handle) const override {
        const auto it = constraints_.find(handle);
        return it == constraints_.end() ? ConstraintLoad{} : it->second;
    }

    void setPayload(std::string_view owner, const PayloadMass& payload) override {
        if (!capabilities_.payload) return;
        if (payload.massKg <= 0.f) {
            payloads_.erase(std::string(owner));
            return;
        }
        payloads_[std::string(owner)] = payload;
    }

    void clearPayload(std::string_view owner) override { payloads_.erase(std::string(owner)); }

    void setCapabilities(CouplingCapabilities capabilities) { capabilities_ = capabilities; }
    void scriptLoad(ConstraintHandle handle, const ConstraintLoad& load) {
        if (constraints_.count(handle) != 0) scriptedLoads_[handle] = load;
    }
    Wrench netPendingWrench() const {
        Wrench total;
        for (const PendingWrench& pending : pending_) {
            total.force += pending.wrench.force;
            total.torque += pending.wrench.torque + cross(pending.point, pending.wrench.force);
        }
        return total;
    }
    std::vector<Wrench> pendingWrenches() const {
        std::vector<Wrench> result;
        result.reserve(pending_.size());
        for (const PendingWrench& pending : pending_) result.push_back(pending.wrench);
        return result;
    }
    std::vector<PendingWrench> consumePendingWrenches() {
        std::vector<PendingWrench> result;
        result.swap(pending_);
        return result;
    }
    void completeStep() {
        for (const auto& [handle, load] : scriptedLoads_) {
            auto it = constraints_.find(handle);
            if (it != constraints_.end()) {
                it->second = load;
                it->second.valid = true;
            }
        }
        scriptedLoads_.clear();
    }
    void flushTo(IPhysicsWorld& world, BodyId chassis) {
        if (hasChassisMass_) world.setMassProperties(chassis, composedMass(chassisMass_));
        const Pose3 chassisPose = world.pose(chassis);
        for (const PendingWrench& pending : consumePendingWrenches()) {
            world.applyForce(chassis, pending.wrench.force, chassisPose.transformPoint(pending.point));
            world.applyTorque(chassis, pending.wrench.torque);
        }
    }
    void endStep() { completeStep(); }
    void setChassisMass(const MassProperties& mass) {
        chassisMass_ = mass;
        hasChassisMass_ = true;
    }
    const std::map<std::string, PayloadMass, std::less<>>& payloads() const { return payloads_; }
    MassProperties composedMass(const MassProperties& base) const {
        std::vector<PayloadMass> values;
        values.reserve(payloads_.size());
        for (const auto& [owner, payload] : payloads_) values.push_back(payload);
        return composeMassProperties(base, values);
    }
    std::size_t constraintCount() const { return constraints_.size(); }

private:
    CouplingCapabilities capabilities_{true, true, true};
    std::uint64_t nextConstraint_ = 1;
    std::vector<PendingWrench> pending_;
    std::map<ConstraintHandle, ConstraintLoad> constraints_;
    std::map<ConstraintHandle, ConstraintLoad> scriptedLoads_;
    std::map<std::string, PayloadMass, std::less<>> payloads_;
    MassProperties chassisMass_;
    bool hasChassisMass_ = false;
};

}  // namespace wpiemu::sdk::testing
