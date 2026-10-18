#pragma once

// Scriptable IMechanismPhysicsCoupling for mechanism / plugin tests (SDK 1.1).
//  * Records what mechanisms submit (wrenches, constraints, payloads).
//  * Models the documented ordering: wrenches/constraints are SUBMITTED before the step and loads
//    are only visible after endStep(), i.e. constraintLoad() always reports the LAST COMPLETED step.
//  * flushTo() forwards the submissions to an IPhysicsWorld (chassis frame -> world frame) so the
//    FakePhysicsWorld moves; FakeSimEnvironment::advance() does this for you.
// Everything is supported by default; narrow it with setCapabilities() to test the unsupported path.

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/mechanisms/physics_coupling.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk::testing {

class FakeMechanismPhysicsCoupling final : public IMechanismPhysicsCoupling {
public:
    struct SubmittedWrench {
        Vec3 chassisLocalPoint;
        Wrench wrench;
    };

    // ---- test knobs ---------------------------------------------------------------------------
    void setCapabilities(CouplingCapabilities caps) { caps_ = caps; }
    // Base chassis mass used by flushTo() to compose payloads (unset: payloads are recorded only).
    void setChassisMass(const MassProperties& chassis) { chassisMass_ = chassis; }
    // The load constraint `h` will report after the NEXT endStep() (the physics step "produced" it).
    void scriptLoad(ConstraintHandle h, const ConstraintLoad& load) { scripted_[h.value] = load; }

    // Call right after the physics step: publishes this step's loads, clears per-step submissions.
    void endStep() {
        lastStepWrenches_ = std::move(pending_);
        pending_.clear();
        committed_.clear();
        for (const auto& [handle, desc] : constraints_) {
            ConstraintLoad load;
            const auto it = scripted_.find(handle);
            if (it != scripted_.end()) load = it->second;
            load.valid = true;
            committed_[handle] = load;
        }
        ++steps_;
    }

    // Hands the submissions to `world`: wrenches as world-frame forces/torques on `chassis`, payloads
    // composed into its mass properties (when setChassisMass() was called).
    void flushTo(IPhysicsWorld& world, BodyId chassis) const {
        if (!chassis.valid()) return;
        const Pose3 pose = world.pose(chassis);
        for (const SubmittedWrench& w : pending_) {
            world.applyForce(chassis, pose.transformVector(w.wrench.force), world.worldPoint(chassis, w.chassisLocalPoint));
            world.applyTorque(chassis, pose.transformVector(w.wrench.torque));
        }
        if (chassisMass_ && !payloads_.empty()) world.setMassProperties(chassis, composedMass(*chassisMass_));
    }

    // ---- inspection ---------------------------------------------------------------------------
    int stepsCompleted() const { return steps_; }
    const std::vector<SubmittedWrench>& pendingWrenches() const { return pending_; }
    const std::vector<SubmittedWrench>& lastStepWrenches() const { return lastStepWrenches_; }
    const std::map<std::uint64_t, AnchorConstraintDesc>& constraints() const { return constraints_; }
    std::size_t constraintCount() const { return constraints_.size(); }
    const std::map<std::string, PayloadMass, std::less<>>& payloads() const { return payloads_; }

    // Sum of the pending wrenches about the chassis COM (chassis frame): force, and couple + r x F.
    Wrench netPendingWrench() const {
        Wrench net;
        for (const SubmittedWrench& w : pending_) {
            net.force += w.wrench.force;
            net.torque += w.wrench.torque + cross(w.chassisLocalPoint, w.wrench.force);
        }
        return net;
    }
    MassProperties composedMass(const MassProperties& chassis) const {
        std::vector<PayloadMass> all;
        for (const auto& [owner, p] : payloads_) all.push_back(p);
        return composeMassProperties(chassis, all);
    }

    // ---- IMechanismPhysicsCoupling --------------------------------------------------------------
    CouplingCapabilities capabilities() const override { return caps_; }

    void applyWrench(const Vec3& chassisLocalPoint, const Wrench& wrench) override {
        if (caps_.wrench) pending_.push_back({chassisLocalPoint, wrench});
    }

    ConstraintHandle addConstraint(const AnchorConstraintDesc& desc) override {
        if (!caps_.constraints) return {};
        const bool finite = std::isfinite(desc.chassisLocalPoint.x + desc.chassisLocalPoint.y + desc.chassisLocalPoint.z +
                                          desc.fieldPoint.x + desc.fieldPoint.y + desc.fieldPoint.z);
        if (!finite) return {};
        if (desc.kind == AnchorKind::Prismatic && length(desc.axisWorld) <= 0.f) return {};
        const std::uint64_t handle = nextHandle_++;
        constraints_[handle] = desc;
        return ConstraintHandle{handle};
    }
    void removeConstraint(ConstraintHandle handle) override {
        constraints_.erase(handle.value);
        committed_.erase(handle.value);
        scripted_.erase(handle.value);
    }
    ConstraintLoad constraintLoad(ConstraintHandle handle) const override {
        const auto it = committed_.find(handle.value);
        return it == committed_.end() ? ConstraintLoad{} : it->second;
    }

    void setPayload(std::string_view owner, const PayloadMass& payload) override {
        if (!caps_.payload) return;
        if (payload.massKg <= 0.f) {
            clearPayload(owner);
            return;
        }
        payloads_[std::string(owner)] = payload;
    }
    void clearPayload(std::string_view owner) override {
        const auto it = payloads_.find(owner);
        if (it != payloads_.end()) payloads_.erase(it);
    }

private:
    CouplingCapabilities caps_{true, true, true};
    std::optional<MassProperties> chassisMass_;
    std::vector<SubmittedWrench> pending_;
    std::vector<SubmittedWrench> lastStepWrenches_;
    std::map<std::uint64_t, AnchorConstraintDesc> constraints_;
    std::map<std::uint64_t, ConstraintLoad> scripted_;
    std::map<std::uint64_t, ConstraintLoad> committed_;
    std::map<std::string, PayloadMass, std::less<>> payloads_;
    std::uint64_t nextHandle_ = 1;
    int steps_ = 0;
};

}  // namespace wpiemu::sdk::testing
