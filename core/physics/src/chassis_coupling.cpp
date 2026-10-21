#include "wpiemu/core/physics/chassis_coupling.h"

#include <cmath>
#include <vector>

namespace wpiemu::core::physics {
namespace {

using namespace sdk;

bool samePayload(const PayloadMass& a, const PayloadMass& b) {
    return a.massKg == b.massKg && a.comChassisLocal == b.comChassisLocal &&
           a.inertiaDiagAboutComKgM2 == b.inertiaDiagAboutComKgM2;
}

}  // namespace

ChassisPhysicsCoupling::ChassisPhysicsCoupling(IPhysicsWorld& world, BodyId chassis, const MassProperties& baseMass)
    : world_(world),
      joints_(dynamic_cast<IPhysicsJointHost*>(&world)),
      body_(chassis),
      base_(baseMass),
      composed_(baseMass) {}

ChassisPhysicsCoupling::~ChassisPhysicsCoupling() { dropConstraints(); }

void ChassisPhysicsCoupling::setMassListener(IChassisMassAware* listener) {
    listener_ = listener;
    if (listener_ != nullptr) listener_->setChassisMass(composed_);
}

CouplingCapabilities ChassisPhysicsCoupling::capabilities() const {
    CouplingCapabilities c;
    c.wrench = true;
    c.payload = true;
    c.constraints = joints_ != nullptr;
    return c;
}

void ChassisPhysicsCoupling::applyWrench(const Vec3& chassisLocalPoint, const Wrench& wrench) {
    if (!finite(chassisLocalPoint) || !finite(wrench.force) || !finite(wrench.torque) || !world_.isValid(body_)) return;
    const Quat rot = world_.pose(body_).rotation;
    world_.applyForce(body_, rot.rotate(wrench.force), world_.worldPoint(body_, chassisLocalPoint));
    world_.applyTorque(body_, rot.rotate(wrench.torque));
}

ConstraintHandle ChassisPhysicsCoupling::addConstraint(const AnchorConstraintDesc& desc) {
    if (joints_ == nullptr || !world_.isValid(body_) || !finite(desc.chassisLocalPoint) || !finite(desc.fieldPoint) ||
        !(desc.maxForceN > 0.f))
        return {};

    AnchoredJointDesc jd;
    jd.body = body_;
    jd.bodyLocalPoint = desc.chassisLocalPoint;
    jd.fieldPoint = desc.fieldPoint;
    switch (desc.kind) {
        case AnchorKind::Distance:
            if (std::isnan(desc.lengthM)) return {};
            jd.kind = JointKind::Distance;
            jd.lengthM = desc.lengthM;
            break;
        case AnchorKind::Prismatic:
            if (!finite(desc.axisWorld) || !(length(desc.axisWorld) > 1e-6f) || std::isnan(desc.minTravelM) ||
                std::isnan(desc.maxTravelM))
                return {};
            jd.kind = JointKind::Prismatic;
            jd.axisWorld = desc.axisWorld;
            jd.minTravelM = desc.minTravelM;
            jd.maxTravelM = desc.maxTravelM;
            break;
        case AnchorKind::ContactAnchor:
            jd.kind = JointKind::Spherical;
            break;
        default:
            return {};
    }
    const JointId joint = joints_->createAnchoredJoint(jd);
    if (!joint.valid()) return {};

    Constraint c;
    c.desc = desc;
    c.joint = joint;
    const std::uint64_t key = nextHandle_++;
    constraints_.emplace(key, std::move(c));
    return ConstraintHandle{key};
}

void ChassisPhysicsCoupling::removeConstraint(ConstraintHandle handle) {
    const auto it = constraints_.find(handle.value);
    if (it == constraints_.end()) return;
    if (joints_ != nullptr && it->second.joint.valid()) joints_->destroyJoint(it->second.joint);
    constraints_.erase(it);
}

ConstraintLoad ChassisPhysicsCoupling::constraintLoad(ConstraintHandle handle) const {
    const auto it = constraints_.find(handle.value);
    return it == constraints_.end() ? ConstraintLoad{} : it->second.load;
}

void ChassisPhysicsCoupling::setPayload(std::string_view owner, const PayloadMass& payload) {
    if (!(payload.massKg > 0.f) || !finite(payload.massKg) || !finite(payload.comChassisLocal) ||
        !finite(payload.inertiaDiagAboutComKgM2)) {
        clearPayload(owner);
        return;
    }
    const auto it = payloads_.find(owner);
    if (it != payloads_.end()) {
        if (samePayload(it->second, payload)) return;
        it->second = payload;
    } else {
        payloads_.emplace(std::string(owner), payload);
    }
    recomposeMass();
}

void ChassisPhysicsCoupling::clearPayload(std::string_view owner) {
    const auto it = payloads_.find(owner);
    if (it == payloads_.end()) return;
    payloads_.erase(it);
    recomposeMass();
}

void ChassisPhysicsCoupling::recomposeMass() {
    std::vector<PayloadMass> list;
    list.reserve(payloads_.size());
    for (const auto& [owner, p] : payloads_) list.push_back(p);
    composed_ = composeMassProperties(base_, list);
    world_.setMassProperties(body_, composed_);
    if (listener_ != nullptr) listener_->setChassisMass(composed_);
}

void ChassisPhysicsCoupling::onStepCompleted() {
    if (joints_ == nullptr) return;
    for (auto& [key, c] : constraints_) {
        if (!c.engaged) continue;
        const JointLoad jl = joints_->jointLoad(c.joint);
        if (!jl.valid) continue;
        c.load.valid = true;
        c.load.force = jl.force;
        c.load.torque = jl.torque;
        c.load.forceMagnitudeN = length(jl.force);
        c.load.travelM = jl.travelM;
        c.load.engaged = true;
        if (c.load.forceMagnitudeN > c.desc.maxForceN) {  // grip / anchor lets go
            joints_->destroyJoint(c.joint);
            c.joint = {};
            c.engaged = false;
            c.load.engaged = false;
        }
    }
}

void ChassisPhysicsCoupling::dropConstraints() {
    if (joints_ != nullptr)
        for (auto& [key, c] : constraints_)
            if (c.joint.valid()) joints_->destroyJoint(c.joint);
    constraints_.clear();
}

void ChassisPhysicsCoupling::reset() {
    dropConstraints();
    if (!payloads_.empty()) {
        payloads_.clear();
        recomposeMass();
    }
}

}  // namespace wpiemu::core::physics
