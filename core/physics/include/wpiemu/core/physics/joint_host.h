#pragma once

// Anchored joints between a dynamic body and a point of the (static) field, used by the mechanism coupling
// (ADR 0008). Engine-agnostic: the Box3D world implements it, no Box3D type appears here. Not part of the
// SDK interface IPhysicsWorld because only the chassis coupling needs it; reach it with
// dynamic_cast<IPhysicsJointHost*>(&world).

#include <limits>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::core::physics {

using JointId = sdk::StrongId<struct JointIdTag>;  // invalid (0) = failure

enum class JointKind {
    Distance,   // rigid rod/rope of fixed length between the body point and the field point
    Prismatic,  // body point slides along an axis through the field point, rotation and other translations locked
    Spherical,  // body point pinned to the field point, rotation free
};

struct AnchoredJointDesc {
    JointKind kind = JointKind::Distance;
    sdk::BodyId body;                 // dynamic body
    sdk::Vec3 bodyLocalPoint;         // attachment point in the body frame
    sdk::Vec3 fieldPoint;             // world-frame anchor
    sdk::Vec3 axisWorld{0.f, 0.f, 1.f};  // Prismatic only, need not be unit length
    float lengthM = -1.f;             // Distance only; < 0 = current distance
    float minTravelM = 0.f;           // Prismatic only: limits relative to fieldPoint along the axis
    float maxTravelM = std::numeric_limits<float>::infinity();
};

struct JointLoad {
    bool valid = false;
    sdk::Vec3 force;    // N, world frame, applied TO THE BODY at its attachment point
    sdk::Vec3 torque;   // N m, world frame, about the body's centre of mass
    float travelM = 0.f;  // Prismatic: translation; Distance: current length; Spherical: distance to the anchor
};

class IPhysicsJointHost {
    WPIEMU_INTERFACE(IPhysicsJointHost)
public:
    // Invalid id for unusable descriptions (unknown body, non-finite values, zero prismatic axis).
    virtual JointId createAnchoredJoint(const AnchoredJointDesc& desc) = 0;
    // Idempotent; also releases the internal anchor.
    virtual void destroyJoint(JointId joint) = 0;
    // Reaction of the last completed step; JointLoad{} for unknown ids.
    virtual JointLoad jointLoad(JointId joint) const = 0;
};

}  // namespace wpiemu::core::physics
