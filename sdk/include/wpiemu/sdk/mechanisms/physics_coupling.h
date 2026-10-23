#pragma once

// Mechanism <-> chassis physics coupling (docs/ARCHITECTURE.md).
//
// Mechanisms own their local plants (IPlant) but some of them push on, or are pushed by, the robot
// chassis: a climber hooking a field point, an elevator whose carriage load shifts the COM, an
// intake wedging against a wall. IPhysicsWorld::applyForce* is body-level and needs BodyIds; this
// service is the mechanism-facing, chassis-centric view of it, reached through SimContext::coupling().
//
// What it offers
//  * applyWrench      a force + couple on the chassis at a chassis-local point (this step only).
//  * addConstraint    an anchored constraint between a chassis-local point and a FIELD point
//                     (prismatic / distance / contact anchor) with reaction-load reporting.
//  * setPayload       payload mass / COM / inertia that the runtime composes with the chassis
//                     (parallel-axis theorem, composeMassProperties) before the physics step.
//
// Ordering inside one tick (see docs/ARCHITECTURE.md)
//   1. mechanisms tick: they SUBMIT wrenches, add/remove constraints, update payloads; they may READ
//      constraintLoad(), which is the load of the LAST COMPLETED physics step (tick N-1 here);
//   2. the runtime composes payloads into the chassis MassProperties, hands wrenches to the world and
//      (re)creates constraints;
//   3. drivetrain + physics step;
//   4. the runtime stores the step's constraint reactions: from now on constraintLoad() returns tick N's.
// Submissions made AFTER the physics step (game state, onTick) take effect on the NEXT step.
//
// Lifetime of submissions
//   wrench       consumed by the next step only (like IPhysicsWorld::applyForce); re-submit each tick.
//   constraint   persistent until removeConstraint(). The runtime drops every constraint and payload
//                on a session reset; mechanisms re-register from IMechanism::reset()/the next tick.
//   payload      latched per owner until changed or cleared; the runtime drops them on reset.
//
// Support is optional: a coupling that cannot do something returns the documented neutral value
// (invalid handle, no effect). Check capabilities() when a mechanism needs a feature to be correct.
// Frames/units: SI; the chassis frame of core/math.h (+X forward, +Y left, +Z up, origin = chassis COM);
// field points are world (field) frame. Sim thread only.

#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk {

using ConstraintHandle = StrongId<struct ConstraintHandleTag>;  // invalid (0) = unsupported / failed

// Force and couple, both in the CHASSIS frame. The couple is applied in addition to the moment of
// `force` about the chassis COM implied by the application point.
struct Wrench {
    Vec3 force;    // N
    Vec3 torque;   // N m
};

enum class AnchorKind : std::uint8_t {
    Distance,       // keeps |anchor - fieldPoint| at `lengthM` (rope, cable, tether)
    Prismatic,      // slides along `axisWorld` through fieldPoint, locks the other translations
    ContactAnchor,  // friction/stick contact with a field feature (grabbed rung, wall wedge); `maxForceN` bounds the grip
};

struct AnchorConstraintDesc {
    AnchorKind kind = AnchorKind::Distance;
    std::string owner;                  // mechanism id (diagnostics, bulk removal on reset)
    Vec3 chassisLocalPoint;             // attachment on the chassis, chassis frame
    Vec3 fieldPoint;                    // anchor on the field, world frame
    Vec3 axisWorld{0.f, 0.f, 1.f};      // Prismatic only: slide axis (need not be unit length)
    float lengthM = -1.f;               // Distance only: target length; < 0 = distance at registration
    float minTravelM = 0.f;             // Prismatic only: slide limits along the axis from fieldPoint
    float maxTravelM = std::numeric_limits<float>::infinity();
    float maxForceN = std::numeric_limits<float>::infinity();  // breaks/slips above this; inf = rigid
    std::uint64_t userTag = 0;
};

// Reaction of one constraint in the last completed physics step. Force/torque are what the
// constraint applied TO THE CHASSIS (world frame, torque about the chassis COM).
struct ConstraintLoad {
    bool valid = false;        // false: invalid handle, unsupported, or no step completed since it was added
    bool engaged = false;      // still enforced (false once broken/slipped/removed by the world)
    Vec3 force;                // N
    Vec3 torque;               // N m
    float forceMagnitudeN = 0.f;
    float travelM = 0.f;       // Prismatic: current slide; Distance: current length
};

// Payload carried by the chassis, per owner.
struct PayloadMass {
    float massKg = 0.f;
    Vec3 comChassisLocal;                    // payload COM in the chassis frame
    Vec3 inertiaDiagAboutComKgM2;            // principal moments about its own COM, axes parallel to the chassis axes
};

struct CouplingCapabilities {
    bool wrench = false;
    bool constraints = false;
    bool payload = false;
};

class IMechanismPhysicsCoupling {
    WPIEMU_INTERFACE(IMechanismPhysicsCoupling)
public:
    virtual CouplingCapabilities capabilities() const = 0;

    // Force + couple on the chassis at `chassisLocalPoint`, consumed by the next step. Additive: several
    // calls in one tick sum. No-op when unsupported.
    virtual void applyWrench(const Vec3& chassisLocalPoint, const Wrench& wrench) = 0;

    // Registers an anchored constraint. Returns an invalid handle when unsupported or the description is
    // unusable (zero prismatic axis, non-finite points). The caller removes it (idempotent).
    virtual ConstraintHandle addConstraint(const AnchorConstraintDesc& desc) = 0;
    virtual void removeConstraint(ConstraintHandle handle) = 0;
    // Load of the last completed step; ConstraintLoad{} (valid == false) for unknown handles.
    virtual ConstraintLoad constraintLoad(ConstraintHandle handle) const = 0;

    // Sets/replaces the payload of `owner` (a mechanism id). massKg <= 0 clears it.
    virtual void setPayload(std::string_view owner, const PayloadMass& payload) = 0;
    virtual void clearPayload(std::string_view owner) = 0;
};

// Default service for contexts without a physics-backed coupling (SimContext::coupling() falls back to it).
class NullMechanismPhysicsCoupling final : public IMechanismPhysicsCoupling {
public:
    CouplingCapabilities capabilities() const override { return {}; }
    void applyWrench(const Vec3&, const Wrench&) override {}
    ConstraintHandle addConstraint(const AnchorConstraintDesc&) override { return {}; }
    void removeConstraint(ConstraintHandle) override {}
    ConstraintLoad constraintLoad(ConstraintHandle) const override { return {}; }
    void setPayload(std::string_view, const PayloadMass&) override {}
    void clearPayload(std::string_view) override {}
};

// What the runtime sets as the chassis body's mass properties: chassis + payloads, about the combined
// COM, inertia by the parallel-axis theorem (diagonal terms only; off-diagonal products are neglected).
inline MassProperties composeMassProperties(const MassProperties& chassis, std::span<const PayloadMass> payloads) {
    float mass = chassis.massKg;
    Vec3 weighted = chassis.comLocal * chassis.massKg;
    for (const PayloadMass& p : payloads) {
        if (p.massKg <= 0.f) continue;
        mass += p.massKg;
        weighted += p.comChassisLocal * p.massKg;
    }
    if (mass <= 0.f) return chassis;

    MassProperties out;
    out.massKg = mass;
    out.comLocal = weighted / mass;

    auto add = [&](float m, const Vec3& com, const Vec3& inertiaAboutOwnCom) {
        const Vec3 d = com - out.comLocal;
        out.inertiaDiagLocal.x += inertiaAboutOwnCom.x + m * (d.y * d.y + d.z * d.z);
        out.inertiaDiagLocal.y += inertiaAboutOwnCom.y + m * (d.x * d.x + d.z * d.z);
        out.inertiaDiagLocal.z += inertiaAboutOwnCom.z + m * (d.x * d.x + d.y * d.y);
    };
    out.inertiaDiagLocal = {};
    add(chassis.massKg, chassis.comLocal, chassis.inertiaDiagLocal);
    for (const PayloadMass& p : payloads)
        if (p.massKg > 0.f) add(p.massKg, p.comChassisLocal, p.inertiaDiagAboutComKgM2);
    return out;
}

}  // namespace wpiemu::sdk
