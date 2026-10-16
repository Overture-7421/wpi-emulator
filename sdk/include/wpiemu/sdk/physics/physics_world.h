#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk {

// Engine-agnostic rigid-body world (implemented by core over Box3D, ADR 0003; fakes in sdk/testing).
//
// Threading: the world is owned by the SIM THREAD. Only that thread may call any method. Other
// threads (UI) never touch it; they read ISimSnapshot copies instead.
//
// Frame/units: Z-up NWU, SI. All poses/vectors are world-frame unless the name says "Local".
//
// Lifetime & ids: BodyId/ShapeId are opaque; invalid (default) ids are tolerated by getters
// (neutral value returned: identity pose, zero vector, false) and ignored by setters. Destroying
// a body destroys its shapes. Do NOT create/destroy bodies while iterating the event spans:
// copy what you need, then mutate (see ADR 0003 notes).
//
// Forces: applyForce*/applyTorque are consumed by the NEXT step() only; re-apply every step.
//
// Determinism: step() is a pure function of the previous state, the applied forces and dt, for
// worker count 1. stateHash() makes that testable.
class IPhysicsWorld {
    WPIEMU_INTERFACE(IPhysicsWorld)
public:
    // ---- lifecycle -----------------------------------------------------------------------
    // Returns an invalid id on failure (e.g. bad descriptor).
    virtual BodyId createBody(const BodyDesc& desc) = 0;
    virtual void destroyBody(BodyId body) = 0;
    virtual ShapeId addShape(BodyId body, const ShapeDesc& desc) = 0;
    virtual void destroyShape(ShapeId shape) = 0;
    // Overrides shape-derived mass; call after all shapes were added.
    virtual void setMassProperties(BodyId body, const MassProperties& mp) = 0;
    virtual void setShapeFilter(ShapeId shape, std::uint64_t category, std::uint64_t mask) = 0;
    virtual void setShapeMaterial(ShapeId shape, const Material& material) = 0;

    // ---- stepping ------------------------------------------------------------------------
    // Advance by a fixed dt chosen by the caller (default 0.005 s). No internal accumulator.
    virtual void step(float dtSeconds) = 0;

    // ---- state ---------------------------------------------------------------------------
    virtual Pose3 pose(BodyId body) const = 0;
    virtual Vec3 linearVelocity(BodyId body) const = 0;
    virtual Vec3 angularVelocity(BodyId body) const = 0;  // world frame, rad/s
    virtual Vec3 pointVelocity(BodyId body, const Vec3& worldPoint) const = 0;
    virtual Vec3 worldPoint(BodyId body, const Vec3& localPoint) const = 0;
    virtual void setPose(BodyId body, const Pose3& pose) = 0;
    virtual void setLinearVelocity(BodyId body, const Vec3& v) = 0;
    virtual void setAngularVelocity(BodyId body, const Vec3& w) = 0;
    virtual void setEnabled(BodyId body, bool enabled) = 0;

    // ---- forces (this step only) ---------------------------------------------------------
    virtual void applyForce(BodyId body, const Vec3& force, const Vec3& worldPoint) = 0;
    virtual void applyForceToCenter(BodyId body, const Vec3& force) = 0;
    virtual void applyTorque(BodyId body, const Vec3& torque) = 0;
    virtual void applyImpulse(BodyId body, const Vec3& impulse, const Vec3& worldPoint) = 0;

    // ---- queries (thread-affine to the sim thread; read-only) ------------------------------
    // `direction` need not be normalised. `maskBits` is tested against shape category.
    virtual RayHit rayCastClosest(const Vec3& origin, const Vec3& direction, float maxDistance,
                                  std::uint64_t maskBits) const = 0;
    // Appends shapes overlapping the sphere to `out` (does not clear it).
    virtual void overlapSphere(const Vec3& center, float radius, std::uint64_t maskBits,
                               std::vector<ShapeId>& out) const = 0;
    // Appends the shapes currently inside `sensor` to `out` (does not clear it).
    virtual void shapesOverlapping(ShapeId sensor, std::vector<ShapeId>& out) const = 0;

    // ---- events of the LAST step; valid until the next step()/destroy ----------------------
    virtual std::span<const SensorEvent> sensorEvents() const = 0;
    virtual std::span<const ContactEvent> contactEvents() const = 0;

    // ---- diagnostics / introspection ---------------------------------------------------------
    virtual PhysicsStats stats() const = 0;
    // FNV-1a over pose + velocity of all bodies in creation order; equal hash <=> bit-identical run.
    virtual std::uint64_t stateHash() const = 0;
    virtual std::uint64_t shapeTag(ShapeId shape) const = 0;
    virtual BodyId shapeBody(ShapeId shape) const = 0;
    virtual bool isValid(ShapeId shape) const = 0;
    virtual bool isValid(BodyId body) const = 0;
};

// Composition-root hook: core registers the Box3D-backed factory; tests inject the fake.
using PhysicsWorldFactory = std::function<std::unique_ptr<IPhysicsWorld>(const PhysicsConfig&)>;

}  // namespace wpiemu::sdk
