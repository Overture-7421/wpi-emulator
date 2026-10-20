#pragma once

// Physics layer types: engine-agnostic (no Box3D), SI units, Z-up (see core/math.h).
// Shape of the API follows docs/reference/physics.md.

#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class BodyType { Static, Kinematic, Dynamic };

// Collision categories form a 64-bit bitmask. Bits [0, 16) are reserved for core; year plugins
// define their own bits starting at `layer::kPluginBase` (document them in the plugin).
namespace layer {
inline constexpr std::uint64_t Field = 1ull << 0;
inline constexpr std::uint64_t Robot = 1ull << 1;
inline constexpr std::uint64_t Fuel = 1ull << 2;  // generic game-piece layer (name is historical; any piece may use it)
inline constexpr std::uint64_t Sensor = 1ull << 3;
inline constexpr std::uint64_t Projectile = 1ull << 4;
inline constexpr std::uint64_t kPluginBase = 1ull << 16;
inline constexpr std::uint64_t All = ~0ull;
}  // namespace layer

struct Material {
    float friction = 0.6f;
    float restitution = 0.f;
    float rollingResistance = 0.f;
};

struct BoxShape {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
};
struct SphereShape {
    float radius = 0.5f;
};
struct CylinderShape {
    float radius = 0.5f;
    float halfHeight = 0.5f;  // axis = local Z
};
// Static bodies only. The adapter COPIES the data during addShape; the pointers need not outlive the call.
struct MeshShape {
    const float* xyz = nullptr;           // vertexCount * 3 floats
    std::size_t vertexCount = 0;
    const std::uint32_t* triangles = nullptr;  // triangleCount * 3 indices
    std::size_t triangleCount = 0;
};

struct ShapeDesc {
    std::variant<BoxShape, SphereShape, CylinderShape, MeshShape> geometry{BoxShape{}};
    Pose3 local;                       // relative to the body
    Material material;
    float density = 0.f;               // kg/m^3; 0 for pads/sensors (use setMassProperties for the chassis)
    std::uint64_t category = layer::Field;
    std::uint64_t mask = layer::All;
    bool sensor = false;               // overlap volume only, no collision response
    bool sensorEvents = false;         // must be true on BOTH the sensor and the visitor shape
    bool contactEvents = false;
    std::uint64_t userTag = 0;         // returned in events / ray hits (entity id, wheel index, ...)
};

struct BodyDesc {
    BodyType type = BodyType::Static;
    Pose3 pose;
    Vec3 linearVelocity;
    bool allowSleep = true;
    bool bullet = false;
    std::uint64_t userTag = 0;
};

struct MassProperties {
    float massKg = 1.f;
    Vec3 comLocal;                // centre of mass in body frame
    Vec3 inertiaDiagLocal{1.f, 1.f, 1.f};  // kg m^2 about the principal axes in the body frame
};

struct RayHit {
    bool hit = false;
    Vec3 point;
    Vec3 normal;
    float fraction = 0.f;  // 0..1 along maxDistance
    ShapeId shape;
    std::uint64_t userTag = 0;
};

struct SensorEvent {
    ShapeId sensor;
    ShapeId visitor;
    std::uint64_t sensorTag = 0;
    std::uint64_t visitorTag = 0;
    bool begin = true;  // false => end
};

struct ContactEvent {
    ShapeId a;
    ShapeId b;
    std::uint64_t tagA = 0;
    std::uint64_t tagB = 0;
    Vec3 point;
    Vec3 normal;
    float approachSpeed = 0.f;
};

// Default gravity magnitude of the simulation (PhysicsConfig, the chassis IMU and the traction model all use it).
inline constexpr float kDefaultGravityMps2 = 9.81f;

struct PhysicsConfig {
    int workerCount = 1;  // 1 = deterministic default
    int subSteps = 4;     // at the 200 Hz fixed step (docs/reference/physics.md)
    bool sleeping = true;
    Vec3 gravity{0.f, 0.f, -kDefaultGravityMps2};
};

struct PhysicsStats {
    int bodies = 0;
    int awakeBodies = 0;
    int contacts = 0;
    double lastStepMs = 0.0;
};

}  // namespace wpiemu::sdk
