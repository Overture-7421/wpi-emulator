#include "wpiemu/core/physics/chassis_builder.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <variant>

namespace wpiemu::core::physics {
namespace {

using namespace sdk;

constexpr float kSettleMarginM = 0.002f;

bool positive(const Vec3& v) { return v.x > 0.f && v.y > 0.f && v.z > 0.f && std::isfinite(v.x + v.y + v.z); }

Vec3 hullCenterLocal(const ChassisSpec& c) {
    // Frame box centre in the COM-origin body frame: the COM is offset from the box centre by comOffsetM (xy).
    return {-c.comOffsetM.x, -c.comOffsetM.y, c.groundClearanceM + 0.5f * c.sizeM.z - c.comHeightM};
}

ShapeDesc robotShape(const ChassisSpec& chassis, std::uint64_t tag) {
    ShapeDesc d;
    d.material = chassis.material;
    d.density = 0.f;
    d.category = layer::Robot;
    d.mask = kRobotHullMask;
    d.sensorEvents = true;
    d.contactEvents = true;
    d.userTag = tag;
    return d;
}

}  // namespace

Vec3 wheelHubLocal(const ChassisSpec& chassis, const WheelSpec& wheel) {
    const float hubHeight = wheel.positionM.z > 0.f ? wheel.positionM.z : wheel.radiusM;
    return {wheel.positionM.x, wheel.positionM.y, hubHeight - chassis.comHeightM};
}

MassProperties chassisMassProperties(const ChassisSpec& c) {
    MassProperties mp;
    mp.massKg = c.massKg;
    mp.comLocal = {};
    const bool explicitInertia = c.inertiaDiagKgM2.x > 0.f && c.inertiaDiagKgM2.y > 0.f && c.inertiaDiagKgM2.z > 0.f;
    if (explicitInertia) {
        mp.inertiaDiagLocal = c.inertiaDiagKgM2;
        return mp;
    }
    const float k = c.massKg / 12.f;
    const Vec3 s = c.sizeM;
    const Vec3 d = hullCenterLocal(c);
    mp.inertiaDiagLocal = {k * (s.y * s.y + s.z * s.z) + c.massKg * (d.y * d.y + d.z * d.z),
                           k * (s.x * s.x + s.z * s.z) + c.massKg * (d.x * d.x + d.z * d.z),
                           k * (s.x * s.x + s.y * s.y) + c.massKg * (d.x * d.x + d.y * d.y)};
    return mp;
}

Result<ChassisBody> buildChassisBody(IPhysicsWorld& world, const ChassisSpec& chassis,
                                     const DrivetrainSpec& drivetrain, const ChassisPlacement& placement,
                                     const ChassisBuildOptions& options) {
    if (!(chassis.massKg > 0.f) || !std::isfinite(chassis.massKg))
        return makeError(ErrorCode::InvalidArgument, "chassis mass must be positive");
    if (!positive(chassis.sizeM)) return makeError(ErrorCode::InvalidArgument, "chassis size must be positive");
    if (!(chassis.comHeightM > 0.f) || !std::isfinite(chassis.comHeightM))
        return makeError(ErrorCode::InvalidArgument, "chassis COM height must be positive");
    for (const WheelSpec& w : drivetrain.wheels)
        if (!(w.radiusM > 0.f) || !std::isfinite(w.radiusM))
            return makeError(ErrorCode::InvalidArgument, "wheel '" + w.name + "' needs a positive radius");

    ChassisBody out;
    out.baseMass = chassisMassProperties(chassis);

    float lift = 0.f;  // how far the COM must sit above comHeightM so every pad touches the floor
    for (const WheelSpec& w : drivetrain.wheels) {
        out.wheelHubLocal.push_back(wheelHubLocal(chassis, w));
        lift = std::max(lift, w.radiusM - (w.positionM.z > 0.f ? w.positionM.z : w.radiusM));
    }
    out.restHeightM = chassis.comHeightM + lift;

    BodyDesc bd;
    bd.type = BodyType::Dynamic;
    bd.allowSleep = false;  // forces are applied every step anyway
    bd.pose = chassisSpawnPose(out, placement);
    bd.userTag = options.userTag;
    out.body = world.createBody(bd);
    if (!out.body.valid()) return makeError(ErrorCode::Internal, "physics world refused the chassis body");

    ShapeDesc hull = robotShape(chassis, options.userTag);
    hull.geometry = BoxShape{chassis.sizeM * 0.5f};
    hull.local = Pose3{hullCenterLocal(chassis), Quat::identity()};
    out.hull = world.addShape(out.body, hull);
    if (!out.hull.valid()) return makeError(ErrorCode::Internal, "physics world refused the chassis hull");

    for (const PrimitiveSpec& prim : chassis.primitives) {
        if (!prim.collides) continue;
        ShapeDesc d = robotShape(chassis, options.userTag);
        d.local = prim.local;
        if (prim.kind == PrimitiveKind::Box) {
            if (!positive(prim.sizeM)) return makeError(ErrorCode::InvalidArgument, "primitive '" + prim.name + "' has a non-positive size");
            d.geometry = BoxShape{prim.sizeM * 0.5f};
        } else {
            if (!(prim.radiusM > 0.f && prim.heightM > 0.f))
                return makeError(ErrorCode::InvalidArgument, "primitive '" + prim.name + "' has a non-positive size");
            d.geometry = CylinderShape{prim.radiusM, 0.5f * prim.heightM};
        }
        const ShapeId id = world.addShape(out.body, d);
        if (!id.valid()) return makeError(ErrorCode::Internal, "physics world refused primitive '" + prim.name + "'");
        out.extraShapes.push_back(id);
    }

    for (std::size_t i = 0; i < drivetrain.wheels.size(); ++i) {
        ShapeDesc pad;
        pad.geometry = SphereShape{drivetrain.wheels[i].radiusM};
        pad.local = Pose3{out.wheelHubLocal[i], Quat::identity()};
        pad.material = Material{0.f, 0.f, 0.f};  // zero friction: all traction comes from the tire model
        pad.density = 0.f;
        pad.category = layer::Robot;
        pad.mask = kWheelPadMask;
        pad.userTag = options.userTag;
        const ShapeId id = world.addShape(out.body, pad);
        if (!id.valid()) return makeError(ErrorCode::Internal, "physics world refused a wheel pad");
        out.wheelPads.push_back(id);
    }

    world.setMassProperties(out.body, out.baseMass);
    return out;
}

Pose3 chassisSpawnPose(const ChassisBody& body, const ChassisPlacement& placement) {
    return Pose3{{placement.xM, placement.yM, body.restHeightM + kSettleMarginM}, Quat::fromYaw(placement.yawRad)};
}

}  // namespace wpiemu::core::physics
