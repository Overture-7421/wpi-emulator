#pragma once

// Qt side of the 3D scenes the editor and the run view both feed from SDK data: conversions from SDK types, the
// conventions of Qt Quick 3D's built-in meshes and colour parsing. SDK data stays Z-up NWU; the one conversion to
// Qt's Y-up space is the ZUpWorld node in qml/ZUpWorld.qml, never an adapter here.

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <string>

#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/physics/chassis_spec.h"

namespace wpiemu::app::ui {

// Edge length, in scene units, of Qt Quick 3D's built-in #Cube / #Cylinder / #Sphere / #Cone meshes.
inline constexpr float kBuiltinMeshSize = 100.f;

// +90 deg X-rotation aligning a local-Z cylinder with a wheel's Y axle.
inline const sdk::Quat kWheelAxisXToY = sdk::Quat::fromAxisAngle({1.f, 0.f, 0.f}, sdk::kPiF * 0.5f);

inline QVector3D toQt(const sdk::Vec3& v) { return {v.x, v.y, v.z}; }
inline QQuaternion toQt(const sdk::Quat& q) { return {q.w, q.x, q.y, q.z}; }

struct SceneBoxOrCylinder {
    bool cylinder = false;
    sdk::Pose3 local;
    sdk::Vec3 sizeM;
    std::string color;
};

// Converts an SDK primitive (Box or Cylinder with 2*radiusM diameter and heightM) into its local pose, extents and colour.
SceneBoxOrCylinder describePrimitive(const sdk::PrimitiveSpec& p);

// Chassis-frame local pose of the fallback hull box (origin at COM).
sdk::Pose3 chassisHullLocalPose(const sdk::ChassisSpec& ch);

// Front nose-marker box dimensions and chassis-frame local pose at `zInBody` on the front of the hull.
SceneBoxOrCylinder describeChassisNose(const sdk::ChassisSpec& ch, float zInBody, std::string color = {});

// Clamped cylinder extents (diameter, diameter, width) for a wheel.
sdk::Vec3 wheelCylinderSize(const sdk::WheelSpec& w);

// Rest position of a wheel hub in the chassis frame (origin at COM).
inline sdk::Vec3 wheelHubLocalPosition(const sdk::ChassisSpec& ch, const sdk::WheelSpec& w) {
    return {w.positionM.x, w.positionM.y, w.positionM.z - ch.comHeightM};
}

// The colour of sdk::kDefaultPrimitiveColor.
QColor defaultColor();

// A CSS colour name or "#rrggbb" string, `fallback` when it is empty or not a colour.
QColor parseColor(const std::string& css, const QColor& fallback = defaultColor());

// Orientation and scale that make a built-in mesh draw a primitive of `kind` with the SDK pose rotation `rotation`
// and full extents `size`. SDK cylinders have their axis on local Z and are sized (diameter, diameter, height);
// #Cylinder has its axis on Y, so it is turned a quarter about X and its Y and Z scales swap.
QQuaternion meshRotation(sdk::PrimitiveKind kind, const sdk::Quat& rotation);
QVector3D meshScale(sdk::PrimitiveKind kind, const sdk::Vec3& size);

}  // namespace wpiemu::app::ui
