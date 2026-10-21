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

namespace wpiemu::app::ui {

// Edge length, in scene units, of Qt Quick 3D's built-in #Cube / #Cylinder / #Sphere / #Cone meshes.
inline constexpr float kBuiltinMeshSize = 100.f;

inline QVector3D toQt(const sdk::Vec3& v) { return {v.x, v.y, v.z}; }
inline QQuaternion toQt(const sdk::Quat& q) { return {q.w, q.x, q.y, q.z}; }

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
