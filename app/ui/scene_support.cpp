#include "ui/scene_support.h"

#include <QString>

#include <algorithm>
#include <utility>

namespace wpiemu::app::ui {

namespace {

constexpr float kNoseLengthM = 0.024f;
constexpr float kNoseMaxWidthM = 0.25f;
constexpr float kNoseMinHeightM = 0.02f;

}  // namespace

SceneBoxOrCylinder describePrimitive(const sdk::PrimitiveSpec& p) {
    return {p.kind == sdk::PrimitiveKind::Cylinder, p.local, p.extentsM(), p.color};
}

sdk::Pose3 chassisHullLocalPose(const sdk::ChassisSpec& ch) {
    return sdk::Pose3{sdk::Vec3{-ch.comOffsetM.x, -ch.comOffsetM.y, ch.groundClearanceM + ch.sizeM.z * 0.5f - ch.comHeightM},
                      sdk::Quat{}};
}

SceneBoxOrCylinder describeChassisNose(const sdk::ChassisSpec& ch, float zInBody, std::string color) {
    SceneBoxOrCylinder out;
    out.local = sdk::Pose3{sdk::Vec3{ch.sizeM.x * 0.5f + kNoseLengthM * 0.5f - ch.comOffsetM.x, -ch.comOffsetM.y, zInBody},
                           sdk::Quat{}};
    out.sizeM = sdk::Vec3{kNoseLengthM, std::min(kNoseMaxWidthM, ch.sizeM.y * 0.5f), std::max(kNoseMinHeightM, ch.sizeM.z * 0.5f)};
    out.color = std::move(color);
    return out;
}

sdk::Vec3 wheelCylinderSize(const sdk::WheelSpec& w) {
    const float d = std::max(w.radiusM, 0.01f) * 2.f;
    return {d, d, std::max(w.widthM, 0.005f)};
}

QColor defaultColor() { return QColor(QString::fromLatin1(sdk::kDefaultPrimitiveColor)); }

QColor parseColor(const std::string& css, const QColor& fallback) {
    const QColor c(QString::fromStdString(css));
    return c.isValid() ? c : fallback;
}

QQuaternion meshRotation(sdk::PrimitiveKind kind, const sdk::Quat& rotation) {
    static const QQuaternion kCylinderToZ = QQuaternion::fromAxisAndAngle(1.f, 0.f, 0.f, 90.f);
    return kind == sdk::PrimitiveKind::Cylinder ? toQt(rotation) * kCylinderToZ : toQt(rotation);
}

QVector3D meshScale(sdk::PrimitiveKind kind, const sdk::Vec3& size) {
    return kind == sdk::PrimitiveKind::Cylinder
               ? QVector3D(size.x / kBuiltinMeshSize, size.z / kBuiltinMeshSize, size.y / kBuiltinMeshSize)
               : QVector3D(size.x / kBuiltinMeshSize, size.y / kBuiltinMeshSize, size.z / kBuiltinMeshSize);
}

}  // namespace wpiemu::app::ui
