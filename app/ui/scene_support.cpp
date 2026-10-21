#include "ui/scene_support.h"

#include <QString>

namespace wpiemu::app::ui {

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
