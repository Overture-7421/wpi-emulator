#include <doctest/doctest.h>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include "ui/scene_support.h"
#include "wpiemu/sdk/core/geometry.h"

namespace sdk = wpiemu::sdk;
namespace ui = wpiemu::app::ui;

namespace {

bool near(const QVector3D& a, const QVector3D& b) { return (a - b).length() < 1e-5f; }

}  // namespace

TEST_CASE("parseColor reads CSS colours and falls back for empty or invalid input") {
    CHECK(ui::parseColor("#ff8000") == QColor(255, 128, 0));
    CHECK(ui::parseColor("red") == QColor(Qt::red));
    CHECK(ui::parseColor("#80ff0000").alpha() == 0x80);
    CHECK(ui::parseColor("transparent").alpha() == 0);   // valid, so not replaced by the fallback

    const QColor fallback("#abcdef");
    CHECK(ui::parseColor("", fallback) == fallback);
    CHECK(ui::parseColor("not a colour", fallback) == fallback);
    CHECK(ui::parseColor("#12345", fallback) == fallback);
    CHECK(ui::parseColor("#ff8000", fallback) == QColor(255, 128, 0));
}

TEST_CASE("the default colour is the one an SDK primitive starts with") {
    CHECK(ui::defaultColor() == QColor(QString::fromStdString(sdk::PrimitiveSpec{}.color)));
    CHECK(ui::parseColor("") == ui::defaultColor());
    CHECK(ui::parseColor("nope") == ui::defaultColor());
}

TEST_CASE("toQt keeps SDK vectors and quaternions meaning the same rotation") {
    CHECK(ui::toQt(sdk::Vec3{1.f, -2.f, 3.f}) == QVector3D(1.f, -2.f, 3.f));

    // SDK quaternions are (x, y, z, w); Qt's are (scalar, x, y, z).
    const sdk::Quat q = sdk::Quat::fromAxisAngle(sdk::normalized({1.f, 2.f, 3.f}), 0.8f);
    const QQuaternion qt = ui::toQt(q);
    CHECK(qt.scalar() == q.w);
    CHECK(qt.x() == q.x);
    const sdk::Vec3 v{0.3f, -1.f, 2.f};
    CHECK(near(qt.rotatedVector(ui::toQt(v)), ui::toQt(q.rotate(v))));
}

TEST_CASE("mesh transforms draw boxes as they are and turn SDK cylinders onto the built-in Y axis") {
    const sdk::Quat pose = sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, 0.5f * sdk::kPiF);   // SDK +Z -> +X

    CHECK(ui::meshRotation(sdk::PrimitiveKind::Box, pose) == ui::toQt(pose));
    CHECK(near(ui::meshScale(sdk::PrimitiveKind::Box, {1.f, 2.f, 3.f}), QVector3D(0.01f, 0.02f, 0.03f)));

    // A cylinder's axis (SDK local Z) is the built-in mesh's Y axis, so its height scales Y and the diameter X and Z.
    CHECK(near(ui::meshScale(sdk::PrimitiveKind::Cylinder, {0.2f, 0.2f, 0.5f}), QVector3D(0.002f, 0.005f, 0.002f)));
    CHECK(near(ui::meshRotation(sdk::PrimitiveKind::Cylinder, sdk::Quat{}).rotatedVector({0.f, 1.f, 0.f}), QVector3D(0.f, 0.f, 1.f)));
    CHECK(near(ui::meshRotation(sdk::PrimitiveKind::Cylinder, pose).rotatedVector({0.f, 1.f, 0.f}), QVector3D(1.f, 0.f, 0.f)));
}
