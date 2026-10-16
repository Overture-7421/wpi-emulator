#include <cmath>

#include "test_support.h"
#include "model/quat_utils.h"
#include "model/scene_builder.h"
#include "model/transform_access.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/orbit_camera.h"
#include "viewmodel/viewport_controller.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

// ---- camera and gizmo math --------------------------------------------------------------------------

TEST_CASE("orbit camera: basis is orthonormal and looks at the target") {
    OrbitCamera cam;
    const auto b = cam.basis();
    CHECK(approx(sdk::length(b.right), 1.f));
    CHECK(approx(sdk::length(b.up), 1.f));
    CHECK(approx(sdk::dot(b.right, b.up), 0.f));
    CHECK(approx(sdk::dot(b.forward, b.up), 0.f));
    CHECK(approx(sdk::dot(b.forward, b.right), 0.f));
    CHECK(sdk::dot(b.up, sdk::Vec3{0.f, 0.f, 1.f}) > 0.f);        // Z stays "up" on screen
    CHECK(approx(sdk::length(cam.position() - cam.target), cam.distance));
    CHECK(approx(b.forward, sdk::normalized(cam.target - cam.position())));
}

TEST_CASE("orbit camera: the quaternion maps -Z to the view direction and +Y to up") {
    OrbitCamera cam;
    cam.yawRad = 0.7f;
    cam.pitchRad = 0.4f;
    const auto q = cam.rotation();
    const auto b = cam.basis();
    CHECK(approx(q.rotate({0.f, 0.f, -1.f}), b.forward, 1e-3f));
    CHECK(approx(q.rotate({0.f, 1.f, 0.f}), b.up, 1e-3f));
    CHECK(approx(q.rotate({1.f, 0.f, 0.f}), b.right, 1e-3f));
}

TEST_CASE("orbit camera: rays and projection agree") {
    OrbitCamera cam;
    const float w = 800.f, h = 600.f;
    const Ray center = cam.rayAt(w / 2, h / 2, w, h);
    CHECK(approx(center.direction, cam.basis().forward, 1e-3f));

    const sdk::Vec3 point{0.4f, -0.2f, 0.5f};
    const auto px = cam.project(point, w, h);
    REQUIRE(px);
    const Ray ray = cam.rayAt(px->first, px->second, w, h);
    const sdk::Vec3 toPoint = point - ray.origin;
    const sdk::Vec3 closest = ray.origin + ray.direction * sdk::dot(toPoint, ray.direction);
    CHECK(sdk::length(closest - point) < 1e-3f);
    CHECK_FALSE(cam.project(cam.position() * 2.f - cam.target, w, h));   // behind the camera
}

TEST_CASE("orbit camera: orbit, pan, zoom and presets") {
    OrbitCamera cam;
    cam.orbit(0.f, 100000.f);
    CHECK(cam.pitchRad <= OrbitCamera::degToRad(89.f) + 1e-4f);
    cam.orbit(0.f, -100000.f);
    CHECK(cam.pitchRad >= OrbitCamera::degToRad(-89.f) - 1e-4f);
    for (int i = 0; i < 100; ++i) cam.zoom(5.f);
    CHECK(cam.distance >= 0.25f);
    for (int i = 0; i < 100; ++i) cam.zoom(-5.f);
    CHECK(cam.distance <= 60.f);

    OrbitCamera p;
    const sdk::Vec3 t0 = p.target;
    p.pan(50.f, 0.f, 600.f);
    CHECK(sdk::dot(p.target - t0, p.basis().right) < 0.f);   // dragging right drags the scene right = target left
    CHECK(approx(sdk::dot(p.target - t0, sdk::Vec3{0.f, 0.f, 1.f}), 0.f, 0.05f));

    p.preset("top");
    CHECK(p.basis().up.x > 0.95f);                            // +X points up on screen in the top view
    CHECK(p.basis().forward.z < -0.95f);
    p.preset("front");
    CHECK(p.position().x > p.target.x);                       // front view looks at the robot's nose
    CHECK(approx(p.position().z, p.target.z, 1e-3f));
    p.preset("side");
    CHECK(p.position().y < p.target.y);                       // right-hand side view

    p.frame({1.f, 0.f, 0.f}, 0.5f);
    CHECK(approx(p.target.x, 1.f));
    CHECK(p.distance > 0.5f);
}

TEST_CASE("gizmo math: closest point on an axis and plane intersection") {
    const Ray down{{1.f, 0.f, 5.f}, {0.f, 0.f, -1.f}};
    const auto t = closestParamOnAxis(down, {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f});
    REQUIRE(t);
    CHECK(approx(*t, 1.f));
    CHECK_FALSE(closestParamOnAxis({{0.f, 1.f, 0.f}, {1.f, 0.f, 0.f}}, {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}));   // parallel
    const auto hit = intersectPlane(down, {0.f, 0.f, 1.f}, {0.f, 0.f, 1.f});
    REQUIRE(hit);
    CHECK(approx(*hit, {1.f, 0.f, 1.f}));
    CHECK_FALSE(intersectPlane({{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}}, {0.f, 0.f, 1.f}, {0.f, 0.f, 1.f}));
    CHECK(approx(snapValue(0.037f, 0.0254f), 0.0254f));
    CHECK(approx(snapValue(0.04f, 0.0254f), 0.0508f));
    CHECK(approx(snapValue(0.123f, 0.f), 0.123f));
}

TEST_CASE("euler helpers round-trip") {
    const Euler e{0.2f, -0.4f, 1.1f};
    const Euler back = toEuler(fromEuler(e));
    CHECK(approx(back.rollRad, e.rollRad, 1e-4f));
    CHECK(approx(back.pitchRad, e.pitchRad, 1e-4f));
    CHECK(approx(back.yawRad, e.yawRad, 1e-4f));
}

// ---- scene -------------------------------------------------------------------------------------------

TEST_CASE("scene builder: hull, wheels, COM, mechanism shapes and overlays are all present") {
    const auto cat = catalog();
    const auto c = robot();
    const auto scene = buildScene(c, cat, SceneTheme::dark());
    CHECK(scene.find("hull") != nullptr);
    CHECK(scene.find("wheel:fl") != nullptr);
    CHECK(scene.find("com") != nullptr);
    CHECK(scene.find("chassis:0") != nullptr);
    CHECK(scene.find("mech:intake:0") != nullptr);
    CHECK(scene.find("overlay:intake:pickupZone") != nullptr);
    CHECK(scene.find("overlay::hopperVolume") != nullptr);
    CHECK(scene.find("link:turret:top") != nullptr);
    CHECK(scene.find("attach:shooter") != nullptr);   // line from the turret's top link to the shooter

    // Ids are unique (QML delegates and diffing rely on it).
    std::vector<std::string> ids;
    for (const auto& i : scene.items) ids.push_back(i.id);
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
}

TEST_CASE("scene builder: placement follows the display frame (floor at z = 0)") {
    const auto cat = catalog();
    const auto c = robot();
    const auto scene = buildScene(c, cat, SceneTheme::dark());
    const auto* hull = scene.find("hull");
    CHECK(approx(hull->world.position.z, c.chassis.groundClearanceM + c.chassis.sizeM.z * 0.5f));
    const auto* wheel = scene.find("wheel:fl");
    CHECK(approx(wheel->world.position, {0.28f, 0.28f, 0.0508f}));
    CHECK(wheel->shape == ItemShape::Cylinder);
    // The wheel axis (cylinder local Z) lies along the robot's Y axis.
    CHECK(approx(wheel->world.rotation.rotate({0.f, 0.f, 1.f}).y * wheel->world.rotation.rotate({0.f, 0.f, 1.f}).y, 1.f, 1e-3f));
    const auto* com = scene.find("com");
    CHECK(approx(com->world.position.z, c.chassis.comHeightM));
    // The shooter hangs on the turret's "top" link.
    const auto* shooter = scene.find("mech:shooter:0");
    const auto* turretLink = scene.find("link:turret:top");
    REQUIRE((shooter && turretLink));
    CHECK(shooter->world.position.z > turretLink->world.position.z - 0.2f);
}

TEST_CASE("scene builder: overlay opacity and picking flags") {
    const auto cat = catalog();
    const auto scene = buildScene(robot(), cat, SceneTheme::light());
    const auto* zone = scene.find("overlay:intake:pickupZone");
    CHECK(zone->opacity < 0.5f);
    CHECK(zone->role == ItemRole::Overlay);
    CHECK(zone->ref == SelectionRef::overlay("intake", "pickupZone"));
    CHECK(scene.find("nose")->ref.isNone());       // decorations are not pickable
    CHECK_FALSE(scene.find("nose")->pickable);
    CHECK(scene.find("wheel:fl")->pickable);
}

TEST_CASE("scene builder: link lines point from the parent to the child") {
    const auto q = rotationFromX({0.f, 1.f, 0.f});
    CHECK(approx(q.rotate({1.f, 0.f, 0.f}), {0.f, 1.f, 0.f}, 1e-3f));
    CHECK(approx(rotationFromX({1.f, 0.f, 0.f}).rotate({1.f, 0.f, 0.f}), {1.f, 0.f, 0.f}));
    CHECK(approx(rotationFromX({-1.f, 0.f, 0.f}).rotate({1.f, 0.f, 0.f}), {-1.f, 0.f, 0.f}, 1e-3f));
}

// ---- viewport controller ----------------------------------------------------------------------------

namespace {

struct ViewFixture {
    FakeEditorBackend backend;
    EditorController editor{backend};
    ViewportController view{editor};
    ViewFixture() {
        editor.setAutosaveDelayMs(0);
        view.setViewSize(800, 600);
        editor.settings().setSnapEnabled(false);
    }
    QPointF screen(const sdk::Vec3& world) {
        const auto px = view.camera().project(world, 800.f, 600.f);
        REQUIRE(px);
        return {static_cast<qreal>(px->first), static_cast<qreal>(px->second)};
    }
};

}  // namespace

TEST_CASE("viewport: the gizmo follows the selection and the allowed modes") {
    ViewFixture f;
    CHECK(f.view.gizmoHandles().isEmpty());                          // robot overview has no gizmo
    CHECK_FALSE(f.view.effectiveMode().has_value());

    f.editor.selection().set(SelectionRef::mechanism("intake"));
    REQUIRE(f.view.effectiveMode());
    CHECK(*f.view.effectiveMode() == GizmoMode::Move);
    CHECK(f.view.gizmoHandles().size() == 6);                        // shaft + tip per axis
    CHECK_FALSE(f.view.selectionEdges().isEmpty());

    f.view.setGizmoMode(GizmoMode::Rotate);
    CHECK(f.view.gizmoHandles().size() == 3 * 40);                   // a ring of segments per axis
    f.view.setGizmoMode(GizmoMode::Scale);
    CHECK(*f.view.effectiveMode() == GizmoMode::Move);               // mechanisms cannot scale: falls back

    f.editor.selection().set(SelectionRef::chassis());
    CHECK(*f.view.effectiveMode() == GizmoMode::Scale);              // the hull can only be resized
    f.editor.selection().set(SelectionRef::com());
    f.view.setGizmoMode(GizmoMode::Rotate);
    CHECK(*f.view.effectiveMode() == GizmoMode::Move);               // the COM marker only moves
    f.editor.selection().set(SelectionRef::slot("shooter", "flywheel"));
    CHECK(f.view.effectiveMode().has_value());                       // a slot manipulates its mechanism
    f.editor.selection().set(SelectionRef::slot("robot", "imu"));
    CHECK_FALSE(f.view.effectiveMode().has_value());
}

TEST_CASE("viewport: dragging an axis handle moves the selection and is one undo step") {
    ViewFixture f;
    f.editor.selection().set(SelectionRef::mechanism("intake"));
    f.view.setGizmoMode(GizmoMode::Move);
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::mechanism("intake"));
    REQUIRE(start);
    const sdk::Vec3 origin = start->world().position;
    const sdk::Vec3 axis = start->parentWorld.rotation.rotate({1.f, 0.f, 0.f});

    const QPointF p0 = f.screen(origin + axis * 0.2f);
    REQUIRE(f.view.beginGizmoDrag("h:m:0", p0.x(), p0.y()));
    for (int i = 1; i <= 5; ++i) {
        const QPointF p = f.screen(origin + axis * (0.2f + 0.05f * static_cast<float>(i)));
        f.view.updateGizmoDrag(p.x(), p.y(), false);
    }
    f.view.endGizmoDrag();

    const auto& mount = f.editor.document().config().findMechanism("intake")->mount;
    CHECK(approx(mount.position.x, start->transform.local.position.x + 0.25f, 2e-3f));
    CHECK(approx(mount.position.y, start->transform.local.position.y, 2e-3f));
    CHECK(f.editor.document().undoStack()->count() == 1);
    f.editor.document().undoStack()->undo();
    CHECK(approx(f.editor.document().config().findMechanism("intake")->mount.position.x, start->transform.local.position.x));
}

TEST_CASE("viewport: snapping rounds to the grid, Ctrl flips it") {
    ViewFixture f;
    f.editor.settings().setSnapEnabled(true);
    f.editor.selection().set(SelectionRef::wheel("fl"));
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::wheel("fl"));
    const sdk::Vec3 origin = start->world().position;
    const QPointF p0 = f.screen(origin + sdk::Vec3{0.2f, 0.f, 0.f});
    REQUIRE(f.view.beginGizmoDrag("h:m:0", p0.x(), p0.y()));
    const QPointF p1 = f.screen(origin + sdk::Vec3{0.2f + 0.0713f, 0.f, 0.f});
    f.view.updateGizmoDrag(p1.x(), p1.y(), false);
    const float snapped = ops::findWheel(f.editor.document().config(), "fl")->positionM.x;
    const float step = static_cast<float>(f.editor.settings().moveStepM());
    CHECK(approx(std::fmod(std::fabs(snapped) / step + 1e-3f, 1.f), 0.f, 0.01f));   // a multiple of one inch
    f.view.updateGizmoDrag(p1.x(), p1.y(), true);                                     // Ctrl: snapping off
    CHECK(approx(ops::findWheel(f.editor.document().config(), "fl")->positionM.x, 0.28f + 0.0713f, 2e-3f));
    f.view.endGizmoDrag();
}

TEST_CASE("viewport: rotating around Z turns the mechanism") {
    ViewFixture f;
    f.editor.selection().set(SelectionRef::mechanism("intake"));
    f.view.setGizmoMode(GizmoMode::Rotate);
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::mechanism("intake"));
    const sdk::Vec3 o = start->world().position;
    const QPointF p0 = f.screen(o + sdk::Vec3{0.3f, 0.f, 0.f});
    REQUIRE(f.view.beginGizmoDrag("h:r:2", p0.x(), p0.y()));
    const QPointF p1 = f.screen(o + sdk::Vec3{0.f, 0.3f, 0.f});   // a quarter turn counter-clockwise seen from above
    f.view.updateGizmoDrag(p1.x(), p1.y(), false);
    f.view.endGizmoDrag();
    const auto yaw = toEuler(f.editor.document().config().findMechanism("intake")->mount.rotation).yawRad;
    CHECK(approx(yaw, static_cast<float>(sdk::kPi * 0.5), 0.05f));
}

TEST_CASE("viewport: scale handles resize shapes symmetrically and respect a minimum") {
    ViewFixture f;
    f.editor.selection().set(SelectionRef::primitive("chassis", 0));   // front bumper, a box
    f.view.setGizmoMode(GizmoMode::Scale);
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::primitive("chassis", 0));
    REQUIRE(start);
    REQUIRE(start->transform.canScale);
    const sdk::Vec3 o = start->world().position;
    const sdk::Vec3 axis = sdk::normalized(start->world().rotation.rotate({0.f, 1.f, 0.f}));
    const QPointF p0 = f.screen(o + axis * 0.5f);
    REQUIRE(f.view.beginGizmoDrag("h:s:1", p0.x(), p0.y()));
    const QPointF p1 = f.screen(o + axis * 0.6f);
    f.view.updateGizmoDrag(p1.x(), p1.y(), false);
    CHECK(approx(f.editor.document().config().chassis.primitives[0].sizeM.y, start->transform.size.y + 0.2f, 5e-3f));
    const QPointF tiny = f.screen(o - axis * 1.f);
    f.view.updateGizmoDrag(tiny.x(), tiny.y(), false);
    CHECK(f.editor.document().config().chassis.primitives[0].sizeM.y >= 0.005f);
    f.view.endGizmoDrag();
}

TEST_CASE("viewport: picking prefers solid parts over volumes and drills down into shapes") {
    ViewFixture f;
    CHECK(f.view.chooseItem({"i:hull"}) == SelectionRef::chassis());
    CHECK(f.view.chooseItem({"i:overlay:intake:pickupZone", "i:hull"}) == SelectionRef::chassis());
    CHECK(f.view.chooseItem({"i:overlay:intake:pickupZone"}) == SelectionRef::overlay("intake", "pickupZone"));
    CHECK(f.view.chooseItem({"i:nose", "i:link:turret:top"}).isNone());
    CHECK(f.view.chooseItem({"h:m:0"}).isNone());

    // First click on a mechanism's shape selects the mechanism; the next one drills into the shape.
    CHECK(f.view.chooseItem({"i:mech:intake:0"}) == SelectionRef::mechanism("intake"));
    f.editor.selection().set(SelectionRef::mechanism("intake"));
    CHECK(f.view.chooseItem({"i:mech:intake:0"}) == SelectionRef::primitive("intake", 0));
    f.editor.selection().set(SelectionRef::primitive("intake", 0));
    CHECK(f.view.chooseItem({"i:mech:intake:1"}) == SelectionRef::primitive("intake", 1));
    CHECK(f.view.chooseItem({"i:mech:turret:0"}) == SelectionRef::mechanism("turret"));
}

TEST_CASE("viewport: pointer events select, deselect and drive the camera") {
    ViewFixture f;
    f.view.press(100, 100, Qt::LeftButton, 0, {"i:wheel:fl"});
    f.view.release(100, 100, Qt::LeftButton);
    CHECK(f.editor.selection().current() == SelectionRef::wheel("fl"));

    f.view.press(300, 300, Qt::LeftButton, 0, {});                  // click on empty space
    f.view.release(300, 300, Qt::LeftButton);
    CHECK(f.editor.selection().current() == SelectionRef::robot());

    const float yaw = f.view.camera().yawRad;
    f.view.press(300, 300, Qt::RightButton, 0, {});
    f.view.move(340, 300, Qt::RightButton, 0);
    f.view.release(340, 300, Qt::RightButton);
    CHECK(f.view.camera().yawRad != yaw);

    f.editor.selection().set(SelectionRef::mechanism("intake"));
    f.view.press(300, 300, Qt::LeftButton, 0, {});                  // drag on empty space orbits and keeps the selection
    f.view.move(360, 320, Qt::LeftButton, 0);
    f.view.release(360, 320, Qt::LeftButton);
    CHECK(f.editor.selection().current() == SelectionRef::mechanism("intake"));

    const float dist = f.view.camera().distance;
    f.view.wheel(120);
    CHECK(f.view.camera().distance < dist);
}

TEST_CASE("viewport: numeric entry edits the selection in the active mode") {
    ViewFixture f;
    f.editor.selection().set(SelectionRef::mechanism("intake"));
    f.view.setGizmoMode(GizmoMode::Move);
    auto fields = f.view.transformFields();
    REQUIRE(fields.valid);
    CHECK(fields.mode == GizmoMode::Move);
    CHECK(approx(fields.value.x, 0.5f));
    CHECK(f.view.setTransformComponent(0, 0.62));
    CHECK(approx(f.editor.document().config().findMechanism("intake")->mount.position.x, 0.62f));

    f.view.setGizmoMode(GizmoMode::Rotate);
    CHECK(f.view.setTransformComponent(2, 90.0));
    CHECK(approx(toEuler(f.editor.document().config().findMechanism("intake")->mount.rotation).yawRad, static_cast<float>(sdk::kPi * 0.5), 1e-3f));
    CHECK(approx(f.view.transformFields().value.z, 90.f, 0.05f));

    f.editor.selection().set(SelectionRef::chassis());
    f.view.setGizmoMode(GizmoMode::Scale);
    CHECK(f.view.setTransformComponent(0, 0.9));
    CHECK(approx(f.editor.document().config().chassis.sizeM.x, 0.9f));
    CHECK_FALSE(f.view.setTransformComponent(7, 1.0));
    f.editor.selection().set(SelectionRef::robot());
    CHECK_FALSE(f.view.transformFields().valid);
    CHECK_FALSE(f.view.setTransformComponent(0, 1.0));
}

TEST_CASE("viewport: dragging the COM marker keeps the rest of the robot where it is") {
    ViewFixture f;
    f.editor.selection().set(SelectionRef::com());
    const auto wheelBefore = ops::wheelWorld(f.editor.document().config().chassis, f.editor.document().config().drivetrain.spec.wheels[0]).position;
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::com());
    const sdk::Vec3 o = start->world().position;
    const QPointF p0 = f.screen(o + sdk::Vec3{0.15f, 0.f, 0.f});
    REQUIRE(f.view.beginGizmoDrag("h:m:0", p0.x(), p0.y()));
    const QPointF p1 = f.screen(o + sdk::Vec3{0.25f, 0.f, 0.f});
    f.view.updateGizmoDrag(p1.x(), p1.y(), false);
    f.view.endGizmoDrag();
    const auto& c = f.editor.document().config();
    CHECK(approx(c.chassis.comOffsetM.x, 0.1f, 3e-3f));
    CHECK(approx(ops::wheelWorld(c.chassis, c.drivetrain.spec.wheels[0]).position, wheelBefore, 1e-3f));
}

TEST_CASE("viewport: views, framing, theme and highlighting") {
    ViewFixture f;
    const auto before = f.view.camera().position();
    f.view.setView("top");
    CHECK(f.view.camera().position().z > before.z);
    f.view.resetCamera();
    f.editor.selection().set(SelectionRef::mechanism("climber"));
    f.view.frameSelection();
    const auto target = ops::mechanismRootWorld(f.editor.document().config(), f.editor.document().catalog(), "climber").position;
    CHECK(sdk::length(f.view.camera().target - target) < 0.5f);

    const QColor dark = f.view.backgroundColor();
    f.view.setTheme(SceneTheme::light());
    CHECK(f.view.backgroundColor() != dark);
    CHECK(f.view.backgroundColor().lightness() > 128);

    // Items of the selected mechanism are highlighted, others are not.
    const auto* mine = f.view.scene().itemById("mech:climber:0");
    const auto* other = f.view.scene().itemById("mech:intake:0");
    REQUIRE((mine && other));
    CHECK(f.view.scene().isHighlighted(*mine));
    CHECK_FALSE(f.view.scene().isHighlighted(*other));
    f.editor.selection().set(SelectionRef::slot("climber", "motor"));      // a slot highlights its mechanism
    CHECK(f.view.scene().isHighlighted(*f.view.scene().itemById("mech:climber:0")));
}

TEST_CASE("viewport: the scene model feeds QML with Z-up poses and cylinder axes") {
    ViewFixture f;
    auto& model = f.view.scene();
    REQUIRE(model.rowCount() > 10);
    int cylinders = 0;
    for (int r = 0; r < model.rowCount(); ++r) {
        const QModelIndex idx = model.index(r, 0);
        const QString source = model.data(idx, SceneModel::SourceRole).toString();
        const QVector3D scl = model.data(idx, SceneModel::SclRole).value<QVector3D>();
        CHECK(scl.x() > 0.f);
        CHECK(scl.y() > 0.f);
        CHECK(scl.z() > 0.f);
        if (source == "#Cylinder") ++cylinders;
        else CHECK(source == "#Cube");
    }
    CHECK(cylinders >= 4);   // the wheels
    const auto roles = model.roleNames();
    CHECK(roles.value(SceneModel::PosRole) == "pos");
    CHECK(roles.value(SceneModel::ColorRole) == "tint");
}

TEST_CASE("viewport: hints guide an empty robot") {
    FakeEditorBackend backend;
    auto empty = robot();
    empty.mechanisms.clear();
    backend.setConfig(empty);
    EditorController editor(backend);
    ViewportController view(editor);
    CHECK(view.hintText().contains("first mechanism"));
    editor.addMechanism("roller", {});
    CHECK(view.hintText().isEmpty());
}
