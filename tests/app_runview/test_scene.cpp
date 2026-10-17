#include <doctest/doctest.h>

#include <QSignalSpy>

#include <cmath>

#include "common/orbit_camera.h"
#include "fake_run_backend.h"
#include "mechanism_overlay_model.h"
#include "scene_model.h"
#include "scene_parts.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

constexpr float kEps = 1e-3f;

bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = kEps) { return sdk::length(a - b) < eps; }

std::shared_ptr<const sdk::RobotConfig> demoConfig() {
    return std::make_shared<const sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
}

sdk::SimSnapshotData baseSnapshot() {
    sdk::SimSnapshotData d;
    d.chassis.pose = {{3.f, 2.f, 0.2f}, sdk::Quat::fromYaw(0.f)};
    d.robotConnected = true;
    return d;
}

const ScenePart* findPart(const RobotRig& rig, const std::string& name) {
    for (const auto& p : rig.parts())
        if (p.name == name) return &p;
    return nullptr;
}

}  // namespace

TEST_CASE("primitive to link assignment uses the longest matching link name") {
    const std::vector<std::string> links{"arm", "arm.tip", "carriage"};
    CHECK(linkForPrimitive("arm.bar", links) == "arm");
    CHECK(linkForPrimitive("arm.tip.pad", links) == "arm.tip");
    CHECK(linkForPrimitive("arm", links) == "arm");
    CHECK(linkForPrimitive("armor", links) == "root");   // not a separator boundary
    CHECK(linkForPrimitive("base", links) == "root");
    CHECK(linkForPrimitive("carriage_plate", links) == "carriage");
}

TEST_CASE("robot rig builds chassis, wheel and mechanism parts") {
    RobotRig rig;
    rig.setConfig(demoConfig());
    // 6 chassis primitives + front marker, 4 wheels x (tire + marker), 7 mechanism primitives
    CHECK(rig.parts().size() == 7 + 8 + 7);
    CHECK(rig.staticCount() == 7);

    RobotRig empty;
    empty.setConfig(nullptr);
    CHECK(empty.parts().empty());
    CHECK_FALSE(empty.update(nullptr));
}

TEST_CASE("rig without chassis primitives draws a default frame box") {
    auto cfg = std::make_shared<sdk::RobotConfig>();
    RobotRig rig;
    rig.setConfig(cfg);
    REQUIRE(rig.parts().size() >= 1);
    CHECK(rig.parts()[0].name == "chassis");
    CHECK(rig.parts()[0].size.x == doctest::Approx(cfg->chassis.sizeM.x));
}

TEST_CASE("rig places wheels from the physics contact points") {
    RobotRig rig;
    rig.setConfig(demoConfig());
    sdk::SimSnapshotData d = baseSnapshot();
    sdk::WheelSample w;
    w.name = "fl";
    w.contactPoint = {3.3f, 2.3f, 0.f};
    w.steerAngleRad = 0.5;
    w.spinAngleRad = 1.0;
    d.wheels.push_back(w);
    const sdk::SimSnapshot snap(d);
    CHECK(rig.update(&snap));
    const ScenePart* tire = findPart(rig, "fl");
    REQUIRE(tire != nullptr);
    CHECK(near(tire->pose.position, {0.3f, 0.3f, 0.0508f - 0.2f}));
    CHECK(tire->shape == PartShape::Cylinder);
    // The cylinder axis (local Z) must point along the chassis Y axis (wheel axle) when steer = spin = 0.
    sdk::WheelSample straight = w;
    straight.steerAngleRad = 0.0;
    straight.spinAngleRad = 0.0;
    d.wheels = {straight};
    const sdk::SimSnapshot snap2(d);
    rig.update(&snap2);
    CHECK(near(findPart(rig, "fl")->pose.rotation.rotate({0.f, 0.f, 1.f}), {0.f, 1.f, 0.f}));
    // No change on identical input.
    CHECK_FALSE(rig.update(&snap2));
}

TEST_CASE("mechanism primitives follow their link pose") {
    RobotRig rig;
    rig.setConfig(demoConfig());

    sdk::SimSnapshotData d = baseSnapshot();
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.links.push_back({"arm", sdk::Pose3{{}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -1.5707963f)}});
    d.mechanisms.push_back(intake);
    sdk::MechanismState turret;
    turret.id = "turret";
    turret.links.push_back({"yaw", sdk::Pose3{{}, sdk::Quat::fromYaw(1.5707963f)}});
    d.mechanisms.push_back(turret);
    const sdk::SimSnapshot snap(d);
    rig.update(&snap);

    // arm.bar: local (0.17,0,0), arm raised 90 deg about -Y => straight up from the mount (0.38,0,-0.06).
    const ScenePart* bar = findPart(rig, "arm.bar");
    REQUIRE(bar != nullptr);
    CHECK(near(bar->pose.position, {0.38f, 0.f, -0.06f + 0.17f}));
    // turret base is on the root link: not rotated
    CHECK(near(findPart(rig, "base")->pose.position, {-0.1f, 0.f, 0.03f}));
    // turret barrel at local (0.2,0,0.12) rotated 90 deg about Z => (0,0.2,0.12) from the mount (-0.1,0,0)
    CHECK(near(findPart(rig, "yaw.barrel")->pose.position, {-0.1f, 0.2f, 0.12f}));
}

TEST_CASE("an explicit PrimitiveSpec::link overrides the name-prefix convention") {
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::MechanismConfig* intakeCfg = nullptr;
    for (auto& m : cfg->mechanisms)
        if (m.id == "intake") intakeCfg = &m;
    REQUIRE(intakeCfg != nullptr);
    sdk::PrimitiveSpec plate;
    plate.name = "plate";  // no link-name prefix: would stay on the root link without the explicit field
    plate.local.position = {0.17f, 0.f, 0.f};
    plate.link = "arm";
    intakeCfg->geometry.push_back(plate);

    RobotRig rig;
    rig.setConfig(cfg);
    sdk::SimSnapshotData d = baseSnapshot();
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.links.push_back({"arm", sdk::Pose3{{}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -1.5707963f)}});
    d.mechanisms.push_back(intake);
    const sdk::SimSnapshot snap(d);
    rig.update(&snap);
    const ScenePart* part = findPart(rig, "plate");
    REQUIRE(part != nullptr);
    CHECK(near(part->pose.position, {0.38f, 0.f, -0.06f + 0.17f}));  // same as arm.bar
}

TEST_CASE("zone overlays resolve through plugin sections and mechanism properties") {
    const auto cfg = demoConfig();
    sdk::EditorPanelDescriptor panel;
    panel.id = "demo_game.zones";
    panel.scope = sdk::PanelScope::PluginConfig;
    panel.overlays.push_back({"intake_zone", "Intake zone", "#22c55e", true});
    panel.overlays.push_back({"missing_key", "Nothing", "#ffffff", true});
    const auto zones = resolveZoneOverlays(*cfg, {panel}, nullptr);
    REQUIRE(zones.size() == 1);
    CHECK(zones[0].label == "Intake zone");
    CHECK(near(zones[0].pose.position, {0.78f, 0.f, -0.08f}));
    CHECK(near(zones[0].size, {0.5f, 0.7f, 0.2f}));

    // Mechanism-scope: the box lives in the properties of matching mechanisms, relative to their root frame.
    auto custom = std::make_shared<sdk::RobotConfig>(*cfg);
    sdk::Box3 box;
    box.pose.position = {0.1f, 0.f, 0.f};
    box.sizeM = {0.2f, 0.2f, 0.2f};
    custom->mechanisms[0].properties["zone"] = sdk::toValue(box);
    sdk::EditorPanelDescriptor mech;
    mech.id = "demo_game.mech";
    mech.scope = sdk::PanelScope::Mechanism;
    mech.mechanismTypes = {"roller"};
    mech.overlays.push_back({"zone", "Roller zone", "#ff0000", true});
    const auto mz = resolveZoneOverlays(*custom, {mech}, nullptr);
    REQUIRE(mz.size() == 1);
    CHECK(near(mz[0].pose.position, {0.38f + 0.1f, 0.f, -0.06f}));
}

TEST_CASE("field parts centre the floor on the visuals") {
    const auto field = FakeRunBackend::makeDemoField();
    const FieldParts fp = buildFieldParts(&field);
    CHECK(fp.parts.size() == field.visuals.size() + 1);
    CHECK(fp.parts.front().name == "field.floor");
    CHECK(fp.center.x == doctest::Approx(field.lengthM / 2).epsilon(0.02));
    CHECK(fp.center.y == doctest::Approx(field.widthM / 2).epsilon(0.02));

    const FieldParts none = buildFieldParts(nullptr);
    CHECK(none.parts.empty());
    CHECK(none.lengthM > 0.f);
}

TEST_CASE("run view camera uses the editor's orbit camera around a field-framed home view") {
    SceneModel scene;
    wpiemu::app::OrbitCamera editorLike;   // the editor viewport drives this same type
    const float home = scene.camera().distance;
    CHECK(home > 0.f);
    // Side-on home view: the camera sits on the -Y side, centred on the field length, above the floor.
    CHECK(scene.cameraPosition().x() == doctest::Approx(scene.fieldCenter().x()).epsilon(1e-3));
    CHECK(scene.cameraPosition().y() < scene.fieldCenter().y());
    CHECK(scene.cameraPosition().z() > 0.f);

    scene.zoom(2.f);
    CHECK(scene.camera().distance < home);
    editorLike.zoom(2.f);
    CHECK(scene.camera().distance / home == doctest::Approx(editorLike.distance / 2.8f));   // same zoom step

    scene.orbit(0.f, 100000.f);   // pitch is clamped like the editor's, never flipped over the pole
    CHECK(scene.camera().pitchRad <= wpiemu::app::OrbitCamera::degToRad(89.f) + 1e-4f);
    scene.orbit(0.f, -100000.f);
    CHECK(scene.camera().pitchRad >= wpiemu::app::OrbitCamera::degToRad(-89.f) - 1e-4f);

    const sdk::Vec3 target = scene.camera().target;
    scene.setViewHeight(600.f);
    scene.pan(50.f, 0.f);
    CHECK(!near(scene.camera().target, target));

    scene.resetCamera();
    CHECK(scene.camera().distance == doctest::Approx(home));
    CHECK(near(scene.camera().target, target));
}
TEST_CASE("part list model converts SDK primitives to Qt Quick 3D transforms") {
    PartListModel model;
    ScenePart box;
    box.name = "b";
    box.size = {1.f, 2.f, 3.f};
    box.pose.position = {1.f, 2.f, 3.f};
    box.color = "#ff0000";
    ScenePart cyl;
    cyl.name = "c";
    cyl.shape = PartShape::Cylinder;
    cyl.size = {0.2f, 0.2f, 0.5f};
    cyl.translucent = true;
    model.setParts({box, cyl});
    REQUIRE(model.rowCount() == 2);
    const QVector3D scale = model.index(0).data(PartListModel::ScaleRole).value<QVector3D>();
    CHECK(scale.x() == doctest::Approx(0.01f));
    CHECK(scale.z() == doctest::Approx(0.03f));
    CHECK(model.index(1).data(PartListModel::ShapeRole).toInt() == 1);
    const QVector3D cscale = model.index(1).data(PartListModel::ScaleRole).value<QVector3D>();
    CHECK(cscale.y() == doctest::Approx(0.005f));   // height maps to the mesh's Y axis
    const QColor translucent = model.index(1).data(PartListModel::ColorRole).value<QColor>();
    CHECK(translucent.alphaF() < 1.0);

    QSignalSpy spy(&model, &QAbstractItemModel::dataChanged);
    box.pose.position = {5.f, 5.f, 5.f};
    model.updatePoses({box, cyl}, {1, 0});
    CHECK(spy.count() == 1);
    CHECK(model.index(0).data(PartListModel::PositionRole).value<QVector3D>() == QVector3D(5.f, 5.f, 5.f));
}

TEST_CASE("entity group model instances every position and tracks structure") {
    EntityGroupModel model;
    sdk::EntityGroupSample fuel;
    fuel.group = "fuel";
    fuel.radiusM = 0.075f;
    for (int i = 0; i < 300; ++i) fuel.positions.push_back({static_cast<float>(i) * 0.01f, 0.f, 0.075f});
    const std::vector<sdk::EntityGroupSample> groups{fuel};
    model.update(groups);
    CHECK(model.rowCount() == 1);
    CHECK(model.totalInstances() == 300);
    CHECK(model.index(0).data(EntityGroupModel::SphereRole).toBool());

    fuel.positions.resize(120);
    model.update(std::vector<sdk::EntityGroupSample>{fuel});
    CHECK(model.totalInstances() == 120);

    model.update({});
    CHECK(model.rowCount() == 0);
}

TEST_CASE("scene model follows the snapshot and exposes zone toggles") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    opts.startupDelayMs = 0;
    FakeRunBackend backend(opts);
    SceneModel scene;
    scene.setSceneInfo(backend.sceneInfo());
    CHECK_FALSE(scene.robotVisible());
    CHECK(scene.zoneLabels().contains(QStringLiteral("Intake zone")));
    CHECK(scene.zoneLabels().contains(QStringLiteral("Shooter zone")));
    CHECK(scene.robotModel().rowCount() == 22);

    backend.startSimulation();
    backend.tick(10);
    const auto snap = backend.latestSnapshot();
    REQUIRE(snap != nullptr);
    scene.update(snap.get());
    CHECK(scene.robotVisible());
    CHECK(scene.entityModel().totalInstances() > 100);
    CHECK(scene.chassisPosition().x() == doctest::Approx(snap->chassis().pose.position.x));

    scene.setZoneLabelShown(QStringLiteral("Intake zone"), false);
    CHECK_FALSE(scene.zoneLabelShown(QStringLiteral("Intake zone")));
    CHECK(scene.zoneLabelShown(QStringLiteral("Shooter zone")));

    QSignalSpy spy(&scene, &SceneModel::cameraChanged);
    scene.zoom(3.f);
    CHECK(spy.count() == 1);
}

TEST_CASE("mechanism overlay model shows labels and human units") {
    MechanismOverlayModel model;
    model.setConfig(demoConfig());
    sdk::SimSnapshotData d = baseSnapshot();
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.type = "roller";
    intake.active = true;
    intake.values = {{"angle", 3.14159265358979, "rad"}, {"speed", 62.8318530718, "rad/s"}, {"height", 0.5, "m"}};
    d.mechanisms.push_back(intake);
    const sdk::SimSnapshot snap(d);
    model.update(&snap);
    REQUIRE(model.rowCount() == 1);
    CHECK(model.index(0).data(MechanismOverlayModel::LabelRole).toString() == QStringLiteral("Intake"));
    CHECK(model.index(0).data(MechanismOverlayModel::ActiveRole).toBool());
    const QString values = model.index(0).data(MechanismOverlayModel::ValuesRole).toString();
    CHECK(values.contains(QStringLiteral("180.0 °")));   // rad -> deg
    CHECK(values.contains(QStringLiteral("600.0 rpm")));    // rad/s -> rpm
    CHECK(values.contains(QStringLiteral("0.50 m")));

    model.update(nullptr);
    CHECK(model.rowCount() == 0);
}
