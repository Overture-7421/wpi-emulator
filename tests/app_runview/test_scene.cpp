#include <doctest/doctest.h>

#include <QSignalSpy>

#include <cmath>

#include "common/orbit_camera.h"
#include "fake_run_backend.h"
#include "scene_model.h"
#include "scene_parts.h"
#include "test_support.h"
#include "wpiemu/sdk/testing/demo_game.h"

using namespace wpiemu::app::runview;
using namespace runview_test;
using wpiemu::app::HostScene;
namespace sdk = wpiemu::sdk;
namespace demo = wpiemu::sdk::testing::demo_game;

namespace {

std::shared_ptr<const sdk::RobotConfig> demoConfig() {
    return std::make_shared<const sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
}

const ScenePart* findPart(const RobotRig& rig, const std::string& name) {
    for (const auto& p : rig.parts())
        if (p.name == name) return &p;
    return nullptr;
}

// The demo intake with its arm raised 90 degrees about -Y.
sdk::MechanismState intakeArmRaised() {
    sdk::MechanismState intake;
    intake.id = "intake";
    intake.links.push_back({"arm", sdk::Pose3{{}, sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -1.5707963f)}});
    return intake;
}

// The demo turret turned 90 degrees about Z.
sdk::MechanismState turretQuarterTurn() {
    sdk::MechanismState turret;
    turret.id = "turret";
    turret.links.push_back({"yaw", sdk::Pose3{{}, sdk::Quat::fromYaw(1.5707963f)}});
    return turret;
}

}  // namespace

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
    CHECK(tire->shape == sdk::PrimitiveKind::Cylinder);
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
    d.mechanisms.push_back(intakeArmRaised());
    d.mechanisms.push_back(turretQuarterTurn());
    const sdk::SimSnapshot snap(d);
    rig.update(&snap);

    // arm.bar: local (0.17,0,0), arm raised 90 deg about -Y => straight up from the mount (0.38,0,-0.06).
    const ScenePart* bar = findPart(rig, "arm.bar");
    REQUIRE(bar != nullptr);
    CHECK(near(bar->pose.position, {0.38f, 0.f, -0.06f + 0.17f}));
    // turret base sits on the yaw axis, so turning about Z leaves it where it is
    CHECK(near(findPart(rig, "base")->pose.position, {-0.1f, 0.f, 0.03f}));
    // turret barrel at local (0.2,0,0.12) rotated 90 deg about Z => (0,0.2,0.12) from the mount (-0.1,0,0)
    CHECK(near(findPart(rig, "yaw.barrel")->pose.position, {-0.1f, 0.2f, 0.12f}));
}

TEST_CASE("every shape of a mechanism follows its first link, whatever the shape is called") {
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::MechanismConfig* intakeCfg = nullptr;
    for (auto& m : cfg->mechanisms)
        if (m.id == "intake") intakeCfg = &m;
    REQUIRE(intakeCfg != nullptr);
    sdk::PrimitiveSpec plate;
    plate.name = "plate";   // unrelated to the link name
    plate.local.position = {0.17f, 0.f, 0.f};
    intakeCfg->geometry.push_back(plate);

    RobotRig rig;
    rig.setConfig(cfg);
    sdk::SimSnapshotData d = baseSnapshot();
    d.mechanisms.push_back(intakeArmRaised());
    const sdk::SimSnapshot snap(d);
    rig.update(&snap);
    const ScenePart* part = findPart(rig, "plate");
    REQUIRE(part != nullptr);
    CHECK(near(part->pose.position, {0.38f, 0.f, -0.06f + 0.17f}));  // same as arm.bar
}

TEST_CASE("zone overlays resolve through plugin sections and mechanism properties") {
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::Box3 intakeBox;
    intakeBox.pose.position = {0.78f, 0.f, -0.08f};
    intakeBox.sizeM = {0.5f, 0.7f, 0.2f};
    cfg->pluginSection("demo_game").data["intake_zone"] = sdk::toValue(intakeBox);
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
    mech.mechanismTypes = {"mechanism"};
    mech.overlays.push_back({"zone", "Roller zone", "#ff0000", true});
    const auto mz = resolveZoneOverlays(*custom, {mech}, nullptr);
    REQUIRE(mz.size() == 1);
    CHECK(near(mz[0].pose.position, {0.38f + 0.1f, 0.f, -0.06f}));
}

TEST_CASE("robot zones resolve to chassis-frame poses with the kind's shape and colour") {
    const auto cfg = demoConfig();
    const auto kinds = demo::zoneKinds();
    const auto zones = resolveRobotZones(*cfg, kinds, nullptr);
    REQUIRE(zones.size() == 2);

    // Volume zone on the intake root: mount (0.38,0,-0.06) * local (0.4,0,-0.02)
    CHECK(zones[0].id == "pickup");
    CHECK(zones[0].label == "Intake zone");
    CHECK_FALSE(zones[0].muzzle);
    CHECK(zones[0].tracksState);
    CHECK(zones[0].color == kinds[0].color);
    CHECK(near(zones[0].pose.position, {0.78f, 0.f, -0.08f}));
    CHECK(near(zones[0].size, {0.5f, 0.7f, 0.2f}));

    // Muzzle zone: on the turret's yaw link at rest, direction along local +X
    CHECK(zones[1].muzzle);
    CHECK(zones[1].color == kinds[1].color);
    CHECK(near(zones[1].pose.position, {-0.1f + 0.38f, 0.f, 0.12f}));
}

TEST_CASE("robot zones follow the pose of the attached mechanism link") {
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::ZoneConfig onArm;
    onArm.id = "arm.zone";
    onArm.kind = demo::kPickupZone;
    onArm.attachment = {"intake", "arm"};
    onArm.local.position = {0.17f, 0.f, 0.f};
    cfg->zones.push_back(onArm);
    sdk::ZoneConfig onChassis;
    onChassis.id = "chassis.zone";
    onChassis.kind = demo::kPickupZone;
    onChassis.local.position = {0.1f, 0.2f, 0.3f};
    cfg->zones.push_back(onChassis);

    sdk::SimSnapshotData d = baseSnapshot();
    d.mechanisms.push_back(intakeArmRaised());
    d.mechanisms.push_back(turretQuarterTurn());
    const sdk::SimSnapshot snap(d);

    const auto zones = resolveRobotZones(*cfg, demo::zoneKinds(), &snap);
    REQUIRE(zones.size() == 4);
    CHECK(near(zones[2].pose.position, {0.38f, 0.f, -0.06f + 0.17f}));   // arm raised 90 deg, same as arm.bar
    CHECK(near(zones[3].pose.position, {0.1f, 0.2f, 0.3f}));              // chassis frame, no link involved
    // The muzzle turned with the turret: (0.38,0,0.12) rotated 90 deg about Z from the mount (-0.1,0,0)
    CHECK(near(zones[1].pose.position, {-0.1f, 0.38f, 0.12f}));
    CHECK(near(zones[1].pose.rotation.rotate({1.f, 0.f, 0.f}), {0.f, 1.f, 0.f}));
}

TEST_CASE("robot zones of an unknown kind are ignored") {
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    sdk::ZoneConfig other;
    other.id = "other";
    other.kind = "another_game.thing";
    cfg->zones.push_back(other);
    CHECK(resolveRobotZones(*cfg, demo::zoneKinds(), nullptr).size() == 2);
    CHECK(resolveRobotZones(*cfg, {}, nullptr).empty());
}

TEST_CASE("robot zone active flag comes from the snapshot zones") {
    const auto cfg = demoConfig();
    const auto kinds = demo::zoneKinds();
    for (const auto& z : resolveRobotZones(*cfg, kinds, nullptr)) CHECK_FALSE(z.active);

    sdk::SimSnapshotData d = baseSnapshot();
    d.zones.push_back({"shooter", true});
    d.zones.push_back({"pickup", false});
    const sdk::SimSnapshot snap(d);
    const auto zones = resolveRobotZones(*cfg, kinds, &snap);
    REQUIRE(zones.size() == 2);
    CHECK_FALSE(zones[0].active);
    CHECK(zones[1].active);

    // A zone missing from the snapshot is inactive
    sdk::SimSnapshotData partial = baseSnapshot();
    partial.zones.push_back({"pickup", true});
    const sdk::SimSnapshot partialSnap(partial);
    const auto some = resolveRobotZones(*cfg, kinds, &partialSnap);
    CHECK(some[0].active);
    CHECK_FALSE(some[1].active);
}

TEST_CASE("zone rig builds a marker and an arrow for muzzles and highlights active zones") {
    ZoneRig rig;
    rig.setScene(demoConfig(), {}, demo::zoneKinds());
    REQUIRE(rig.parts().size() == 3);   // pickup box, muzzle marker, muzzle arrow
    CHECK(rig.labels()[0] == "Intake zone");
    CHECK(rig.labels()[1] == "Shooter zone");
    CHECK(rig.labels()[2] == "Shooter zone");
    CHECK(near(rig.parts()[0].size, {0.5f, 0.7f, 0.2f}));
    // The arrow starts at the muzzle point and extends along local +X
    CHECK(rig.parts()[2].pose.position.x > rig.parts()[1].pose.position.x);
    CHECK(rig.parts()[2].size.x > rig.parts()[2].size.y);
    CHECK_FALSE(rig.parts()[0].highlighted);
    const float dim = rig.parts()[0].opacity;

    sdk::SimSnapshotData d = baseSnapshot();
    d.zones.push_back({"pickup", true});
    const sdk::SimSnapshot snap(d);
    CHECK(rig.update(&snap));
    CHECK(rig.changed()[0] == 1);
    CHECK(rig.changed()[1] == 0);
    CHECK(rig.parts()[0].highlighted);
    CHECK(rig.parts()[0].opacity > dim);
    CHECK_FALSE(rig.update(&snap));
}

TEST_CASE("field parts centre the floor on the visuals") {
    const auto field = demo::field();
    const FieldParts fp = buildFieldParts(&field);
    CHECK(fp.parts.size() == field.visuals.size() + 1);
    CHECK(fp.parts.front().name == "field.floor");
    CHECK(fp.center.x == doctest::Approx(field.lengthM / 2).epsilon(0.02));
    CHECK(fp.center.y == doctest::Approx(field.widthM / 2).epsilon(0.02));

    const FieldParts none = buildFieldParts(nullptr);
    CHECK(none.parts.empty());
    CHECK(none.lengthM > 0.f);
}

TEST_CASE("field parts frame the default FRC field around its centre") {
    const sdk::FieldDescription field = sdk::defaultFieldDescription();
    const FieldParts fp = buildFieldParts(&field);
    CHECK(fp.parts.size() == field.visuals.size() + 1);
    CHECK(fp.center.x == doctest::Approx(sdk::default_field::kLengthM / 2).epsilon(0.02));
    CHECK(fp.center.y == doctest::Approx(sdk::default_field::kWidthM / 2).epsilon(0.02));
    CHECK(fp.lengthM == sdk::default_field::kLengthM);
    CHECK(fp.widthM == sdk::default_field::kWidthM);
}

TEST_CASE("field parts keep opaque visuals opaque and draw see-through ones translucent") {
    sdk::FieldDescription field;
    field.lengthM = 10.f;
    field.widthM = 5.f;
    sdk::FieldVisual wall;
    wall.name = "wall";
    field.visuals.push_back(wall);
    sdk::FieldVisual glass;
    glass.name = "glass";
    glass.opacity = 0.25f;
    field.visuals.push_back(glass);

    const FieldParts fp = buildFieldParts(&field);
    REQUIRE(fp.parts.size() == 3);
    CHECK(fp.parts[1].name == "wall");
    CHECK_FALSE(fp.parts[1].translucent);
    CHECK(fp.parts[2].name == "glass");
    CHECK(fp.parts[2].translucent);
    CHECK(fp.parts[2].opacity == doctest::Approx(0.25f));

    PartListModel model;
    model.setParts(fp.parts);
    CHECK(model.data(model.index(1), PartListModel::ColorRole).value<QColor>().alphaF() == doctest::Approx(1.0));
    CHECK(model.data(model.index(2), PartListModel::ColorRole).value<QColor>().alphaF() == doctest::Approx(0.25));
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
    CHECK(scene.camera().pitchRad <= wpiemu::sdk::degToRadF(89.f) + 1e-4f);
    scene.orbit(0.f, -100000.f);
    CHECK(scene.camera().pitchRad >= wpiemu::sdk::degToRadF(-89.f) - 1e-4f);

    const sdk::Vec3 target = scene.camera().target;
    scene.setViewHeight(600.f);
    scene.pan(50.f, 0.f);
    CHECK(!near(scene.camera().target, target));

    scene.resetCamera();
    CHECK(scene.camera().distance == doctest::Approx(home));
    CHECK(near(scene.camera().target, target));
}

TEST_CASE("scene camera rig preserves driver views across scene changes and follows snapshot chassis poses") {
    SceneModel scene;
    auto field = std::make_shared<sdk::FieldDescription>();
    field->lengthM = 16.f;
    field->widthM = 8.f;
    field->driverStations.push_back({sdk::Alliance::Blue, 2, {-1.f, 4.f, 1.6f}});
    HostScene info;
    info.field = field;
    info.config = demoConfig();
    scene.setSceneInfo(info);
    scene.showStationView(sdk::Alliance::Blue, 2);
    CHECK(scene.cameraPosition().x() == doctest::Approx(-1.f));
    CHECK(scene.cameraPosition().y() == doctest::Approx(4.f));
    CHECK(scene.cameraPosition().z() == doctest::Approx(1.6f));
    QSignalSpy viewSpy(&scene, &SceneModel::cameraViewChanged);
    QSignalSpy cameraSpy(&scene, &SceneModel::cameraChanged);
    scene.setSceneInfo(info);
    CHECK(scene.cameraView().kind == CameraView::Kind::Station);
    CHECK(scene.cameraPosition().x() == doctest::Approx(-1.f));
    CHECK(viewSpy.count() == 0);

    sdk::SimSnapshotData data = baseSnapshot();
    data.chassis.pose.position = {10.f, 4.f, 0.f};
    const sdk::SimSnapshot snapshot(data);
    scene.update(&snapshot);
    scene.showDriverView();
    const QVector3D eye = scene.cameraPosition();
    CHECK(eye.x() < 0.f);
    const auto centered = scene.camera().project(data.chassis.pose.position, 800.f, 600.f);
    REQUIRE(centered);
    CHECK(centered->first == doctest::Approx(400.f).epsilon(1e-3));
    data.chassis.pose.position = {12.f, 6.f, 0.f};
    const sdk::SimSnapshot moved(data);
    scene.update(&moved);
    scene.advanceCamera(0.1);
    CHECK(cameraSpy.count() > 0);
    CHECK(viewSpy.count() == 1);

    scene.resetCamera();
    CHECK(scene.cameraView().kind == CameraView::Kind::Home);
    CHECK(viewSpy.count() == 2);
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
    cyl.shape = sdk::PrimitiveKind::Cylinder;
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
    scene.setSceneInfo(backend.scene());
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

TEST_CASE("scene model draws zones and updates their parts only when something changed") {
    SceneModel scene;
    HostScene info;
    info.config = demoConfig();
    info.zoneKinds = demo::zoneKinds();
    scene.setSceneInfo(info);
    PartListModel& zones = *static_cast<PartListModel*>(scene.zones());
    CHECK(zones.rowCount() == 3);   // pickup volume + muzzle marker + muzzle arrow
    CHECK(scene.zoneLabels() == QStringList{QStringLiteral("Intake zone"), QStringLiteral("Shooter zone")});
    const QColor dim = zones.index(0).data(PartListModel::ColorRole).value<QColor>();

    sdk::SimSnapshotData d = baseSnapshot();
    d.zones.push_back({"pickup", false});
    d.zones.push_back({"shooter", false});
    const sdk::SimSnapshot rest(d);
    QSignalSpy changed(&zones, &QAbstractItemModel::dataChanged);
    scene.update(&rest);
    CHECK(changed.count() == 0);   // rest pose, everything inactive: nothing to update
    scene.update(&rest);
    CHECK(changed.count() == 0);

    d.zones[0].active = true;
    const sdk::SimSnapshot pickupOn(d);
    scene.update(&pickupOn);
    REQUIRE(changed.count() == 1);   // only the pickup volume
    CHECK(changed.at(0).at(0).toModelIndex().row() == 0);
    const QColor lit = zones.index(0).data(PartListModel::ColorRole).value<QColor>();
    CHECK(lit.alphaF() > dim.alphaF());
    CHECK(lit.lightnessF() > dim.lightnessF());
    scene.update(&pickupOn);
    CHECK(changed.count() == 1);

    // The shooter follows its turret link: moving the link updates its marker and arrow only.
    sdk::MechanismState turret;
    turret.id = "turret";
    turret.links.push_back({"yaw", sdk::Pose3{{}, sdk::Quat::fromYaw(0.5f)}});
    d.mechanisms.push_back(turret);
    const sdk::SimSnapshot turned(d);
    scene.update(&turned);
    CHECK(changed.count() == 3);
    CHECK(changed.at(1).at(0).toModelIndex().row() == 1);
    CHECK(changed.at(2).at(0).toModelIndex().row() == 2);
}

TEST_CASE("fake backend publishes zone states that follow the intake arm and the flywheel") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    opts.startupDelayMs = 0;
    FakeRunBackend backend(opts);
    const HostScene info = backend.scene();
    CHECK(info.zoneKinds.size() == 2);
    CHECK(info.config->zones.size() == 2);

    backend.startSimulation();
    backend.tick(1);
    auto snap = backend.latestSnapshot();
    REQUIRE(snap != nullptr);
    REQUIRE(snap->zones().size() == 2);
    CHECK(snap->zones()[0].id == "pickup");
    CHECK_FALSE(snap->zones()[0].active);
    CHECK_FALSE(snap->zones()[1].active);

    sdk::proto::DsState ds;
    ds.enabled = true;
    ds.joysticks[0].buttons = 1u;   // A: deploy the intake
    backend.setDriverStation(ds);
    backend.tick(600);
    snap = backend.latestSnapshot();
    CHECK(snap->zones()[0].active);
    CHECK_FALSE(snap->zones()[1].active);

    ds.joysticks[0].buttons = 2u;   // B: spin up the flywheel
    backend.setDriverStation(ds);
    backend.tick(1000);
    snap = backend.latestSnapshot();
    CHECK_FALSE(snap->zones()[0].active);
    CHECK(snap->zones()[1].active);
}
