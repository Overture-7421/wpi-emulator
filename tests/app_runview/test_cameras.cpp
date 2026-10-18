#include <doctest/doctest.h>

#include <QSignalSpy>

#include <cmath>
#include <set>

#include "fake_run_backend.h"
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

sdk::CameraState cameraState(const char* id, std::vector<int> tags) {
    sdk::CameraState c;
    c.id = id;
    c.tagsVisible = !tags.empty();
    c.visibleTagIds = std::move(tags);
    return c;
}

sdk::MechanismState turretAt(float yawRad) {
    sdk::MechanismState turret;
    turret.id = "turret";
    turret.links.push_back({"yaw", sdk::Pose3{{}, sdk::Quat::fromYaw(yawRad)}});
    return turret;
}

const CameraOverlay* findCamera(const std::vector<CameraOverlay>& cams, const std::string& id) {
    for (const auto& c : cams)
        if (c.id == id) return &c;
    return nullptr;
}

}  // namespace

TEST_CASE("camera rig: no cameras means no parts") {
    CameraRig rig;
    rig.setScene(nullptr);
    CHECK(rig.parts().empty());
    CHECK_FALSE(rig.update(nullptr));

    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    cfg->cameras.clear();
    rig.setScene(cfg);
    CHECK(rig.parts().empty());
    CHECK(rig.cameras().empty());
    CHECK_FALSE(rig.update(nullptr));
    CHECK(resolveRobotCameras(*cfg, nullptr).empty());

    SceneModel scene;
    RunSceneInfo info;
    info.config = cfg;
    scene.setSceneInfo(info);
    CHECK(static_cast<PartListModel*>(scene.simCameras())->rowCount() == 0);
    CHECK(static_cast<PartListModel*>(scene.tags())->rowCount() == 0);
}

TEST_CASE("camera rig builds a body and a frustum per camera sized by its field of view") {
    CameraRig rig;
    rig.setScene(demoConfig());
    REQUIRE(rig.cameras().size() == 2);
    REQUIRE(rig.parts().size() == 2 * CameraRig::kPartsPerCamera);
    CHECK(rig.labels().size() == rig.parts().size());
    CHECK(rig.labels()[0] == "Rear Limelight");
    CHECK(rig.labels()[CameraRig::kPartsPerCamera] == "Turret camera");

    const CameraOverlay& rear = rig.cameras()[0];
    CHECK(rear.kind == sdk::CameraKind::Limelight);
    CHECK(rig.cameras()[1].kind == sdk::CameraKind::PhotonVision);
    CHECK(rig.parts()[0].color != rig.parts()[CameraRig::kPartsPerCamera].color);   // colour per kind
    // The display length is capped by maxRangeM (5 m here is longer than the display length, 0.5 m is not).
    auto cfg = std::make_shared<sdk::RobotConfig>(FakeRunBackend::makeDemoConfig());
    cfg->cameras[0].maxRangeM = 0.5;
    CameraRig shortRange;
    shortRange.setScene(cfg);
    CHECK(shortRange.cameras()[0].frustumLengthM == doctest::Approx(0.5f));
    CHECK(rear.frustumLengthM > 0.5f);

    // Far-plane corners: the first four edges run from the apex to a corner at (L, +-L tan(h/2), +-L tan(v/2)).
    const float length = rear.frustumLengthM;
    const float halfWidth = length * std::tan(static_cast<float>(sdk::degToRad(75.0)) * 0.5f);
    const float halfHeight = length * std::tan(static_cast<float>(sdk::degToRad(50.0)) * 0.5f);
    const ScenePart& firstEdge = rig.parts()[1];
    const sdk::Vec3 camOrigin = rig.cameras()[0].pose.position;
    const sdk::Vec3 end = camOrigin + (firstEdge.pose.position - camOrigin) * 2.f;   // apex edge: centre is the midpoint
    const sdk::Vec3 far = rig.cameras()[0].pose.inverse().transformPoint(end);
    CHECK(far.x == doctest::Approx(length).epsilon(1e-3));
    CHECK(std::fabs(far.y) == doctest::Approx(halfWidth).epsilon(1e-3));
    CHECK(std::fabs(far.z) == doctest::Approx(halfHeight).epsilon(1e-3));
}

TEST_CASE("camera poses resolve in the chassis frame and follow the mechanism link they are mounted on") {
    const auto cfg = demoConfig();
    {
        const auto rest = resolveRobotCameras(*cfg, nullptr);
        const CameraOverlay* rear = findCamera(rest, "cam_rear");
        REQUIRE(rear != nullptr);
        CHECK(near(rear->pose.position, {-0.38f, 0.f, 0.1f}));
        const sdk::Vec3 forward = rear->pose.rotation.rotate({1.f, 0.f, 0.f});
        CHECK(forward.x < -0.9f);   // looks backwards, tilted slightly up
        CHECK(forward.z > 0.f);
        const CameraOverlay* turret = findCamera(rest, "cam_turret");
        REQUIRE(turret != nullptr);
        // Turret mount (-0.1,0,0) + local (0.1,0,0.22) at rest
        CHECK(near(turret->pose.position, {0.f, 0.f, 0.22f}));
    }

    sdk::SimSnapshotData d = baseSnapshot();
    d.mechanisms.push_back(turretAt(static_cast<float>(sdk::kPi) * 0.5f));
    const sdk::SimSnapshot turned(d);
    const auto cams = resolveRobotCameras(*cfg, &turned);
    const CameraOverlay* rear = findCamera(cams, "cam_rear");
    const CameraOverlay* turret = findCamera(cams, "cam_turret");
    REQUIRE((rear != nullptr && turret != nullptr));
    CHECK(near(rear->pose.position, {-0.38f, 0.f, 0.1f}));   // chassis-mounted: unaffected by the turret
    // Turret turned 90 deg: the local x offset swings onto +Y and the camera looks along +Y.
    CHECK(near(turret->pose.position, {-0.1f, 0.1f, 0.22f}));
    CHECK(turret->pose.rotation.rotate({1.f, 0.f, 0.f}).y > 0.9f);

    // The rig's parts follow the link too, and only the turret camera's parts move.
    CameraRig rig;
    rig.setScene(cfg);
    const ScenePart bodyAtRest = rig.parts()[CameraRig::kPartsPerCamera];
    CHECK(rig.update(&turned));
    CHECK(near(rig.cameras()[1].pose.position, turret->pose.position));
    CHECK_FALSE(near(rig.parts()[CameraRig::kPartsPerCamera].pose.position, bodyAtRest.pose.position));
    for (std::size_t i = 0; i < CameraRig::kPartsPerCamera; ++i) CHECK(rig.changed()[i] == 0);
    for (std::size_t i = CameraRig::kPartsPerCamera; i < rig.parts().size(); ++i) CHECK(rig.changed()[i] == 1);
    CHECK_FALSE(rig.update(&turned));

    // A camera on an unknown mechanism falls back to the chassis frame instead of failing.
    auto broken = std::make_shared<sdk::RobotConfig>(*cfg);
    broken->cameras[1].attachment = {"missing", "yaw"};
    const auto resolved = resolveRobotCameras(*broken, &turned);
    CHECK(near(resolved[1].pose.position, broken->cameras[1].local.position));
}

TEST_CASE("camera look follows the snapshot by camera id") {
    CameraRig rig;
    rig.setScene(demoConfig());
    CHECK_FALSE(rig.cameras()[0].active);
    const ScenePart dimEdge = rig.parts()[1];
    const ScenePart otherEdge = rig.parts()[CameraRig::kPartsPerCamera + 1];
    CHECK_FALSE(rig.parts()[0].highlighted);

    sdk::SimSnapshotData d = baseSnapshot();
    d.cameras.push_back(cameraState("cam_rear", {2}));
    d.cameras.push_back(cameraState("unknown_cam", {1}));   // ids that are not configured are ignored
    d.cameras.push_back(cameraState("cam_turret", {}));     // listed, but sees nothing
    const sdk::SimSnapshot snap(d);
    CHECK(rig.update(&snap));
    CHECK(rig.cameras()[0].active);
    CHECK_FALSE(rig.cameras()[1].active);
    CHECK(rig.parts()[0].highlighted);
    CHECK(rig.parts()[1].opacity > dimEdge.opacity);
    CHECK(rig.parts()[1].color != dimEdge.color);   // distinct "sees tags" colour
    CHECK(rig.parts()[CameraRig::kPartsPerCamera + 1].opacity == doctest::Approx(otherEdge.opacity));
    CHECK(rig.parts()[CameraRig::kPartsPerCamera + 1].color == otherEdge.color);
    for (std::size_t i = 0; i < CameraRig::kPartsPerCamera; ++i) CHECK(rig.changed()[i] == 1);
    for (std::size_t i = CameraRig::kPartsPerCamera; i < rig.parts().size(); ++i) CHECK(rig.changed()[i] == 0);
    CHECK_FALSE(rig.update(&snap));

    // Back to dim when the snapshot stops listing the camera as seeing tags.
    sdk::SimSnapshotData none = baseSnapshot();
    const sdk::SimSnapshot idle(none);
    CHECK(rig.update(&idle));
    CHECK_FALSE(rig.cameras()[0].active);
    CHECK(rig.parts()[1].opacity == doctest::Approx(dimEdge.opacity));
    CHECK(rig.parts()[1].color == dimEdge.color);
}

TEST_CASE("tag rig draws a plate per tag and highlights the union of the cameras' visible ids") {
    TagRig rig;
    rig.setLayout(FakeRunBackend::makeDemoTags());
    REQUIRE(rig.tags().size() == 8);
    CHECK(rig.parts().size() == 8 * TagRig::kPartsPerTag);
    // +X out of the face: the plate is thin along X and sits at the tag pose.
    const ScenePart& border = rig.parts()[0];
    CHECK(border.size.x < border.size.y);
    CHECK(near(border.pose.position, rig.tags()[0].pose.position));
    const std::string idleColor = border.color;

    sdk::SimSnapshotData d = baseSnapshot();
    d.cameras.push_back(cameraState("cam_rear", {1, 2}));
    d.cameras.push_back(cameraState("cam_turret", {2, 5}));
    const sdk::SimSnapshot snap(d);
    CHECK(rig.update(&snap));
    std::set<int> seen;
    for (const auto& t : rig.tags())
        if (t.seen) seen.insert(t.id);
    CHECK(seen == std::set<int>{1, 2, 5});
    for (std::size_t i = 0; i < rig.tags().size(); ++i) {
        const bool expect = seen.count(rig.tags()[i].id) != 0;
        CHECK(rig.tagChanged()[i] == (expect ? 1 : 0));
        CHECK(rig.changed()[i * TagRig::kPartsPerTag] == (expect ? 1 : 0));   // only the border recolours
        CHECK(rig.changed()[i * TagRig::kPartsPerTag + 1] == 0);
        CHECK((rig.parts()[i * TagRig::kPartsPerTag].color != idleColor) == expect);
    }
    CHECK_FALSE(rig.update(&snap));

    sdk::SimSnapshotData later = baseSnapshot();
    later.cameras.push_back(cameraState("cam_turret", {5}));
    const sdk::SimSnapshot snap2(later);
    CHECK(rig.update(&snap2));
    CHECK(rig.tagChanged()[0] == 1);   // tag 1 dropped out
    CHECK(rig.tagChanged()[1] == 1);
    CHECK(rig.tagChanged()[4] == 0);   // tag 5 stays seen

    // Tags a camera lists but the layout does not have are ignored.
    sdk::SimSnapshotData stray = baseSnapshot();
    stray.cameras.push_back(cameraState("cam_rear", {99}));
    const sdk::SimSnapshot snap3(stray);
    rig.update(&snap3);
    for (const auto& t : rig.tags()) CHECK_FALSE(t.seen);

    TagRig empty;
    empty.setLayout({});
    CHECK(empty.parts().empty());
    CHECK_FALSE(empty.update(&snap));
}

TEST_CASE("scene model emits changes only for cameras and tags that moved or switched state") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    FakeRunBackend backend(opts);
    SceneModel scene;
    scene.setSceneInfo(backend.sceneInfo());
    auto& cameras = *static_cast<PartListModel*>(scene.simCameras());
    auto& tags = *static_cast<PartListModel*>(scene.tags());
    auto& labels = *static_cast<TagLabelModel*>(scene.tagLabels());
    CHECK(cameras.rowCount() == static_cast<int>(2 * CameraRig::kPartsPerCamera));
    CHECK(tags.rowCount() == static_cast<int>(8 * TagRig::kPartsPerTag));
    REQUIRE(labels.rowCount() == 8);
    CHECK(labels.index(1).data(TagLabelModel::TextRole).toString() == QStringLiteral("2"));

    sdk::SimSnapshotData d = baseSnapshot();
    const sdk::SimSnapshot rest(d);
    QSignalSpy cameraSpy(&cameras, &QAbstractItemModel::dataChanged);
    QSignalSpy tagSpy(&tags, &QAbstractItemModel::dataChanged);
    QSignalSpy labelSpy(&labels, &QAbstractItemModel::dataChanged);
    scene.update(&rest);
    scene.update(&rest);
    CHECK(cameraSpy.count() == 0);
    CHECK(tagSpy.count() == 0);

    d.cameras.push_back(cameraState("cam_rear", {2}));
    const sdk::SimSnapshot seeing(d);
    scene.update(&seeing);
    CHECK(cameraSpy.count() == static_cast<int>(CameraRig::kPartsPerCamera));   // only the rear camera's parts
    CHECK(cameraSpy.at(0).at(0).toModelIndex().row() == 0);
    REQUIRE(tagSpy.count() == 1);
    CHECK(tagSpy.at(0).at(0).toModelIndex().row() == static_cast<int>(1 * TagRig::kPartsPerTag));
    REQUIRE(labelSpy.count() == 1);
    CHECK(labels.index(1).data(TagLabelModel::SeenRole).toBool());
    scene.update(&seeing);
    CHECK(cameraSpy.count() == static_cast<int>(CameraRig::kPartsPerCamera));
    CHECK(tagSpy.count() == 1);

    // Moving the turret link only touches the turret camera.
    d.mechanisms.push_back(turretAt(0.4f));
    const sdk::SimSnapshot turned(d);
    cameraSpy.clear();
    scene.update(&turned);
    REQUIRE(cameraSpy.count() == static_cast<int>(CameraRig::kPartsPerCamera));
    CHECK(cameraSpy.at(0).at(0).toModelIndex().row() == static_cast<int>(CameraRig::kPartsPerCamera));
}

TEST_CASE("scene model toggles cameras and tags independently") {
    SceneModel scene;
    CHECK(scene.simCamerasVisible());
    CHECK(scene.tagsVisible());
    QSignalSpy cameraSpy(&scene, &SceneModel::simCamerasVisibleChanged);
    QSignalSpy tagSpy(&scene, &SceneModel::tagsVisibleChanged);
    scene.setSimCamerasVisible(false);
    scene.setSimCamerasVisible(false);
    CHECK(cameraSpy.count() == 1);
    CHECK_FALSE(scene.simCamerasVisible());
    CHECK(scene.tagsVisible());
    scene.setTagsVisible(false);
    CHECK(tagSpy.count() == 1);
    CHECK_FALSE(scene.tagsVisible());
}

TEST_CASE("fake backend cameras see tags and the visible set changes as the robot moves") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    opts.startupDelayMs = 0;
    FakeRunBackend backend(opts);
    CHECK(backend.sceneInfo().tags.tags.size() == 8);
    CHECK(backend.sceneInfo().config->cameras.size() == 2);

    backend.startSimulation();
    backend.tick(1);
    auto snap = backend.latestSnapshot();
    REQUIRE(snap != nullptr);
    REQUIRE(snap->cameras().size() == 2);
    CHECK(snap->cameras()[0].id == "cam_rear");
    CHECK(snap->cameras()[0].tagsVisible);   // the blue wall tag behind the robot at the start
    CHECK(snap->cameras()[0].visibleTagIds == std::vector<int>{2});

    sdk::proto::DsState ds;
    ds.enabled = true;
    ds.mode = sdk::proto::RobotMode::Autonomous;
    backend.setDriverStation(ds);
    std::set<std::vector<int>> rearSets, turretSets;
    for (int i = 0; i < 3000; ++i) {
        backend.tick(1);
        snap = backend.latestSnapshot();
        rearSets.insert(snap->cameras()[0].visibleTagIds);
        turretSets.insert(snap->cameras()[1].visibleTagIds);
    }
    CHECK(rearSets.size() > 1);
    CHECK(turretSets.size() > 1);
}
