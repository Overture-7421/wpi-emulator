#include <doctest/doctest.h>

#include <QQuick3DGeometry>
#include <QSignalSpy>
#include <cmath>

#include "camera_rig.h"
#include "fake_run_backend.h"
#include "scene_model.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

constexpr float kPi = static_cast<float>(sdk::kPi);

bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = 1e-3f) {
    return sdk::length(a - b) < eps;
}

sdk::FiducialSample tag(int id, sdk::Vec3 position, float normalYaw) {
    sdk::FiducialSample t;
    t.id = id;
    t.pose = {position, sdk::Quat::fromYaw(normalYaw)};
    t.sizeM = 0.1651f;
    return t;
}

sdk::CameraSample camera(const char* id, sdk::Vec3 position, std::vector<int> visible) {
    sdk::CameraSample c;
    c.id = id;
    c.label = id;
    c.pose = {position, sdk::Quat{}};
    c.hfovRad = 1.4;
    c.vfovRad = 0.97;
    c.widthPx = 1280;
    c.heightPx = 800;
    c.robotConnected = true;
    c.visibleTagIds = std::move(visible);
    return c;
}

// Three tags on the far wall (ids 1-2, facing the robot) and the side wall (id 3), and a camera
// that sees the first two.
sdk::SimSnapshotData visionSnapshot() {
    sdk::SimSnapshotData d;
    d.chassis.pose = {{3.f, 2.f, 0.2f}, sdk::Quat{}};
    d.robotConnected = true;
    d.fiducials = {tag(1, {10.f, 2.f, 1.f}, kPi), tag(2, {10.f, 4.f, 1.f}, kPi),
                   tag(3, {6.f, 0.f, 1.f}, kPi * 0.5f)};
    d.cameras = {camera("front", {3.3f, 2.f, 0.4f}, {1, 2})};
    return d;
}

}  // namespace

TEST_CASE("vision overlay: a camera is drawn by its state, enabled and robot connection") {
    sdk::CameraSample c = camera("front", {}, {});
    CHECK(cameraStateOf(c) == CameraState::Ok);
    c.robotConnected = false;
    CHECK(cameraStateOf(c) == CameraState::Waiting);
    c.status = "photon_uuid_unsupported";
    CHECK(cameraStateOf(c) == CameraState::Problem);
    c.status = "foreign_publisher";
    CHECK(cameraStateOf(c) == CameraState::Problem);
    c.status = "disabled";
    CHECK(cameraStateOf(c) == CameraState::Disabled);
    c.status.clear();
    c.enabled = false;
    CHECK(cameraStateOf(c) == CameraState::Disabled);  // switched off wins over everything else
}

TEST_CASE("vision overlay: the rig builds cameras, tag plates and a ray pool from the snapshot") {
    CameraRig rig;
    CHECK_FALSE(rig.update(nullptr));
    CHECK(rig.cameras().empty());

    const sdk::SimSnapshot snap(visionSnapshot());
    CHECK(rig.update(&snap));
    CHECK(rig.camerasRebuilt());
    CHECK(rig.tagsRebuilt());

    REQUIRE(rig.cameras().size() == 1);
    CHECK(rig.cameras()[0].state == CameraState::Ok);
    CHECK(rig.cameras()[0].seesTags);
    CHECK(near(rig.cameras()[0].pose.position, {3.3f, 2.f, 0.4f}));

    // One plate per tag, thin along the tag's normal and sitting proud of the surface it is on.
    REQUIRE(rig.tagParts().size() == 3);
    const ScenePart& plate = rig.tagParts()[0];
    CHECK(plate.size.x == doctest::Approx(kTagPlateThicknessM));
    CHECK(plate.size.y == doctest::Approx(0.1651f));
    CHECK(near(plate.pose.position,
               {10.f - kTagPlateThicknessM * 0.5f, 2.f, 1.f}));  // the wall tag faces -X

    // One ray per (camera, tag), camera-major, visible only for the tags the camera sees.
    REQUIRE(rig.rayParts().size() == 3);
    CHECK(rig.rayParts()[0].visible);
    CHECK(rig.rayParts()[1].visible);
    CHECK_FALSE(rig.rayParts()[2].visible);

    // Labels: the three tag ids, then the camera.
    REQUIRE(rig.labels().size() == 4);
    CHECK(rig.labels()[0].text == "1");
    CHECK(rig.labels()[0].kind == LabelKind::Tag);
    CHECK(rig.labels()[3].text == "front");
    CHECK(rig.labels()[3].kind == LabelKind::Camera);
}

TEST_CASE("vision overlay: tags a camera sees are highlighted, the others stay dim") {
    CameraRig rig;
    sdk::SimSnapshotData d = visionSnapshot();
    const sdk::SimSnapshot both(d);
    rig.update(&both);
    CHECK(rig.tagParts()[0].highlighted);
    CHECK(rig.tagParts()[1].highlighted);
    CHECK_FALSE(rig.tagParts()[2].highlighted);
    CHECK(rig.tagParts()[0].opacity > rig.tagParts()[2].opacity);
    CHECK(rig.labels()[0].emphasized);
    CHECK_FALSE(rig.labels()[2].emphasized);

    // Another camera sees tag 3: highlighted although the first camera does not see it.
    d.cameras.push_back(camera("rear", {3.f, 2.f, 0.4f}, {3}));
    const sdk::SimSnapshot two(d);
    rig.update(&two);
    CHECK(rig.tagParts()[2].highlighted);

    // Nobody sees tag 1 any more.
    d.cameras[0].visibleTagIds = {2};
    const sdk::SimSnapshot lost(d);
    rig.update(&lost);
    CHECK_FALSE(rig.tagParts()[0].highlighted);
    CHECK(rig.tagsChanged()[0] == 1);
    CHECK(rig.tagsChanged()[1] == 0);
}

TEST_CASE("vision overlay: a ray runs from the camera to the centre of the tag") {
    CameraRig rig;
    sdk::SimSnapshotData d;
    d.fiducials = {tag(1, {3.f, 4.f, 0.f}, kPi), tag(2, {0.f, 0.f, 2.f}, 0.f)};
    d.cameras = {camera("c", {0.f, 0.f, 0.f}, {1, 2})};
    const sdk::SimSnapshot snap(d);
    rig.update(&snap);

    const ScenePart& flat = rig.rayParts()[0];  // 3-4-5 in the floor plane
    CHECK(flat.size.x == doctest::Approx(5.f));
    CHECK(near(flat.pose.position, {1.5f, 2.f, 0.f}));
    CHECK(near(flat.pose.rotation.rotate({1.f, 0.f, 0.f}), {0.6f, 0.8f, 0.f}));

    const ScenePart& up = rig.rayParts()[1];  // straight up
    CHECK(up.size.x == doctest::Approx(2.f));
    CHECK(near(up.pose.rotation.rotate({1.f, 0.f, 0.f}), {0.f, 0.f, 1.f}));
    CHECK(flat.size.y == doctest::Approx(kRayThicknessM));
}

TEST_CASE("vision overlay: update reports only what moved and rebuilds when the shape changes") {
    CameraRig rig;
    sdk::SimSnapshotData d = visionSnapshot();
    const sdk::SimSnapshot first(d);
    rig.update(&first);
    CHECK_FALSE(rig.update(&first));  // identical input: nothing to report
    CHECK_FALSE(rig.camerasRebuilt());
    CHECK_FALSE(rig.tagsRebuilt());

    // The camera moves 10 cm: the camera, its two rays and its label change; the tags do not.
    d.cameras[0].pose.position.x += 0.1f;
    const sdk::SimSnapshot moved(d);
    CHECK(rig.update(&moved));
    CHECK_FALSE(rig.camerasRebuilt());
    CHECK(rig.camerasChanged()[0] == 1);
    CHECK(rig.tagsChanged() == std::vector<char>{0, 0, 0});
    CHECK(rig.raysChanged() == std::vector<char>{1, 1, 0});  // the unseen ray is not recomputed
    CHECK(rig.labelsChanged() == std::vector<char>{0, 0, 0, 1});

    // A different field of view is a different frustum: rebuild instead of updating row by row.
    d.cameras[0].hfovRad = 1.0;
    const sdk::SimSnapshot narrower(d);
    CHECK(rig.update(&narrower));
    CHECK(rig.camerasRebuilt());
    CHECK_FALSE(rig.tagsRebuilt());

    CHECK(rig.update(nullptr));  // the run ended
    CHECK(rig.cameras().empty());
    CHECK(rig.tagParts().empty());
    CHECK(rig.rayParts().empty());
    CHECK(rig.labels().empty());
}

TEST_CASE(
    "vision overlay: labels hide behind the camera, off screen and for far tags nobody sees") {
    wpiemu::app::OrbitCamera eye;
    eye.target = {0.f, 0.f, 0.f};
    eye.yawRad = 0.f;  // on +X, looking back at the origin
    eye.pitchRad = 0.f;
    eye.distance = 5.f;
    const float w = 800.f, h = 600.f;

    OverlayLabel label;
    label.text = "7";
    label.kind = LabelKind::Tag;
    label.emphasized = false;
    label.anchor = {0.f, 0.f, 0.f};
    const LabelPlacement centre = placeLabel(label, eye, w, h);
    REQUIRE(centre.visible);
    CHECK(centre.xPx == doctest::Approx(w * 0.5f).epsilon(0.01));
    CHECK(centre.yPx == doctest::Approx(h * 0.5f).epsilon(0.01));

    label.anchor = {20.f, 0.f, 0.f};  // behind the camera
    CHECK_FALSE(placeLabel(label, eye, w, h).visible);
    label.anchor = {0.f, 30.f, 0.f};  // far to the side: off screen
    CHECK_FALSE(placeLabel(label, eye, w, h).visible);

    label.anchor = {-3.f, 0.f,
                    0.f};  // 8 m from the viewer: farther than the range for a tag nobody sees
    CHECK_FALSE(placeLabel(label, eye, w, h).visible);
    label.emphasized = true;  // but a tag a camera sees is always labelled
    CHECK(placeLabel(label, eye, w, h).visible);
    label.anchor = {-15.f, 0.f, 0.f};  // 20 m away
    CHECK(placeLabel(label, eye, w, h).visible);
    label.emphasized = false;
    label.kind = LabelKind::Camera;  // a camera is labelled while the viewer is within 12 m
    CHECK_FALSE(placeLabel(label, eye, w, h).visible);
    label.anchor = {-3.f, 0.f, 0.f};
    CHECK(placeLabel(label, eye, w, h).visible);

    label.anchor = {0.f, 0.f, 0.f};
    label.kind = LabelKind::Tag;
    eye.distance = 3.f;  // close enough: the dim label of an unseen tag shows
    CHECK(placeLabel(label, eye, w, h).visible);
}

TEST_CASE(
    "vision overlay: the scene model keeps cameras, tags, rays and labels and emits only for rows "
    "that changed") {
    SceneModel scene;
    scene.setViewSize(1000.f, 700.f);
    scene.camera().target = {6.f, 2.f, 0.5f};
    scene.camera().distance = 5.f;
    scene.zoom(0.f);

    sdk::SimSnapshotData d = visionSnapshot();
    const sdk::SimSnapshot first(d);
    scene.update(&first);
    CHECK(scene.cameraModel().rowCount() == 1);
    CHECK(scene.tagModel().rowCount() == 3);
    CHECK(scene.rayModel().rowCount() == 3);
    CHECK(scene.labelModel().rowCount() == 4);

    // The frustum geometry is a pyramid: four sides and the far face, 18 vertices of position and
    // normal.
    auto* frustum = qobject_cast<QQuick3DGeometry*>(
        scene.cameraModel().index(0).data(CameraListModel::FrustumRole).value<QObject*>());
    REQUIRE(frustum != nullptr);
    CHECK(frustum->vertexData().size() == 18 * 6 * static_cast<qsizetype>(sizeof(float)));
    CHECK(frustum->boundsMax().x() == doctest::Approx(static_cast<float>(kFrustumDepthM)));

    // Rays show for what the camera sees; plates of seen tags are more opaque.
    CHECK(scene.rayModel().index(0).data(PartListModel::ShownRole).toBool());
    CHECK_FALSE(scene.rayModel().index(2).data(PartListModel::ShownRole).toBool());
    const QColor dim = scene.tagModel().index(2).data(PartListModel::ColorRole).value<QColor>();
    const QColor seen = scene.tagModel().index(0).data(PartListModel::ColorRole).value<QColor>();
    CHECK(seen.alphaF() > dim.alphaF());

    QSignalSpy cameras(&scene.cameraModel(), &QAbstractItemModel::dataChanged);
    QSignalSpy tags(&scene.tagModel(), &QAbstractItemModel::dataChanged);
    QSignalSpy rays(&scene.rayModel(), &QAbstractItemModel::dataChanged);
    scene.update(&first);
    CHECK(cameras.count() == 0);
    CHECK(tags.count() == 0);
    CHECK(rays.count() == 0);

    d.cameras[0].pose.position.y += 0.2f;
    const sdk::SimSnapshot moved(d);
    scene.update(&moved);
    CHECK(cameras.count() == 1);
    CHECK(tags.count() == 0);  // the tags do not move
    CHECK(rays.count() == 2);  // the two rays that exist follow the camera

    d.cameras[0].visibleTagIds = {2};
    const sdk::SimSnapshot lost(d);
    scene.update(&lost);
    CHECK_FALSE(scene.rayModel().index(0).data(PartListModel::ShownRole).toBool());
    CHECK(scene.tagModel().index(0).data(PartListModel::ColorRole).value<QColor>().alphaF() ==
          doctest::Approx(dim.alphaF()));

    scene.update(nullptr);  // no snapshot: the run is over, nothing is drawn
    scene.setSceneInfo({});
    CHECK(scene.cameraModel().rowCount() == 0);
    CHECK(scene.tagModel().rowCount() == 0);
}

TEST_CASE(
    "vision overlay: a camera that sees no tag is dimmer than one that does, and the colour "
    "follows its state") {
    SceneModel scene;
    sdk::SimSnapshotData d = visionSnapshot();
    d.cameras.push_back(camera("rear", {3.f, 2.f, 0.4f}, {}));
    sdk::CameraSample waiting = camera("waiting", {2.f, 2.f, 0.4f}, {});
    waiting.robotConnected = false;
    d.cameras.push_back(waiting);
    sdk::CameraSample broken = camera("broken", {1.f, 2.f, 0.4f}, {1});
    broken.status = "foreign_publisher";
    d.cameras.push_back(broken);
    const sdk::SimSnapshot snap(d);
    scene.update(&snap);

    const auto frustum = [&](int row) {
        return scene.cameraModel()
            .index(row)
            .data(CameraListModel::FrustumColorRole)
            .value<QColor>();
    };
    const auto body = [&](int row) {
        return scene.cameraModel().index(row).data(CameraListModel::BodyColorRole).value<QColor>();
    };
    REQUIRE(scene.cameraModel().rowCount() == 4);
    CHECK(frustum(0).alphaF() > frustum(1).alphaF());  // sees tags / sees none
    CHECK(body(0).alphaF() > body(1).alphaF());
    CHECK(frustum(0).rgb() == frustum(1).rgb());  // same state, same hue
    CHECK(frustum(2).rgb() != frustum(1).rgb());  // waiting for the robot is another hue
    CHECK(frustum(3).rgb() != frustum(1).rgb());  // so is a problem
    CHECK(frustum(3).rgb() != frustum(2).rgb());
}

TEST_CASE("vision overlay: the layer toggles hide labels of their layer and keep the others") {
    SceneModel scene;
    scene.setViewSize(1000.f, 700.f);
    scene.camera().target = {6.5f, 2.f, 0.8f};
    scene.camera().distance = 3.f;
    scene.camera().yawRad = kPi;  // the viewer stands on the -X side and looks towards the far wall
    scene.camera().pitchRad = 0.1f;
    scene.zoom(0.f);

    sdk::SimSnapshotData d = visionSnapshot();
    d.cameras[0].pose.position = {7.f, 2.f, 0.6f};
    const sdk::SimSnapshot snap(d);
    scene.update(&snap);
    const auto shown = [&](int row) {
        return scene.labelModel().index(row).data(LabelListModel::ShownRole).toBool();
    };
    REQUIRE(scene.labelModel().rowCount() == 4);
    CHECK(shown(0));  // tag 1 (a seen tag) is in view
    CHECK(shown(3));  // so is the camera's name

    QSignalSpy toggled(&scene, &SceneModel::tagsVisibleChanged);
    scene.setTagsVisible(false);
    CHECK(toggled.count() == 1);
    CHECK_FALSE(shown(0));
    CHECK(shown(3));
    scene.setTagsVisible(false);  // no change, no signal
    CHECK(toggled.count() == 1);

    scene.setCamerasVisible(false);
    CHECK_FALSE(shown(3));
    scene.setTagsVisible(true);
    scene.setCamerasVisible(true);
    CHECK(shown(0));
    CHECK(shown(3));
}
