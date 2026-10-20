#include <doctest/doctest.h>

#include <QImage>
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


}  // namespace

TEST_CASE("tag rig draws a plate per tag and highlights the union of the cameras' visible ids") {
    TagRig rig;
    rig.setLayout(FakeRunBackend::makeDemoTags());
    REQUIRE(rig.tags().size() == 8);
    CHECK(rig.parts().size() == 8);
    // +X out of the face: the panel is thin along X and sits just behind the tag pose.
    const ScenePart& panel = rig.parts()[0];
    CHECK(panel.size.x < panel.size.y);
    const sdk::Vec3 normal = rig.tags()[0].pose.rotation.rotate({1.f, 0.f, 0.f});
    const float behind = sdk::dot(panel.pose.position - rig.tags()[0].pose.position, normal);
    CHECK(behind < 0.f);
    CHECK(behind > -0.02f);
    const std::string idleColor = panel.color;

    // The decal faces the same way, image up = tag +Z, image right = tag +Y, at 1.25 x the black square.
    REQUIRE(rig.decals().size() == 8);
    const TagDecal& decal = rig.decals()[0];
    CHECK(decal.id == rig.tags()[0].id);
    CHECK(near(decal.pose.position, rig.tags()[0].pose.position));
    CHECK(near(decal.pose.rotation.rotate({0.f, 0.f, 1.f}), normal));
    CHECK(near(decal.pose.rotation.rotate({1.f, 0.f, 0.f}), rig.tags()[0].pose.rotation.rotate({0.f, 1.f, 0.f})));
    CHECK(near(decal.pose.rotation.rotate({0.f, 1.f, 0.f}), rig.tags()[0].pose.rotation.rotate({0.f, 0.f, 1.f})));
    CHECK(decal.sizeM == doctest::Approx(rig.tags()[0].sizeM * 1.25f));

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
        CHECK(rig.changed()[i] == (expect ? 1 : 0));
        CHECK(rig.changed()[i] == (expect ? 1 : 0));   // only the panel recolours
        CHECK((rig.parts()[i].color != idleColor) == expect);
    }
    CHECK_FALSE(rig.update(&snap));

    sdk::SimSnapshotData later = baseSnapshot();
    later.cameras.push_back(cameraState("cam_turret", {5}));
    const sdk::SimSnapshot snap2(later);
    CHECK(rig.update(&snap2));
    CHECK(rig.changed()[0] == 1);   // tag 1 dropped out
    CHECK(rig.changed()[1] == 1);
    CHECK(rig.changed()[4] == 0);   // tag 5 stays seen

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


TEST_CASE("scene model emits tag changes only when a tag switches state") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    FakeRunBackend backend(opts);
    SceneModel scene;
    scene.setSceneInfo(backend.scene());
    auto& tags = *static_cast<PartListModel*>(scene.tags());
    CHECK(tags.rowCount() == 8);
    auto& markers = *static_cast<TagMarkerModel*>(scene.tagMarkers());
    REQUIRE(markers.rowCount() == 8);
    CHECK(markers.index(0).data(TagMarkerModel::ImageRole).toUrl().toString() ==
          QStringLiteral("qrc:/wpiemu/runview/assets/apriltags/36h11/tag36_11_00001.png"));

    sdk::SimSnapshotData d = baseSnapshot();
    const sdk::SimSnapshot rest(d);
    QSignalSpy tagSpy(&tags, &QAbstractItemModel::dataChanged);
    scene.update(&rest);
    scene.update(&rest);
    CHECK(tagSpy.count() == 0);

    d.cameras.push_back(cameraState("cam_rear", {2}));
    const sdk::SimSnapshot seeing(d);
    scene.update(&seeing);
    REQUIRE(tagSpy.count() == 1);
    CHECK(tagSpy.at(0).at(0).toModelIndex().row() == 1);
    scene.update(&seeing);
    CHECK(tagSpy.count() == 1);
}

TEST_CASE("scene model toggles tag visibility") {
    SceneModel scene;
    CHECK(scene.tagsVisible());
    QSignalSpy tagSpy(&scene, &SceneModel::tagsVisibleChanged);
    scene.setTagsVisible(false);
    scene.setTagsVisible(false);
    CHECK(tagSpy.count() == 1);
    CHECK_FALSE(scene.tagsVisible());
}

TEST_CASE("fake backend cameras see tags and the visible set changes as the robot moves") {
    FakeRunBackend::Options opts;
    opts.autoRun = false;
    opts.startupDelayMs = 0;
    FakeRunBackend backend(opts);
    CHECK(backend.scene().tags.tags.size() == 8);
    CHECK(backend.scene().config->cameras.size() == 2);

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

TEST_CASE("tag images: ids 1-32 are embedded 36h11 markers and ids without an image are skipped") {
    std::set<QByteArray> seen;
    for (int id = 1; id <= 32; ++id) {
        const QImage img(QStringLiteral(":/wpiemu/runview/assets/apriltags/36h11/tag36_11_%1.png").arg(id, 5, 10, QLatin1Char('0')));
        INFO("tag " << id);
        REQUIRE(img.size() == QSize(10, 10));
        for (int y = 0; y < 10; ++y)
            for (int x = 0; x < 10; ++x) {
                const bool outer = x == 0 || y == 0 || x == 9 || y == 9;
                const bool ring = !outer && (x == 1 || y == 1 || x == 8 || y == 8);
                if (outer) CHECK(qGray(img.pixel(x, y)) == 255);
                if (ring) CHECK(qGray(img.pixel(x, y)) == 0);
            }
        seen.insert(QByteArray(reinterpret_cast<const char*>(img.constBits()), static_cast<qsizetype>(img.sizeInBytes())));
    }
    CHECK(seen.size() == 32);

    TagRig rig;
    sdk::AprilTagLayout layout = FakeRunBackend::makeDemoTags();
    sdk::AprilTag extra;
    extra.id = 999;
    layout.tags.push_back(extra);
    rig.setLayout(layout);
    TagMarkerModel model;
    model.setDecals(rig.decals());
    CHECK(rig.decals().size() == 9);
    CHECK(model.rowCount() == 8);
}
