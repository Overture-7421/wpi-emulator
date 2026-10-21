#include <QFrame>
#include <QLabel>
#include <QMimeData>

#include <algorithm>
#include <memory>

#include "common/camera_frustum.h"
#include "editor_page.h"
#include "model/editor_document.h"
#include "model/property_groups.h"
#include "model/scene_builder.h"
#include "model/slot_status.h"
#include "model/transform_access.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"
#include "viewmodel/viewport_controller.h"
#include "views/inspector_panel.h"
#include "views/property_form.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

struct CameraPage {
    FakeEditorBackend backend;
    EditorPage page{backend};
    CameraPage() {
        page.setAutosaveDelayMs(0);
        page.resize(1400, 900);
        page.show();
        pumpEvents(60);
    }
    EditorController& c() { return page.controller(); }
};

sdk::Pose3 cameraWorldPose(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return ops::cameraWorld(c, cat, *c.findCamera(id));
}

sdk::Vec3 cameraPos(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return cameraWorldPose(c, cat, id).position;
}

sdk::RobotConfig withoutCameras() {
    auto c = robot();
    c.cameras.clear();
    return c;
}

}  // namespace

// ---- ops ---------------------------------------------------------------------------------------------

TEST_CASE("camera ops: the sample robot carries a Limelight on the chassis and a PhotonVision camera on the turret") {
    const auto c = robot();
    REQUIRE(c.cameras.size() == 2);
    CHECK(c.findCamera("limelight")->attachment.onChassis());
    CHECK(c.findCamera("photon")->attachment.parentMechanismId == "turret");
    CHECK(c.findCamera("photon")->attachment.parentLink == "mover");
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("camera ops: a new camera faces forward near the front of the hull with a unique id, label and name") {
    const auto cat = catalog();
    auto c = withoutCameras();

    const std::string first = ops::addCamera(c, cat, sdk::CameraKind::Limelight, {});
    CHECK(first == "limelight");
    const sdk::CameraConfig* cam = c.findCamera(first);
    REQUIRE(cam != nullptr);
    CHECK(cam->kind == sdk::CameraKind::Limelight);
    CHECK(cam->label == "Limelight");
    CHECK(cam->name.empty());   // "limelight" is the default table name
    CHECK(cam->attachment.onChassis());
    CHECK(cam->hFovDeg == sdk::CameraConfig{}.hFovDeg);
    CHECK(cam->fps == sdk::CameraConfig{}.fps);
    const sdk::Pose3 world = cameraWorldPose(c, cat, first);
    CHECK(approx(world.position, {c.chassis.sizeM.x * 0.5f - 0.05f, 0.f, c.chassis.groundClearanceM + c.chassis.sizeM.z * 0.8f}));
    CHECK(approx(world.rotation.rotate({1.f, 0.f, 0.f}), {1.f, 0.f, 0.f}));   // looks along +X

    const std::string second = ops::addCamera(c, cat, sdk::CameraKind::Limelight, {});
    CHECK(second == "limelight_2");
    CHECK(c.findCamera(second)->label == "Limelight 2");
    CHECK(c.findCamera(second)->name == "cam2");   // unique per kind

    const std::string photon = ops::addCamera(c, cat, sdk::CameraKind::PhotonVision, {});
    CHECK(photon == "photon");
    CHECK(c.findCamera(photon)->label == "PhotonVision");
    CHECK(c.findCamera(photon)->name == "photon");
    CHECK(c.findCamera(ops::addCamera(c, cat, sdk::CameraKind::PhotonVision, {}))->name == "photon2");

    // Names only clash within a kind: this Limelight may reuse the PhotonVision name.
    CHECK(ops::uniqueCameraName(c, sdk::CameraKind::Limelight, "photon") == "photon");
}

TEST_CASE("camera ops: on a mechanism it sits at the link origin, an unknown attachment falls back to the chassis") {
    const auto cat = catalog();
    auto c = withoutCameras();
    const std::string id = ops::addCamera(c, cat, sdk::CameraKind::PhotonVision, {"turret", "mover"});
    const sdk::CameraConfig* cam = c.findCamera(id);
    REQUIRE(cam != nullptr);
    CHECK(cam->attachment.parentMechanismId == "turret");
    CHECK(cam->attachment.parentLink == "mover");
    CHECK(approx(cam->local.position, {0.f, 0.f, 0.f}));
    const auto link = ops::linkInChassis(c, cat, "turret", "mover");
    REQUIRE(link);
    CHECK(approx(cameraPos(c, cat, id), (ops::chassisFrameWorld(c.chassis) * *link).position));

    ops::addCamera(c, cat, sdk::CameraKind::Limelight, {"ghost", "root"});
    CHECK(c.cameras.back().attachment.onChassis());
    ops::addCamera(c, cat, sdk::CameraKind::Limelight, {"turret", "no_such_link"});
    CHECK(c.cameras.back().attachment.onChassis());
}

TEST_CASE("camera ops: ids stay distinct from mechanism, zone and camera ids in every generator") {
    auto c = robot();
    CHECK(ops::uniqueCameraId(c, "intake") == "intake_2");     // a mechanism
    CHECK(ops::uniqueCameraId(c, "pickup") == "pickup_2");     // a zone
    CHECK(ops::uniqueCameraId(c, "limelight") == "limelight_2");
    CHECK(ops::uniqueCameraId(c, "") == "camera");
    CHECK(ops::uniqueMechanismId(c, "limelight") == "limelight_2");   // mechanisms and zones avoid camera ids too
    CHECK(ops::uniqueZoneId(c, "photon") == "photon_2");

    auto d = robot();
    d.cameras[0].id = "indexer_2";
    CHECK(ops::duplicateMechanism(d, "indexer") == "indexer_3");
}

TEST_CASE("camera ops: duplicate copies the setup with a fresh id and name, remove deletes") {
    auto c = robot();
    c.findCamera("photon")->hFovDeg = 90.0;
    const std::string copy = ops::duplicateCamera(c, "photon");
    CHECK(copy == "photon_2");
    const auto* z = c.findCamera(copy);
    REQUIRE(z != nullptr);
    CHECK(z->attachment == c.findCamera("photon")->attachment);
    CHECK(z->kind == sdk::CameraKind::PhotonVision);
    CHECK(z->hFovDeg == 90.0);
    CHECK(z->name == "photon2");
    CHECK(z->label == "Turret camera (copy)");
    CHECK(ops::duplicateCamera(c, "nobody").empty());
    CHECK(ops::removeCamera(c, copy));
    CHECK_FALSE(ops::removeCamera(c, copy));
    CHECK(c.cameras.size() == 2);
}

TEST_CASE("camera ops: re-attaching keeps the world pose and refuses unknown parents or links") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Pose3 before = cameraWorldPose(c, cat, "photon");

    REQUIRE(ops::reattachCamera(c, cat, "photon", "chassis", "root"));
    CHECK(c.findCamera("photon")->attachment.onChassis());
    CHECK(approx(cameraPos(c, cat, "photon"), before.position));
    REQUIRE(ops::reattachCamera(c, cat, "photon", "intake", "root"));
    CHECK(approx(cameraPos(c, cat, "photon"), before.position));
    CHECK(approx(cameraWorldPose(c, cat, "photon").rotation.rotate({1.f, 0.f, 0.f}), before.rotation.rotate({1.f, 0.f, 0.f})));

    CHECK_FALSE(ops::reattachCamera(c, cat, "photon", "ghost", "root"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "photon", "turret", "no_such_link"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "photon", "chassis", "mover"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "ghost", "chassis", "root"));
    CHECK(approx(cameraPos(c, cat, "photon"), before.position));

    const sdk::Pose3 local = c.findCamera("photon")->local;
    REQUIRE(ops::reattachCamera(c, cat, "photon", "turret", "mover", false));
    CHECK(approx(c.findCamera("photon")->local.position, local.position));   // jumps: the local pose is untouched
}

TEST_CASE("camera ops: deleting a mechanism keeps its cameras in place") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 before = cameraPos(c, cat, "photon");
    REQUIRE(ops::removeMechanism(c, cat, "turret"));
    CHECK(c.findCamera("photon")->attachment.onChassis());   // the turret hung on the chassis
    CHECK(approx(cameraPos(c, cat, "photon"), before));

    auto d = robot();
    REQUIRE(ops::reattachCamera(d, cat, "limelight", "shooter", "root"));
    const sdk::Vec3 onShooter = cameraPos(d, cat, "limelight");
    REQUIRE(ops::removeMechanism(d, cat, "shooter"));
    CHECK(d.findCamera("limelight")->attachment.parentMechanismId == "turret");   // up to the parent, not the chassis
    CHECK(d.findCamera("limelight")->attachment.parentLink == "mover");
    CHECK(approx(cameraPos(d, cat, "limelight"), onShooter));
}

TEST_CASE("camera ops: moving the COM keeps cameras on the chassis where they are") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 before = cameraPos(c, cat, "limelight");
    const sdk::Vec3 turretCam = cameraPos(c, cat, "photon");
    ops::setCom(c, 0.05f, -0.03f, 0.30f, true, &cat);
    CHECK(approx(cameraPos(c, cat, "limelight"), before));
    CHECK(approx(cameraPos(c, cat, "photon"), turretCam));
}

TEST_CASE("camera ops: the robot-to-camera pose follows the mechanism link it hangs on at its rest pose") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Pose3 onTurret = ops::cameraRobotPose(c, cat, *c.findCamera("photon"));
    CHECK(approx(onTurret.position, cameraPos(c, cat, "photon")));

    for (auto& m : c.mechanisms)
        if (m.id == "turret") m.mount.position.x += 0.2f;
    CHECK(approx(ops::cameraRobotPose(c, cat, *c.findCamera("photon")).position.x, onTurret.position.x + 0.2f, 1e-3f));
    // The origin is on the floor below the hull centre, whatever the COM is.
    ops::setCom(c, 0.1f, 0.f, 0.3f, true, &cat);
    CHECK(approx(ops::cameraRobotPose(c, cat, *c.findCamera("limelight")).position, cameraPos(c, cat, "limelight")));
}

// ---- document and controller -------------------------------------------------------------------------

TEST_CASE("camera document: add, duplicate, re-attach and remove are undoable") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());

    const QString id = doc.addCamera(sdk::CameraKind::PhotonVision, {"turret", "mover"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(doc.config().cameras.size() == 3);
    doc.undoStack()->undo();
    CHECK(doc.config().cameras.size() == 2);
    doc.undoStack()->redo();
    CHECK(doc.config().findCamera(id.toStdString()) != nullptr);

    const QString copy = doc.duplicateCamera("limelight");
    CHECK(copy == "limelight_2");
    CHECK(doc.duplicateCamera("ghost").isEmpty());
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("limelight_2") == nullptr);

    const sdk::Vec3 before = cameraPos(doc.config(), doc.catalog(), "photon");
    REQUIRE(doc.reattachCamera("photon", "chassis"));
    CHECK(approx(cameraPos(doc.config(), doc.catalog(), "photon"), before));
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon")->attachment.parentMechanismId == "turret");
    CHECK_FALSE(doc.reattachCamera("ghost", "chassis"));

    REQUIRE(doc.removeCamera("photon"));
    CHECK(doc.config().findCamera("photon") == nullptr);
    CHECK_FALSE(doc.removeCamera("photon"));
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon") != nullptr);
}

TEST_CASE("camera controller: adding selects the camera and delete / duplicate act on it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    const QString id = c.addCamera(sdk::CameraKind::Limelight, {"intake", "root"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(c.selection().current() == SelectionRef::camera(id.toStdString()));
    CHECK(c.selectionExists(SelectionRef::camera(id.toStdString())));
    CHECK_FALSE(c.selectionExists(SelectionRef::camera("ghost")));

    REQUIRE(c.duplicateSelection());
    const SelectionRef copy = c.selection().current();
    CHECK(copy.kind == SelKind::Camera);
    CHECK(copy.owner != id.toStdString());
    CHECK(c.document().config().cameras.size() == 4);

    REQUIRE(c.deleteSelection());
    CHECK(c.document().config().cameras.size() == 3);
    CHECK(c.selection().current() == SelectionRef::robot());   // the selected camera is gone
}

// ---- project tree ------------------------------------------------------------------------------------

TEST_CASE("camera tree: cameras are listed in a group under the robot with their attachment") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex group = tree.indexForKey("cameras");
    REQUIRE(group.isValid());
    CHECK(tree.parent(group) == tree.index(0, 0));
    CHECK(tree.rowCount(group) == 2);
    CHECK(tree.data(group, ProjectTreeModel::DetailRole).toString() == "2");

    const QModelIndex limelight = tree.indexFor(SelectionRef::camera("limelight"));
    const QModelIndex photon = tree.indexFor(SelectionRef::camera("photon"));
    REQUIRE(limelight.isValid());
    CHECK(tree.parent(limelight) == group);
    CHECK(tree.data(limelight).toString() == "Limelight");
    CHECK(tree.data(limelight, ProjectTreeModel::KindRole).toInt() == static_cast<int>(NodeKind::Camera));
    CHECK(tree.data(limelight, ProjectTreeModel::DetailRole).toString() == "@ chassis");
    CHECK(tree.data(photon, ProjectTreeModel::DetailRole).toString() == "@ turret.mover");

    REQUIRE(doc.reattachCamera("photon", "intake"));
    CHECK(doc.addCamera(sdk::CameraKind::Limelight).size() > 0);
    tree.refresh();
    CHECK(tree.data(tree.indexFor(SelectionRef::camera("photon")), ProjectTreeModel::DetailRole).toString() == "@ intake");
    CHECK(tree.rowCount(tree.indexForKey("cameras")) == 3);
}

TEST_CASE("camera tree: validator issues put a warning on the camera they point at") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);
    const auto warning = [&](const char* id) {
        return tree.data(tree.indexFor(SelectionRef::camera(id)), ProjectTreeModel::WarningRole).toBool();
    };
    CHECK_FALSE(warning("photon"));
    tree.setIssues({{sdk::Severity::Error, "cameras[1].hFovDeg", "bad", "camera_optics_invalid"}});
    CHECK(warning("photon"));
    CHECK_FALSE(warning("limelight"));
    tree.setIssues({});
    CHECK_FALSE(warning("photon"));
}

TEST_CASE("camera tree: dragging a camera onto a mechanism or the chassis offers re-attaching it") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex photon = tree.indexFor(SelectionRef::camera("photon"));
    CHECK((tree.flags(photon) & Qt::ItemIsDragEnabled));
    std::unique_ptr<QMimeData> mime(tree.mimeData({photon}));
    REQUIRE(mime != nullptr);
    CHECK(mime->hasFormat(QString::fromLatin1(kCameraMime)));

    const QModelIndex turret = tree.indexFor(SelectionRef::mechanism("turret"));
    const QModelIndex intake = tree.indexFor(SelectionRef::mechanism("intake"));
    const QModelIndex chassis = tree.indexFor(SelectionRef::chassis());
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, intake));
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, chassis));
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, turret));   // another link of the turret is possible: root
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::camera("limelight"))));

    std::unique_ptr<QMimeData> limelightMime(tree.mimeData({tree.indexFor(SelectionRef::camera("limelight"))}));
    CHECK_FALSE(tree.canDropMimeData(limelightMime.get(), Qt::MoveAction, -1, -1, chassis));   // already there

    QString droppedId;
    sdk::MechanismAttachment droppedOn;
    QObject::connect(&tree, &ProjectTreeModel::cameraDropped, &tree, [&](const QString& id, const sdk::MechanismAttachment& att) {
        droppedId = id;
        droppedOn = att;
    });
    REQUIRE(tree.dropMimeData(mime.get(), Qt::MoveAction, -1, -1, intake));
    CHECK(droppedId == "photon");
    CHECK(droppedOn.parentMechanismId == "intake");
}

// ---- property groups and the inspector ---------------------------------------------------------------

TEST_CASE("camera property groups: general, optics and placement apply to the camera") {
    const auto cat = catalog();
    auto c = robot();
    const SelectionRef sel = SelectionRef::camera("photon");

    const auto groups = propertyGroupsFor(c, cat, sel);
    std::vector<std::string> ids;
    for (const auto& g : groups) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"camera.general", "camera.optics", "camera.placement"});
    CHECK(groups[0].schema.find("kind") != nullptr);
    CHECK(groups[0].schema.find("name") != nullptr);
    CHECK(groups[0].hint.contains("limelight-<name>"));
    CHECK(groups[0].hint.contains("PhotonCamera"));
    for (const char* key : {"hFovDeg", "vFovDeg", "maxRangeM", "maxIncidenceDeg", "fps", "latencyMs", "widthPx", "heightPx", "positionNoiseM", "yawNoiseDeg"})
        CHECK(groups[1].schema.find(key) != nullptr);
    CHECK(groups[2].schema.find("local") != nullptr);
    CHECK(groups[2].section == sdk::PropertySection::Placement);
    CHECK(groups[1].values.find("widthPx")->asInt() == 1280);
    CHECK(propertyGroupsFor(c, cat, SelectionRef::camera("ghost")).empty());
    CHECK(describeSelection(c, cat, sel) == "Turret camera");

    CHECK(applyProperty(c, cat, sel, "camera.general", "label", sdk::Value("Left cam")));
    CHECK(c.findCamera("photon")->label == "Left cam");
    CHECK(applyProperty(c, cat, sel, "camera.general", "name", sdk::Value("left")));
    CHECK(c.findCamera("photon")->name == "left");
    CHECK(applyProperty(c, cat, sel, "camera.optics", "hFovDeg", sdk::Value(90.0)));
    CHECK(c.findCamera("photon")->hFovDeg == 90.0);
    CHECK(applyProperty(c, cat, sel, "camera.optics", "widthPx", sdk::Value(640)));
    CHECK(c.findCamera("photon")->widthPx == 640);
    CHECK(applyProperty(c, cat, sel, "camera.optics", "yawNoiseDeg", sdk::Value(0.5)));
    CHECK(c.findCamera("photon")->yawNoiseDeg == 0.5);
    CHECK_FALSE(applyProperty(c, cat, sel, "camera.optics", "bogus", sdk::Value(1.0)));
    CHECK_FALSE(applyProperty(c, cat, SelectionRef::camera("ghost"), "camera.optics", "fps", sdk::Value(10.0)));

    sdk::Pose3 pose;
    pose.position = {0.1f, 0.2f, 0.3f};
    CHECK(applyProperty(c, cat, sel, "camera.placement", "local", sdk::toValue(pose)));
    CHECK(approx(c.findCamera("photon")->local.position, pose.position));
    const sdk::Vec3 before = cameraPos(c, cat, "photon");
    CHECK(applyProperty(c, cat, sel, "camera.placement", "parent", sdk::Value("intake")));   // keeps the world pose
    CHECK(c.findCamera("photon")->attachment.parentMechanismId == "intake");
    CHECK(approx(cameraPos(c, cat, "photon"), before));
    CHECK_FALSE(applyProperty(c, cat, sel, "camera.placement", "parentLink", sdk::Value("nowhere")));
}

TEST_CASE("camera property groups: changing the type keeps the robot-code name meaningful and unique") {
    const auto cat = catalog();
    auto c = robot();
    const SelectionRef photon = SelectionRef::camera("photon");
    REQUIRE(applyProperty(c, cat, photon, "camera.general", "kind", sdk::Value("limelight")));
    CHECK(c.findCamera("photon")->kind == sdk::CameraKind::Limelight);
    CHECK(c.findCamera("photon")->name == "cam2");   // "" is taken by the other Limelight, "photon" was only the PhotonVision default
    CHECK_FALSE(applyProperty(c, cat, photon, "camera.general", "kind", sdk::Value("limelight")));   // unchanged
    REQUIRE(applyProperty(c, cat, photon, "camera.general", "kind", sdk::Value("photonvision")));
    CHECK(c.findCamera("photon")->kind == sdk::CameraKind::PhotonVision);

    auto d = robot();
    REQUIRE(applyProperty(d, cat, SelectionRef::camera("limelight"), "camera.general", "kind", sdk::Value("photonvision")));
    CHECK(d.findCamera("limelight")->name == "photon2");   // "photon" is the other camera's name
}

TEST_CASE("camera inspector: the page shows General, Optics and Placement with the robot-to-camera transform") {
    CameraPage f;
    f.c().selection().set(SelectionRef::camera("photon"));
    pumpEvents(30);
    auto* insp = f.page.inspector();
    CHECK(insp->titleText() == "Turret camera");
    for (const char* card : {"card.info", "card.configure", "card.placement"})
        CHECK(insp->findChild<QFrame*>(card) != nullptr);
    CHECK(insp->form("camera.general")->hasField("label"));
    CHECK(insp->form("camera.general")->hasField("kind"));
    CHECK(insp->form("camera.general")->hasField("name"));
    CHECK(insp->form("camera.optics")->hasField("hFovDeg"));
    CHECK(insp->form("camera.placement")->hasField("local"));
    CHECK(insp->findChild<QLabel*>("camera.id")->text().contains("photon"));
    CHECK(insp->findChild<QLabel*>("camera.nameHint")->text().contains("limelight-<name>"));

    QLabel* transform = insp->robotToCameraLabel();
    REQUIRE(transform != nullptr);
    CHECK(transform->text().contains("roll"));
    const auto xOf = [&] { return transform->toolTip().section("x ", 1, 1).section(",", 0, 0).toDouble(); };
    const double before = xOf();
    CHECK(before == doctest::Approx(static_cast<double>(cameraPos(f.c().document().config(), f.c().document().catalog(), "photon").x)).epsilon(1e-3));

    // The line follows the camera: moving the turret it rides on moves its rest pose.
    f.c().document().edit(QStringLiteral("move turret"), [](sdk::RobotConfig& c) {
        for (auto& m : c.mechanisms)
            if (m.id == "turret") m.mount.position.x += 0.2f;
        return true;
    });
    pumpEvents(30);
    CHECK(xOf() == doctest::Approx(before + 0.2).epsilon(1e-3));

    // Editing a field goes through the document (undoable).
    auto* label = insp->form("camera.general")->editorFor("label");
    emit label->edited(sdk::Value("Rear cam"));
    CHECK(f.c().document().config().findCamera("photon")->label == "Rear cam");
    auto* kind = insp->form("camera.general")->editorFor("kind");
    emit kind->edited(sdk::Value("limelight"));
    CHECK(f.c().document().config().findCamera("photon")->kind == sdk::CameraKind::Limelight);
    f.c().document().undoStack()->undo();
    CHECK(f.c().document().config().findCamera("photon")->kind == sdk::CameraKind::PhotonVision);
    f.c().document().undoStack()->undo();
    CHECK(f.c().document().config().findCamera("photon")->label == "Turret camera");
}

// ---- validation --------------------------------------------------------------------------------------

TEST_CASE("camera validation: issue paths are located to their camera and field") {
    const auto cat = catalog();
    const auto c = robot();
    const auto at = [&](const std::string& path) { return locateIssue(c, cat, {sdk::Severity::Error, path, "x", {}}); };
    CHECK(at("cameras[1].attachment").selection == SelectionRef::camera("photon"));
    CHECK(at("cameras[1].attachment").fieldKey == "parent");
    CHECK(at("cameras[0].hFovDeg").fieldKey == "hFovDeg");
    CHECK(at("cameras[0].name").fieldKey == "name");
    CHECK(at("cameras[0].local").fieldKey == "local");
    CHECK(at("cameras[0].pose").fieldKey == "local");   // the shared validator reports the mount pose as ".pose"
    CHECK(at("cameras[0].id").fieldKey == "id");
    CHECK(at("cameras[9].attachment").selection.isNone());
}

TEST_CASE("camera validation: the editor backend reports duplicate names and bad optics through the issue paths") {
    FakeEditorBackend backend;
    const auto cat = catalog();
    auto c = robot();
    c.findCamera("photon")->kind = sdk::CameraKind::Limelight;   // the other Limelight is named "" too
    c.findCamera("photon")->name = "";
    c.findCamera("limelight")->vFovDeg = 0.0;
    std::vector<std::string> codes;
    for (const auto& issue : backend.validate(c)) {
        codes.push_back(issue.code);
        if (issue.code == "camera_name_duplicate") CHECK(locateIssue(c, cat, issue).selection == SelectionRef::camera("photon"));
        if (issue.code == "camera_optics_invalid") {
            CHECK(locateIssue(c, cat, issue).selection == SelectionRef::camera("limelight"));
            CHECK(locateIssue(c, cat, issue).fieldKey == "vFovDeg");
        }
    }
    CHECK(std::count(codes.begin(), codes.end(), "camera_name_duplicate") == 1);
    CHECK(std::count(codes.begin(), codes.end(), "camera_optics_invalid") == 1);

    auto d = robot();
    d.findCamera("photon")->id = "intake";   // collides with a mechanism
    bool clash = false;
    for (const auto& issue : backend.validate(d)) clash = clash || issue.code == "camera_id_duplicate";
    CHECK(clash);
}

// ---- viewport ----------------------------------------------------------------------------------------

TEST_CASE("camera scene: a body marker, eight frustum edges and a translucent far plane in the kind colour") {
    const auto cat = catalog();
    const auto c = robot();
    const SceneTheme theme = SceneTheme::dark();
    const auto scene = buildScene(c, cat, theme);

    const auto* body = scene.find("camera:limelight");
    REQUIRE(body != nullptr);
    CHECK(body->ref == SelectionRef::camera("limelight"));
    CHECK(body->pickable);
    CHECK(body->color == theme.limelight);
    CHECK(approx(body->world.position, cameraPos(c, cat, "limelight")));
    CHECK(scene.find("camera:photon")->color == theme.photonVision);   // distinct per kind
    CHECK(theme.limelight != theme.photonVision);

    const auto& cam = *c.findCamera("limelight");
    const wpiemu::app::FrustumGeometry g = wpiemu::app::cameraFrustum(cam);
    const sdk::Pose3 camWorld = ops::cameraWorld(c, cat, cam);

    for (std::size_t i = 0; i < wpiemu::app::kFrustumEdges.size(); ++i) {
        const auto* edge = scene.find("camera:limelight:edge:" + std::to_string(i));
        REQUIRE(edge != nullptr);
        CHECK(edge->group == SelectionRef::camera("limelight"));
        CHECK(edge->role == ItemRole::Link);
        const auto [from, to] = wpiemu::app::kFrustumEdges[i];
        const sdk::Vec3 a = camWorld.transformPoint(g.pts[from]);
        const sdk::Vec3 b = camWorld.transformPoint(g.pts[to]);
        CHECK(approx(edge->world.position, (a + b) * 0.5f, 1e-3f));                              // centred on the segment
        CHECK(approx(edge->world.transformVector({edge->size.x, 0.f, 0.f}), b - a, 1e-3f));    // and runs from a to b
    }
    CHECK(scene.find("camera:limelight:edge:8") == nullptr);

    const auto* plane = scene.find("camera:limelight:fov");
    REQUIRE(plane != nullptr);
    CHECK(plane->role == ItemRole::Overlay);
    CHECK(plane->opacity < 0.5f);
    CHECK(plane->pickable);
    // The far plane lies along the optical axis (+X of the camera) at the frustum length.
    CHECK(approx(plane->world.position, camWorld.transformPoint({g.lengthM, 0.f, 0.f})));
    CHECK(approx(plane->size.y, 2.f * g.pts[wpiemu::app::kFrustumTopLeft].y, 1e-3f));

    // Moving the turret moves the camera that rides on it.
    auto moved = c;
    for (auto& m : moved.mechanisms)
        if (m.id == "turret") m.mount.position.x += 0.2f;
    CHECK(approx(buildScene(moved, cat, theme).find("camera:photon")->world.position.x, scene.find("camera:photon")->world.position.x + 0.2f, 1e-3f));

    std::vector<std::string> ids;
    for (const auto& i : scene.items) ids.push_back(i.id);
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
}

TEST_CASE("camera transform: pose round-trips and a camera can be moved and rotated but not scaled") {
    const auto cat = catalog();
    auto c = robot();
    const auto t0 = readTransform(c, cat, SelectionRef::camera("photon"));
    REQUIRE(t0);
    CHECK(t0->transform.canMove);
    CHECK(t0->transform.canRotate);
    CHECK_FALSE(t0->transform.canScale);
    CHECK(approx(t0->world().position, cameraPos(c, cat, "photon")));

    EditableTransform t = t0->transform;
    t.local.position = {0.5f, 0.1f, 0.2f};
    t.local.rotation = sdk::Quat::fromYaw(0.5f);
    REQUIRE(writeTransform(c, cat, SelectionRef::camera("photon"), t));
    CHECK(approx(c.findCamera("photon")->local.position, {0.5f, 0.1f, 0.2f}));
    CHECK(approx(c.findCamera("photon")->local.rotation.yaw(), 0.5f));
    CHECK_FALSE(readTransform(c, cat, SelectionRef::camera("ghost")));
    CHECK_FALSE(writeTransform(c, cat, SelectionRef::camera("ghost"), t));
}

TEST_CASE("camera viewport: picking, gizmo modes and a drag that is one undo step") {
    FakeEditorBackend backend;
    EditorController editor{backend};
    ViewportController view{editor};
    editor.setAutosaveDelayMs(0);
    view.setViewSize(800, 600);
    editor.settings().setSnapEnabled(false);

    CHECK(view.chooseItem({"i:camera:limelight:fov", "i:hull"}) == SelectionRef::chassis());   // solid parts win over the far plane
    CHECK(view.chooseItem({"i:camera:limelight:fov"}) == SelectionRef::camera("limelight"));
    CHECK(view.chooseItem({"i:camera:limelight"}) == SelectionRef::camera("limelight"));

    editor.selection().set(SelectionRef::camera("limelight"));
    REQUIRE(view.effectiveMode());
    view.setGizmoMode(GizmoMode::Scale);
    CHECK(*view.effectiveMode() == GizmoMode::Move);   // a camera cannot scale
    view.setGizmoMode(GizmoMode::Rotate);
    CHECK(*view.effectiveMode() == GizmoMode::Rotate);
    CHECK_FALSE(view.selectionEdges().isEmpty());

    view.setGizmoMode(GizmoMode::Move);
    const auto start = readTransform(editor.document().config(), editor.document().catalog(), SelectionRef::camera("limelight"));
    REQUIRE(start);
    const sdk::Vec3 origin = start->world().position;
    const sdk::Vec3 axis = start->parentWorld.rotation.rotate({0.f, 1.f, 0.f});
    const auto screen = [&](const sdk::Vec3& world) {
        const auto px = view.camera().project(world, 800.f, 600.f);
        REQUIRE(px);
        return QPointF(static_cast<qreal>(px->first), static_cast<qreal>(px->second));
    };
    const QPointF p0 = screen(origin + axis * 0.2f);
    REQUIRE(view.beginGizmoDrag("h:m:1", p0.x(), p0.y()));
    for (int i = 1; i <= 4; ++i) {
        const QPointF p = screen(origin + axis * (0.2f + 0.05f * static_cast<float>(i)));
        view.updateGizmoDrag(p.x(), p.y(), false);
    }
    view.endGizmoDrag();
    CHECK(approx(editor.document().config().findCamera("limelight")->local.position.y, start->transform.local.position.y + 0.2f, 2e-3f));
    CHECK(editor.document().undoStack()->count() == 1);
    editor.document().undoStack()->undo();
    CHECK(approx(editor.document().config().findCamera("limelight")->local.position.y, start->transform.local.position.y));
}
