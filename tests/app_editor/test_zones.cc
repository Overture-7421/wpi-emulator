#include <QAbstractButton>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QToolButton>

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
#include "views/activation_card.h"
#include "views/inspector_panel.h"
#include "views/property_form.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/testing/demo_game.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

constexpr const char* kPickup = sdk::testing::demo_game::kPickupZone;
constexpr const char* kShooter = sdk::testing::demo_game::kShooterZone;

struct ZonePage {
    FakeEditorBackend backend;
    EditorPage page{backend};
    ZonePage() {
        page.setAutosaveDelayMs(0);
        page.resize(1400, 900);
        page.show();
        pumpEvents(60);
    }
    EditorController& c() { return page.controller(); }
};

sdk::Vec3 zoneWorldPos(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return ops::zoneWorld(c, cat, *c.findZone(id)).position;
}

}  // namespace

// ---- ops ---------------------------------------------------------------------------------------------

TEST_CASE("zone ops: the sample robot carries a pickup on the intake and a muzzle on the turret and validates") {
    const auto c = robot();
    REQUIRE(c.zones.size() == 2);
    CHECK(c.findZone("pickup")->attachment.parentMechanismId == "intake");
    CHECK(c.findZone("muzzle")->attachment.parentLink == "mover");
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("zone ops: adding derives a unique id and takes label, pose, size and properties from the kind") {
    const auto cat = catalog();
    auto c = robot();
    c.zones.clear();
    const auto* pickup = cat.findZoneKind(kPickup);
    REQUIRE(pickup != nullptr);

    const std::string id = ops::addZone(c, cat, *pickup, {});
    CHECK(id == "pickup");
    const sdk::ZoneConfig* z = c.findZone(id);
    REQUIRE(z != nullptr);
    CHECK(z->kind == kPickup);
    CHECK(z->label == "Pickup");
    CHECK(z->attachment.onChassis());
    CHECK(approx(z->local.position, pickup->defaultLocal.position));
    CHECK(approx(z->sizeM, pickup->defaultSizeM));
    CHECK(z->properties.find("capacity")->asInt() == 30);
    CHECK(z->activation.conditions.empty());
    CHECK(ops::addZone(c, cat, *pickup, {}) == "pickup_2");

    // A zone id never equals a mechanism id (the validator rejects that), and attachments fall back to the chassis.
    const auto* shooter = cat.findZoneKind(kShooter);
    REQUIRE(shooter != nullptr);
    const std::string muzzle = ops::addZone(c, cat, *shooter, {"turret", "mover"});
    CHECK(muzzle == "shooter_2");   // "shooter" is the sample mechanism
    CHECK(c.findZone(muzzle)->attachment.parentLink == "mover");
    CHECK(ops::addZone(c, cat, *shooter, {"ghost", "root"}).size() > 0);
    CHECK(c.zones.back().attachment.onChassis());
    CHECK(ops::uniqueMechanismId(c, "pickup") == "pickup_3");   // mechanisms avoid zone ids as well
}

TEST_CASE("zone ops: duplicate copies attachment and activation, remove deletes") {
    auto c = robot();
    const std::string copy = ops::duplicateZone(c, "muzzle");
    CHECK(copy == "muzzle_2");
    const auto* z = c.findZone(copy);
    REQUIRE(z != nullptr);
    CHECK(z->attachment == c.findZone("muzzle")->attachment);
    CHECK(z->activation == c.findZone("muzzle")->activation);
    CHECK(z->label == "Shooter muzzle (copy)");
    CHECK(ops::duplicateZone(c, "nobody").empty());
    CHECK(ops::removeZone(c, copy));
    CHECK_FALSE(ops::removeZone(c, copy));
    CHECK(c.zones.size() == 2);
}

TEST_CASE("zone ops: re-attaching keeps the world pose and refuses unknown parents or links") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 before = zoneWorldPos(c, cat, "pickup");

    REQUIRE(ops::reattachZone(c, cat, "pickup", "turret", "mover"));
    CHECK(c.findZone("pickup")->attachment.parentMechanismId == "turret");
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));
    REQUIRE(ops::reattachZone(c, cat, "pickup", "chassis", "root"));
    CHECK(c.findZone("pickup")->attachment.onChassis());
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));

    CHECK_FALSE(ops::reattachZone(c, cat, "pickup", "ghost", "root"));
    CHECK_FALSE(ops::reattachZone(c, cat, "pickup", "turret", "no_such_link"));
    CHECK_FALSE(ops::reattachZone(c, cat, "pickup", "chassis", "mover"));
    CHECK_FALSE(ops::reattachZone(c, cat, "ghost", "chassis", "root"));
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));

    // Without keeping the world pose the local pose is untouched and the zone jumps.
    const sdk::Pose3 local = c.findZone("pickup")->local;
    REQUIRE(ops::reattachZone(c, cat, "pickup", "turret", "mover", false));
    CHECK(approx(c.findZone("pickup")->local.position, local.position));
}

TEST_CASE("zone ops: deleting a mechanism keeps its zones in place and conditions keep the id") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 muzzleBefore = zoneWorldPos(c, cat, "muzzle");
    const sdk::Vec3 pickupBefore = zoneWorldPos(c, cat, "pickup");

    REQUIRE(ops::removeMechanism(c, cat, "turret"));
    CHECK(c.findZone("muzzle")->attachment.onChassis());   // the turret hung on the chassis
    CHECK(approx(zoneWorldPos(c, cat, "muzzle"), muzzleBefore));

    REQUIRE(ops::removeMechanism(c, cat, "intake"));
    CHECK(c.findZone("pickup")->attachment.onChassis());
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), pickupBefore));
    CHECK(c.findZone("pickup")->activation.conditions.at(0).mechanismId == "intake");   // dangling, the validator warns

    FakeEditorBackend backend;
    bool warned = false;
    for (const auto& issue : backend.validate(c)) {
        warned = warned || issue.code == "zone_condition_mechanism_unknown";
        CHECK(issue.severity != sdk::Severity::Error);
    }
    CHECK(warned);

    // A zone on a mechanism below another one moves up to that parent, not to the chassis.
    auto d = robot();
    REQUIRE(ops::reattachZone(d, cat, "pickup", "shooter", "root"));
    const sdk::Vec3 pickup2 = zoneWorldPos(d, cat, "pickup");
    REQUIRE(ops::removeMechanism(d, cat, "shooter"));
    CHECK(d.findZone("pickup")->attachment.parentMechanismId == "turret");
    CHECK(d.findZone("pickup")->attachment.parentLink == "mover");
    CHECK(approx(zoneWorldPos(d, cat, "pickup"), pickup2));
}

TEST_CASE("zone ops: moving the COM keeps zones on the chassis where they are") {
    const auto cat = catalog();
    auto c = robot();
    REQUIRE(ops::reattachZone(c, cat, "pickup", "chassis", "root"));
    const sdk::Vec3 before = zoneWorldPos(c, cat, "pickup");
    ops::setCom(c, 0.05f, -0.03f, 0.30f, true, &cat);
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));
}

TEST_CASE("zone ops: activation edits add, change, remove and switch the mode") {
    auto c = robot();
    CHECK(ops::addZoneCondition(c, "pickup") == 1);
    CHECK(c.findZone("pickup")->activation.conditions[1].mechanismId == c.mechanisms.front().id);
    CHECK(ops::addZoneCondition(c, "ghost") == -1);

    sdk::ZoneCondition cond = c.findZone("pickup")->activation.conditions[1];
    cond.value = "position";
    cond.op = sdk::ConditionOp::Within;
    cond.threshold = 1.5;
    cond.tolerance = 0.2;
    cond.absolute = false;
    CHECK(ops::setZoneCondition(c, "pickup", 1, cond));
    CHECK(c.findZone("pickup")->activation.conditions[1] == cond);
    CHECK_FALSE(ops::setZoneCondition(c, "pickup", 1, cond));   // unchanged
    CHECK_FALSE(ops::setZoneCondition(c, "pickup", 5, cond));

    CHECK(ops::setZoneActivationMode(c, "pickup", sdk::ActivationMode::Any));
    CHECK_FALSE(ops::setZoneActivationMode(c, "pickup", sdk::ActivationMode::Any));
    CHECK(ops::removeZoneCondition(c, "pickup", 0));
    CHECK_FALSE(ops::removeZoneCondition(c, "pickup", 1));
    CHECK(c.findZone("pickup")->activation.conditions.size() == 1);
}

// ---- document and controller -------------------------------------------------------------------------

TEST_CASE("zone document: add, re-attach and activation edits are undoable") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());

    const QString id = doc.addZone(kShooter, {"turret", "mover"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(doc.config().zones.size() == 3);
    CHECK(doc.addZone("demo_game.nope").isEmpty());
    doc.undoStack()->undo();
    CHECK(doc.config().zones.size() == 2);
    doc.undoStack()->redo();
    CHECK(doc.config().findZone(id.toStdString()) != nullptr);

    const sdk::Vec3 before = zoneWorldPos(doc.config(), doc.catalog(), "pickup");
    REQUIRE(doc.reattachZone("pickup", "chassis"));
    CHECK(approx(zoneWorldPos(doc.config(), doc.catalog(), "pickup"), before));
    doc.undoStack()->undo();
    CHECK(doc.config().findZone("pickup")->attachment.parentMechanismId == "intake");

    const int index = doc.addZoneCondition("pickup");
    CHECK(index == 1);
    sdk::ZoneCondition cond = doc.config().findZone("pickup")->activation.conditions[1];
    const int steps = doc.undoStack()->count();
    cond.threshold = 4;
    REQUIRE(doc.setZoneCondition("pickup", 1, cond));
    cond.threshold = 5;
    REQUIRE(doc.setZoneCondition("pickup", 1, cond));
    CHECK(doc.undoStack()->count() == steps + 1);   // typing in one field is one undo step
    CHECK(doc.setZoneActivationMode("pickup", sdk::ActivationMode::Any));
    doc.undoStack()->undo();
    CHECK(doc.config().findZone("pickup")->activation.mode == sdk::ActivationMode::All);
    CHECK(doc.removeZoneCondition("pickup", 0));
    CHECK(doc.config().findZone("pickup")->activation.conditions.size() == 1);
    doc.undoStack()->undo();
    CHECK(doc.config().findZone("pickup")->activation.conditions.size() == 2);
}

TEST_CASE("zone controller: adding selects the zone and delete / duplicate act on it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    const QString id = c.addZone(kPickup, {"intake", "root"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(c.selection().current() == SelectionRef::zone(id.toStdString()));
    CHECK(c.selectionExists(SelectionRef::zone(id.toStdString())));
    CHECK(c.addZone("demo_game.nope").isEmpty());

    REQUIRE(c.duplicateSelection());
    const SelectionRef copy = c.selection().current();
    CHECK(copy.kind == SelKind::Zone);
    CHECK(copy.owner != id.toStdString());
    CHECK(c.document().config().zones.size() == 4);

    REQUIRE(c.deleteSelection());
    CHECK(c.document().config().zones.size() == 3);
    CHECK(c.selection().current() == SelectionRef::robot());   // the selected zone is gone
}

// ---- project tree ------------------------------------------------------------------------------------

TEST_CASE("zone tree: zones are listed under the plugin node with their attachment and keep its volumes") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex plugin = tree.indexForKey("plugin");
    REQUIRE(plugin.isValid());
    const auto volumes = ops::volumesFor(doc.config(), doc.catalog(), "");
    CHECK(tree.rowCount(plugin) == static_cast<int>(volumes.size() + 2));
    CHECK(tree.refAt(tree.index(0, 0, plugin)).kind == SelKind::Overlay);   // existing volume children stay first

    const QModelIndex pickup = tree.indexFor(SelectionRef::zone("pickup"));
    const QModelIndex muzzle = tree.indexFor(SelectionRef::zone("muzzle"));
    REQUIRE(pickup.isValid());
    CHECK(tree.parent(pickup) == plugin);
    CHECK(tree.data(pickup).toString() == "Pickup");
    CHECK(tree.data(pickup, ProjectTreeModel::KindRole).toInt() == static_cast<int>(NodeKind::Zone));
    CHECK(tree.data(pickup, ProjectTreeModel::DetailRole).toString() == "@ intake");
    CHECK(tree.data(muzzle, ProjectTreeModel::DetailRole).toString() == "@ turret.mover");

    REQUIRE(doc.reattachZone("pickup", "chassis"));
    tree.refresh();
    CHECK(tree.data(tree.indexFor(SelectionRef::zone("pickup")), ProjectTreeModel::DetailRole).toString() == "@ chassis");
    CHECK(doc.addZone(kShooter).size() > 0);
    tree.refresh();
    const QModelIndex pluginAfter = tree.indexForKey("plugin");
    CHECK(tree.rowCount(pluginAfter) == static_cast<int>(volumes.size() + 3));
}

TEST_CASE("zone tree: validator issues put a warning on the zone they point at") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);
    const auto warning = [&](const char* id) {
        return tree.data(tree.indexFor(SelectionRef::zone(id)), ProjectTreeModel::WarningRole).toBool();
    };
    CHECK_FALSE(warning("pickup"));
    tree.setIssues({{sdk::Severity::Warning, "zones[1].activation.conditions[0].mechanism", "missing", "zone_condition_mechanism_unknown"}});
    CHECK(warning("muzzle"));
    CHECK_FALSE(warning("pickup"));
    tree.setIssues({});
    CHECK_FALSE(warning("muzzle"));
}

TEST_CASE("zone tree: dragging a zone onto a mechanism or the chassis offers re-attaching it") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex pickup = tree.indexFor(SelectionRef::zone("pickup"));
    CHECK((tree.flags(pickup) & Qt::ItemIsDragEnabled));
    std::unique_ptr<QMimeData> mime(tree.mimeData({pickup}));
    REQUIRE(mime != nullptr);

    const QModelIndex turret = tree.indexFor(SelectionRef::mechanism("turret"));
    const QModelIndex intake = tree.indexFor(SelectionRef::mechanism("intake"));
    const QModelIndex chassis = tree.indexFor(SelectionRef::chassis());
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, turret));
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, chassis));
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, intake));   // already there
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::zone("muzzle"))));

    QString droppedId;
    sdk::MechanismAttachment droppedOn;
    QObject::connect(&tree, &ProjectTreeModel::zoneDropped, &tree, [&](const QString& id, const sdk::MechanismAttachment& att) {
        droppedId = id;
        droppedOn = att;
    });
    REQUIRE(tree.dropMimeData(mime.get(), Qt::MoveAction, -1, -1, turret));
    CHECK(droppedId == "pickup");
    CHECK(droppedOn.parentMechanismId == "turret");
}

// ---- property groups and the inspector ---------------------------------------------------------------

TEST_CASE("zone property groups: general, placement and the kind's properties apply to the zone") {
    const auto cat = catalog();
    auto c = robot();
    const SelectionRef sel = SelectionRef::zone("pickup");

    std::vector<std::string> ids;
    for (const auto& g : propertyGroupsFor(c, cat, sel)) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"zone.general", "zone.placement", "zone.props"});
    const auto groups = propertyGroupsFor(c, cat, sel);
    CHECK(groups[1].schema.find("sizeM") != nullptr);
    CHECK(groups[2].schema.find("capacity") != nullptr);
    for (const auto& g : propertyGroupsFor(c, cat, SelectionRef::zone("muzzle")))
        if (g.id == "zone.placement") CHECK(g.schema.find("sizeM") == nullptr);   // a muzzle has no size
    CHECK(propertyGroupsFor(c, cat, SelectionRef::zone("ghost")).empty());
    CHECK(describeSelection(c, cat, sel) == "Pickup");

    CHECK(applyProperty(c, cat, sel, "zone.general", "label", sdk::Value("Front pickup")));
    CHECK(c.findZone("pickup")->label == "Front pickup");
    CHECK(applyProperty(c, cat, sel, "zone.props", "capacity", sdk::Value(12)));
    CHECK(c.findZone("pickup")->properties.find("capacity")->asInt() == 12);
    CHECK(applyProperty(c, cat, sel, "zone.placement", "sizeM", sdk::toValue(sdk::Vec3{1.f, 2.f, 3.f})));
    CHECK(approx(c.findZone("pickup")->sizeM, {1.f, 2.f, 3.f}));
    sdk::Pose3 pose;
    pose.position = {0.1f, 0.2f, 0.3f};
    CHECK(applyProperty(c, cat, sel, "zone.placement", "local", sdk::toValue(pose)));
    CHECK(approx(c.findZone("pickup")->local.position, pose.position));

    const sdk::Vec3 before = zoneWorldPos(c, cat, "pickup");
    CHECK(applyProperty(c, cat, sel, "zone.placement", "parent", sdk::Value("turret")));   // keeps the world pose
    CHECK(c.findZone("pickup")->attachment.parentMechanismId == "turret");
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));
    CHECK(applyProperty(c, cat, sel, "zone.placement", "parentLink", sdk::Value("mover")));
    CHECK(approx(zoneWorldPos(c, cat, "pickup"), before));
    CHECK_FALSE(applyProperty(c, cat, sel, "zone.placement", "parentLink", sdk::Value("nowhere")));
}

TEST_CASE("zone inspector: the page shows General, Placement, Activation and Configure cards") {
    ZonePage f;
    f.c().selection().set(SelectionRef::zone("pickup"));
    pumpEvents(30);
    auto* insp = f.page.inspector();
    CHECK(insp->titleText() == "Pickup");   // the kind's display name
    for (const char* card : {"card.info", "card.placement", "card.activation", "card.configure"})
        CHECK(insp->findChild<QFrame*>(card) != nullptr);
    CHECK(insp->form("zone.general")->hasField("label"));
    CHECK(insp->form("zone.placement")->hasField("sizeM"));
    CHECK(insp->form("zone.props")->hasField("capacity"));
    CHECK(insp->findChild<QLabel*>("zone.id")->text().contains("pickup"));
    CHECK(insp->activationCard() != nullptr);

    f.c().selection().set(SelectionRef::zone("muzzle"));
    pumpEvents(30);
    CHECK_FALSE(insp->form("zone.placement")->hasField("sizeM"));

    // Editing the label goes through the document (undoable).
    auto* label = insp->form("zone.general")->editorFor("label");
    emit label->edited(sdk::Value("Barrel"));
    CHECK(f.c().document().config().findZone("muzzle")->label == "Barrel");
    f.c().document().undoStack()->undo();
    CHECK(f.c().document().config().findZone("muzzle")->label == "Shooter muzzle");
}

TEST_CASE("zone inspector: the Activation card adds, edits and removes condition rows") {
    ZonePage f;
    auto cfg = robot();
    cfg.zones[0].activation.conditions.clear();
    f.backend.setConfig(cfg);
    pumpEvents(30);
    f.c().selection().set(SelectionRef::zone("pickup"));
    pumpEvents(30);
    auto* insp = f.page.inspector();
    auto* card = insp->activationCard();
    REQUIRE(card != nullptr);
    const auto zone = [&]() -> const sdk::ZoneConfig& { return *f.c().document().config().findZone("pickup"); };

    CHECK(card->findChild<QLabel*>("activation.empty")->isVisibleTo(card));   // "Always active"
    CHECK(card->conditionCount() == 0);

    card->findChild<QToolButton*>("activation.add")->click();
    pumpEvents(30);   // new rows are shown by the layout from the event loop
    REQUIRE(zone().activation.conditions.size() == 1);
    CHECK(card->conditionCount() == 1);
    CHECK_FALSE(card->findChild<QLabel*>("activation.empty")->isVisibleTo(card));
    CHECK(card->findChild<QFrame*>("activation.row:0") != nullptr);

    auto* mechanism = card->findChild<QComboBox*>("activation.mechanism:0");
    mechanism->setCurrentIndex(mechanism->findData("turret"));
    CHECK(zone().activation.conditions[0].mechanismId == "turret");

    auto* value = card->findChild<QLineEdit*>("activation.value:0");
    value->setText("position");
    emit value->editingFinished();
    CHECK(zone().activation.conditions[0].value == "position");

    auto* op = card->findChild<QComboBox*>("activation.op:0");
    auto* tolerance = card->findChild<QDoubleSpinBox*>("activation.tolerance:0");
    auto* absolute = card->findChild<QCheckBox*>("activation.absolute:0");
    CHECK(absolute->isVisibleTo(card));
    CHECK_FALSE(tolerance->isVisibleTo(card));
    op->setCurrentIndex(op->findData(static_cast<int>(sdk::ConditionOp::Within)));
    CHECK(zone().activation.conditions[0].op == sdk::ConditionOp::Within);
    CHECK(tolerance->isVisibleTo(card));
    CHECK_FALSE(absolute->isVisibleTo(card));
    card->findChild<QDoubleSpinBox*>("activation.threshold:0")->setValue(12.5);
    tolerance->setValue(0.25);
    CHECK(zone().activation.conditions[0].threshold == doctest::Approx(12.5));
    CHECK(zone().activation.conditions[0].tolerance == doctest::Approx(0.25));
    op->setCurrentIndex(op->findData(static_cast<int>(sdk::ConditionOp::Below)));
    absolute->setChecked(false);
    CHECK_FALSE(zone().activation.conditions[0].absolute);

    auto* mode = card->findChild<QComboBox*>("activation.mode");
    mode->setCurrentIndex(mode->findData(static_cast<int>(sdk::ActivationMode::Any)));
    CHECK(zone().activation.mode == sdk::ActivationMode::Any);

    card->findChild<QToolButton*>("activation.remove:0")->click();
    CHECK(zone().activation.conditions.empty());
    CHECK(card->conditionCount() == 0);
    CHECK(card->findChild<QLabel*>("activation.empty")->isVisibleTo(card));

    f.c().document().undoStack()->undo();   // the removed row comes back with its values
    REQUIRE(zone().activation.conditions.size() == 1);
    CHECK(card->conditionCount() == 1);
    CHECK(card->findChild<QDoubleSpinBox*>("activation.threshold:0")->value() == doctest::Approx(12.5));
    CHECK(card->findChild<QLineEdit*>("activation.value:0")->text() == "position");
}

TEST_CASE("zone inspector: a condition on a deleted mechanism stays listed and is flagged") {
    ZonePage f;
    REQUIRE(f.c().document().removeMechanism("intake"));
    f.c().selection().set(SelectionRef::zone("pickup"));
    pumpEvents(250);   // validation is debounced
    auto* card = f.page.inspector()->activationCard();
    REQUIRE(card != nullptr);
    auto* mechanism = card->findChild<QComboBox*>("activation.mechanism:0");
    CHECK(mechanism->currentData().toString() == "intake");
    CHECK(mechanism->currentText().contains("missing"));
    CHECK(card->findChild<QLabel*>("activation.badge:0")->isVisibleTo(card));
}

// ---- validation --------------------------------------------------------------------------------------

TEST_CASE("zone validation: issue paths are located to their zone and field") {
    const auto cat = catalog();
    const auto c = robot();
    const auto at = [&](const std::string& path) { return locateIssue(c, cat, {sdk::Severity::Error, path, "x", {}}); };
    CHECK(at("zones[1].attachment").selection == SelectionRef::zone("muzzle"));
    CHECK(at("zones[1].attachment").fieldKey == "parent");
    CHECK(at("zones[0].properties.capacity").fieldKey == "capacity");
    CHECK(at("zones[0].local").fieldKey == "local");
    CHECK(at("zones[0].sizeM").fieldKey == "sizeM");
    CHECK(at("zones[0].kind").fieldKey == "kind");
    CHECK(at("zones[0].id").fieldKey == "id");
    CHECK(at("zones[1].activation.conditions[1].mechanism").selection == SelectionRef::zone("muzzle"));
    CHECK(at("zones[1].activation.conditions[1].mechanism").fieldKey == "activation:1");
    CHECK(at("zones[9].attachment").selection.isNone());
}

// ---- viewport ----------------------------------------------------------------------------------------

TEST_CASE("zone scene: volumes are translucent boxes in the kind colour, muzzles a marker with an arrow along +X") {
    const auto cat = catalog();
    const auto c = robot();
    const auto scene = buildScene(c, cat, SceneTheme::dark());

    const auto* pickup = scene.find("zone:pickup");
    REQUIRE(pickup != nullptr);
    CHECK(pickup->ref == SelectionRef::zone("pickup"));
    CHECK(pickup->pickable);
    CHECK(pickup->opacity < 0.5f);
    CHECK(pickup->color == QColor(QString::fromStdString(cat.findZoneKind(kPickup)->color)));
    CHECK(approx(pickup->size, c.findZone("pickup")->sizeM));
    // Link pose * local: the pickup rides on the intake.
    CHECK(approx(pickup->world.position, (ops::mechanismRootWorld(c, cat, "intake") * c.findZone("pickup")->local).position));

    const auto* muzzle = scene.find("zone:muzzle");
    const auto* shaft = scene.find("zone:muzzle:dir");
    const auto* head = scene.find("zone:muzzle:head");
    REQUIRE((muzzle && shaft && head));
    CHECK(muzzle->ref == SelectionRef::zone("muzzle"));
    CHECK(approx(muzzle->world.position, zoneWorldPos(c, cat, "muzzle")));
    CHECK(shaft->ref == SelectionRef::zone("muzzle"));
    const sdk::Pose3 world = ops::zoneWorld(c, cat, *c.findZone("muzzle"));
    const sdk::Vec3 dir = sdk::normalized(head->world.position - muzzle->world.position);
    CHECK(approx(dir, world.rotation.rotate({1.f, 0.f, 0.f}), 1e-3f));

    // Moving the turret's link moves the muzzle with it.
    auto moved = c;
    for (auto& m : moved.mechanisms)
        if (m.id == "turret") m.mount.position.x += 0.2f;
    CHECK(approx(buildScene(moved, cat, SceneTheme::dark()).find("zone:muzzle")->world.position.x, muzzle->world.position.x + 0.2f, 1e-3f));

    std::vector<std::string> ids;
    for (const auto& i : scene.items) ids.push_back(i.id);
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
}

TEST_CASE("zone transform: pose and size round-trip, a muzzle has no size") {
    const auto cat = catalog();
    auto c = robot();
    const auto volume = readTransform(c, cat, SelectionRef::zone("pickup"));
    REQUIRE(volume);
    CHECK(volume->transform.canScale);
    CHECK(volume->transform.sizeKind == SizeKind::Box);
    CHECK(approx(volume->world().position, zoneWorldPos(c, cat, "pickup")));

    EditableTransform t = volume->transform;
    t.local.position = {0.5f, 0.1f, 0.2f};
    t.size = {0.7f, 0.8f, 0.9f};
    REQUIRE(writeTransform(c, cat, SelectionRef::zone("pickup"), t));
    CHECK(approx(c.findZone("pickup")->local.position, {0.5f, 0.1f, 0.2f}));
    CHECK(approx(c.findZone("pickup")->sizeM, {0.7f, 0.8f, 0.9f}));

    const auto muzzle = readTransform(c, cat, SelectionRef::zone("muzzle"));
    REQUIRE(muzzle);
    CHECK_FALSE(muzzle->transform.canScale);
    CHECK(muzzle->transform.canRotate);
    const sdk::Vec3 size = c.findZone("muzzle")->sizeM;
    EditableTransform m = muzzle->transform;
    m.local.position.x += 0.1f;
    m.size = {9.f, 9.f, 9.f};
    REQUIRE(writeTransform(c, cat, SelectionRef::zone("muzzle"), m));
    CHECK(approx(c.findZone("muzzle")->sizeM, size));
    CHECK_FALSE(readTransform(c, cat, SelectionRef::zone("ghost")));
}

TEST_CASE("zone viewport: picking, gizmo modes and a drag that is one undo step") {
    ViewFixture f;
    CHECK(f.view.chooseItem({"i:zone:pickup", "i:hull"}) == SelectionRef::chassis());   // solid parts win over volumes
    CHECK(f.view.chooseItem({"i:zone:pickup"}) == SelectionRef::zone("pickup"));
    CHECK(f.view.chooseItem({"i:zone:muzzle:dir"}) == SelectionRef::zone("muzzle"));

    f.editor.selection().set(SelectionRef::zone("muzzle"));
    REQUIRE(f.view.effectiveMode());
    f.view.setGizmoMode(GizmoMode::Scale);
    CHECK(*f.view.effectiveMode() == GizmoMode::Move);   // a muzzle cannot scale
    f.view.setGizmoMode(GizmoMode::Rotate);
    CHECK(*f.view.effectiveMode() == GizmoMode::Rotate);
    CHECK_FALSE(f.view.selectionEdges().isEmpty());

    f.editor.selection().set(SelectionRef::zone("pickup"));
    f.view.setGizmoMode(GizmoMode::Scale);
    CHECK(*f.view.effectiveMode() == GizmoMode::Scale);

    f.view.setGizmoMode(GizmoMode::Move);
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::zone("pickup"));
    REQUIRE(start);
    const sdk::Vec3 origin = start->world().position;
    const sdk::Vec3 axis = start->parentWorld.rotation.rotate({1.f, 0.f, 0.f});
    const QPointF p0 = f.screen(origin + axis * 0.2f);
    REQUIRE(f.view.beginGizmoDrag("h:m:0", p0.x(), p0.y()));
    for (int i = 1; i <= 4; ++i) {
        const QPointF p = f.screen(origin + axis * (0.2f + 0.05f * static_cast<float>(i)));
        f.view.updateGizmoDrag(p.x(), p.y(), false);
    }
    f.view.endGizmoDrag();
    CHECK(approx(f.editor.document().config().findZone("pickup")->local.position.x, start->transform.local.position.x + 0.2f, 2e-3f));
    CHECK(f.editor.document().undoStack()->count() == 1);
    f.editor.document().undoStack()->undo();
    CHECK(approx(f.editor.document().config().findZone("pickup")->local.position.x, start->transform.local.position.x));
}
