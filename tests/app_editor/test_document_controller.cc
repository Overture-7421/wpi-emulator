#include <QMimeData>

#include <vector>

#include "test_support.h"
#include "model/editor_document.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "viewmodel/detected_devices_model.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/issue_model.h"
#include "viewmodel/project_tree_model.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

struct DocFixture {
    EditorDocument doc;
    int changed = 0;
    int modified = 0;
    DocFixture() {
        doc.setCatalog(catalog());
        doc.load(robot());
        QObject::connect(&doc, &EditorDocument::changed, &doc, [this] { ++changed; });
        QObject::connect(&doc, &EditorDocument::modified, &doc, [this] { ++modified; });
    }
};

}  // namespace

TEST_CASE("document: every edit is undoable and redoable") {
    DocFixture f;
    const auto before = f.doc.config().mechanisms.size();
    const QString id = f.doc.addMechanism("mechanism", {});
    CHECK(id == "mechanism");
    CHECK(f.doc.config().mechanisms.size() == before + 1);
    CHECK(f.doc.undoStack()->count() == 1);
    CHECK(f.modified == 1);

    f.doc.undoStack()->undo();
    CHECK(f.doc.config().mechanisms.size() == before);
    f.doc.undoStack()->redo();
    CHECK(f.doc.config().mechanisms.size() == before + 1);
    CHECK(f.doc.undoStack()->undoText().contains("Mechanism"));
}

TEST_CASE(
    "document: motion links follow duplication and remain repairable after deletion") {
    DocFixture f;
    f.doc.edit("Link motion", [](sdk::RobotConfig& c) {
        c.motionLinks = {{"follow", {"turret", "axis"}, {"intake", "axis"}, -2}};
        return true;
    });
    const auto copied = f.doc.duplicateMechanism("intake");
    REQUIRE_FALSE(copied.isEmpty());
    REQUIRE(f.doc.config().motionLinks.size() == 2);
    CHECK(f.doc.config().motionLinks[1].target.owner == copied.toStdString());
    CHECK(f.doc.config().motionLinks[1].source.owner == "turret");
    f.doc.undoStack()->undo();
    CHECK(f.doc.config().motionLinks.size() == 1);
    REQUIRE(f.doc.removeMechanism("turret"));
    CHECK(f.doc.config().motionLinks.size() == 1);
    f.doc.undoStack()->undo();
    CHECK(f.doc.config().findMechanism("turret") != nullptr);
}

TEST_CASE("document: load clears history and does not count as a user edit") {
    DocFixture f;
    f.doc.addMechanism("mechanism", {});
    f.modified = 0;
    f.doc.load(robot());
    CHECK(f.doc.undoStack()->count() == 0);
    CHECK(f.modified == 0);
    CHECK(f.changed >= 1);
}

TEST_CASE("document: rapid edits of the same property merge into one undo step, different ones do not") {
    DocFixture f;
    const auto sel = SelectionRef::mechanism("turret");
    f.doc.setProperty(sel, "mech.props", "gearRatio", sdk::Value(41.0));
    f.doc.setProperty(sel, "mech.props", "gearRatio", sdk::Value(42.0));
    f.doc.setProperty(sel, "mech.props", "gearRatio", sdk::Value(43.0));
    CHECK(f.doc.undoStack()->count() == 1);
    f.doc.setProperty(sel, "mech.props", "inertiaKgM2", sdk::Value(0.1));
    CHECK(f.doc.undoStack()->count() == 2);
    CHECK(f.doc.config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 43.0);

    f.doc.undoStack()->undo();
    f.doc.undoStack()->undo();
    CHECK(f.doc.config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 40.0);   // template default
    CHECK(f.doc.undoStack()->canRedo());
}

TEST_CASE("document: a gizmo drag is a single undo step") {
    DocFixture f;
    auto t = readTransform(f.doc.config(), f.doc.catalog(), SelectionRef::mechanism("intake"));
    REQUIRE(t);
    const float startX = t->transform.local.position.x;
    for (int i = 1; i <= 10; ++i) {
        auto step = t->transform;
        step.local.position.x = startX + 0.01f * static_cast<float>(i);
        f.doc.setTransform(SelectionRef::mechanism("intake"), step, "g1");
    }
    CHECK(f.doc.undoStack()->count() == 1);
    f.doc.setTransform(SelectionRef::mechanism("intake"), t->transform, "g2");   // next drag
    CHECK(f.doc.undoStack()->count() == 2);
    f.doc.undoStack()->undo();
    CHECK(approx(f.doc.config().findMechanism("intake")->mount.position.x, startX + 0.1f));
    f.doc.undoStack()->undo();
    CHECK(approx(f.doc.config().findMechanism("intake")->mount.position.x, startX));
}

TEST_CASE("document: slot mapping, structure edits and shapes go through the undo stack") {
    DocFixture f;
    const sdk::DeviceKey key{"phoenix6", "talonfx", 12, "canivore", -1};
    CHECK(f.doc.mapSlot({"shooter", "feeder"}, key));
    CHECK(ops::findMapping(f.doc.config(), {"shooter", "feeder"}) != nullptr);
    CHECK(f.doc.clearSlot({"shooter", "feeder"}));
    CHECK_FALSE(f.doc.clearSlot({"shooter", "feeder"}));    // nothing to clear: no empty undo entry
    CHECK(f.doc.undoStack()->count() == 2);

    CHECK(f.doc.reparentMechanism("climber", "turret", "mover"));
    CHECK(f.doc.duplicateMechanism("indexer") == "indexer_2");
    CHECK(f.doc.removeMechanism("indexer_2"));
    CHECK(f.doc.addPrimitive("chassis", sdk::PrimitiveKind::Box) >= 0);
    CHECK(f.doc.addWheel() == "wheel");
    CHECK(f.doc.removeWheel("wheel"));
    CHECK(f.doc.addMechanism("does.not.exist", {}).isEmpty());

    while (f.doc.undoStack()->canUndo()) f.doc.undoStack()->undo();
    CHECK(f.doc.config().mechanisms.size() == robot().mechanisms.size());
    CHECK(f.doc.config().chassis.primitives.size() == robot().chassis.primitives.size());
}

TEST_CASE("document: setProperty on a COM key compensates and the whole change undoes at once") {
    DocFixture f;
    const sdk::Vec3 wheel = ops::wheelWorld(f.doc.config().chassis, f.doc.config().drivetrain.spec.wheels[0]).position;
    f.doc.setProperty(SelectionRef::chassis(), "chassis", "comOffsetX", sdk::Value(0.08));
    CHECK(approx(ops::wheelWorld(f.doc.config().chassis, f.doc.config().drivetrain.spec.wheels[0]).position, wheel));
    f.doc.undoStack()->undo();
    CHECK(approx(f.doc.config().chassis.comOffsetM.x, 0.f));
    CHECK(approx(f.doc.config().drivetrain.spec.wheels[0].positionM.x, 0.28f));
}

// ---- controller ------------------------------------------------------------------------------------

TEST_CASE("controller: edits are pushed to the backend and autosaved after the delay") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(15);
    int autosaves = 0;
    QObject::connect(&c, &EditorController::autosaveRequested, &c, [&] { ++autosaves; });

    CHECK(backend.submitCount() == 0);
    c.addMechanism("mechanism", {});
    CHECK(backend.submitCount() == 1);
    CHECK(backend.lastSubmitted().findMechanism("mechanism") != nullptr);
    CHECK(c.hasPendingChanges());
    CHECK(c.saveStatusText().contains("Unsaved"));

    pumpEvents(120);
    CHECK(autosaves == 1);
    CHECK(backend.saveCount() == 1);
    CHECK_FALSE(c.hasPendingChanges());

    c.addMechanism("mechanism", {});
    CHECK(c.flushSave().ok());   // Ctrl+S does not wait for the timer
    CHECK(backend.saveCount() == 2);
}

TEST_CASE("controller: undo and redo also reach the backend") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    c.addMechanism("mechanism", {});
    c.document().undoStack()->undo();
    CHECK(backend.lastSubmitted().findMechanism("mechanism") == nullptr);
    c.document().undoStack()->redo();
    CHECK(backend.lastSubmitted().findMechanism("mechanism") != nullptr);
}

TEST_CASE("controller: failed autosave stays dirty until a successful explicit retry") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(15);
    backend.setSaveError(QStringLiteral("disk full"));
    int saved = 0;
    QObject::connect(&c, &EditorController::autosaveRequested, &c, [&] { ++saved; });
    c.addMechanism("mechanism", {});
    pumpEvents(120);
    CHECK(c.hasPendingChanges());
    CHECK(c.saveStatusText().contains("Save failed"));
    CHECK_FALSE(c.saveStatusText().contains("Saved "));
    CHECK(saved == 0);
    CHECK_FALSE(c.flushSave().ok());
    backend.setSaveError({});
    CHECK(c.flushSave().ok());
    CHECK_FALSE(c.hasPendingChanges());
    CHECK(c.saveStatusText().contains("Saved "));
    CHECK(saved == 1);
}

TEST_CASE("controller: a replaced model reloads without echoing it back and resets the history") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.addMechanism("mechanism", {});
    const int submits = backend.submitCount();
    auto other = robot();
    other.name = "Other robot";
    other.mechanisms.pop_back();
    backend.setConfig(other);
    CHECK(c.document().config().name == "Other robot");
    CHECK(c.document().undoStack()->count() == 0);
    CHECK(backend.submitCount() == submits);
    CHECK(c.selection().current() == SelectionRef::robot());
}

TEST_CASE("controller: mapping a device onto a slot, a mechanism or a wheel") {
    FakeEditorBackend backend;
    EditorController c(backend);
    const auto caps = sdk::CapabilitySet(sdk::Capability::MotorOutput) | sdk::CapabilitySet(sdk::Capability::PositionSensor);
    const sdk::DeviceKey key{"phoenix6", "talonfx", 12, sdk::kBusUnknown, -1};

    // Onto a specific slot.
    CHECK(c.mapDevice(SelectionRef::slot("shooter", "feeder"), key, caps).isEmpty());
    CHECK(ops::findMapping(c.document().config(), {"shooter", "feeder"})->device.id == 12);
    CHECK(c.selection().current() == SelectionRef::slot("shooter", "feeder"));

    // Onto a mechanism: the best free compatible slot is chosen (optional deploy motor of the intake).
    CHECK(c.mapDevice(SelectionRef::mechanism("intake"), {"phoenix6", "talonfx", 13, sdk::kBusUnknown, -1}, caps).isEmpty());
    CHECK(ops::findMapping(c.document().config(), {"intake", "deploy"}) != nullptr);

    // Incompatible and unsupported targets explain themselves.
    const QString imuError = c.mapDevice(SelectionRef::slot("indexer", "motor"), {"phoenix6", "pigeon2", 99, "", -1}, sdk::Capability::Imu);
    CHECK(imuError.contains("cannot act as"));
    CHECK(c.mapDevice(SelectionRef::chassis(), key, caps).contains("Drop a device"));
    CHECK(c.mapDevice(SelectionRef::mechanism("shooter"), {"hal", "dio", 4, "rio", -1}, sdk::Capability::DigitalInput).contains("No free slot"));

    // A wheel only considers its own slots; the sample has all of them mapped, so free one first.
    c.document().clearSlot({"drivetrain", "fr.steer"});
    CHECK(c.mapDevice(SelectionRef::wheel("fr"), {"phoenix6", "talonfx", 31, sdk::kBusUnknown, -1}, caps).isEmpty());
    CHECK(ops::findMapping(c.document().config(), {"drivetrain", "fr.steer"}) != nullptr);
}

TEST_CASE("controller: driving a part from another axis is one undoable edit and frees its motor") {
    FakeEditorBackend backend;
    EditorController c(backend);
    REQUIRE(ops::findMapping(c.document().config(), {"intake", "motor"}) != nullptr);
    CHECK(c.setDrivenBy({"intake", "axis"}, {"drivetrain", "fl.drive"}, 2.0).isEmpty());
    REQUIRE(c.document().config().motionLinks.size() == 1);
    CHECK(c.document().config().motionLinks[0].ratio == doctest::Approx(2.0));
    CHECK(ops::findMapping(c.document().config(), {"intake", "motor"}) == nullptr);
    CHECK(c.document().undoStack()->count() == 1);

    CHECK(c.mapDevice(SelectionRef::slot("intake", "motor"), {"phoenix6", "talonfx", 40, "", -1}, sdk::Capability::MotorOutput).contains("motion link"));
    CHECK_FALSE(c.setDrivenBy({"drivetrain", "fl.drive"}, {"intake", "axis"}, 1.0).isEmpty());   // would close a loop
    CHECK_FALSE(c.setDrivenBy({"intake", "axis"}, {"intake", "axis"}, 1.0).isEmpty());          // itself
    CHECK_FALSE(c.setDrivenBy({"intake", "axis"}, {"drivetrain", "fl.drive"}, 0.0).isEmpty());    // zero ratio
    CHECK(c.document().config().motionLinks.size() == 1);

    CHECK(c.clearDrivenBy({"intake", "axis"}));
    CHECK(c.document().config().motionLinks.empty());
    c.document().undoStack()->undo();
    c.document().undoStack()->undo();
    CHECK(c.document().config().motionLinks.empty());
    CHECK(ops::findMapping(c.document().config(), {"intake", "motor"}) != nullptr);
}
TEST_CASE("controller: mapping through a different bus label never creates an alias conflict") {
    FakeEditorBackend backend;
    EditorController c(backend);
    const sdk::CapabilitySet motor(sdk::Capability::MotorOutput);
    // TalonFX 1 is already mapped (on canivore) to fl.drive. Map it to the optional deploy motor via "rio".
    CHECK(c.mapDevice(SelectionRef::slot("intake", "deploy"), {"phoenix6", "talonfx", 1, "rio", -1}, motor).isEmpty());
    CHECK(ops::findMapping(c.document().config(), {"intake", "deploy"})->device.bus == "canivore");
    pumpEvents(250);
    for (const auto& issue : c.issues()) CHECK(issue.code != "device_alias_conflict");
}

TEST_CASE("controller: validation merges backend, local and pushed issues and refreshes after edits") {
    FakeEditorBackend backend;
    EditorController c(backend);
    auto hasCode = [&](const char* code) {
        for (const auto& i : c.issues())
            if (i.code == code) return true;
        return false;
    };
    CHECK(hasCode("slot_unfilled"));
    CHECK_FALSE(hasCode("slot_incompatible"));

    c.document().mapSlot({"intake", "motor"}, {"phoenix6", "pigeon2", 13, "canivore", -1});
    pumpEvents(250);   // validation is coalesced
    CHECK(hasCode("slot_incompatible"));

    backend.pushIssues({{sdk::Severity::Warning, "device:phoenix6/talonfx[1]", "runtime warning", "hub_runtime"}});
    pumpEvents(250);
    CHECK(hasCode("hub_runtime"));
    // Errors sort first.
    CHECK(c.issues().front().severity == sdk::Severity::Error);
}

TEST_CASE("controller: detection streams devices in, raises toasts and updates the directory") {
    FakeEditorBackend backend;
    backend.setDetectionStepMs(5);
    EditorController c(backend);
    const std::size_t known = c.devices().all().size();
    std::vector<QString> toasts;
    bool running = false;
    QObject::connect(&c, &EditorController::deviceToast, &c, [&](const QString&, const QString& detail) { toasts.push_back(detail); });
    QObject::connect(&c, &EditorController::detectionChanged, &c, [&](bool r) { running = r; });

    c.toggleDetection();
    CHECK(running);
    CHECK(c.detecting());
    pumpEvents(250);
    CHECK_FALSE(c.detecting());
    CHECK(c.devices().all().size() == known + 7);
    CHECK(toasts.size() == 7);
    CHECK(c.devices().find(sdk::DeviceKey{"phoenix6", "talonfx", 10, "", -1}) != nullptr);
    // A re-announce of a known device updates it silently.
    toasts.clear();
    auto again = *c.devices().find(sdk::DeviceKey{"phoenix6", "talonfx", 10, "", -1});
    backend.announce(again);
    CHECK(toasts.empty());
}

TEST_CASE("controller: delete, duplicate and clear act on the selection") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.selection().set(SelectionRef::mechanism("indexer"));
    CHECK(c.duplicateSelection());
    CHECK(c.selection().current() == SelectionRef::mechanism("indexer_2"));
    CHECK(c.deleteSelection());
    CHECK(c.selection().current() == SelectionRef::robot());    // the deleted selection falls back to the robot

    c.selection().set(SelectionRef::slot("intake", "motor"));
    CHECK(c.deleteSelection());                                 // delete on a slot unmaps it
    CHECK(ops::findMapping(c.document().config(), {"intake", "motor"}) == nullptr);
    c.selection().set(SelectionRef::chassis());
    CHECK_FALSE(c.deleteSelection());
    CHECK_FALSE(c.duplicateSelection());
}

TEST_CASE("controller: adding with a mechanism selected attaches to it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.selection().set(SelectionRef::mechanism("turret"));
    const QString id = c.addMechanismToSelection("mechanism");
    REQUIRE(id == "mechanism");
    CHECK(c.document().config().findMechanism("mechanism")->attachment.parentMechanismId == "turret");
    CHECK(c.selection().current() == SelectionRef::mechanism("mechanism"));

    c.selection().set(SelectionRef::chassis());
    const QString free = c.addMechanismToSelection("mechanism");
    CHECK(c.document().config().findMechanism(free.toStdString())->attachment.onChassis());
}

TEST_CASE("controller: clicking an issue selects its target and asks the inspector to focus the field") {
    FakeEditorBackend backend;
    EditorController c(backend);
    QString focused;
    QObject::connect(&c.selection(), &SelectionState::fieldFocusRequested, &c, [&](const QString& k) { focused = k; });
    c.focusIssue({sdk::Severity::Error, "mechanisms[2].properties.gearRatio", "bad", {}});
    CHECK(c.selection().current() == SelectionRef::mechanism("turret"));
    CHECK(focused == "gearRatio");
    c.focusIssue({sdk::Severity::Warning, "shooter.feeder", "empty", "slot_unfilled"});
    CHECK(c.selection().current() == SelectionRef::slot("shooter", "feeder"));
}

// ---- item models -----------------------------------------------------------------------------------

namespace {
QModelIndex findRow(const QAbstractItemModel& m, const QString& text, const QModelIndex& parent = {}) {
    for (int r = 0; r < m.rowCount(parent); ++r) {
        const QModelIndex i = m.index(r, 0, parent);
        if (m.data(i).toString() == text) return i;
        const QModelIndex deeper = findRow(m, text, i);
        if (deeper.isValid()) return deeper;
    }
    return {};
}
}  // namespace

TEST_CASE("project tree: structure, slot rows and selection lookup") {
    DocFixture f;
    DeviceDirectory devices;
    devices.setDevices(FakeEditorBackend::sampleDevices());
    ProjectTreeModel tree(f.doc, devices);

    REQUIRE(tree.rowCount() == 1);   // the robot
    const QModelIndex robotIdx = tree.index(0, 0);
    CHECK(tree.data(robotIdx).toString() == "Sample Robot");
    CHECK(tree.refAt(robotIdx) == SelectionRef::robot());

    const QModelIndex turret = tree.indexFor(SelectionRef::mechanism("turret"));
    REQUIRE(turret.isValid());
    const QModelIndex shooter = tree.indexFor(SelectionRef::mechanism("shooter"));
    CHECK(tree.parent(shooter) == turret);                         // children nest under their parent mechanism
    CHECK(tree.data(shooter, ProjectTreeModel::DetailRole).toString().contains("turret.mover"));

    const QModelIndex slot = tree.indexFor(SelectionRef::slot("shooter", "feeder"));
    REQUIRE(slot.isValid());
    CHECK(tree.data(slot, ProjectTreeModel::StateRole).toInt() == static_cast<int>(SlotState::Unmapped));
    CHECK(tree.data(slot, ProjectTreeModel::DetailRole).toString() == "not mapped");
    CHECK(tree.data(shooter, ProjectTreeModel::WarningRole).toBool());   // unfilled required slot below

    const QModelIndex wheelSlot = tree.indexFor(SelectionRef::slot("drivetrain", "fl.drive"));
    REQUIRE(wheelSlot.isValid());
    CHECK(tree.parent(wheelSlot) == tree.indexFor(SelectionRef::wheel("fl")));
    CHECK(tree.indexFor(SelectionRef::overlay("intake", "pickupZone")).isValid());
    CHECK(tree.indexFor(SelectionRef::overlay("", "hopperVolume")).isValid());
    CHECK(tree.indexFor(SelectionRef::com()).isValid());
    CHECK(tree.indexFor(SelectionRef::slot("robot", "imu")).isValid());
}

TEST_CASE("project tree: the drivetrain is a child of the chassis, next to its centre of mass and shapes") {
    DocFixture f;
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);
    const sdk::RobotConfig& c = f.doc.config();

    const QModelIndex robot = tree.index(0, 0);
    std::vector<QString> robotKids;
    for (int r = 0; r < tree.rowCount(robot); ++r) robotKids.push_back(tree.keyAt(tree.index(r, 0, robot)));
    REQUIRE(robotKids.size() >= 3);
    CHECK(robotKids[0] == "chassis");
    CHECK(robotKids[1] == "mechanisms");
    CHECK(robotKids[2] == "sensors");
    REQUIRE(robotKids.size() >= 4);
    CHECK(robotKids[3] == "cameras");
    for (std::size_t i = 4; i < robotKids.size(); ++i) CHECK(robotKids[i] == "plugin");
    CHECK(tree.indexForKey("drivetrain").parent() != robot);

    const QModelIndex chassis = tree.indexFor(SelectionRef::chassis());
    REQUIRE(chassis.isValid());
    CHECK(tree.parent(chassis) == robot);
    CHECK(tree.data(chassis).toString() == "Chassis");
    REQUIRE(tree.rowCount(chassis) == 2 + static_cast<int>(c.chassis.primitives.size()));
    CHECK(tree.keyAt(tree.index(0, 0, chassis)) == "com");
    for (std::size_t i = 0; i < c.chassis.primitives.size(); ++i)
        CHECK(tree.refAt(tree.index(1 + static_cast<int>(i), 0, chassis)) == SelectionRef::primitive("chassis", static_cast<int>(i)));
    const QModelIndex drive = tree.index(tree.rowCount(chassis) - 1, 0, chassis);
    CHECK(drive == tree.indexFor(SelectionRef::drivetrain()));
    CHECK(tree.data(drive).toString() == "Drivetrain");
    CHECK(tree.data(drive, ProjectTreeModel::DetailRole).toString() == QString::fromStdString(c.drivetrain.spec.kind));

    REQUIRE_FALSE(c.drivetrain.spec.wheels.empty());
    for (const auto& w : c.drivetrain.spec.wheels) {
        QModelIndex p = tree.indexFor(SelectionRef::wheel(w.name)).parent();
        while (p.isValid() && p != drive && tree.refAt(p).kind == SelKind::Wheel) p = p.parent();   // followers nest under wheels
        CHECK(p == drive);
    }
}

TEST_CASE("project tree: followers nest under the part that drives them and are read-only") {
    DocFixture f;
    auto config = robot();
    config.drivetrain.spec.kind = sdk::drivetrain_kind::kTank;
    for (auto& w : config.drivetrain.spec.wheels) w.steered = false;
    config.drivetrain.mappings = {{{"drivetrain", "fl.drive"}, {"phoenix6", "talonfx", 1, "canivore", -1}},
                                  {{"drivetrain", "fr.drive"}, {"phoenix6", "talonfx", 2, "canivore", -1}}};
    config.motionLinks.clear();
    config.motionLinks = sdk::tankMotionLinks(config).value();
    f.doc.load(config);
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);

    const QModelIndex fl = tree.indexFor(SelectionRef::wheel("fl"));
    const QModelIndex bl = tree.indexFor(SelectionRef::wheel("bl"));
    const QModelIndex fr = tree.indexFor(SelectionRef::wheel("fr"));
    const QModelIndex br = tree.indexFor(SelectionRef::wheel("br"));
    REQUIRE(bl.isValid());
    CHECK(tree.parent(fl) == tree.indexFor(SelectionRef::drivetrain()));
    CHECK(tree.parent(bl) == fl);
    CHECK(tree.parent(br) == fr);
    CHECK(tree.rowCount(bl) == 0);   // no motor slot on a follower
    CHECK(tree.data(bl, ProjectTreeModel::FollowerRole).toBool());
    CHECK_FALSE(tree.data(fl, ProjectTreeModel::FollowerRole).toBool());
    CHECK(tree.data(bl, ProjectTreeModel::DetailRole).toString().contains("follows fl"));
    CHECK(tree.data(fl, ProjectTreeModel::DetailRole).toString().contains("drives 1"));

    // A powered wheel shows its only motor on its own row.
    CHECK_FALSE(tree.indexFor(SelectionRef::slot("drivetrain", "fl.drive")).isValid());
    CHECK_FALSE(tree.indexFor(SelectionRef::slot("drivetrain", "bl.drive")).isValid());
    CHECK(tree.data(fl, ProjectTreeModel::DetailRole).toString().contains("TalonFX 1"));
    CHECK(tree.data(fl, ProjectTreeModel::StateRole).toInt() == static_cast<int>(SlotState::Mapped));

    sdk::DeviceDescriptor d;
    d.family = "phoenix6";
    d.type = "talonfx";
    d.id = 50;
    d.capabilities = sdk::Capability::MotorOutput;
    sdk::DiscoveredDevice dev;
    dev.descriptor = d;
    QMimeData motor;
    motor.setData(kDeviceMime, encodeDevice(dev));
    CHECK_FALSE(tree.canDropMimeData(&motor, Qt::CopyAction, -1, 0, bl));   // a follower takes no motor
}

TEST_CASE("project tree: a mechanism driven by a wheel nests under it, and link loops cannot hide parts") {
    DocFixture f;
    REQUIRE(f.doc.edit("link", [&](sdk::RobotConfig& c) {
        return applyDrivenBy(c, f.doc.catalog(), {"indexer", "axis"}, {"drivetrain", "fl.drive"}, 1.0).empty();
    }));
    {
        DeviceDirectory devices;
        ProjectTreeModel tree(f.doc, devices);
        const QModelIndex indexer = tree.indexFor(SelectionRef::mechanism("indexer"));
        REQUIRE(indexer.isValid());
        CHECK(tree.parent(indexer) == tree.indexFor(SelectionRef::wheel("fl")));
        CHECK(tree.data(indexer, ProjectTreeModel::FollowerRole).toBool());
        const QString detail = tree.data(indexer, ProjectTreeModel::DetailRole).toString();
        CHECK(detail.contains("follows fl"));
        CHECK(detail.contains("@ chassis"));   // where it is attached is still visible
        CHECK_FALSE(tree.flags(indexer).testFlag(Qt::ItemIsDragEnabled));
        CHECK(tree.flags(tree.indexFor(SelectionRef::mechanism("turret"))).testFlag(Qt::ItemIsDragEnabled));
        CHECK_FALSE(tree.indexFor(SelectionRef::slot("indexer", "motor")).isValid());
    }
    f.doc.edit("loop", [](sdk::RobotConfig& c) {
        c.motionLinks = {{"a", {"drivetrain", "fl.drive"}, {"indexer", "axis"}, 1.0}, {"b", {"indexer", "axis"}, {"drivetrain", "fl.drive"}, 1.0}};
        return true;
    });
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);
    const QModelIndex fl = tree.indexFor(SelectionRef::wheel("fl"));
    const QModelIndex indexer = tree.indexFor(SelectionRef::mechanism("indexer"));
    REQUIRE(fl.isValid());
    REQUIRE(indexer.isValid());
    CHECK(tree.parent(fl) == tree.indexFor(SelectionRef::drivetrain()));   // the loop is broken at the wheel
    CHECK(tree.parent(indexer) == fl);
}
TEST_CASE("project tree: refresh keeps the model stable when only contents change") {
    DocFixture f;
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);
    int resets = 0, dataChanges = 0;
    QObject::connect(&tree, &QAbstractItemModel::modelReset, &tree, [&] { ++resets; });
    QObject::connect(&tree, &QAbstractItemModel::dataChanged, &tree, [&] { ++dataChanges; });

    f.doc.mapSlot({"shooter", "feeder"}, {"phoenix6", "talonfx", 12, "canivore", -1});
    tree.refresh();
    CHECK(resets == 0);
    CHECK(dataChanges > 0);
    CHECK(tree.data(tree.indexFor(SelectionRef::slot("shooter", "feeder")), ProjectTreeModel::DetailRole).toString() == "TalonFX 12");

    f.doc.addMechanism("mechanism", {});   // the shape changes: full reset
    tree.refresh();
    CHECK(resets == 1);
    CHECK(tree.indexFor(SelectionRef::mechanism("mechanism")).isValid());
}

TEST_CASE("project tree: device drops are only accepted where they fit") {
    DocFixture f;
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);
    auto mimeFor = [](const char* type, int id, sdk::CapabilitySet caps) {
        sdk::DeviceDescriptor d;
        d.family = "phoenix6";
        d.type = type;
        d.id = id;
        d.bus = "";
        d.capabilities = caps;
        sdk::DiscoveredDevice dev;
        dev.descriptor = d;
        auto* mime = new QMimeData;
        mime->setData(kDeviceMime, encodeDevice(dev));
        return mime;
    };
    std::unique_ptr<QMimeData> motor(mimeFor("talonfx", 50, sdk::Capability::MotorOutput));
    std::unique_ptr<QMimeData> imu(mimeFor("pigeon2", 51, sdk::Capability::Imu));

    const QModelIndex feeder = tree.indexFor(SelectionRef::slot("shooter", "feeder"));
    CHECK(tree.canDropMimeData(motor.get(), Qt::CopyAction, -1, 0, feeder));
    CHECK_FALSE(tree.canDropMimeData(imu.get(), Qt::CopyAction, -1, 0, feeder));          // an IMU is not a motor
    CHECK(tree.canDropMimeData(motor.get(), Qt::CopyAction, -1, 0, tree.indexFor(SelectionRef::mechanism("shooter"))));
    CHECK_FALSE(tree.canDropMimeData(imu.get(), Qt::CopyAction, -1, 0, tree.indexFor(SelectionRef::mechanism("shooter"))));
    CHECK(tree.canDropMimeData(imu.get(), Qt::CopyAction, -1, 0, tree.indexFor(SelectionRef::slot("robot", "imu"))));
    CHECK_FALSE(tree.canDropMimeData(motor.get(), Qt::CopyAction, -1, 0, tree.indexFor(SelectionRef::chassis())));
    CHECK_FALSE(tree.canDropMimeData(motor.get(), Qt::CopyAction, -1, 0, {}));

    SelectionRef target;
    sdk::DeviceKey dropped;
    QObject::connect(&tree, &ProjectTreeModel::deviceDropped, &tree, [&](const SelectionRef& t, const sdk::DeviceKey& k, quint32) {
        target = t;
        dropped = k;
    });
    CHECK(tree.dropMimeData(motor.get(), Qt::CopyAction, -1, 0, feeder));
    CHECK(target == SelectionRef::slot("shooter", "feeder"));
    CHECK(dropped.id == 50);
}

TEST_CASE("project tree: mechanisms can be dropped to re-attach, templates cannot be dropped") {
    DocFixture f;
    DeviceDirectory devices;
    ProjectTreeModel tree(f.doc, devices);
    QMimeData mech;
    mech.setData(kMechanismMime, QByteArray("climber"));
    QMimeData self;
    self.setData(kMechanismMime, QByteArray("turret"));

    const QModelIndex turret = tree.indexFor(SelectionRef::mechanism("turret"));
    CHECK(tree.canDropMimeData(&mech, Qt::CopyAction, -1, 0, turret));
    CHECK_FALSE(tree.canDropMimeData(&self, Qt::CopyAction, -1, 0, turret));                                     // onto itself
    CHECK_FALSE(tree.canDropMimeData(&self, Qt::CopyAction, -1, 0, tree.indexFor(SelectionRef::mechanism("shooter"))));   // onto its own child
}


TEST_CASE("detected devices model: rows, status text and drag payload") {
    DocFixture f;
    DeviceDirectory devices;
    auto list = FakeEditorBackend::sampleDevices();
    list.push_back(FakeEditorBackend::makeDevice("phoenix6", "talonfx", 77, sdk::kBusUnknown, sdk::Capability::MotorOutput));
    devices.setDevices(list);
    DetectedDevicesModel model(devices, f.doc);
    REQUIRE(model.rowCount() == 10);

    const QModelIndex fl = model.indexOf("phoenix6/talonfx[1]");
    REQUIRE(fl.isValid());
    CHECK(model.data(model.index(fl.row(), DetectedDevicesModel::ColDevice)).toString() == "TalonFX 1");
    CHECK(model.data(model.index(fl.row(), DetectedDevicesModel::ColStatus)).toString().contains("fl drive"));
    CHECK(model.data(model.index(fl.row(), DetectedDevicesModel::ColBus)).toString() == "canivore");
    CHECK(model.data(fl, DetectedDevicesModel::MappedRole).toBool());

    const QModelIndex loose = model.indexOf("phoenix6/talonfx[77]");
    CHECK(model.data(model.index(loose.row(), DetectedDevicesModel::ColStatus)).toString().contains("loopback"));
    CHECK(model.unmappedLiveCount() == 1);

    std::unique_ptr<QMimeData> mime(model.mimeData({loose}));
    REQUIRE(mime);
    const auto payload = devicePayload(mime.get());
    REQUIRE(payload);
    CHECK(payload->key.id == 77);
    CHECK(payload->capabilities.has(sdk::Capability::MotorOutput));
    CHECK((model.flags(loose) & Qt::ItemIsDragEnabled) != 0);
}

TEST_CASE("detected devices model: alias conflicts are surfaced") {
    DocFixture f;
    DeviceDirectory devices;
    auto d = FakeEditorBackend::makeDevice("phoenix6", "talonfx", 19, sdk::kBusUnknown, sdk::Capability::MotorOutput);
    d.aliasState = sdk::AliasState::Conflict;
    devices.setDevices({d});
    DetectedDevicesModel model(devices, f.doc);
    const QModelIndex idx = model.index(0, DetectedDevicesModel::ColStatus);
    CHECK(model.data(idx).toString().contains("alias conflict"));
    CHECK(model.data(idx, DetectedDevicesModel::AliasRole).toInt() == static_cast<int>(sdk::AliasState::Conflict));
}

TEST_CASE("issue model: counts, summary and rows") {
    IssueModel model;
    CHECK(model.summary() == "No problems");
    model.setIssues({{sdk::Severity::Error, "a", "boom", "c1"}, {sdk::Severity::Warning, "b", "hmm", {}}, {sdk::Severity::Warning, "c", "hmm2", {}}});
    CHECK(model.rowCount() == 3);
    CHECK(model.count(sdk::Severity::Warning) == 2);
    CHECK(model.summary().contains("1 error"));
    CHECK(model.summary().contains("2 warning"));
    CHECK(model.data(model.index(0, IssueModel::ColMessage)).toString() == "boom");
    CHECK(model.data(model.index(0, 0), IssueModel::SeverityRole).toInt() == static_cast<int>(sdk::Severity::Error));
    CHECK(model.issueAt(model.index(2, 0))->path == "c");
    CHECK(model.issueAt(model.index(9, 0)) == nullptr);
}

TEST_CASE("controller: dropping more controllers on a mechanism adds motors, one undo step each") {
    FakeEditorBackend backend;
    EditorController c(backend);
    const sdk::CapabilitySet motor(sdk::Capability::MotorOutput);
    const auto target = SelectionRef::mechanism("climber");
    auto motors = [&] { return ops::motorGroups(c.document().config(), c.document().catalog(), "climber").front().count; };
    CHECK(motors() == 1);
    CHECK(c.mapDevice(target, {"phoenix6", "talonfx", 41, "rio", -1}, motor).isEmpty());   // fills the free motor row
    CHECK(motors() == 1);
    CHECK(c.mapDevice(target, {"phoenix6", "talonfx", 42, "rio", -1}, motor).isEmpty());   // grows to motor 2
    CHECK(motors() == 2);
    CHECK(ops::findMapping(c.document().config(), {"climber", "motor2"})->device.id == 42);
    CHECK(c.mapDevice(target, {"phoenix6", "talonfx", 43, "rio", -1}, motor).isEmpty());
    CHECK(motors() == 3);
    CHECK_FALSE(c.mapDevice(target, {"phoenix6", "talonfx", 44, "rio", -1}, motor).isEmpty());   // template maximum reached

    c.document().undoStack()->undo();   // the third motor goes away together with its mapping
    CHECK(motors() == 2);
    CHECK(ops::findMapping(c.document().config(), {"climber", "motor3"}) == nullptr);

    // A tank/swerve wheel grows the same way.
    const auto wheel = SelectionRef::wheel("fl");
    CHECK(c.mapDevice(wheel, {"phoenix6", "talonfx", 45, "rio", -1}, motor).isEmpty());
    CHECK(ops::findWheel(c.document().config(), "fl")->driveMotorCount == 2);
    CHECK(ops::findMapping(c.document().config(), {"drivetrain", "fl.drive2"}) != nullptr);
}

TEST_CASE("controller: the reversed flag and the motor count are undoable edits") {
    FakeEditorBackend backend;
    EditorController c(backend);
    REQUIRE(c.addMotor("climber", "motorCount"));
    REQUIRE(ops::motorGroups(c.document().config(), c.document().catalog(), "climber").front().count == 2);
    c.document().mapSlot({"climber", "motor2"}, {"phoenix6", "talonfx", 46, "rio", -1});
    REQUIRE(c.setMotorReversed({"climber", "motor2"}, true));
    CHECK(ops::findMapping(c.document().config(), {"climber", "motor2"})->reversed);
    c.document().undoStack()->undo();
    CHECK_FALSE(ops::findMapping(c.document().config(), {"climber", "motor2"})->reversed);
    c.document().undoStack()->undo();   // the mapping
    c.document().undoStack()->undo();   // the added motor
    CHECK(ops::motorGroups(c.document().config(), c.document().catalog(), "climber").front().count == 1);
    CHECK_FALSE(c.removeMotor("climber", "motorCount"));   // never below one motor
}

