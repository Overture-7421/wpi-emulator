#include <QAbstractButton>
#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QLabel>
#include <QListWidget>
#include <QMimeData>
#include <QPushButton>
#include <QQuickItem>
#include <QScrollArea>
#include <QScrollBar>
#include <QTableView>
#include <QToolButton>
#include <QWheelEvent>
#include <QtTest/qtestspontaneevent.h>

#include "editor_page.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "model/spec_values.h"
#include "test_support.h"
#include "viewmodel/detected_devices_model.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"
#include "views/bottom_panel.h"
#include "views/device_panel.h"
#include "views/drive_card.h"
#include "views/field_editors.h"
#include "views/inspector_panel.h"
#include "views/project_tree_view.h"
#include "views/property_form.h"
#include "views/device_row.h"
#include "views/toast_overlay.h"
#include "views/viewport_widget.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

struct PageFixture {
    FakeEditorBackend backend;
    EditorPage page{backend};
    PageFixture() {
        page.setAutosaveDelayMs(0);
        page.resize(1400, 900);
        page.show();
        pumpEvents(60);
    }
    EditorController& c() { return page.controller(); }
};

}  // namespace

TEST_CASE("driven-by panel: mixed units, signed ratios, undo and own motor") {
    FakeEditorBackend backend;
    EditorController controller(backend);
    IconFactory icons;
    auto config = robot();
    sdk::MechanismConfig source;
    source.id = "source";
    source.type = "mechanism";
    sdk::MechanismConfig follower;
    follower.id = "follower";
    follower.type = "mechanism";
    follower.properties["motion"] = "linear";
    config.mechanisms = {source, follower};
    controller.document().load(config);
    const auto& catalog = controller.document().catalog();
    const QString sourceText = QStringLiteral("Driven by %1").arg(
        QString::fromStdString(axisLabel(controller.document().config(), catalog, {"source", "axis"})));

    {
        DriveCard panel(controller, icons, SelectionRef::mechanism("follower"));
        auto* src = panel.findChild<QComboBox*>("drivenBy.source:follower.axis");
        auto* ratio = panel.findChild<QDoubleSpinBox*>("drivenBy.ratio:follower.axis");
        REQUIRE(src != nullptr);
        REQUIRE(ratio != nullptr);
        const int index = src->findText(sourceText);
        REQUIRE(index > 0);
        src->setCurrentIndex(index);
        src->activated(index);
        REQUIRE(controller.document().config().motionLinks.size() == 1);
        ratio->setValue(-0.1);   // m per rev: stored in SI (m per rad)
        ratio->editingFinished();
        REQUIRE(controller.document().config().motionLinks.size() == 1);
        CHECK(controller.document().config().motionLinks[0].ratio == doctest::Approx(-0.1 / sdk::kTwoPi));
    }
    controller.document().undoStack()->undo();
    controller.document().undoStack()->undo();
    CHECK(controller.document().config().motionLinks.empty());
    controller.document().undoStack()->redo();
    controller.document().undoStack()->redo();
    REQUIRE(controller.document().config().motionLinks.size() == 1);
    {
        DriveCard panel(controller, icons, SelectionRef::mechanism("follower"));
        auto* ratio = panel.findChild<QDoubleSpinBox*>("drivenBy.ratio:follower.axis");
        CHECK(ratio->value() == doctest::Approx(-0.1));
        CHECK(panel.findChild<QLabel*>("drivenBy.units:follower.axis")->text() == "m / rev");
        auto* src = panel.findChild<QComboBox*>("drivenBy.source:follower.axis");
        src->setCurrentIndex(0);
        src->activated(0);   // back to its own motor
        CHECK(controller.document().config().motionLinks.empty());
    }
    controller.document().undoStack()->undo();
    REQUIRE(controller.document().config().motionLinks.size() == 1);
    const sdk::DeviceKey motor{"phoenix6", "talonfx", 1, "", -1};
    CHECK_FALSE(controller.mapDevice(SelectionRef::slot("follower", "motor"), motor, sdk::CapabilitySet{sdk::Capability::MotorOutput}).isEmpty());
}

TEST_CASE("driven-by panel: linking a part drops its motor mapping in the same undo step") {
    PageFixture f;
    auto& doc = f.c().document();
    REQUIRE(ops::findMapping(doc.config(), {"intake", "motor"}) != nullptr);
    {
        IconFactory icons;
        DriveCard panel(f.c(), icons, SelectionRef::mechanism("intake"));
        auto* src = panel.findChild<QComboBox*>("drivenBy.source:intake.axis");
        REQUIRE(src != nullptr);
        const int index = src->findText("Driven by fl");
        REQUIRE(index > 0);
        src->setCurrentIndex(index);
        src->activated(index);
    }
    REQUIRE(doc.config().motionLinks.size() == 1);
    CHECK(ops::findMapping(doc.config(), {"intake", "motor"}) == nullptr);
    doc.undoStack()->undo();
    CHECK(doc.config().motionLinks.empty());
    CHECK(ops::findMapping(doc.config(), {"intake", "motor"}) != nullptr);
}

TEST_CASE("inspector: a follower wheel has no drive slot but shows what drives it") {
    PageFixture f;
    REQUIRE(f.c().setDrivenBy({"drivetrain", "fl.drive"}, {"drivetrain", "fr.drive"}, 1.0).isEmpty());
    f.c().selection().set(SelectionRef::wheel("fl"));
    pumpEvents(20);
    bool steer = false;
    for (auto* row : f.page.inspector()->slotRows()) {
        CHECK(row->address().slot != "fl.drive");
        steer = steer || row->address().slot == "fl.steer";
    }
    CHECK(steer);   // steering is not part of the link
    CHECK(f.page.inspector()->findChild<QComboBox*>("drivenBy.source:drivetrain.fl.drive") != nullptr);

    f.c().selection().set(SelectionRef::wheel("fr"));
    pumpEvents(20);
    for (auto* b : f.page.inspector()->findChildren<QToolButton*>())
        CHECK(b->text() != "fl");   // followers are shown in the tree, not listed in the source's inspector
}

// ---- editors and forms -------------------------------------------------------------------------------

TEST_CASE("field editors: numbers are shown in the user's units and stored in SI") {
    sdk::PropertySpec spec;
    spec.key = "len";
    spec.type = sdk::PropertyType::Double;
    spec.unit = sdk::PropertyUnit::Meters;
    spec.min = 0.0;
    spec.max = 1.0;
    NumberEditor inches(spec, UnitSystem::Imperial);
    inches.setValue(sdk::Value(0.0254));
    auto* spin = qobject_cast<QDoubleSpinBox*>(inches.focusTarget());
    REQUIRE(spin != nullptr);
    CHECK(approx(static_cast<float>(spin->value()), 1.f, 1e-3f));
    CHECK(spin->suffix().contains("in"));
    CHECK(approx(static_cast<float>(spin->maximum()), 39.37f, 0.01f));

    double emitted = -1;
    QObject::connect(&inches, &FieldEditor::edited, &inches, [&](const sdk::Value& v) { emitted = v.asDouble(); });
    spin->setValue(10.0);   // the user types 10 in
    CHECK(approx(static_cast<float>(emitted), 0.254f, 1e-4f));

    NumberEditor cm(spec, UnitSystem::Metric);
    cm.setValue(sdk::Value(0.5));
    CHECK(approx(static_cast<float>(qobject_cast<QDoubleSpinBox*>(cm.focusTarget())->value()), 50.f, 1e-3f));
    CHECK(qobject_cast<QDoubleSpinBox*>(cm.focusTarget())->suffix().contains("cm"));

    // Pushing a value never counts as a user edit.
    emitted = -1;
    inches.setValue(sdk::Value(0.5));
    CHECK(emitted == -1);

    sdk::PropertySpec angle = spec;
    angle.unit = sdk::PropertyUnit::Radians;
    angle.max = 7.0;
    NumberEditor deg(angle, UnitSystem::Metric);
    deg.setValue(sdk::Value(sdk::kPi));
    CHECK(approx(static_cast<float>(qobject_cast<QDoubleSpinBox*>(deg.focusTarget())->value()), 180.f, 1e-2f));
}

TEST_CASE("field editors: integers, enums, bools, strings and colours") {
    sdk::PropertySpec count;
    count.type = sdk::PropertyType::Int;
    count.unit = sdk::PropertyUnit::Count;
    NumberEditor n(count, UnitSystem::Imperial);
    n.setValue(sdk::Value(7));
    CHECK(n.value().isInt());
    CHECK(n.value().asInt() == 7);
    CHECK(qobject_cast<QDoubleSpinBox*>(n.focusTarget())->decimals() == 0);

    sdk::PropertySpec e;
    e.type = sdk::PropertyType::Enum;
    e.options = {{"a", "Alpha"}, {"b", "Beta"}};
    ChoiceEditor choice(e);
    choice.setValue(sdk::Value("b"));
    CHECK(choice.value().asString() == "b");
    choice.setValue(sdk::Value("zzz"));   // unknown persisted values stay visible instead of silently changing
    CHECK(choice.value().asString() == "zzz");

    BoolEditor b;
    b.setValue(sdk::Value(true));
    CHECK(b.value().asBool());
    TextEditor t;
    t.setValue(sdk::Value("hello"));
    CHECK(t.value().asString() == "hello");
    ColorEditor col;
    col.setValue(sdk::Value("#112233"));
    CHECK(col.value().asString() == "#112233");
}

TEST_CASE("field editors: a pose round-trips exactly until the user edits a component") {
    sdk::Pose3 pose;
    pose.position = {0.1f, 0.2f, 0.3f};
    pose.rotation = sdk::Quat::fromEuler(0.3f, -0.2f, 1.0f);
    sdk::PropertySpec spec;
    spec.type = sdk::PropertyType::Pose3;
    spec.unit = sdk::PropertyUnit::Meters;
    auto* editor = createFieldEditor(spec, UnitSystem::Metric);
    editor->setValue(sdk::toValue(pose));
    sdk::Pose3 out;
    REQUIRE(sdk::fromValue(editor->value(), out));
    CHECK(out.rotation == pose.rotation);      // bit-exact: no Euler drift
    CHECK(approx(out.position, pose.position));
    delete editor;

    spec.type = sdk::PropertyType::Box3;
    auto* box = createFieldEditor(spec, UnitSystem::Imperial);
    sdk::Box3 b;
    b.pose = pose;
    b.sizeM = {0.4f, 0.5f, 0.6f};
    box->setValue(sdk::toValue(b));
    sdk::Box3 back;
    REQUIRE(sdk::fromValue(box->value(), back));
    CHECK(approx(back.sizeM, b.sizeM, 1e-3f));
    CHECK(approx(back.pose.position, b.pose.position, 1e-3f));
    delete box;
}

TEST_CASE("property form: conditional rows follow the values and edits are reported") {
    const sdk::PrimitiveSpec box;
    PropertyGroup group;
    group.id = "primitive";
    group.title = "Shape";
    group.schema = spec::primitiveSchema();
    group.values = spec::primitiveValues(box);
    PropertyFormWidget form(group, UnitSystem::Imperial);
    form.show();
    pumpEvents(20);
    CHECK(form.editorFor("size")->isVisibleTo(&form));
    CHECK_FALSE(form.editorFor("radius")->isVisibleTo(&form));

    QString key;
    sdk::Value value;
    QObject::connect(&form, &PropertyFormWidget::valueEdited, &form, [&](const QString&, const QString& k, const sdk::Value& v) {
        key = k;
        value = v;
    });
    auto* kind = qobject_cast<QComboBox*>(form.editorFor("kind")->focusTarget());
    kind->setCurrentIndex(1);
    QMetaObject::invokeMethod(kind, "activated", Q_ARG(int, 1));
    CHECK(key == "kind");
    CHECK(value.asString() == "cylinder");
    CHECK_FALSE(form.editorFor("size")->isVisibleTo(&form));   // the form re-evaluates visibility on its own
    CHECK(form.editorFor("radius")->isVisibleTo(&form));
    CHECK(form.hasField("color"));
    CHECK_FALSE(form.hasField("nope"));
    CHECK_FALSE(form.focusField("nope"));
    CHECK(form.editorFor("massKg")->isVisibleTo(&form));       // there is no Advanced fold: every field is shown
    CHECK(form.focusField("massKg"));
}

TEST_CASE("property form: field issues show a badge") {
    PropertyGroup group;
    group.id = "chassis";
    group.title = "Chassis";
    group.schema = spec::chassisSchema();
    group.values = spec::chassisValues(sdk::ChassisSpec{});
    PropertyFormWidget form(group, UnitSystem::Metric);
    form.show();
    form.setFieldIssues({{"massKg", {2, "too light"}}});
    const auto badges = form.findChildren<QLabel*>();
    bool found = false;
    for (auto* l : badges) found = found || (l->toolTip() == "too light" && l->isVisibleTo(&form));
    CHECK(found);
    form.setFieldIssues({});
    found = false;
    for (auto* l : badges) found = found || (l->toolTip() == "too light" && l->isVisibleTo(&form));
    CHECK_FALSE(found);
}

// ---- the page ----------------------------------------------------------------------------------------

TEST_CASE("page: the three columns are populated from the backend") {
    PageFixture f;
    CHECK(f.page.tree()->model()->rowCount() == 1);
    CHECK(f.page.devicePanel()->table()->model()->rowCount() == 9);
    CHECK(f.page.inspector()->titleText() == "Sample Robot");
    CHECK(f.page.inspector()->form("robot") != nullptr);
    CHECK(f.page.bottomPanel()->issueTable()->model()->rowCount() > 0);
    CHECK(f.page.viewport() != nullptr);
    // The 3D scene either loaded or told the user why it could not (e.g. no GPU on a CI box).
    CHECK((f.page.viewport()->sceneReady() || !f.page.viewport()->failureReason().isEmpty() || f.page.viewport()->quick()->status() == QQuickWidget::Loading));
}

TEST_CASE("page: the Chassis node starts collapsed and is revealed when one of its children is selected") {
    PageFixture f;
    auto* tree = f.page.tree();
    auto* model = static_cast<ProjectTreeModel*>(tree->model());
    CHECK(tree->isExpanded(model->indexForKey(QStringLiteral("robot"))));
    CHECK_FALSE(tree->isExpanded(model->indexForKey(QStringLiteral("chassis"))));
    CHECK_FALSE(tree->isExpanded(model->indexForKey(QStringLiteral("drivetrain"))));
    f.c().selection().set(SelectionRef::com());
    CHECK(tree->isExpanded(model->indexForKey(QStringLiteral("chassis"))));
    CHECK_FALSE(tree->isExpanded(model->indexForKey(QStringLiteral("drivetrain"))));
    f.c().selection().set(SelectionRef::wheel("fl"));
    CHECK(tree->isExpanded(model->indexForKey(QStringLiteral("drivetrain"))));
    CHECK(tree->currentIndex() == model->indexFor(SelectionRef::wheel("fl")));
}

TEST_CASE("page: selecting a mechanism shows its slots and generated property forms") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("intake"));
    auto* insp = f.page.inspector();
    CHECK(insp->titleText() == "Intake");
    CHECK(insp->slotRows().size() == 3);
    CHECK(insp->form("mech.props") != nullptr);
    CHECK(insp->form("mech.placement") != nullptr);
    CHECK(insp->form("mech.general") == nullptr);   // the instance id is not shown
    CHECK(insp->form("mech.props")->hasField("pickupZone"));

    f.c().selection().set(SelectionRef::mechanism("shooter"));
    CHECK(insp->form("panel:demo_game.aiming") != nullptr);       // plugin panel contributed to matching templates only
    f.c().selection().set(SelectionRef::mechanism("indexer"));
    CHECK(insp->form("panel:demo_game.aiming") == nullptr);
    f.c().selection().set(SelectionRef::plugin());
    CHECK(insp->form("panel:demo_game.hopper") != nullptr);
    CHECK(insp->form("plugin")->hasField("preloadFuel"));
    f.c().selection().set(SelectionRef::wheel("fr"));
    CHECK(insp->slotRows().size() == 3);  // drive + steer + optional azimuth encoder
    f.c().selection().set(SelectionRef::primitive("chassis", 0));
    CHECK(insp->form("primitive") != nullptr);
}

TEST_CASE("page: editing a value in the inspector updates the model and can be undone") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("turret"));
    auto* form = f.page.inspector()->form("mech.drive:axis");
    REQUIRE(form != nullptr);
    auto* spin = qobject_cast<QDoubleSpinBox*>(form->editorFor("gearRatio")->focusTarget());
    REQUIRE(spin != nullptr);
    spin->setFocus();
    pumpEvents(10);
    REQUIRE(spin->hasFocus());
    spin->setValue(55.0);
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);
    CHECK(f.backend.lastSubmitted().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);

    f.page.undoAction()->trigger();
    CHECK(spin->value() == 40.0);
    CHECK(spin->hasFocus());
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 40.0);
    f.page.redoAction()->trigger();
    CHECK(spin->value() == 55.0);
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);

    // The form was refreshed in place, not rebuilt (the user keeps focus while typing).
    const int rebuilds = f.page.inspector()->rebuildCount();
    spin->setValue(60.0);
    CHECK(f.page.inspector()->rebuildCount() == rebuilds);
}

TEST_CASE("page: focused pose undo restores every component before the next edit") {
    PageFixture f;
    f.c().settings().setUnits(UnitSystem::Metric);
    f.c().selection().set(SelectionRef::mechanism("turret"));
    auto* form = f.page.inspector()->form("mech.placement");
    REQUIRE(form != nullptr);
    auto* poseEditor = form->editorFor("mount");
    REQUIRE(poseEditor != nullptr);
    const auto spins = poseEditor->findChildren<QDoubleSpinBox*>();
    REQUIRE(spins.size() >= 6);
    auto* x = spins[0];
    auto* y = spins[1];
    const double oldX = x->value();
    const double oldY = y->value();
    x->setFocus();
    pumpEvents(10);
    REQUIRE(x->hasFocus());
    x->setValue(oldX + 10.0);
    f.page.undoAction()->trigger();
    CHECK(x->value() == oldX);
    CHECK(x->hasFocus());
    y->setValue(oldY + 5.0);
    sdk::Pose3 pose;
    REQUIRE(sdk::fromValue(poseEditor->value(), pose));
    CHECK(approx(pose.position.x, static_cast<float>(oldX / 100.0)));
    CHECK(approx(pose.position.y, static_cast<float>((oldY + 5.0) / 100.0)));
    const auto groups = propertyGroupsFor(f.c().document().config(), f.c().document().catalog(), SelectionRef::mechanism("turret"));
    const auto group = std::find_if(groups.begin(), groups.end(), [](const auto& g) { return g.id == "mech.placement"; });
    REQUIRE(group != groups.end());
    CHECK(group->values.get("mount") == poseEditor->value());
}

TEST_CASE("property form: typing echoes preserve cursor but authoritative load replaces focused text") {
    PropertyGroup group;
    group.id = "text";
    sdk::PropertySpec text;
    text.key = "name";
    text.type = sdk::PropertyType::String;
    group.schema.properties.push_back(text);
    group.values["name"] = sdk::Value("robot");
    PropertyFormWidget form(group, UnitSystem::Metric);
    form.show();
    auto* edit = qobject_cast<QLineEdit*>(form.editorFor("name")->focusTarget());
    REQUIRE(edit != nullptr);
    edit->setFocus();
    pumpEvents(10);
    REQUIRE(edit->hasFocus());
    edit->setText(QStringLiteral("roXbot"));
    edit->setCursorPosition(3);
    form.setValues(group.values, false);   // validation or another field's edit is not a load
    CHECK(edit->text() == QStringLiteral("roXbot"));
    CHECK(edit->cursorPosition() == 3);
    form.setValues(group.values, true);
    CHECK(edit->text() == QStringLiteral("robot"));
    CHECK(edit->hasFocus());
}

TEST_CASE("page: units follow the toolbar choice") {
    PageFixture f;
    f.c().selection().set(SelectionRef::chassis());
    auto massSpin = [&] { return qobject_cast<QDoubleSpinBox*>(f.page.inspector()->form("chassis")->editorFor("massKg")->focusTarget()); };
    CHECK(massSpin()->suffix().contains("lb"));
    CHECK(approx(static_cast<float>(massSpin()->value()), 52.f * 2.20462f, 0.05f));
    f.c().settings().setUnits(UnitSystem::Metric);
    CHECK(massSpin()->suffix().contains("kg"));
    CHECK(approx(static_cast<float>(massSpin()->value()), 52.f, 0.01f));
    massSpin()->setValue(60.0);
    CHECK(approx(f.c().document().config().chassis.massKg, 60.f));
}

TEST_CASE("page: a mapped controller row shows the device, Reversed and unmap edit the model, there is no picker") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("shooter"));
    DeviceRowWidget* flywheel = nullptr;
    DeviceRowWidget* feeder = nullptr;
    for (auto* row : f.page.inspector()->slotRows()) {
        if (row->address().slot == "flywheel") flywheel = row;
        if (row->address().slot == "feeder") feeder = row;
    }
    REQUIRE(flywheel != nullptr);
    REQUIRE(feeder != nullptr);
    CHECK(flywheel->deviceText().contains("TalonFX 11"));
    CHECK_FALSE(flywheel->reversedBox()->isHidden());
    CHECK(feeder->deviceText().contains("Not mapped"));
    CHECK(feeder->reversedBox()->isHidden());
    CHECK(feeder->findChildren<QComboBox*>().isEmpty());

    flywheel->reversedBox()->setChecked(true);
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "flywheel"})->reversed);
    f.c().document().undoStack()->undo();
    CHECK_FALSE(ops::findMapping(f.c().document().config(), {"shooter", "flywheel"})->reversed);
    pumpEvents(20);
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "flywheel") flywheel = row;
    flywheel->unmapButton()->click();
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "flywheel"}) == nullptr);
}
TEST_CASE("page: dropping a device on the tree maps it and selects the slot") {
    PageFixture f;
    auto* model = f.page.tree()->model();
    const QModelIndex slot = static_cast<ProjectTreeModel*>(model)->indexFor(SelectionRef::slot("shooter", "feeder"));
    REQUIRE(slot.isValid());
    sdk::DiscoveredDevice motor = FakeEditorBackend::makeDevice("phoenix6", "talonfx", 63, sdk::kBusUnknown, sdk::Capability::MotorOutput);
    QMimeData mime;
    mime.setData(kDeviceMime, encodeDevice(motor));
    CHECK(model->dropMimeData(&mime, Qt::CopyAction, -1, 0, slot));
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "feeder"})->device.id == 63);
    CHECK(f.c().selection().current() == SelectionRef::slot("shooter", "feeder"));
    f.c().document().undoStack()->undo();
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "feeder"}) == nullptr);
}

TEST_CASE("page: Detect devices streams devices into the list and raises toasts") {
    PageFixture f;
    f.backend.setDetectionStepMs(10);
    auto* button = f.page.devicePanel()->detectButton();
    CHECK(button->text() == "Detect devices");
    button->click();
    CHECK(f.backend.detecting());
    CHECK(button->text().contains("Stop"));
    pumpEvents(50);
    CHECK(f.page.toasts()->activeCount() > 0);
    pumpEvents(250);
    CHECK_FALSE(f.backend.detecting());
    CHECK(button->text() == "Detect devices");
    CHECK(f.page.devicePanel()->table()->model()->rowCount() == 16);
    CHECK(f.page.bottomPanel()->console()->toPlainText().contains("announced talonfx 10"));

    // The alias-conflict device is flagged in its status column.
    auto* model = f.page.devicePanel()->table()->model();
    bool flagged = false;
    for (int r = 0; r < model->rowCount(); ++r)
        flagged = flagged || model->index(r, DetectedDevicesModel::ColStatus).data().toString().contains("alias conflict");
    CHECK(flagged);
    // Toggling again while idle starts a new run.
    f.page.detectAction()->trigger();
    CHECK(f.backend.detecting());
    f.page.detectAction()->trigger();
    CHECK_FALSE(f.backend.detecting());
}

TEST_CASE("page: clicking a validation row jumps to the offending item") {
    PageFixture f;
    pumpEvents(200);
    auto* table = f.page.bottomPanel()->issueTable();
    int row = -1;
    for (int r = 0; r < table->model()->rowCount(); ++r)
        if (table->model()->index(r, IssueModel::ColWhere).data().toString() == "shooter.feeder") row = r;
    REQUIRE(row >= 0);
    QMetaObject::invokeMethod(table, "clicked", Q_ARG(QModelIndex, table->model()->index(row, 0)));
    CHECK(f.c().selection().current() == SelectionRef::slot("shooter", "feeder"));
    CHECK(f.page.inspector()->titleText() == "Shooter");
}

TEST_CASE("page: selection stays in sync between the tree and the controller") {
    PageFixture f;
    auto* tree = f.page.tree();
    auto* model = static_cast<ProjectTreeModel*>(tree->model());
    f.c().selection().set(SelectionRef::wheel("br"));
    CHECK(tree->currentIndex() == model->indexFor(SelectionRef::wheel("br")));   // ancestors were expanded to reveal it
    tree->setCurrentIndex(model->indexFor(SelectionRef::mechanism("climber")));
    CHECK(f.c().selection().current() == SelectionRef::mechanism("climber"));
    f.c().selection().set(SelectionRef::device("phoenix6/talonfx[1]"));
    CHECK_FALSE(tree->currentIndex().isValid());
}

TEST_CASE("page: backend signals reach the editor (config, catalog, issues, log, telemetry)") {
    PageFixture f;
    auto other = robot();
    other.name = "Swapped";
    f.backend.setConfig(other);
    CHECK(f.page.inspector()->titleText() == "Swapped");

    f.backend.log(sdk::Severity::Error, "gradle failed");
    CHECK(f.page.bottomPanel()->console()->toPlainText().contains("gradle failed"));

    f.backend.pushIssues({{sdk::Severity::Error, "chassis.massKg", "pushed problem", "x"}});
    pumpEvents(250);
    bool seen = false;
    for (int r = 0; r < f.page.bottomPanel()->issueTable()->model()->rowCount(); ++r)
        seen = seen || f.page.bottomPanel()->issueTable()->model()->index(r, IssueModel::ColMessage).data().toString() == "pushed problem";
    CHECK(seen);

    sdk::SimSnapshotData data;
    data.robotConnected = true;
    data.batteryVoltageV = 11.9;
    f.backend.snapshotChanged(std::make_shared<const sdk::SimSnapshot>(data));
    CHECK(f.page.bottomPanel()->telemetryTable()->rowCount() >= 3);

}

TEST_CASE("page: autosave fires after edits and Save persists immediately") {
    PageFixture f;
    f.page.setAutosaveDelayMs(20);
    int autosaves = 0;
    QObject::connect(&f.page, &EditorPage::autosaveRequested, &f.page, [&] { ++autosaves; });
    f.c().addMechanism("mechanism", {});
    CHECK(f.page.hasUnsavedChanges());
    pumpEvents(150);
    CHECK(autosaves == 1);
    CHECK(f.backend.saveCount() == 1);
    CHECK_FALSE(f.page.hasUnsavedChanges());
    f.page.saveAction()->trigger();
    CHECK(f.backend.saveCount() == 2);
}

TEST_CASE("page: keyboard actions") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("indexer"));
    f.page.duplicateAction()->trigger();
    CHECK(f.c().document().config().findMechanism("indexer_2") != nullptr);
    f.page.deleteAction()->trigger();
    CHECK(f.c().document().config().findMechanism("indexer_2") == nullptr);
    CHECK(f.page.undoStack()->count() == 2);
}

TEST_CASE("page: toasts stack, expire and can be dismissed") {
    PageFixture f;
    auto* toasts = f.page.toasts();
    toasts->showToast("One", "first", 0, 40);
    toasts->showToast("Two", "second", 1, 40);
    CHECK(toasts->activeCount() == 2);
    for (int i = 0; i < 6; ++i) toasts->showToast("More", {}, 2, 5000);
    CHECK(toasts->activeCount() <= 4);
    pumpEvents(700);
    CHECK(toasts->activeCount() <= 4);
}

TEST_CASE("page: an empty robot shows onboarding instead of blank panels") {
    FakeEditorBackend backend;
    auto empty = robot();
    empty.mechanisms.clear();
    empty.drivetrain.mappings.clear();
    backend.setConfig(empty);
    backend.setDevices({});
    EditorPage page(backend);
    page.show();
    pumpEvents(40);
    CHECK(page.devicePanel()->table()->model()->rowCount() == 0);
    // Empty list -> the stacked page with the coaching text is shown, not the empty table.
    CHECK_FALSE(page.devicePanel()->table()->isVisibleTo(&page));
    CHECK(page.inspector()->titleText() == "Sample Robot");
    CHECK(page.controller().issues().size() >= 1);
}


TEST_CASE("page: the QML viewport answers pick queries with the ids the controller understands") {
    PageFixture f;
    pumpEvents(600);   // let Qt Quick render a couple of frames
    auto* quick = f.page.viewport()->quick();
    QQuickItem* root = quick->rootObject();
    if (!f.page.viewport()->sceneReady() || root == nullptr) {
        MESSAGE("Qt Quick 3D is not available on this machine: skipping the pick probe");
        return;
    }
    QVariant hits;
    QVariant cx(quick->width() / 2);
    QVariant cy(quick->height() / 2);
    QMetaObject::invokeMethod(root, "hitsAt", Q_RETURN_ARG(QVariant, hits), Q_ARG(QVariant, cx), Q_ARG(QVariant, cy));
    // Whatever is at the centre of the view, every id must be well formed.
    for (const QVariant& id : hits.toList()) CHECK((id.toString().startsWith("i:") || id.toString().startsWith("h:")));
}
TEST_CASE("inspector: motor rows follow the motor count field and Reversed edits the mapping") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("climber"));
    pumpEvents(20);
    auto motorRows = [&] {
        int n = 0;
        for (auto* row : f.page.inspector()->slotRows())
            if (row->address().owner == "climber" && row->address().slot.rfind("motor", 0) == 0) ++n;
        return n;
    };
    CHECK(motorRows() == 1);
    auto* drive = f.page.inspector()->form("mech.drive:axis");
    REQUIRE(drive != nullptr);
    auto* count = qobject_cast<QDoubleSpinBox*>(drive->editorFor("motorCount")->focusTarget());
    REQUIRE(count != nullptr);
    count->setFocus();
    pumpEvents(10);
    count->setValue(2);
    pumpEvents(20);
    CHECK(motorRows() == 2);

    REQUIRE(f.c().mapDevice(SelectionRef::slot("climber", "motor2"), {"phoenix6", "talonfx", 46, "rio", -1}, sdk::CapabilitySet{sdk::Capability::MotorOutput}).isEmpty());
    pumpEvents(20);
    QCheckBox* reversed = nullptr;
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "motor2") reversed = row->reversedBox();
    REQUIRE(reversed != nullptr);
    CHECK_FALSE(reversed->isHidden());
    reversed->setChecked(true);
    CHECK(ops::findMapping(f.c().document().config(), {"climber", "motor2"})->reversed);

    pumpEvents(20);
    drive = f.page.inspector()->form("mech.drive:axis");
    count = qobject_cast<QDoubleSpinBox*>(drive->editorFor("motorCount")->focusTarget());
    count->setFocus();
    pumpEvents(10);
    count->setValue(1);
    pumpEvents(20);
    CHECK(motorRows() == 1);
    CHECK(ops::findMapping(f.c().document().config(), {"climber", "motor2"}) == nullptr);
}

TEST_CASE("inspector: a part page is Name, Drive, Configure, Placement cards and the name is editable") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("climber"));
    pumpEvents(20);
    auto* insp = f.page.inspector();
    REQUIRE(insp->nameEdit() != nullptr);
    CHECK(insp->nameEdit()->text() == "Climber");
    // Cards in order, each its own role.
    QStringList order;
    for (auto* card : insp->findChildren<SectionCard*>()) order << card->objectName();
    CHECK(order.mid(0, 3) == QStringList{"card.drive", "card.configure", "card.placement"});
    CHECK(insp->findChild<QToolButton*>("drive.own:climber.axis") != nullptr);
    // Type is the motion (Rotary / Arm / Linear); there is no template switch and no repeated drag hint.
    CHECK(insp->findChild<QComboBox*>("configure.typeCombo") == nullptr);
    CHECK(insp->form("mech.props")->hasField("motion"));
    for (auto* label : insp->findChildren<QLabel*>()) CHECK_FALSE(label->text().contains("Drag a detected device"));
    CHECK(insp->form("mech.drive:axis")->hasField("gearRatio"));   // motor model and gearing live in Drive, not Configure
    CHECK_FALSE(insp->form("mech.props")->hasField("gearRatio"));

    // No "Mechanism · Mechanism" subtitle; the tree row names the Type instead of the template.
    for (auto* label : insp->findChildren<QLabel*>()) CHECK(label->text() != "Mechanism · Mechanism");
    CHECK(mechanismKindLabel(*f.c().document().config().findMechanism("climber"), *f.c().document().catalog().findTemplate("mechanism")) == "Linear");

    insp->nameEdit()->setText("Lift");
    insp->nameEdit()->editingFinished();
    CHECK(f.c().document().config().findMechanism("climber")->label == "Lift");

    // A wheel renames through the same field and the selection follows.
    f.c().selection().set(SelectionRef::wheel("fl"));
    pumpEvents(20);
    insp->nameEdit()->setText("front_left");
    insp->nameEdit()->editingFinished();
    CHECK(ops::findWheel(f.c().document().config(), "front_left") != nullptr);
    CHECK(f.c().selection().current() == SelectionRef::wheel("front_left"));
}

TEST_CASE("inspector: the Own motors / Linked choice follows the link state and hides the motor rows") {
    PageFixture f;
    f.c().selection().set(SelectionRef::wheel("fl"));
    pumpEvents(20);
    auto* own = f.page.inspector()->findChild<QToolButton*>("drive.own:drivetrain.fl.drive");
    auto* linked = f.page.inspector()->findChild<QToolButton*>("drive.linked:drivetrain.fl.drive");
    REQUIRE(own != nullptr);
    REQUIRE(linked != nullptr);
    CHECK(own->isChecked());
    linked->click();
    CHECK(f.page.inspector()->findChild<QComboBox*>("drivenBy.source:drivetrain.fl.drive")->isVisibleTo(f.page.inspector()));
    REQUIRE(f.c().setDrivenBy({"drivetrain", "fl.drive"}, {"drivetrain", "fr.drive"}, 1.0).isEmpty());
    pumpEvents(20);
    CHECK(f.page.inspector()->findChild<QToolButton*>("drive.linked:drivetrain.fl.drive")->isChecked());
    f.page.inspector()->findChild<QToolButton*>("drive.own:drivetrain.fl.drive")->click();   // back to own motors
    pumpEvents(20);
    CHECK(f.c().document().config().motionLinks.empty());
}

TEST_CASE("inspector: Drive is device-agnostic and a swerve wheel's steering lives in Configure") {
    PageFixture f;
    f.c().selection().set(SelectionRef::wheel("fl"));
    pumpEvents(20);
    auto* insp = f.page.inspector();
    auto* drive = insp->form("wheel.drive");
    REQUIRE(drive != nullptr);
    for (const char* key : {"driveMotor", "driveMotorCount", "driveGearRatio"}) CHECK(drive->hasField(key));
    CHECK_FALSE(drive->hasField("steerMotor"));
    CHECK_FALSE(drive->hasField("steered"));
    auto* configure = insp->form("wheel");
    REQUIRE(configure != nullptr);
    CHECK(configure->hasField("kind"));
    CHECK(configure->hasField("steerMotor"));
    for (auto* row : insp->slotRows()) {
        const bool inDrive = insp->findChild<SectionCard*>("card.drive")->isAncestorOf(row);
        if (row->address().slot == "fl.steer") CHECK_FALSE(inDrive);
        if (row->address().slot == "fl.drive") {
            CHECK(inDrive);
            CHECK(row->titleText() == "Motor 1");
        }
    }
    f.c().document().setProperty(SelectionRef::wheel("fl"), "wheel", "kind", sdk::Value("standard"));
    pumpEvents(20);
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "fl.drive") CHECK(row->titleText() == "Motor 1");   // refreshes keep the generic label
    for (auto* row : f.page.inspector()->slotRows()) CHECK(row->address().slot != "fl.steer");
}

TEST_CASE("inspector: the wheel scrolls the panel instead of changing a dropdown") {
    PageFixture f;
    f.page.resize(1400, 420);
    f.c().selection().set(SelectionRef::wheel("fl"));
    pumpEvents(40);
    auto* insp = f.page.inspector();
    auto* configure = insp->form("wheel");
    REQUIRE(configure != nullptr);
    auto* combo = qobject_cast<QComboBox*>(configure->editorFor("kind")->focusTarget());
    REQUIRE(combo != nullptr);
    auto* scroll = insp->findChild<QScrollArea*>();
    REQUIRE(scroll != nullptr);
    REQUIRE(scroll->verticalScrollBar()->maximum() > 0);

    const int before = combo->currentIndex();
    const QPointF pos(combo->width() / 2, combo->height() / 2);
    QWheelEvent e(pos, combo->mapToGlobal(pos), QPoint(), QPoint(0, -120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
    QSpontaneKeyEvent::setSpontaneous(&e);
    scroll->verticalScrollBar()->setValue(0);
    qApp->notify(combo, &e);
    CHECK(combo->currentIndex() == before);
    CHECK_FALSE(combo->hasFocus());
    CHECK(scroll->verticalScrollBar()->value() > 0);
}

TEST_CASE("inspector: the Robot and Drivetrain pages have no Getting started or Wheels card") {
    PageFixture f;
    auto* insp = f.page.inspector();
    for (const SelectionRef& sel : {SelectionRef::robot(), SelectionRef::drivetrain()}) {
        f.c().selection().set(sel);
        pumpEvents(20);
        for (auto* card : insp->findChildren<SectionCard*>()) {
            const QString title = card->header()->text();
            CHECK_FALSE(title.contains("Getting started"));
            CHECK_FALSE(title.contains("Wheels"));
        }
        CHECK(insp->findChild<QPushButton*>("drivetrain.linkBySide") == nullptr);
        CHECK(insp->findChild<QPushButton*>("drivetrain.independent") == nullptr);
    }
    f.c().selection().set(SelectionRef::drivetrain());
    pumpEvents(20);
    CHECK(insp->form("drivetrain") != nullptr);                  // Configure stays: type, tire model, load transfer
    CHECK(insp->findChildren<SectionCard*>().size() == 1);
    f.c().selection().set(SelectionRef::robot());
    pumpEvents(20);
    CHECK(insp->form("robot") != nullptr);
}