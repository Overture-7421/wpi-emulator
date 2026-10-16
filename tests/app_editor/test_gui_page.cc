#include <QAbstractButton>
#include <QApplication>
#include <QAction>
#include <QComboBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QQuickItem>
#include <QTableView>
#include <QToolButton>

#include "test_support.h"
#include "editor_page.h"
#include "model/property_groups.h"
#include "model/spec_values.h"
#include "viewmodel/detected_devices_model.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"
#include "views/bottom_panel.h"
#include "views/device_panel.h"
#include "views/field_editors.h"
#include "views/inspector_panel.h"
#include "views/palette_panel.h"
#include "views/project_tree_view.h"
#include "views/property_form.h"
#include "views/slot_row.h"
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
    CHECK(form.focusField("massKg"));                          // advanced fields are revealed when focused
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
    CHECK(f.page.palette()->templateCount() == 10);              // 8 core + 2 plugin templates
    CHECK(f.page.tree()->model()->rowCount() == 1);
    CHECK(f.page.devicePanel()->table()->model()->rowCount() == 9);
    CHECK(f.page.inspector()->titleText() == "Sample Robot");
    CHECK(f.page.inspector()->form("robot") != nullptr);
    CHECK(f.page.bottomPanel()->issueTable()->model()->rowCount() > 0);
    CHECK(f.page.viewport() != nullptr);
    // The 3D scene either loaded or told the user why it could not (e.g. no GPU on a CI box).
    CHECK((f.page.viewport()->sceneReady() || !f.page.viewport()->failureReason().isEmpty() || f.page.viewport()->quick()->status() == QQuickWidget::Loading));
}

TEST_CASE("page: selecting a mechanism shows its slots and generated property forms") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("intake"));
    auto* insp = f.page.inspector();
    CHECK(insp->titleText() == "Intake");
    CHECK(insp->slotRows().size() == 3);
    CHECK(insp->form("mech.props") != nullptr);
    CHECK(insp->form("mech.placement") != nullptr);
    CHECK(insp->form("mech.general") != nullptr);
    CHECK(insp->form("mech.props")->hasField("pickupZone"));

    f.c().selection().set(SelectionRef::mechanism("indexer"));
    CHECK(insp->form("panel:game_2026.feeding") != nullptr);      // plugin panel contributed to matching templates only
    f.c().selection().set(SelectionRef::plugin());
    CHECK(insp->form("panel:game_2026.hopper") != nullptr);
    CHECK(insp->form("plugin")->hasField("preloadFuel"));
    f.c().selection().set(SelectionRef::wheel("fr"));
    CHECK(insp->slotRows().size() == 2);                          // drive + steer of that wheel only
    f.c().selection().set(SelectionRef::primitive("chassis", 0));
    CHECK(insp->form("primitive") != nullptr);
}

TEST_CASE("page: editing a value in the inspector updates the model and can be undone") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("turret"));
    auto* form = f.page.inspector()->form("mech.props");
    REQUIRE(form != nullptr);
    auto* spin = qobject_cast<QDoubleSpinBox*>(form->editorFor("gearRatio")->focusTarget());
    REQUIRE(spin != nullptr);
    spin->setValue(55.0);
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);
    CHECK(f.backend.lastSubmitted().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);

    f.page.undoAction()->trigger();
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 40.0);
    f.page.redoAction()->trigger();
    CHECK(f.c().document().config().findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);

    // The form was refreshed in place, not rebuilt (the user keeps focus while typing).
    const int rebuilds = f.page.inspector()->rebuildCount();
    spin->setValue(60.0);
    CHECK(f.page.inspector()->rebuildCount() == rebuilds);
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

TEST_CASE("page: picking a device from a slot's list maps it") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("shooter"));
    SlotRowWidget* feeder = nullptr;
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "feeder") feeder = row;
    REQUIRE(feeder != nullptr);
    CHECK(feeder->picker()->currentIndex() == 0);                    // unmapped
    int pick = -1;
    for (int i = 1; i < feeder->picker()->count(); ++i)
        if (feeder->picker()->itemText(i).startsWith("TalonFX 5")) pick = i;
    REQUIRE(pick > 0);
    feeder->picker()->setCurrentIndex(pick);
    QMetaObject::invokeMethod(feeder->picker(), "activated", Q_ARG(int, pick));
    const auto* mapping = ops::findMapping(f.c().document().config(), {"shooter", "feeder"});
    REQUIRE(mapping != nullptr);
    CHECK(mapping->device.id == 5);
    CHECK(mapping->device.bus == "canivore");   // the existing declared alias wins
    // Unmapping through the combo's first entry.
    QMetaObject::invokeMethod(feeder->picker(), "activated", Q_ARG(int, 0));
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "feeder"}) == nullptr);
}

TEST_CASE("page: incompatible devices are disabled in a slot's picker") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("shooter"));
    SlotRowWidget* feeder = nullptr;
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "feeder") feeder = row;
    REQUIRE(feeder != nullptr);
    bool pigeonDisabled = false;
    for (int i = 0; i < feeder->picker()->count(); ++i)
        if (feeder->picker()->itemText(i).startsWith("Pigeon")) pigeonDisabled = !(feeder->picker()->model()->flags(feeder->picker()->model()->index(i, 0)) & Qt::ItemIsEnabled);
    CHECK(pigeonDisabled);
}

TEST_CASE("page: dropping a device on a slot row maps it, a wrong kind is refused") {
    PageFixture f;
    f.c().selection().set(SelectionRef::mechanism("shooter"));
    SlotRowWidget* feeder = nullptr;
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "feeder") feeder = row;
    REQUIRE(feeder != nullptr);

    sdk::DiscoveredDevice motor = FakeEditorBackend::makeDevice("phoenix6", "talonfx", 61, sdk::kBusUnknown, sdk::Capability::MotorOutput);
    QMimeData good;
    good.setData(kDeviceMime, encodeDevice(motor));
    QDragEnterEvent enter(QPoint(5, 5), Qt::CopyAction, &good, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(feeder, &enter);
    CHECK(enter.isAccepted());
    QDropEvent drop(QPointF(5, 5), Qt::CopyAction, &good, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(feeder, &drop);
    CHECK(ops::findMapping(f.c().document().config(), {"shooter", "feeder"})->device.id == 61);

    sdk::DiscoveredDevice imu = FakeEditorBackend::makeDevice("phoenix6", "pigeon2", 62, sdk::kBusUnknown, sdk::Capability::Imu);
    QMimeData bad;
    bad.setData(kDeviceMime, encodeDevice(imu));
    f.c().selection().set(SelectionRef::mechanism("shooter"));
    for (auto* row : f.page.inspector()->slotRows())
        if (row->address().slot == "feeder") feeder = row;
    QDragEnterEvent enterBad(QPoint(5, 5), Qt::CopyAction, &bad, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(feeder, &enterBad);
    CHECK_FALSE(enterBad.isAccepted());
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

TEST_CASE("page: dropping a template on a mechanism attaches it as a child") {
    PageFixture f;
    auto* model = f.page.tree()->model();
    const QModelIndex turret = static_cast<ProjectTreeModel*>(model)->indexFor(SelectionRef::mechanism("turret"));
    QMimeData mime;
    mime.setData(kTemplateMime, QByteArray("hood"));
    CHECK(model->dropMimeData(&mime, Qt::CopyAction, -1, 0, turret));
    CHECK(f.c().document().config().findMechanism("hood")->resolvedAttachment().parentMechanismId == "turret");
}

TEST_CASE("page: the palette adds templates to the robot") {
    PageFixture f;
    auto* list = f.page.palette()->list();
    QListWidgetItem* roller = nullptr;
    for (int i = 0; i < list->count(); ++i)
        if (list->item(i)->text() == "Generic roller") roller = list->item(i);
    REQUIRE(roller != nullptr);
    list->setCurrentItem(roller);
    QMetaObject::invokeMethod(list, "itemDoubleClicked", Q_ARG(QListWidgetItem*, roller));
    CHECK(f.c().document().config().findMechanism("roller") != nullptr);
    CHECK(f.c().selection().current() == SelectionRef::mechanism("roller"));
    CHECK(f.page.inspector()->slotRows().size() == 1);              // generic roller: just the motor

    // Templates can be dragged: the list produces the template mime.
    auto* mime = list->model()->mimeData({list->model()->index(list->row(roller), 0)});
    REQUIRE(mime != nullptr);
    CHECK(mime->data(kTemplateMime) == "roller");
    delete mime;
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

    auto catalog = FakeEditorBackend::sampleCatalog();
    catalog.mechanismTemplates.resize(2);
    f.backend.setCatalog(catalog);
    CHECK(f.page.palette()->templateCount() == 2);
}

TEST_CASE("page: autosave fires after edits and Save persists immediately") {
    PageFixture f;
    f.page.setAutosaveDelayMs(20);
    int autosaves = 0;
    QObject::connect(&f.page, &EditorPage::autosaveRequested, &f.page, [&] { ++autosaves; });
    f.c().addMechanism("roller", {});
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