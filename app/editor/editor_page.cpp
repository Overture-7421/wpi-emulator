#include "editor_page.h"

#include <QAction>
#include <QComboBox>
#include <QCursor>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QPointer>
#include <QSplitter>
#include <QToolButton>
#include <QUndoStack>
#include <QVBoxLayout>

#include <optional>
#include "model/config_ops.h"
#include "model/tr.h"
#include "ui/widget_support.h"
#include "viewmodel/detected_devices_model.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/issue_model.h"
#include "viewmodel/project_tree_model.h"
#include "viewmodel/viewport_controller.h"
#include "views/bottom_panel.h"
#include "views/device_panel.h"
#include "views/icon_factory.h"
#include "views/inspector_panel.h"
#include "views/project_tree_view.h"
#include "views/toast_overlay.h"
#include "views/viewport_widget.h"

namespace wpiemu::app::editor {

namespace {

QWidget* titled(const QString& title, QWidget* body, QWidget* parent) {
    auto* box = new QWidget(parent);
    auto* v = new QVBoxLayout(box);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);
    auto* label = ui::headingLabel(title, box);
    label->setContentsMargins(10, 8, 10, 4);
    v->addWidget(label);
    v->addWidget(body, 1);
    return box;
}

}  // namespace

struct EditorPage::Impl {
    IEditorBackend& backend;
    IconFactory icons;
    EditorController controller;
    ViewportController viewportController;
    ProjectTreeModel treeModel;
    DetectedDevicesModel deviceModel;
    IssueModel issueModel;

    ProjectTreeView* tree = nullptr;
    DevicePanel* devices = nullptr;
    ViewportWidget* viewport = nullptr;
    BottomPanel* bottom = nullptr;
    InspectorPanel* inspector = nullptr;
    ToastOverlay* toasts = nullptr;

    QAction* undo = nullptr;
    QAction* redo = nullptr;
    QAction* save = nullptr;
    QAction* detect = nullptr;
    QAction* duplicate = nullptr;
    QAction* remove = nullptr;
    QLabel* status = nullptr;
    QLabel* saveState = nullptr;
    QToolButton* validation = nullptr;
    QComboBox* units = nullptr;

    explicit Impl(IEditorBackend& b)
        : backend(b),
          controller(b),
          viewportController(controller),
          treeModel(controller.document(), controller.devices()),
          deviceModel(controller.devices(), controller.document()) {}
};

EditorPage::EditorPage(IEditorBackend& backend, QWidget* parent) : QWidget(parent), d(std::make_unique<Impl>(backend)) {
    Impl& s = *d;
    EditorController& c = s.controller;

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // ---- actions ----
    s.undo = c.document().undoStack()->createUndoAction(this, ETR("Undo"));
    s.undo->setShortcut(QKeySequence::Undo);
    s.redo = c.document().undoStack()->createRedoAction(this, ETR("Redo"));
    s.redo->setShortcuts({QKeySequence::Redo, QKeySequence(QStringLiteral("Ctrl+Y"))});
    s.save = new QAction(ETR("Save"), this);
    s.save->setShortcut(QKeySequence::Save);
    s.detect = new QAction(ETR("Detect devices"), this);
    s.detect->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+D")));
    s.duplicate = new QAction(ETR("Duplicate"), this);
    s.duplicate->setShortcut(QKeySequence(QStringLiteral("Ctrl+D")));
    s.remove = new QAction(ETR("Delete"), this);
    s.remove->setShortcut(QKeySequence::Delete);
    for (QAction* a : {s.undo, s.redo, s.save, s.detect, s.duplicate, s.remove}) {
        a->setShortcutContext(Qt::WidgetWithChildrenShortcut);
        addAction(a);
    }
    auto addShortcutAction = [this](const QString& key, const std::function<void()>& fn) {
        auto* a = new QAction(this);
        a->setShortcut(QKeySequence(key));
        a->setShortcutContext(Qt::WidgetWithChildrenShortcut);
        connect(a, &QAction::triggered, this, [fn] { fn(); });
        addAction(a);
    };
    addShortcutAction(QStringLiteral("W"), [this] { d->viewportController.setGizmoMode(GizmoMode::Move); });
    addShortcutAction(QStringLiteral("E"), [this] { d->viewportController.setGizmoMode(GizmoMode::Rotate); });
    addShortcutAction(QStringLiteral("R"), [this] { d->viewportController.setGizmoMode(GizmoMode::Scale); });
    addShortcutAction(QStringLiteral("F"), [this] { d->viewportController.frameSelection(); });
    addShortcutAction(QStringLiteral("1"), [this] { d->viewportController.setView(QStringLiteral("front")); });
    addShortcutAction(QStringLiteral("2"), [this] { d->viewportController.setView(QStringLiteral("side")); });
    addShortcutAction(QStringLiteral("3"), [this] { d->viewportController.setView(QStringLiteral("top")); });
    addShortcutAction(QStringLiteral("4"), [this] { d->viewportController.setView(QStringLiteral("iso")); });
    addShortcutAction(QStringLiteral("Esc"), [this] { d->controller.selection().set(SelectionRef::robot()); });
    connect(s.save, &QAction::triggered, this, [this] { (void)d->controller.flushSave(); });
    connect(s.detect, &QAction::triggered, this, [this] { d->controller.toggleDetection(); });
    connect(s.duplicate, &QAction::triggered, this, [this] { d->controller.duplicateSelection(); });
    connect(s.remove, &QAction::triggered, this, [this] { d->controller.deleteSelection(); });

    // ---- top bar ----
    auto* bar = new QWidget(this);
    auto* barLayout = new QHBoxLayout(bar);
    barLayout->setContentsMargins(8, 4, 8, 4);
    barLayout->setSpacing(6);
    for (QAction* a : {s.undo, s.redo}) {
        auto* b = new QToolButton(bar);
        b->setDefaultAction(a);
        b->setAutoRaise(true);
        b->setToolButtonStyle(Qt::ToolButtonTextOnly);
        barLayout->addWidget(b);
    }
    s.units = new QComboBox(bar);
    s.units->addItem(ETR("Units: in / lb"), static_cast<int>(UnitSystem::Imperial));
    s.units->addItem(ETR("Units: cm / kg"), static_cast<int>(UnitSystem::Metric));
    s.units->setToolTip(ETR("Display units. The saved robot model always uses SI units."));
    barLayout->addWidget(s.units);
    connect(s.units, &QComboBox::activated, this, [this](int index) {
        d->controller.settings().setUnits(static_cast<UnitSystem>(d->units->itemData(index).toInt()));
    });
    s.status = new QLabel(bar);
    s.status->setEnabled(false);
    s.status->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    barLayout->addWidget(s.status, 1);
    s.saveState = new QLabel(bar);
    s.saveState->setEnabled(false);
    barLayout->addWidget(s.saveState);
    s.validation = new QToolButton(bar);
    s.validation->setAutoRaise(true);
    s.validation->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    s.validation->setToolTip(ETR("Open the validation list"));
    barLayout->addWidget(s.validation);
    root->addWidget(bar);

    // ---- left column ----
    s.tree = new ProjectTreeView(s.treeModel, s.icons, this);
    s.devices = new DevicePanel(c, s.deviceModel, s.icons, this);
    auto* left = new QSplitter(Qt::Vertical, this);
    left->setChildrenCollapsible(false);
    left->addWidget(titled(ETR("Project"), s.tree, left));
    left->addWidget(s.devices);
    left->setStretchFactor(0, 5);
    left->setStretchFactor(1, 4);
    left->setSizes({420, 380});
    left->setMinimumWidth(250);

    // ---- centre ----
    s.viewport = new ViewportWidget(c, s.viewportController, this);
    s.bottom = new BottomPanel(c, s.issueModel, s.icons, this);
    auto* centre = new QSplitter(Qt::Vertical, this);
    centre->addWidget(s.viewport);
    centre->addWidget(s.bottom);
    centre->setStretchFactor(0, 4);
    centre->setStretchFactor(1, 1);
    centre->setSizes({560, 170});
    centre->setCollapsible(0, false);

    // ---- right ----
    s.inspector = new InspectorPanel(c, s.icons, this);
    s.inspector->setMinimumWidth(320);

    auto* main = new QSplitter(Qt::Horizontal, this);
    main->addWidget(left);
    main->addWidget(centre);
    main->addWidget(s.inspector);
    main->setStretchFactor(0, 0);
    main->setStretchFactor(1, 1);
    main->setStretchFactor(2, 0);
    main->setSizes({300, 700, 360});
    main->setChildrenCollapsible(false);
    root->addWidget(main, 1);

    s.toasts = new ToastOverlay(s.viewport);

    // ---- wiring: model -> views ----
    auto refreshTree = [this] { d->treeModel.refresh(); };
    connect(&c.document(), &EditorDocument::changed, this, refreshTree);
    connect(&c, &EditorController::issuesUpdated, this, [this] { d->treeModel.setIssues(d->controller.issues()); });
    d->treeModel.setIssues(c.issues());
    connect(&c, &EditorController::devicesUpdated, this, refreshTree);
    connect(&s.backend, &IEditorBackend::deviceDiscovered, this, [this](const sdk::DiscoveredDevice& dev) { d->deviceModel.markRecent(dev); });
    connect(&c.selection(), &SelectionState::changed, this, [this](const SelectionRef& ref) {
        d->tree->selectRef(ref.kind == SelKind::Device ? SelectionRef{} : ref);
    });
    connect(s.tree, &ProjectTreeView::refSelected, this, [this](const SelectionRef& ref) { d->controller.selection().set(ref); });
    connect(s.tree, &ProjectTreeView::contextMenuRequestedFor, this, [this](const SelectionRef& ref, const QPoint& global) {
        const sdk::RobotConfig& cfg = d->controller.document().config();
        const EditorCatalog& cat = d->controller.document().catalog();
        QMenu menu(this);
        // Core templates only; the game plugin's own templates are added from the plugin node.
        // A core template with a `motion` choice is offered once per choice (Rotary / Arm / Linear).
        using AddFn = std::function<void(const QString&, const ops::MechanismVariant&)>;
        auto addTemplates = [&](QMenu* into, const AddFn& fn, bool plugin = false) {
            for (const auto& t : cat.mechanismTemplates) {
                if (plugin ? t.origin != cat.pluginId : t.origin != "core") continue;
                const QString type = QString::fromStdString(t.type);
                const QColor color = IconFactory::colorForCategory(QString::fromStdString(t.category));
                const sdk::PropertySpec* motion = nullptr;
                for (const auto& p : t.properties.properties)
                    if (!plugin && p.key == "motion" && p.type == sdk::PropertyType::Enum) motion = &p;
                if (motion == nullptr) {
                    QAction* a = into->addAction(d->icons.glyph(QString::fromStdString(t.displayName).left(1).toUpper(), color), QString::fromStdString(t.displayName));
                    connect(a, &QAction::triggered, this, [fn, type] { fn(type, {}); });
                    continue;
                }
                for (const auto& o : motion->options) {
                    QAction* a = into->addAction(d->icons.glyph(QString::fromStdString(o.label).left(1).toUpper(), color), QString::fromStdString(o.label));
                    connect(a, &QAction::triggered, this, [fn, type, key = motion->key, value = o.value] { fn(type, {key, value}); });
                }
            }
        };
        auto addZoneKinds = [&](QMenu* into, const sdk::MechanismAttachment& att) {
            for (const auto& k : cat.zoneKinds) {
                QAction* a = into->addAction(d->icons.node(NodeKind::Zone), ETR("Add %1").arg(QString::fromStdString(k.displayName)));
                connect(a, &QAction::triggered, this, [this, kind = QString::fromStdString(k.kind), att] { d->controller.addZone(kind, att); });
            }
        };
        auto addCameraMenu = [&](QMenu* into, const sdk::MechanismAttachment& att) {
            QMenu* sub = into->addMenu(d->icons.node(NodeKind::Camera), ETR("Add camera"));
            for (const auto kind : {sdk::CameraKind::Limelight, sdk::CameraKind::PhotonVision}) {
                const QString name = kind == sdk::CameraKind::PhotonVision ? QStringLiteral("PhotonVision") : QStringLiteral("Limelight");
                connect(sub->addAction(name), &QAction::triggered, this, [this, kind, att] { d->controller.addCamera(kind, att); });
            }
        };
        // Sensors have no pose: they are driven by a game value.
        auto addSensorMenu = [&](QMenu* into) {
            QMenu* sub = into->addMenu(d->icons.node(NodeKind::Sensor), ETR("Add sensor"));
            for (const auto& k : sdk::sensorKinds())
                connect(sub->addAction(QString::fromUtf8(k.displayName)), &QAction::triggered, this,
                        [this, kind = k.kind] { d->controller.addSensor(kind); });
        };
        auto addStartPositionAction = [&](QMenu* into) {
            into->addAction(d->icons.node(NodeKind::StartPosition), ETR("Add start position"), this, [this] { d->controller.addStartPosition(); });
        };
        // "Attach to" for something hanging on the chassis or a mechanism link (zones, cameras).
        using AttachFn = std::function<void(const QString& parent, const QString& link)>;
        auto addAttachMenu = [&](QMenu* into, const AttachFn& attach) {
            auto* sub = into->addMenu(ETR("Attach to"));
            sub->addAction(ETR("Chassis"), this, [attach] { attach(QStringLiteral("chassis"), QStringLiteral("root")); });
            for (const auto& m : cfg.mechanisms) {
                const auto* info = cat.findTemplate(m.type);
                const QString name = QString::fromStdString(ops::displayNameFor(m, cat));
                const QString mechId = QString::fromStdString(m.id);
                if (info == nullptr || info->links.empty()) {
                    sub->addAction(name, this, [attach, mechId] { attach(mechId, QStringLiteral("root")); });
                    continue;
                }
                auto* links = sub->addMenu(name);
                links->addAction(ETR("Origin"), this, [attach, mechId] { attach(mechId, QStringLiteral("root")); });
                for (const auto& l : info->links)
                    links->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), this,
                                     [attach, mechId, link = QString::fromStdString(l.name)] { attach(mechId, link); });
            }
        };
        switch (ref.kind) {
            case SelKind::Camera: {
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                menu.addSeparator();
                const QString cameraId = QString::fromStdString(ref.owner);
                addAttachMenu(&menu, [this, cameraId](const QString& parent, const QString& link) {
                    d->controller.document().reattachCamera(cameraId, parent, link);
                });
                break;
            }
            case SelKind::StartPosition: {
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                const bool hasSymmetry = cat.fieldFrame.symmetry != sdk::FieldSymmetry::None;
                const sdk::StartPosition* p = cfg.findStartPosition(ref.owner);
                QAction* copy = menu.addAction(p != nullptr && p->alliance == sdk::Alliance::Red ? ETR("Copy to Blue") : ETR("Copy to Red"), this,
                                               [this] { d->controller.copySelectedStartPositionToOtherAlliance(); });
                copy->setEnabled(hasSymmetry);
                menu.addSeparator();
                addStartPositionAction(&menu);
                break;
            }
            case SelKind::Sensor:
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                menu.addSeparator();
                addSensorMenu(&menu);
                break;
            case SelKind::Zone: {
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                menu.addSeparator();
                const QString zoneId = QString::fromStdString(ref.owner);
                auto* attach = menu.addMenu(ETR("Attach to"));
                attach->addAction(ETR("Chassis"), this, [this, zoneId] { d->controller.document().reattachZone(zoneId, QStringLiteral("chassis")); });
                for (const auto& m : cfg.mechanisms) {
                    const auto* info = cat.findTemplate(m.type);
                    const QString name = QString::fromStdString(ops::displayNameFor(m, cat));
                    const QString mechId = QString::fromStdString(m.id);
                    if (info == nullptr || info->links.empty()) {
                        attach->addAction(name, this, [this, zoneId, mechId] { d->controller.document().reattachZone(zoneId, mechId); });
                        continue;
                    }
                    auto* sub = attach->addMenu(name);
                    sub->addAction(ETR("Origin"), this, [this, zoneId, mechId] { d->controller.document().reattachZone(zoneId, mechId); });
                    for (const auto& l : info->links)
                        sub->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), this,
                                       [this, zoneId, mechId, link = QString::fromStdString(l.name)] { d->controller.document().reattachZone(zoneId, mechId, link); });
                }
                break;
            }
            case SelKind::Mechanism: {
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                menu.addSeparator();
                auto* attach = menu.addMenu(ETR("Attach to"));
                attach->addAction(ETR("Chassis"), this, [this, ref] { d->controller.document().reparentMechanism(QString::fromStdString(ref.owner), QStringLiteral("chassis")); });
                for (const auto& m : cfg.mechanisms) {
                    if (!ops::canReparent(cfg, ref.owner, m.id) || m.id == ref.owner) continue;
                    const auto* info = cat.findTemplate(m.type);
                    const QString name = QString::fromStdString(ops::displayNameFor(m, cat));
                    if (info == nullptr || info->links.empty()) {
                        attach->addAction(name, this, [this, ref, id = m.id] { d->controller.document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id)); });
                        continue;
                    }
                    auto* sub = attach->addMenu(name);
                    sub->addAction(ETR("Origin"), this, [this, ref, id = m.id] { d->controller.document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id)); });
                    for (const auto& l : info->links)
                        sub->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), this, [this, ref, id = m.id, link = l.name] {
                            d->controller.document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id), QString::fromStdString(link));
                        });
                }
                addTemplates(menu.addMenu(ETR("Add attached mechanism")), [this](const QString& type, const ops::MechanismVariant& v) { d->controller.addMechanismToSelection(type, v); });
                if (!cat.zoneKinds.empty()) {
                    menu.addSeparator();
                    sdk::MechanismAttachment onMechanism;
                    onMechanism.parentMechanismId = ref.owner;
                    addZoneKinds(&menu, onMechanism);
                }
                {
                    sdk::MechanismAttachment onMechanism;
                    onMechanism.parentMechanismId = ref.owner;
                    menu.addSeparator();
                    addCameraMenu(&menu, onMechanism);
                }
                break;
            }
            case SelKind::Slot:
                d->controller.selection().set(ref);
                if (ops::findMapping(cfg, {ref.owner, ref.sub}) != nullptr)
                    menu.addAction(ETR("Unmap device"), this, [this, ref] { d->controller.clearSlot({ref.owner, ref.sub}); });
                break;
            case SelKind::Primitive:
                d->controller.selection().set(ref);
                menu.addAction(ETR("Duplicate"), this, [this] { d->controller.duplicateSelection(); });
                menu.addAction(ETR("Delete"), this, [this] { d->controller.deleteSelection(); });
                break;
            case SelKind::Wheel:
                d->controller.selection().set(ref);
                menu.addAction(ETR("Add wheel"), this, [this] {
                    const QString n = d->controller.document().addWheel();
                    if (!n.isEmpty()) d->controller.selection().set(SelectionRef::wheel(n.toStdString()));
                });
                menu.addAction(ETR("Delete wheel"), this, [this] { d->controller.deleteSelection(); });
                break;
            case SelKind::Drivetrain:
                menu.addAction(ETR("Add wheel"), this, [this] {
                    const QString n = d->controller.document().addWheel();
                    if (!n.isEmpty()) d->controller.selection().set(SelectionRef::wheel(n.toStdString()));
                });
                break;
            case SelKind::Chassis:
                addTemplates(menu.addMenu(ETR("Add mechanism")), [this](const QString& type, const ops::MechanismVariant& v) {
                    d->controller.addMechanism(type, sdk::MechanismAttachment{}, v);
                });
                menu.addSeparator();
                for (auto kind : {sdk::PrimitiveKind::Box, sdk::PrimitiveKind::Cylinder})
                    menu.addAction(kind == sdk::PrimitiveKind::Box ? ETR("Add box") : ETR("Add cylinder"), this, [this, kind] {
                        const int i = d->controller.document().addPrimitive(QString::fromLatin1(ops::kChassisOwner), kind);
                        if (i >= 0) d->controller.selection().set(SelectionRef::primitive(ops::kChassisOwner, i));
                    });
                if (!cat.zoneKinds.empty()) {
                    menu.addSeparator();
                    addZoneKinds(&menu, sdk::MechanismAttachment{});
                }
                menu.addSeparator();
                addCameraMenu(&menu, sdk::MechanismAttachment{});
                addSensorMenu(&menu);
                break;
            case SelKind::Plugin:
                addTemplates(&menu, [this](const QString& type, const ops::MechanismVariant& v) { d->controller.addMechanism(type, sdk::MechanismAttachment{}, v); }, true);
                addZoneKinds(&menu, sdk::MechanismAttachment{});
                break;
            default:
                addTemplates(menu.addMenu(ETR("Add mechanism")), [this](const QString& type, const ops::MechanismVariant& v) {
                    d->controller.addMechanism(type, sdk::MechanismAttachment{}, v);
                });
                addCameraMenu(&menu, sdk::MechanismAttachment{});
                addSensorMenu(&menu);
                addStartPositionAction(&menu);
                break;
        }
        if (!menu.isEmpty()) menu.exec(global);
    });
    connect(&s.treeModel, &ProjectTreeModel::deviceDropped, this, [this](const SelectionRef& target, const sdk::DeviceKey& key, quint32 caps) {
        const QString err = d->controller.mapDevice(target, key, sdk::CapabilitySet{caps});
        if (!err.isEmpty()) d->toasts->showToast(ETR("Cannot map the device"), err, 1);
    });
    auto chooseLink = [this](const sdk::MechanismAttachment& att) -> std::optional<QString> {
        const sdk::RobotConfig& cfg = d->controller.document().config();
        const EditorCatalog& cat = d->controller.document().catalog();
        const sdk::MechanismConfig* parent = att.onChassis() ? nullptr : cfg.findMechanism(att.parentMechanismId);
        const auto* info = parent != nullptr ? cat.findTemplate(parent->type) : nullptr;
        if (info == nullptr || info->links.empty()) return QStringLiteral("root");
        QMenu menu(this);
        menu.addAction(ETR("Attach to origin"))->setData(QStringLiteral("root"));
        for (const auto& l : info->links)
            menu.addAction(ETR("Attach to %1").arg(QString::fromStdString(l.label.empty() ? l.name : l.label)))->setData(QString::fromStdString(l.name));
        const QAction* chosen = menu.exec(QCursor::pos());
        if (chosen == nullptr) return std::nullopt;
        return chosen->data().toString();
    };
    connect(&s.treeModel, &ProjectTreeModel::mechanismDropped, this, [this, chooseLink](const QString& id, const sdk::MechanismAttachment& att) {
        if (const auto link = chooseLink(att)) d->controller.document().reparentMechanism(id, QString::fromStdString(att.parentMechanismId), *link);
    });
    connect(&s.treeModel, &ProjectTreeModel::zoneDropped, this, [this, chooseLink](const QString& id, const sdk::MechanismAttachment& att) {
        if (const auto link = chooseLink(att)) d->controller.document().reattachZone(id, QString::fromStdString(att.parentMechanismId), *link);
    });
    connect(&s.treeModel, &ProjectTreeModel::cameraDropped, this, [this, chooseLink](const QString& id, const sdk::MechanismAttachment& att) {
        if (const auto link = chooseLink(att)) d->controller.document().reattachCamera(id, QString::fromStdString(att.parentMechanismId), *link);
    });

    connect(&c, &EditorController::deviceToast, this, [this](const QString& title, const QString& detail) { d->toasts->showToast(title, detail, 0); });
    connect(&c, &EditorController::statusMessage, this, [this](const QString& text, int severity) {
        d->status->setText(text);
        d->status->setToolTip(text);
        d->bottom->appendLog(severity == 2 ? sdk::Severity::Error : severity == 1 ? sdk::Severity::Warning : sdk::Severity::Info, text);
        if (severity >= 1) d->toasts->showToast(severity == 2 ? ETR("Problem") : ETR("Heads up"), text, severity);
    });
    connect(&c, &EditorController::logLine, this, [this](sdk::Severity sev, const QString& text) { d->bottom->appendLog(sev, text); });
    connect(&c, &EditorController::snapshotUpdated, this, [this](std::shared_ptr<const sdk::ISimSnapshot> snap) { d->bottom->setSnapshot(snap); });
    connect(&c, &EditorController::detectionChanged, this, [this](bool running) {
        d->detect->setText(running ? ETR("Stop detecting") : ETR("Detect devices"));
    });
    connect(&c, &EditorController::saveStateChanged, this, [this] { d->saveState->setText(d->controller.saveStatusText()); });
    connect(&c, &EditorController::autosaveRequested, this, &EditorPage::autosaveRequested);
    connect(&c, &EditorController::configEdited, this, &EditorPage::configEdited);
    connect(&c.settings(), &EditorSettings::changed, this, [this] {
        const int index = d->units->findData(static_cast<int>(d->controller.settings().units()));
        if (index >= 0) d->units->setCurrentIndex(index);
    });

    auto updateValidationButton = [this] {
        d->issueModel.setIssues(d->controller.issues());
        const int e = d->issueModel.count(sdk::Severity::Error);
        const int w = d->issueModel.count(sdk::Severity::Warning);
        d->validation->setText(d->issueModel.summary());
        d->validation->setIcon(e > 0 ? d->icons.severity(2) : w > 0 ? d->icons.severity(1) : d->icons.deviceStatus(true, true, 0));
    };
    connect(&c, &EditorController::issuesUpdated, this, updateValidationButton);
    connect(s.validation, &QToolButton::clicked, this, [this] { d->bottom->showValidation(); });
    updateValidationButton();

    d->tree->selectRef(c.selection().current());
    d->saveState->setText(c.saveStatusText());
    d->status->setText(ETR("Pick a part in the viewport or the tree. Drag detected devices onto slots to map them."));
}

EditorPage::~EditorPage() {
    // The views reference models and controllers owned by Impl: destroy them first.
    delete d->toasts;
    delete d->inspector;
    delete d->bottom;
    delete d->viewport;
    delete d->devices;
    delete d->tree;
}

QUndoStack* EditorPage::undoStack() const { return d->controller.document().undoStack(); }
QAction* EditorPage::undoAction() const { return d->undo; }
QAction* EditorPage::redoAction() const { return d->redo; }
QAction* EditorPage::saveAction() const { return d->save; }
QAction* EditorPage::detectAction() const { return d->detect; }
QAction* EditorPage::duplicateAction() const { return d->duplicate; }
QAction* EditorPage::deleteAction() const { return d->remove; }
void EditorPage::setAutosaveDelayMs(int ms) { d->controller.setAutosaveDelayMs(ms); }
bool EditorPage::hasUnsavedChanges() const { return d->controller.hasPendingChanges(); }
EditorController& EditorPage::controller() const { return d->controller; }
ViewportWidget* EditorPage::viewport() const { return d->viewport; }
InspectorPanel* EditorPage::inspector() const { return d->inspector; }
DevicePanel* EditorPage::devicePanel() const { return d->devices; }
BottomPanel* EditorPage::bottomPanel() const { return d->bottom; }
ProjectTreeView* EditorPage::tree() const { return d->tree; }
ToastOverlay* EditorPage::toasts() const { return d->toasts; }

}  // namespace wpiemu::app::editor
