#include "views/tree_context_menu.h"

#include <QAction>
#include <QMenu>

#include <functional>

#include "model/config_ops.h"
#include "model/tr.h"
#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"
#include "wpiemu/sdk/config/sensor.h"

namespace wpiemu::app::editor {

namespace {

using AddFn = std::function<void(const QString&, const ops::MechanismVariant&)>;
using AttachFn = std::function<void(const QString& parent, const QString& link)>;

// The entries of one row's menu, built from the current config and catalog. The entries' callbacks hold the controller, not
// this object: they run while the menu is open, but the controller is what outlives it.
class TreeMenu {
public:
    TreeMenu(QWidget* owner, EditorController& controller, const IconFactory& icons)
        : owner_(owner),
          controller_(controller),
          icons_(icons),
          cfg_(controller.document().config()),
          cat_(controller.document().catalog()),
          menu_(owner) {}

    void show(const SelectionRef& ref, const QPoint& global) {
        switch (ref.kind) {
            case SelKind::Camera: forCamera(ref); break;
            case SelKind::StartPosition: forStartPosition(ref); break;
            case SelKind::Sensor: forSensor(ref); break;
            case SelKind::Zone: forZone(ref); break;
            case SelKind::Mechanism: forMechanism(ref); break;
            case SelKind::Slot: forSlot(ref); break;
            case SelKind::Primitive: forPrimitive(ref); break;
            case SelKind::Wheel: forWheel(ref); break;
            case SelKind::Drivetrain: forDrivetrain(); break;
            case SelKind::Chassis: forChassis(); break;
            case SelKind::Plugin: forPlugin(); break;
            default: forRobot(); break;
        }
        if (!menu_.isEmpty()) menu_.exec(global);
    }

private:
    // ---- entries shared by several kinds ----
    // Core templates only; the game plugin's own templates are added from the plugin node.
    // A core template with a `motion` choice is offered once per choice (Rotary / Arm / Linear).
    void addTemplates(QMenu* into, const AddFn& fn, bool plugin = false) {
        for (const auto& t : cat_.mechanismTemplates) {
            if (plugin ? t.origin != cat_.pluginId : t.origin != "core") continue;
            const QString type = QString::fromStdString(t.type);
            const QColor color = IconFactory::colorForCategory(QString::fromStdString(t.category));
            const sdk::PropertySpec* motion = nullptr;
            for (const auto& p : t.properties.properties)
                if (!plugin && p.key == "motion" && p.type == sdk::PropertyType::Enum) motion = &p;
            if (motion == nullptr) {
                QAction* a = into->addAction(icons_.glyph(QString::fromStdString(t.displayName).left(1).toUpper(), color), QString::fromStdString(t.displayName));
                QObject::connect(a, &QAction::triggered, owner_, [fn, type] { fn(type, {}); });
                continue;
            }
            for (const auto& o : motion->options) {
                QAction* a = into->addAction(icons_.glyph(QString::fromStdString(o.label).left(1).toUpper(), color), QString::fromStdString(o.label));
                QObject::connect(a, &QAction::triggered, owner_, [fn, type, key = motion->key, value = o.value] { fn(type, {key, value}); });
            }
        }
    }

    void addZoneKinds(QMenu* into, const sdk::MechanismAttachment& att) {
        for (const auto& k : cat_.zoneKinds) {
            QAction* a = into->addAction(icons_.node(NodeKind::Zone), ETR("Add %1").arg(QString::fromStdString(k.displayName)));
            QObject::connect(a, &QAction::triggered, owner_, [ctl = &controller_, kind = QString::fromStdString(k.kind), att] { ctl->addZone(kind, att); });
        }
    }

    void addCameraMenu(QMenu* into, const sdk::MechanismAttachment& att) {
        QMenu* sub = into->addMenu(icons_.node(NodeKind::Camera), ETR("Add camera"));
        for (const auto kind : {sdk::CameraKind::Limelight, sdk::CameraKind::PhotonVision}) {
            const QString name = kind == sdk::CameraKind::PhotonVision ? QStringLiteral("PhotonVision") : QStringLiteral("Limelight");
            QObject::connect(sub->addAction(name), &QAction::triggered, owner_, [ctl = &controller_, kind, att] { ctl->addCamera(kind, att); });
        }
    }

    // Sensors have no pose: they are driven by a game value.
    void addSensorMenu(QMenu* into) {
        QMenu* sub = into->addMenu(icons_.node(NodeKind::Sensor), ETR("Add sensor"));
        for (const auto& k : sdk::sensorKinds())
            QObject::connect(sub->addAction(QString::fromUtf8(k.displayName)), &QAction::triggered, owner_,
                             [ctl = &controller_, kind = k.kind] { ctl->addSensor(kind); });
    }

    void addStartPositionAction(QMenu* into) {
        into->addAction(icons_.node(NodeKind::StartPosition), ETR("Add start position"), owner_, [ctl = &controller_] { ctl->addStartPosition(); });
    }

    // "Attach to" for something hanging on the chassis or a mechanism link (zones, cameras).
    void addAttachMenu(QMenu* into, const AttachFn& attach) {
        auto* sub = into->addMenu(ETR("Attach to"));
        sub->addAction(ETR("Chassis"), owner_, [attach] { attach(QStringLiteral("chassis"), QStringLiteral("root")); });
        for (const auto& m : cfg_.mechanisms) {
            const auto* info = cat_.findTemplate(m.type);
            const QString name = QString::fromStdString(ops::displayNameFor(m, cat_));
            const QString mechId = QString::fromStdString(m.id);
            if (info == nullptr || info->links.empty()) {
                sub->addAction(name, owner_, [attach, mechId] { attach(mechId, QStringLiteral("root")); });
                continue;
            }
            auto* links = sub->addMenu(name);
            links->addAction(ETR("Origin"), owner_, [attach, mechId] { attach(mechId, QStringLiteral("root")); });
            for (const auto& l : info->links)
                links->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), owner_,
                                 [attach, mechId, link = QString::fromStdString(l.name)] { attach(mechId, link); });
        }
    }

    void addDuplicateAndDelete() {
        menu_.addAction(ETR("Duplicate"), owner_, [ctl = &controller_] { ctl->duplicateSelection(); });
        menu_.addAction(ETR("Delete"), owner_, [ctl = &controller_] { ctl->deleteSelection(); });
    }

    void addWheelAction() {
        menu_.addAction(ETR("Add wheel"), owner_, [ctl = &controller_] {
            const QString n = ctl->document().addWheel();
            if (!n.isEmpty()) ctl->selection().set(SelectionRef::wheel(n.toStdString()));
        });
    }

    AddFn addToRobot() {
        return [ctl = &controller_](const QString& type, const ops::MechanismVariant& v) {
            ctl->addMechanism(type, sdk::MechanismAttachment{}, v);
        };
    }

    // ---- one row kind each ----
    void forCamera(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
        menu_.addSeparator();
        const QString cameraId = QString::fromStdString(ref.owner);
        addAttachMenu(&menu_, [ctl = &controller_, cameraId](const QString& parent, const QString& link) {
            ctl->document().reattachCamera(cameraId, parent, link);
        });
    }

    void forStartPosition(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
        const bool hasSymmetry = cat_.fieldFrame.symmetry != sdk::FieldSymmetry::None;
        const sdk::StartPosition* p = cfg_.findStartPosition(ref.owner);
        QAction* copy = menu_.addAction(p != nullptr && p->alliance == sdk::Alliance::Red ? ETR("Copy to Blue") : ETR("Copy to Red"), owner_,
                                        [ctl = &controller_] { ctl->copySelectedStartPositionToOtherAlliance(); });
        copy->setEnabled(hasSymmetry);
        menu_.addSeparator();
        addStartPositionAction(&menu_);
    }

    void forSensor(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
        menu_.addSeparator();
        addSensorMenu(&menu_);
    }

    void forZone(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
        menu_.addSeparator();
        const QString zoneId = QString::fromStdString(ref.owner);
        auto* attach = menu_.addMenu(ETR("Attach to"));
        attach->addAction(ETR("Chassis"), owner_, [ctl = &controller_, zoneId] { ctl->document().reattachZone(zoneId, QStringLiteral("chassis")); });
        for (const auto& m : cfg_.mechanisms) {
            const auto* info = cat_.findTemplate(m.type);
            const QString name = QString::fromStdString(ops::displayNameFor(m, cat_));
            const QString mechId = QString::fromStdString(m.id);
            if (info == nullptr || info->links.empty()) {
                attach->addAction(name, owner_, [ctl = &controller_, zoneId, mechId] { ctl->document().reattachZone(zoneId, mechId); });
                continue;
            }
            auto* sub = attach->addMenu(name);
            sub->addAction(ETR("Origin"), owner_, [ctl = &controller_, zoneId, mechId] { ctl->document().reattachZone(zoneId, mechId); });
            for (const auto& l : info->links)
                sub->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), owner_,
                               [ctl = &controller_, zoneId, mechId, link = QString::fromStdString(l.name)] { ctl->document().reattachZone(zoneId, mechId, link); });
        }
    }

    void forMechanism(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
        menu_.addSeparator();
        auto* attach = menu_.addMenu(ETR("Attach to"));
        attach->addAction(ETR("Chassis"), owner_, [ctl = &controller_, ref] { ctl->document().reparentMechanism(QString::fromStdString(ref.owner), QStringLiteral("chassis")); });
        for (const auto& m : cfg_.mechanisms) {
            if (!ops::canReparent(cfg_, ref.owner, m.id) || m.id == ref.owner) continue;
            const auto* info = cat_.findTemplate(m.type);
            const QString name = QString::fromStdString(ops::displayNameFor(m, cat_));
            if (info == nullptr || info->links.empty()) {
                attach->addAction(name, owner_, [ctl = &controller_, ref, id = m.id] { ctl->document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id)); });
                continue;
            }
            auto* sub = attach->addMenu(name);
            sub->addAction(ETR("Origin"), owner_, [ctl = &controller_, ref, id = m.id] { ctl->document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id)); });
            for (const auto& l : info->links)
                sub->addAction(QString::fromStdString(l.label.empty() ? l.name : l.label), owner_, [ctl = &controller_, ref, id = m.id, link = l.name] {
                    ctl->document().reparentMechanism(QString::fromStdString(ref.owner), QString::fromStdString(id), QString::fromStdString(link));
                });
        }
        addTemplates(menu_.addMenu(ETR("Add attached mechanism")), [ctl = &controller_](const QString& type, const ops::MechanismVariant& v) { ctl->addMechanismToSelection(type, v); });
        if (!cat_.zoneKinds.empty()) {
            menu_.addSeparator();
            sdk::MechanismAttachment onMechanism;
            onMechanism.parentMechanismId = ref.owner;
            addZoneKinds(&menu_, onMechanism);
        }
        {
            sdk::MechanismAttachment onMechanism;
            onMechanism.parentMechanismId = ref.owner;
            menu_.addSeparator();
            addCameraMenu(&menu_, onMechanism);
        }
    }

    void forSlot(const SelectionRef& ref) {
        controller_.selection().set(ref);
        if (ops::findMapping(cfg_, {ref.owner, ref.sub}) != nullptr)
            menu_.addAction(ETR("Unmap device"), owner_, [ctl = &controller_, ref] { ctl->clearSlot({ref.owner, ref.sub}); });
    }

    void forPrimitive(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addDuplicateAndDelete();
    }

    void forWheel(const SelectionRef& ref) {
        controller_.selection().set(ref);
        addWheelAction();
        menu_.addAction(ETR("Delete wheel"), owner_, [ctl = &controller_] { ctl->deleteSelection(); });
    }

    void forDrivetrain() { addWheelAction(); }

    void forChassis() {
        addTemplates(menu_.addMenu(ETR("Add mechanism")), addToRobot());
        menu_.addSeparator();
        for (auto kind : {sdk::PrimitiveKind::Box, sdk::PrimitiveKind::Cylinder})
            menu_.addAction(kind == sdk::PrimitiveKind::Box ? ETR("Add box") : ETR("Add cylinder"), owner_, [ctl = &controller_, kind] {
                const int i = ctl->document().addPrimitive(QString::fromLatin1(ops::kChassisOwner), kind);
                if (i >= 0) ctl->selection().set(SelectionRef::primitive(ops::kChassisOwner, i));
            });
        if (!cat_.zoneKinds.empty()) {
            menu_.addSeparator();
            addZoneKinds(&menu_, sdk::MechanismAttachment{});
        }
        menu_.addSeparator();
        addCameraMenu(&menu_, sdk::MechanismAttachment{});
        addSensorMenu(&menu_);
    }

    void forPlugin() {
        addTemplates(&menu_, addToRobot(), true);
        addZoneKinds(&menu_, sdk::MechanismAttachment{});
    }

    void forRobot() {
        addTemplates(menu_.addMenu(ETR("Add mechanism")), addToRobot());
        addCameraMenu(&menu_, sdk::MechanismAttachment{});
        addSensorMenu(&menu_);
        addStartPositionAction(&menu_);
    }

    QWidget* owner_;
    EditorController& controller_;
    const IconFactory& icons_;
    const sdk::RobotConfig& cfg_;
    const EditorCatalog& cat_;
    QMenu menu_;
};

}  // namespace

void showTreeContextMenu(QWidget* owner, EditorController& controller, const IconFactory& icons, const SelectionRef& ref,
                         const QPoint& globalPos) {
    TreeMenu(owner, controller, icons).show(ref, globalPos);
}

}  // namespace wpiemu::app::editor
