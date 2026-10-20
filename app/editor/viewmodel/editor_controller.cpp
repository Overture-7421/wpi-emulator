#include "viewmodel/editor_controller.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

#include <QTime>

#include <algorithm>

#include "model/config_ops.h"
#include "model/motion_links.h"
#include "model/property_groups.h"
#include "model/tr.h"

namespace wpiemu::app::editor {

EditorController::EditorController(IEditorBackend& backend, QObject* parent) : QObject(parent), backend_(backend) {
    validateTimer_.setSingleShot(true);
    validateTimer_.setInterval(120);
    saveTimer_.setSingleShot(true);
    connect(&validateTimer_, &QTimer::timeout, this, [this] { runValidation(); });
    connect(&saveTimer_, &QTimer::timeout, this, [this] { (void)persist(); });

    connect(&document_, &EditorDocument::changed, this, [this] {
        ensureSelectionValid();
        scheduleValidation();
    });
    connect(&document_, &EditorDocument::modified, this, [this] { onDocumentModified(); });

    connect(&backend_, &IEditorBackend::configReplaced, this, [this] { reloadFromBackend(); });
    connect(&backend_, &IEditorBackend::catalogChanged, this, [this] { document_.setCatalog(backend_.catalog()); });
    connect(&backend_, &IEditorBackend::devicesChanged, this, [this] {
        devices_.setDevices(backend_.devices());
        Q_EMIT devicesUpdated();
        scheduleValidation();
    });
    auto onDevice = [this](const sdk::DiscoveredDevice& d, bool announceNew) {
        const bool isNew = devices_.upsert(d);
        if (isNew && announceNew) {
            const QString what = deviceLabel(d.descriptor);
            Q_EMIT deviceToast(ETR("New device detected"),
                               ETR("%1 on bus %2").arg(what, d.observedBus.empty() ? busLabel(d.descriptor.bus) : QString::fromStdString(d.observedBus)));
        }
        Q_EMIT devicesUpdated();
        scheduleValidation();
    };
    connect(&backend_, &IEditorBackend::deviceDiscovered, this, [onDevice](const sdk::DiscoveredDevice& d) { onDevice(d, true); });
    connect(&backend_, &IEditorBackend::deviceUpdated, this, [onDevice](const sdk::DiscoveredDevice& d) { onDevice(d, false); });
    connect(&backend_, &IEditorBackend::issuesChanged, this, [this](std::vector<sdk::Issue> issues) {
        pushedIssues_ = std::move(issues);
        scheduleValidation();
    });
    connect(&backend_, &IEditorBackend::detectionStateChanged, this, [this](bool running) {
        Q_EMIT detectionChanged(running);
        Q_EMIT statusMessage(running ? ETR("Detecting devices: run your robot code, new devices appear here as they announce themselves.")
                                     : ETR("Device detection finished."), 0);
    });
    connect(&backend_, &IEditorBackend::logMessage, this, [this](sdk::Severity s, const QString& t) { Q_EMIT logLine(s, t); });
    connect(&backend_, &IEditorBackend::snapshotChanged, this,
            [this](std::shared_ptr<const sdk::ISimSnapshot> s) { Q_EMIT snapshotUpdated(std::move(s)); });

    document_.setCatalog(backend_.catalog());
    devices_.setDevices(backend_.devices());
    applyingBackend_ = true;
    document_.load(backend_.config());
    applyingBackend_ = false;
    selection_.set(SelectionRef::robot());
    runValidation();
}

void EditorController::reloadFromBackend() {
    applyingBackend_ = true;
    document_.setCatalog(backend_.catalog());
    document_.load(backend_.config());
    applyingBackend_ = false;
    pendingSave_ = false;
    saveError_.clear();
    lastSaved_.clear();
    saveTimer_.stop();
    selection_.set(SelectionRef::robot());
    Q_EMIT saveStateChanged();
    Q_EMIT statusMessage(ETR("Robot model reloaded."), 0);
}

void EditorController::scheduleValidation() { validateTimer_.start(); }

void EditorController::runValidation() {
    const sdk::RobotConfig& c = document_.config();
    issues_ = mergeIssues({backend_.validate(c), localIssues(c, document_.catalog(), devices_), pushedIssues_});
    std::stable_sort(issues_.begin(), issues_.end(), [](const sdk::Issue& a, const sdk::Issue& b) {
        return static_cast<int>(a.severity) > static_cast<int>(b.severity);
    });
    Q_EMIT issuesUpdated();
}

void EditorController::onDocumentModified() {
    if (applyingBackend_) return;
    backend_.submitConfig(document_.config());
    pendingSave_ = true;
    if (autosaveDelayMs_ > 0) saveTimer_.start(autosaveDelayMs_);
    Q_EMIT configEdited();
    Q_EMIT saveStateChanged();
}

sdk::Status EditorController::persist() {
    saveTimer_.stop();
    const sdk::Status result = backend_.saveConfig();
    if (!result) {
        saveError_ = QString::fromStdString(result.error().message);
        Q_EMIT saveStateChanged();
        Q_EMIT statusMessage(ETR("Save failed: %1. Check the app-data location and disk space, then retry Save. "
                                "Your changes are still open.").arg(saveError_), 2);
        return result;
    }
    document_.markSaved();
    pendingSave_ = false;
    saveError_.clear();
    lastSaved_ = QTime::currentTime().toString(QStringLiteral("HH:mm:ss"));
    Q_EMIT saveStateChanged();
    Q_EMIT autosaveRequested();
    return sdk::okStatus();
}

sdk::Status EditorController::flushSave() { return persist(); }

QString EditorController::saveStatusText() const {
    if (!saveError_.isEmpty()) return ETR("Save failed — retry Save: %1").arg(saveError_);
    if (pendingSave_) return ETR("Unsaved changes...");
    return lastSaved_.isEmpty() ? ETR("No changes") : ETR("Saved %1").arg(lastSaved_);
}

bool EditorController::selectionExists(const SelectionRef& ref) const {
    const sdk::RobotConfig& c = document_.config();
    switch (ref.kind) {
        case SelKind::Wheel: return ops::findWheel(c, ref.sub) != nullptr;
        case SelKind::Primitive: {
            const auto* list = ops::primitivesOf(c, ref.owner);
            return list != nullptr && ref.index >= 0 && ref.index < static_cast<int>(list->size());
        }
        case SelKind::Mechanism: return c.findMechanism(ref.owner) != nullptr;
        case SelKind::Zone: return c.findZone(ref.owner) != nullptr;
        case SelKind::Camera: return c.findCamera(ref.owner) != nullptr;
        case SelKind::Sensor: return c.findSensor(ref.owner) != nullptr;
        case SelKind::Slot:
            for (const auto& s : ops::slotSpecsFor(c, document_.catalog(), ref.owner))
                if (s.name == ref.sub) return true;
            return false;
        case SelKind::Overlay:
            for (const auto& v : ops::volumesFor(c, document_.catalog(), ref.owner))
                if (v.key == ref.sub) return true;
            return false;
        case SelKind::Plugin: return document_.catalog().hasPlugin();
        default: return true;
    }
}

void EditorController::ensureSelectionValid() {
    const SelectionRef current = selection_.current();
    if (selectionExists(current)) return;
    // A motor row that went away (motor count lowered): stay on the part it belonged to.
    const SelectionRef part = current.kind == SelKind::Slot ? formTargetOf(document_.config(), current) : SelectionRef::robot();
    selection_.set(selectionExists(part) ? part : SelectionRef::robot());
}

QString EditorController::addMechanism(const QString& type, const sdk::MechanismAttachment& attachment, const ops::MechanismVariant& variant) {
    const QString id = document_.addMechanism(type, attachment, variant);
    if (id.isEmpty()) {
        Q_EMIT statusMessage(ETR("Unknown mechanism template \"%1\".").arg(type), 2);
        return {};
    }
    selection_.set(SelectionRef::mechanism(id.toStdString()));
    Q_EMIT statusMessage(ETR("Added %1. Drag it with the move gizmo, then drop devices on its slots.").arg(id), 0);
    return id;
}

QString EditorController::addMechanismToSelection(const QString& type, const ops::MechanismVariant& variant) {
    sdk::MechanismAttachment att;
    const SelectionRef target = formTargetOf(document_.config(), selection_.current());
    if (target.kind == SelKind::Mechanism && selection_.current().kind != SelKind::Slot) {
        att.parentMechanismId = target.owner;
        att.parentLink = sdk::kRootLink;
    }
    return addMechanism(type, att, variant);
}

QString EditorController::addZone(const QString& kind, const sdk::MechanismAttachment& attachment) {
    const QString id = document_.addZone(kind, attachment);
    if (id.isEmpty()) {
        Q_EMIT statusMessage(ETR("Unknown zone kind \"%1\".").arg(kind), 2);
        return {};
    }
    selection_.set(SelectionRef::zone(id.toStdString()));
    Q_EMIT statusMessage(ETR("Added zone %1. Drag it with the move gizmo and set when it is active.").arg(id), 0);
    return id;
}

QString EditorController::addCamera(sdk::CameraKind kind, const sdk::MechanismAttachment& attachment) {
    const QString id = document_.addCamera(kind, attachment);
    if (id.isEmpty()) return {};
    selection_.set(SelectionRef::camera(id.toStdString()));
    Q_EMIT statusMessage(ETR("Added camera %1. Drag it with the move gizmo and set the camera name your robot code uses.").arg(id), 0);
    return id;
}

QString EditorController::addSensor(sdk::SensorKind kind) {
    const QString id = document_.addSensor(kind);
    if (id.isEmpty()) return {};
    selection_.set(SelectionRef::sensor(id.toStdString()));
    Q_EMIT statusMessage(ETR("Added sensor %1. Pick the game value that drives it, then drop a device on its slot.").arg(id), 0);
    return id;
}

bool EditorController::deleteSelection() {
    const SelectionRef sel = selection_.current();
    switch (sel.kind) {
        case SelKind::Mechanism: return document_.removeMechanism(QString::fromStdString(sel.owner));
        case SelKind::Zone: return document_.removeZone(QString::fromStdString(sel.owner));
        case SelKind::Camera: return document_.removeCamera(QString::fromStdString(sel.owner));
        case SelKind::Sensor: return document_.removeSensor(QString::fromStdString(sel.owner));
        case SelKind::Primitive: return document_.removePrimitive(QString::fromStdString(sel.owner), sel.index);
        case SelKind::Wheel: return document_.removeWheel(QString::fromStdString(sel.sub));
        case SelKind::Slot: return clearSlot({sel.owner, sel.sub});
        default: return false;
    }
}

bool EditorController::duplicateSelection() {
    const SelectionRef sel = selection_.current();
    if (sel.kind == SelKind::Mechanism) {
        const QString id = document_.duplicateMechanism(QString::fromStdString(sel.owner));
        if (!id.isEmpty()) selection_.set(SelectionRef::mechanism(id.toStdString()));
        return !id.isEmpty();
    }
    if (sel.kind == SelKind::Zone) {
        const QString id = document_.duplicateZone(QString::fromStdString(sel.owner));
        if (!id.isEmpty()) selection_.set(SelectionRef::zone(id.toStdString()));
        return !id.isEmpty();
    }
    if (sel.kind == SelKind::Camera) {
        const QString id = document_.duplicateCamera(QString::fromStdString(sel.owner));
        if (!id.isEmpty()) selection_.set(SelectionRef::camera(id.toStdString()));
        return !id.isEmpty();
    }
    if (sel.kind == SelKind::Sensor) {
        const QString id = document_.duplicateSensor(QString::fromStdString(sel.owner));
        if (!id.isEmpty()) selection_.set(SelectionRef::sensor(id.toStdString()));
        return !id.isEmpty();
    }
    if (sel.kind == SelKind::Primitive) {
        const int index = document_.duplicatePrimitive(QString::fromStdString(sel.owner), sel.index);
        if (index >= 0) selection_.set(SelectionRef::primitive(sel.owner, index));
        return index >= 0;
    }
    return false;
}

bool EditorController::clearSlot(const sdk::SlotAddress& slot) { return document_.clearSlot(slot); }

QString EditorController::setDrivenBy(const sdk::SlotAddress& target, const sdk::SlotAddress& source, double displayRatio) {
    std::string error;
    const EditorCatalog& catalog = document_.catalog();
    const bool changed = document_.edit(ETR("Link motion"), [&](sdk::RobotConfig& c) {
        error = applyDrivenBy(c, catalog, target, source, displayRatio);
        return error.empty();
    });
    if (!changed && error.empty()) return {};   // nothing to do
    return QString::fromStdString(error);
}

bool EditorController::clearDrivenBy(const sdk::SlotAddress& target) {
    return document_.edit(ETR("Use own motor"), [&](sdk::RobotConfig& c) {
        const auto before = c.motionLinks.size();
        removeDrivenBy(c, target);
        return c.motionLinks.size() != before;
    });
}

QString EditorController::mapDevice(const SelectionRef& target, const sdk::DeviceKey& device, sdk::CapabilitySet caps) {
    const sdk::RobotConfig& c = document_.config();
    const sdk::DeviceKey key = ops::declaredKeyFor(c, device);
    sdk::SlotAddress address;

    if (target.kind == SelKind::Slot) {
        address = {target.owner, target.sub};
        bool found = false;
        for (const auto& spec : ops::slotSpecsFor(c, document_.catalog(), target.owner))
            if (spec.name == target.sub) {
                found = true;
                if (!slotAccepts(spec, caps))
                    return ETR("%1 cannot act as a %2.").arg(deviceLabel(device), slotTypeLabel(spec.type).toLower());
            }
        if (!found) return ETR("That slot no longer exists.");
    } else if (target.kind == SelKind::Mechanism || target.kind == SelKind::Wheel || target.kind == SelKind::Sensor) {
        const std::string owner = target.kind == SelKind::Wheel ? std::string(sdk::slot_owner::kDrivetrain) : target.owner;
        const std::string prefix = target.kind == SelKind::Wheel ? target.sub + "." : std::string();
        const auto best = bestSlotFor(c, document_.catalog(), owner, caps, prefix);
        bool bestIsMotor = false;
        if (best)
            for (const auto& spec : ops::slotSpecsFor(c, document_.catalog(), owner))
                if (spec.name == best->slot) bestIsMotor = spec.type == sdk::SlotType::Motor;
        // No free motor row: a motor controller becomes one more motor of the part (one undo step).
        if (!bestIsMotor)
            if (const auto group = growableMotorGroup(c, document_.catalog(), owner, caps, prefix)) {
                sdk::SlotAddress added;
                const bool done = document_.edit(ETR("Add motor to %1").arg(describeSelection(c, document_.catalog(), target)), [&](sdk::RobotConfig& cfg) {
                    if (!ops::setMotorCount(cfg, document_.catalog(), owner, group->countKey, group->count + 1)) return false;
                    for (const auto& g : ops::motorGroups(cfg, document_.catalog(), owner))
                        if (g.countKey == group->countKey && !g.slotNames.empty()) added = {owner, g.slotNames.back()};
                    return !added.slot.empty() && ops::setMapping(cfg, added, key);
                });
                if (!done) return ETR("Could not add a motor.");
                selection_.set(SelectionRef::slot(added.owner, added.slot));
                Q_EMIT statusMessage(ETR("Added %1 as motor %2 of %3.").arg(deviceLabel(device)).arg(group->count + 1).arg(describeSelection(document_.config(), document_.catalog(), target)), 0);
                return {};
            }
        if (!best) return ETR("No free slot that accepts %1 on %2.").arg(deviceLabel(device), describeSelection(c, document_.catalog(), target));
        address = *best;
    } else {
        return ETR("Drop a device on a mechanism, a sensor, a wheel or one of their slots.");
    }
    const auto axes = sdk::motionAxisInfos(
        c, [&](std::string_view type) { return document_.catalog().findTemplate(type); });
    if (sdk::motionFollowerMotor(c, axes, address))
        return ETR("This axis is driven by a motion link. Remove its link before mapping a motor.");
    if (!document_.mapSlot(address, key)) return ETR("Could not map the device.");
    selection_.set(SelectionRef::slot(address.owner, address.slot));
    Q_EMIT statusMessage(ETR("Mapped %1 to %2.").arg(deviceLabel(device), QString::fromStdString(ops::slotDisplayName(c, document_.catalog(), address))), 0);
    return {};
}

bool EditorController::setMotorReversed(const sdk::SlotAddress& slot, bool reversed) {
    const QString name = QString::fromStdString(ops::slotDisplayName(document_.config(), document_.catalog(), slot));
    return document_.edit(reversed ? ETR("Reverse %1").arg(name) : ETR("Un-reverse %1").arg(name),
                          [&](sdk::RobotConfig& c) { return ops::setMappingReversed(c, slot, reversed); });
}

bool EditorController::addMotor(const QString& owner, const QString& countKey) {
    const std::string o = owner.toStdString(), key = countKey.toStdString();
    for (const auto& g : ops::motorGroups(document_.config(), document_.catalog(), o))
        if (g.countKey == key)
            return document_.edit(ETR("Add motor"), [&](sdk::RobotConfig& c) { return ops::setMotorCount(c, document_.catalog(), o, key, g.count + 1); });
    return false;
}

bool EditorController::removeMotor(const QString& owner, const QString& countKey) {
    const std::string o = owner.toStdString(), key = countKey.toStdString();
    for (const auto& g : ops::motorGroups(document_.config(), document_.catalog(), o))
        if (g.countKey == key) {
            const SelectionRef before = selection_.current();
            const bool done = document_.edit(ETR("Remove motor"), [&](sdk::RobotConfig& c) { return ops::setMotorCount(c, document_.catalog(), o, key, g.count - 1); });
            // The selected motor row may be the one that just went away: fall back to its part.
            if (done && before.kind == SelKind::Slot && before.owner == o) {
                const auto specs = ops::slotSpecsFor(document_.config(), document_.catalog(), o);
                if (std::none_of(specs.begin(), specs.end(), [&](const sdk::SlotSpec& s) { return s.name == before.sub; }))
                    selection_.set(formTargetOf(document_.config(), before));
            }
            return done;
        }
    return false;
}

void EditorController::toggleDetection() {
    if (backend_.detecting()) backend_.stopDetection();
    else backend_.startDetection();
}

void EditorController::focusIssue(const sdk::Issue& issue) {
    const IssueLocation loc = locateIssue(document_.config(), document_.catalog(), issue);
    if (loc.selection.isNone()) {
        Q_EMIT statusMessage(ETR("This issue is not tied to something that can be selected."), 1);
        return;
    }
    selection_.set(loc.selection, QString::fromStdString(loc.fieldKey));
}

}  // namespace wpiemu::app::editor
