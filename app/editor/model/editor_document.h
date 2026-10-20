#pragma once

// The editable robot model plus its undo history. Every user edit is a snapshot command pushed on the
// QUndoStack, built from the pure operations in config_ops.h / property_groups.h, so undo/redo is
// uniform and a new edit kind never needs its own command class. Views never mutate the config
// directly; they ask the document.

#include <QObject>
#include <QString>
#include <QUndoCommand>
#include <QUndoStack>

#include <functional>

#include "editor_backend.h"
#include "model/config_ops.h"
#include "model/selection.h"
#include "model/transform_access.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor {

class EditorDocument;

// before/after snapshot of the whole (small) config. Consecutive commands with the same merge key
// collapse into one undo step (typing in a spin box, dragging a gizmo).
class ConfigEditCommand final : public QUndoCommand {
public:
    ConfigEditCommand(EditorDocument& doc, const QString& text, sdk::RobotConfig before, sdk::RobotConfig after, QString mergeKey);
    void undo() override;
    void redo() override;
    int id() const override;
    bool mergeWith(const QUndoCommand* other) override;

private:
    EditorDocument& doc_;
    sdk::RobotConfig before_;
    sdk::RobotConfig after_;
    QString mergeKey_;
    qint64 stampMs_ = 0;
    bool firstRedo_ = true;
};

class EditorDocument : public QObject {
    Q_OBJECT
public:
    explicit EditorDocument(QObject* parent = nullptr);

    const sdk::RobotConfig& config() const { return config_; }
    const EditorCatalog& catalog() const { return catalog_; }
    QUndoStack* undoStack() { return &undo_; }

    // Replaces the model without touching the undo history of the NEW model (history is cleared).
    void load(sdk::RobotConfig config);
    void setCatalog(EditorCatalog catalog);

    // Generic edit: `mutator` changes a copy; false = nothing to do. `mergeKey` non-empty merges
    // consecutive edits with the same key ("drag:" keys always merge, others within ~1.5 s).
    bool edit(const QString& text, const std::function<bool(sdk::RobotConfig&)>& mutator, const QString& mergeKey = {});

    // ---- mechanisms
    QString addMechanism(const QString& type, const sdk::MechanismAttachment& attachment = {}, const ops::MechanismVariant& variant = {});
    bool removeMechanism(const QString& id);
    QString duplicateMechanism(const QString& id);
    bool reparentMechanism(const QString& id, const QString& newParent, const QString& link = QStringLiteral("root"));
    // ---- zones
    QString addZone(const QString& kind, const sdk::MechanismAttachment& attachment = {});
    bool removeZone(const QString& id);
    QString duplicateZone(const QString& id);
    bool reattachZone(const QString& id, const QString& newParent, const QString& link = QStringLiteral("root"));
    int addZoneCondition(const QString& zoneId);
    bool removeZoneCondition(const QString& zoneId, int index);
    bool setZoneCondition(const QString& zoneId, int index, const sdk::ZoneCondition& condition);
    bool setZoneActivationMode(const QString& zoneId, sdk::ActivationMode mode);
    // ---- cameras
    QString addCamera(sdk::CameraKind kind, const sdk::MechanismAttachment& attachment = {});
    bool removeCamera(const QString& id);
    QString duplicateCamera(const QString& id);
    bool reattachCamera(const QString& id, const QString& newParent, const QString& link = QStringLiteral("root"));
    // ---- sensors
    QString addSensor(sdk::SensorKind kind);
    bool removeSensor(const QString& id);
    QString duplicateSensor(const QString& id);
    QString addStartPosition();
    bool removeStartPosition(const QString& id);
    QString duplicateStartPosition(const QString& id);
    // New position for the other alliance (field symmetry); empty when the field has none.
    QString copyStartPositionToOtherAlliance(const QString& id);
    // ---- slots
    bool mapSlot(const sdk::SlotAddress& slot, const sdk::DeviceKey& device);
    bool clearSlot(const sdk::SlotAddress& slot);
    // ---- generic property / transform edits
    bool setProperty(const SelectionRef& sel, const std::string& groupId, const std::string& key, const sdk::Value& value);
    bool setTransform(const SelectionRef& sel, const EditableTransform& t, const QString& dragSession = {});
    // ---- chassis / geometry
    QString addWheel();
    bool removeWheel(const QString& name);
    int addPrimitive(const QString& owner, sdk::PrimitiveKind kind);
    bool removePrimitive(const QString& owner, int index);
    int duplicatePrimitive(const QString& owner, int index);

    void markSaved() { undo_.setClean(); }

Q_SIGNALS:
    void changed(bool authoritative);   // undo/redo/load override even focused, uncommitted controls
    void modified();   // user-originated change (edit, undo, redo): autosave + submit to the backend
    void catalogChanged();

private:
    friend class ConfigEditCommand;
    void applySnapshot(const sdk::RobotConfig& config, bool authoritative = true);

    sdk::RobotConfig config_;
    EditorCatalog catalog_;
    QUndoStack undo_;
};

}  // namespace wpiemu::app::editor
