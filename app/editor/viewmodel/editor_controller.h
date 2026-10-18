#pragma once

// Application logic of the editor page (everything that is not drawing): keeps the document, the
// backend and the discovery state in sync, runs validation, and implements the user's actions
// (add / delete / duplicate / map a device / detect). Widgets call it; it never touches widgets.

#include <QObject>
#include <QString>
#include <QTimer>

#include <vector>

#include "editor_backend.h"
#include "model/device_directory.h"
#include "model/editor_document.h"
#include "model/slot_status.h"
#include "viewmodel/selection_state.h"

namespace wpiemu::app::editor {

class EditorController : public QObject {
    Q_OBJECT
public:
    explicit EditorController(IEditorBackend& backend, QObject* parent = nullptr);

    IEditorBackend& backend() { return backend_; }
    EditorDocument& document() { return document_; }
    SelectionState& selection() { return selection_; }
    EditorSettings& settings() { return settings_; }
    const DeviceDirectory& devices() const { return devices_; }
    // The cameras the robot process reports using, as the backend last published them (live during a discovery or run session).
    const std::vector<sdk::CameraRobotState>& detectedCameras() const { return detectedCameras_; }
    // Backend + local + runtime issues, errors first.
    const std::vector<sdk::Issue>& issues() const { return issues_; }
    bool detecting() const { return backend_.detecting(); }
    bool hasPendingChanges() const { return pendingSave_; }
    void setAutosaveDelayMs(int ms) { autosaveDelayMs_ = ms; }

    // ---- actions (all undoable unless noted) ----
    // Adds a mechanism; with a mechanism selected it is attached to that mechanism's `link`.
    QString addMechanism(const QString& type, const sdk::MechanismAttachment& attachment, const ops::MechanismVariant& variant = {});
    QString addMechanismToSelection(const QString& type, const ops::MechanismVariant& variant = {});
    // Adds a zone of `kind` (a ZoneKindInfo::kind) on the chassis or a mechanism link and selects it.
    QString addZone(const QString& kind, const sdk::MechanismAttachment& attachment = {});
    // Adds a PhotonVision / Limelight camera on the chassis or a mechanism link and selects it.
    QString addCamera(sdk::CameraVendor vendor, const sdk::MechanismAttachment& attachment = {});
    // Adds a camera for a name the robot code uses (see detectedCameras()) and selects it. Empty when the model already has it.
    QString addCameraFromDetected(const sdk::CameraRobotState& detected);
    bool deleteSelection();
    bool duplicateSelection();
    // Maps a dropped / chosen device: onto a slot, or onto the best free compatible slot of a
    // mechanism / wheel. Returns a human message on failure (empty on success).
    QString mapDevice(const SelectionRef& target, const sdk::DeviceKey& device, sdk::CapabilitySet caps);
    bool clearSlot(const sdk::SlotAddress& slot);
    // Reversed: the mapped controller spins against the mechanism's positive direction (undoable).
    bool setMotorReversed(const sdk::SlotAddress& slot, bool reversed);
    // Adds/removes the last motor row of a motor group (see ops::MotorGroup); `owner` is a mechanism id or "drivetrain".
    bool addMotor(const QString& owner, const QString& countKey);
    bool removeMotor(const QString& owner, const QString& countKey);
    // Motion links (ADR 0012): make `target` follow `source` (ratio in the units the UI shows) / take its own
    // motor again. setDrivenBy drops the target's own motor mappings in the same undo step and returns why the edit
    // was refused (empty on success).
    QString setDrivenBy(const sdk::SlotAddress& target, const sdk::SlotAddress& source, double displayRatio);
    bool clearDrivenBy(const sdk::SlotAddress& target);
    // Tank presets (ADR 0012).
    void toggleDetection();
    void focusIssue(const sdk::Issue& issue);
    sdk::Status flushSave();             // persist now (Ctrl+S), retain dirty state on failure

    bool selectionExists(const SelectionRef& ref) const;
    QString saveStatusText() const;

Q_SIGNALS:
    void issuesUpdated();
    void devicesUpdated();
    void detectedCamerasUpdated();
    void deviceToast(const QString& title, const QString& detail);
    void statusMessage(const QString& text, int severity);
    void logLine(wpiemu::sdk::Severity severity, const QString& text);
    void detectionChanged(bool running);
    void snapshotUpdated(std::shared_ptr<const wpiemu::sdk::ISimSnapshot> snapshot);
    void saveStateChanged();
    void autosaveRequested();
    void configEdited();

private:
    void reloadFromBackend();
    void scheduleValidation();
    void runValidation();
    void onDocumentModified();
    void ensureSelectionValid();
    sdk::Status persist();

    IEditorBackend& backend_;
    EditorDocument document_;
    SelectionState selection_;
    EditorSettings settings_;
    DeviceDirectory devices_;
    std::vector<sdk::CameraRobotState> detectedCameras_;
    std::vector<sdk::Issue> issues_;
    std::vector<sdk::Issue> pushedIssues_;
    QTimer validateTimer_;
    QTimer saveTimer_;
    int autosaveDelayMs_ = 1500;
    bool pendingSave_ = false;
    bool applyingBackend_ = false;
    QString lastSaved_;
    QString saveError_;
};

}  // namespace wpiemu::app::editor
