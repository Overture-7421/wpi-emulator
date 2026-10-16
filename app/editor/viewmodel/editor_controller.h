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
    // Backend + local + runtime issues, errors first.
    const std::vector<sdk::Issue>& issues() const { return issues_; }
    bool detecting() const { return backend_.detecting(); }
    bool hasPendingChanges() const { return pendingSave_; }
    void setAutosaveDelayMs(int ms) { autosaveDelayMs_ = ms; }

    // ---- actions (all undoable unless noted) ----
    // Adds a mechanism; with a mechanism selected it is attached to that mechanism's `link`.
    QString addMechanism(const QString& type, const sdk::MechanismAttachment& attachment);
    QString addMechanismToSelection(const QString& type);
    bool deleteSelection();
    bool duplicateSelection();
    // Maps a dropped / chosen device: onto a slot, or onto the best free compatible slot of a
    // mechanism / wheel. Returns a human message on failure (empty on success).
    QString mapDevice(const SelectionRef& target, const sdk::DeviceKey& device, sdk::CapabilitySet caps);
    bool clearSlot(const sdk::SlotAddress& slot);
    void toggleDetection();
    void focusIssue(const sdk::Issue& issue);
    void flushSave();                    // persist now (Ctrl+S)

    bool selectionExists(const SelectionRef& ref) const;
    QString saveStatusText() const;

Q_SIGNALS:
    void issuesUpdated();
    void devicesUpdated();
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
    void persist();

    IEditorBackend& backend_;
    EditorDocument document_;
    SelectionState selection_;
    EditorSettings settings_;
    DeviceDirectory devices_;
    std::vector<sdk::Issue> issues_;
    std::vector<sdk::Issue> pushedIssues_;
    QTimer validateTimer_;
    QTimer saveTimer_;
    int autosaveDelayMs_ = 1500;
    bool pendingSave_ = false;
    bool applyingBackend_ = false;
    QString lastSaved_;
};

}  // namespace wpiemu::app::editor
