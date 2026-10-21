#pragma once

// The robot editor: project tree + mechanism palette + detected devices on the left, the Qt Quick 3D
// viewport and console / validation / telemetry tabs in the middle, the generated property inspector
// on the right. The page talks to the rest of the application ONLY through IEditorBackend.
//
//   FakeEditorBackend backend;                  // or the shell's real adapter
//   auto* page = new EditorPage(backend, parent);
//   menu->addAction(page->undoAction());        // Edit menu integration
//   connect(page, &EditorPage::autosaveRequested, ...);

#include <QWidget>

#include <memory>

#include "editor_backend.h"

class QAction;
class QUndoStack;
class QVBoxLayout;

namespace wpiemu::app::editor {

class EditorController;
class ViewportWidget;
class InspectorPanel;
class DevicePanel;
class BottomPanel;
class ProjectTreeView;
class ToastOverlay;

class EditorPage : public QWidget {
    Q_OBJECT
public:
    explicit EditorPage(IEditorBackend& backend, QWidget* parent = nullptr);
    ~EditorPage() override;

    // ---- shell integration ----
    QUndoStack* undoStack() const;
    QAction* undoAction() const;        // text follows the command ("Undo Add Intake")
    QAction* redoAction() const;
    QAction* saveAction() const;        // Ctrl+S: persists immediately
    QAction* detectAction() const;      // starts / stops "Detect devices"
    QAction* duplicateAction() const;   // Ctrl+D
    QAction* deleteAction() const;      // Del
    // Delay between the last edit and the autosave request (default 1500 ms; 0 disables).
    void setAutosaveDelayMs(int milliseconds);
    bool hasUnsavedChanges() const;

    // ---- internals exposed for tests and the demo ----
    EditorController& controller() const;
    ViewportWidget* viewport() const;
    InspectorPanel* inspector() const;
    DevicePanel* devicePanel() const;
    BottomPanel* bottomPanel() const;
    ProjectTreeView* tree() const;
    ToastOverlay* toasts() const;

Q_SIGNALS:
    // The model changed (every edit, undo, redo).
    void configEdited();
    // The autosave timer fired: the page already called IEditorBackend::saveConfig().
    void autosaveRequested();

private:
    // The constructor runs these in order: widgets first, then the connections between models and views.
    void createActions();
    void buildTopBar(QVBoxLayout* root);
    void buildPanes(QVBoxLayout* root);
    void connectTree();
    void connectTreeDrops();
    void connectController();
    void connectValidationButton();

    struct Impl;
    std::unique_ptr<Impl> d;
};

}  // namespace wpiemu::app::editor
