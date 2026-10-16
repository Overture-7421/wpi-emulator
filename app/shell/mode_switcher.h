#pragma once

#include <QButtonGroup>
#include <QToolButton>
#include <QWidget>

#include "common/ui_resources.h"
#include "shell/shell_controller.h"

namespace wpiemu::app {

// Segmented "Editor | Run" control.
class ModeSwitcher : public QWidget {
    Q_OBJECT
public:
    explicit ModeSwitcher(const UiResources& ui, QWidget* parent = nullptr);

    AppMode mode() const { return runButton_->isChecked() ? AppMode::Run : AppMode::Editor; }
    void setMode(AppMode mode);
    void setRunEnabled(bool enabled);
    QToolButton* editorButton() const { return editorButton_; }
    QToolButton* runButton() const { return runButton_; }

signals:
    void modeRequested(wpiemu::app::AppMode mode);

private:
    void refreshIcons();

    const UiResources& ui_;
    QToolButton* editorButton_;
    QToolButton* runButton_;
};

}  // namespace wpiemu::app
