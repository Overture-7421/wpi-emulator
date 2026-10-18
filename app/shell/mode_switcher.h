#pragma once

#include <QTabBar>
#include <QWidget>

#include "common/ui_resources.h"
#include "shell/shell_controller.h"

namespace wpiemu::app {

// "Editor | Run" tabs that live in the menu bar row; same look as the other tab bars in the app.
class ModeSwitcher : public QWidget {
    Q_OBJECT
public:
    explicit ModeSwitcher(const UiResources& ui, QWidget* parent = nullptr);

    AppMode mode() const { return tabs_->currentIndex() == kRunTab ? AppMode::Run : AppMode::Editor; }
    void setMode(AppMode mode);   // silent: does not emit modeRequested
    void setRunEnabled(bool enabled);
    bool runEnabled() const { return tabs_->isTabEnabled(kRunTab); }
    void requestMode(AppMode mode) { tabs_->setCurrentIndex(mode == AppMode::Run ? kRunTab : kEditorTab); }

signals:
    void modeRequested(wpiemu::app::AppMode mode);

private:
    static constexpr int kEditorTab = 0;
    static constexpr int kRunTab = 1;
    void refreshIcons();

    const UiResources& ui_;
    QTabBar* tabs_;
};

}  // namespace wpiemu::app
