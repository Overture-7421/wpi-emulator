#pragma once

#include <QAction>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QStackedWidget>
#include <QToolBar>

#include <functional>

#include "common/app_services.h"
#include "common/ui_resources.h"
#include "shell/mode_switcher.h"
#include "shell/page_factory.h"
#include "shell/shell_controller.h"
#include "shell/welcome_page.h"

namespace wpiemu::app {

class WizardDialog;

// The application window: menus, mode switch and the stacked pages
// (welcome / editor / run). Behaviour lives in ShellController; this class only wires widgets.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(AppServices& services, ShellController& controller, const UiResources& ui, ShellPages pages, QWidget* parent = nullptr);

    // Opens a project (folder or a file inside it) and walks through the wizard when it needs setup.
    void openProjectPath(const QString& path);
    void runSetupWizard(const QString& projectPath);
    void showSettings();
    void showAbout();

    // Replaceable dialogs: the defaults are the real modal dialogs; tests inject non-blocking ones.
    using WizardRunner = std::function<int(WizardDialog&)>;
    using FolderPicker = std::function<QString()>;
    using MessageRunner = std::function<void(const QString&)>;
    void setWizardRunner(WizardRunner runner) { wizardRunner_ = std::move(runner); }
    void setFolderPicker(FolderPicker picker) { folderPicker_ = std::move(picker); }
    void setMessageRunner(MessageRunner runner) { messageRunner_ = std::move(runner); }

    // Tells the user something once, in a modal dialog.
    void showMessage(const QString& text);

    // Introspection used by tests.
    enum class PageId { Welcome = 0, Editor = 1, Run = 2 };
    PageId currentPage() const { return static_cast<PageId>(stack_->currentIndex()); }
    WelcomePage* welcomePage() const { return welcome_; }
    ModeSwitcher* modeSwitcher() const { return modeSwitcher_; }
    QAction* openAction() const { return openAction_; }
    QAction* setupAction() const { return setupAction_; }
    QAction* saveAction() const { return saveAction_; }
    QAction* settingsAction() const { return settingsAction_; }
    QAction* closeAction() const { return closeAction_; }
    QMenu* recentMenu() const { return recentMenu_; }

protected:
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    void buildActions();
    void centerModeSwitcher();
    bool eventFilter(QObject* watched, QEvent* event) override;
    void buildMenus();
    void refresh();
    void refreshIcons();
    void rebuildRecentMenu();
    void browseForProject();

    AppServices& services_;
    ShellController& controller_;
    const UiResources& ui_;
    ShellPages pages_;
    WizardRunner wizardRunner_;
    FolderPicker folderPicker_;
    MessageRunner messageRunner_;
    bool busyOpening_ = false;

    QStackedWidget* stack_;
    WelcomePage* welcome_;
    ModeSwitcher* modeSwitcher_ = nullptr;

    QAction* openAction_ = nullptr;
    QAction* setupAction_ = nullptr;
    QAction* saveAction_ = nullptr;
    QAction* closeAction_ = nullptr;
    QAction* settingsAction_ = nullptr;
    QAction* quitAction_ = nullptr;
    QAction* editorAction_ = nullptr;
    QAction* runAction_ = nullptr;
    QAction* aboutAction_ = nullptr;
    QAction* aboutQtAction_ = nullptr;
    QMenu* recentMenu_ = nullptr;

};

}  // namespace wpiemu::app
