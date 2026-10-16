#include <doctest/doctest.h>

#include <QFileInfo>

#include "gui_support.h"
#include "shell/about_dialog.h"
#include "shell/main_window.h"
#include "shell/page_factory.h"
#include "shell/settings_dialog.h"
#include "shell/shell_controller.h"
#include "wizard/robot_page.h"
#include "wizard/wizard_dialog.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

namespace {

// A window wired like main(): services, controller, pages, window.
struct ShellFixture {
    GuiEnvironment env;
    ShellController controller;
    MainWindow window;
    int wizardRuns = 0;

    explicit ShellFixture(bool healthy = true)
        : env(healthy), controller(*env.services), window(*env.services, controller, env.ui, createShellPages(*env.services, env.ui, &controller)) {}

    // Walks the wizard like a user who accepts the defaults, or cancels.
    void acceptWizard() {
        window.setWizardRunner([this](WizardDialog& dlg) {
            ++wizardRuns;
            waitFor([&] { return dlg.model().toolchainChecked(); });
            while (!dlg.model().isLastStep() && dlg.model().canGoNext()) dlg.nextButton()->click();
            dlg.nextButton()->click();
            return dlg.result();
        });
    }
    void cancelWizard() {
        window.setWizardRunner([this](WizardDialog& dlg) {
            ++wizardRuns;
            dlg.cancelButton()->click();
            return dlg.result();
        });
    }
};

}  // namespace

TEST_CASE("shell starts on the welcome page with project actions disabled") {
    ShellFixture f;
    f.window.show();
    CHECK(f.window.currentPage() == MainWindow::PageId::Welcome);
    CHECK(f.window.windowTitle() == QStringLiteral("WPI Emulator"));
    CHECK(!f.window.setupAction()->isEnabled());
    CHECK(!f.window.closeAction()->isEnabled());
    CHECK(!f.window.detectAction()->isEnabled());
    CHECK(!f.window.startStopAction()->isEnabled());
    CHECK(!f.window.modeSwitcher()->isEnabled());
    CHECK(f.window.openAction()->isEnabled());
    CHECK(f.window.welcomePage()->openButton()->isVisible());
    CHECK(!f.window.projectChip()->text().isEmpty());
}

TEST_CASE("opening an unconfigured project runs the wizard and lands in the editor") {
    ShellFixture f;
    f.window.show();
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);

    f.window.openProjectPath(project.path());
    CHECK(f.wizardRuns == 1);
    CHECK(f.controller.hasProject());
    CHECK(f.controller.canRun());
    CHECK(f.window.currentPage() == MainWindow::PageId::Editor);
    CHECK(f.window.windowTitle().contains(QFileInfo(project.path()).fileName()));
    CHECK(f.window.gameChip()->text().contains(QStringLiteral("2026")));
    CHECK(f.window.setupAction()->isEnabled());
    CHECK(f.window.closeAction()->isEnabled());
    CHECK(f.window.detectAction()->isEnabled());
    CHECK(f.window.modeSwitcher()->runButton()->isEnabled());
    CHECK(QFileInfo::exists(project.path() + "/.wpiemu/robot.json"));
    CHECK(f.env.services->recentProjects().lastProject() == project.path());
    // the "detect devices now" box of the summary was ticked: the robot is launched in discovery mode
    CHECK(f.env.demo->status().active());
    CHECK(f.env.demo->status().mode == RuntimeMode::Discovery);
}

TEST_CASE("an already configured project opens straight into the editor") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    f.controller.closeProject();
    CHECK(f.window.currentPage() == MainWindow::PageId::Welcome);

    f.wizardRuns = 0;
    f.window.openProjectPath(project.path());
    CHECK(f.wizardRuns == 0);   // no wizard the second time
    CHECK(f.window.currentPage() == MainWindow::PageId::Editor);

    // File > Set up robot re-enters the wizard on demand
    f.window.setupAction()->trigger();
    CHECK(f.wizardRuns == 1);
}

TEST_CASE("cancelling the wizard leaves no project half-open") {
    ShellFixture f;
    f.cancelWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    CHECK(f.wizardRuns == 1);
    CHECK(!f.controller.hasProject());
    CHECK(f.window.currentPage() == MainWindow::PageId::Welcome);
    CHECK(!f.env.services->configs().exists(project.path()));

    // an invalid folder also opens the wizard so that step 1 can explain what is wrong
    QTemporaryDir notAProject;
    f.window.openProjectPath(notAProject.path());
    CHECK(f.wizardRuns == 2);
    CHECK(f.window.currentPage() == MainWindow::PageId::Welcome);
}

TEST_CASE("mode switcher and shortcuts switch between editor and run") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    REQUIRE(f.window.currentPage() == MainWindow::PageId::Editor);

    f.window.modeSwitcher()->runButton()->click();
    CHECK(f.controller.mode() == AppMode::Run);
    CHECK(f.window.currentPage() == MainWindow::PageId::Run);
    CHECK(f.window.modeSwitcher()->mode() == AppMode::Run);

    f.window.modeSwitcher()->editorButton()->click();
    CHECK(f.window.currentPage() == MainWindow::PageId::Editor);

    f.window.close();   // closing the window stops a running robot
    CHECK(!f.env.demo->status().active());
}

TEST_CASE("start/stop action follows the robot state") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    f.controller.stopRobot();   // the wizard may have started detection

    CHECK(f.window.startStopAction()->text().contains(QStringLiteral("Start")));
    f.window.startStopAction()->trigger();
    CHECK(f.controller.mode() == AppMode::Run);
    CHECK(f.env.demo->status().active());
    CHECK(f.window.startStopAction()->text().contains(QStringLiteral("Stop")));
    CHECK(f.window.runtimeLabel()->text() == f.controller.runtimeLabel());
    f.window.startStopAction()->trigger();
    CHECK(!f.env.demo->status().active());
    CHECK(f.window.startStopAction()->text().contains(QStringLiteral("Start")));
}

TEST_CASE("recent projects menu lists opened projects and reopens them") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    f.controller.closeProject();
    REQUIRE(f.window.recentMenu()->actions().size() >= 1);
    CHECK(f.window.recentMenu()->actions().front()->text().contains(QFileInfo(project.path()).fileName()));
    f.window.recentMenu()->actions().front()->trigger();
    CHECK(f.controller.hasProject());
    CHECK(f.window.currentPage() == MainWindow::PageId::Editor);
}

TEST_CASE("open project dialog uses the folder picker") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.setFolderPicker([&] { return project.path(); });
    f.window.openAction()->trigger();
    CHECK(f.controller.hasProject());
    f.window.setFolderPicker([] { return QString(); });   // cancelled picker changes nothing
    f.window.openAction()->trigger();
    CHECK(f.controller.hasProject());
}

TEST_CASE("settings dialog edits a copy and applies on OK") {
    GuiEnvironment env;
    SettingsDialog dlg(env.services->settings(), env.services->probe(), env.ui);
    CHECK(dlg.themeCombo()->count() == 3);
    CHECK(dlg.rateCombo()->count() == 5);
    CHECK(dlg.buttons()->button(QDialogButtonBox::Ok)->isEnabled());

    dlg.themeCombo()->setCurrentIndex(dlg.themeCombo()->findData(static_cast<int>(ThemeMode::Dark)));
    dlg.rateCombo()->setCurrentIndex(dlg.rateCombo()->findData(500));
    CHECK(env.services->settings().data().theme == ThemeMode::System);   // not applied yet
    dlg.buttons()->button(QDialogButtonBox::Ok)->click();
    CHECK(env.services->settings().data().theme == ThemeMode::Dark);
    CHECK(env.services->settings().data().physicsRateHz == 500);
    CHECK(env.theme.isDark());   // the theme follows immediately

    dlg.themeCombo()->setCurrentIndex(dlg.themeCombo()->findData(static_cast<int>(ThemeMode::Light)));
    dlg.buttons()->button(QDialogButtonBox::Ok)->click();
    CHECK(!env.theme.isDark());
}

TEST_CASE("settings dialog refuses folders that do not exist") {
    GuiEnvironment env;
    SettingsDialog dlg(env.services->settings(), env.services->probe(), env.ui);
    dlg.jdkEdit()->setText(QStringLiteral("/no/such/jdk"));
    CHECK(!dlg.problems().isEmpty());
    CHECK(!dlg.buttons()->button(QDialogButtonBox::Ok)->isEnabled());
    dlg.jdkEdit()->clear();
    CHECK(dlg.problems().isEmpty());
    CHECK(dlg.buttons()->button(QDialogButtonBox::Ok)->isEnabled());
}

TEST_CASE("about dialog lists versions and game modules") {
    GuiEnvironment env;
    const QString details = aboutDetails(env.services->plugins());
    CHECK(details.contains(QStringLiteral("WPI Emulator")));
    CHECK(details.contains(QStringLiteral("SDK 1.")));
    CHECK(details.contains(QStringLiteral("Qt ")));
    CHECK(details.contains(QStringLiteral("game_2026")));
    AboutDialog dlg(env.services->plugins(), env.ui);
    CHECK(dlg.windowTitle().contains(QStringLiteral("WPI Emulator")));
}

TEST_CASE("icons render in every tone and fall back to empty for unknown names") {
    GuiEnvironment env;
    for (const char* name : {"folder-open", "check-circle", "alert-triangle", "x-circle", "circle", "minus-circle", "refresh", "play",
                             "stop", "settings", "info", "arrow-left", "arrow-right", "search", "edit-3d", "run", "robot",
                             "close-project", "check", "sun", "moon", "upload"}) {
        CAPTURE(name);
        for (const IconTone tone : {IconTone::Text, IconTone::Accent, IconTone::Error}) {
            const QIcon icon = env.ui.icon(QString::fromLatin1(name), tone);
            CHECK(!icon.isNull());
            CHECK(!icon.pixmap(24, 24).isNull());
        }
    }
    CHECK(env.ui.icon(QStringLiteral("does-not-exist")).isNull());
    CHECK(!env.ui.logo().isNull());
    CHECK(IconProvider::coloredSvg(QStringLiteral("check"), QColor("#123456")).contains(QStringLiteral("#123456")));
}
