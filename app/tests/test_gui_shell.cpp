#include <doctest/doctest.h>
#include <QApplication>
#include <QMenuBar>
#include <cmath>
#include <QToolBar>

#include <QFileInfo>
#include <QSignalSpy>
#include <QTest>

#include "gui_support.h"
#include "shell/about_dialog.h"
#include "shell/main_window.h"
#include "shell/page_factory.h"
#include "shell/settings_dialog.h"
#include "shell/shell_controller.h"
#include "wizard/robot_page.h"
#include "wizard/wizard_dialog.h"

#ifdef WPIEMU_APP_HAS_EDITOR
#include "editor/editor_page.h"
#include "editor/viewmodel/editor_controller.h"
#include "editor/viewmodel/detected_devices_model.h"
#include "shell/editor_backend_adapter.h"
#endif
#ifdef WPIEMU_APP_HAS_RUNVIEW
#include "runview/run_page.h"
#endif

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
        : env(healthy), controller(*env.services), window(*env.services, controller, env.ui, createShellPages(*env.services, env.ui, &controller)) {
        window.setMessageRunner([](const QString&) {});
    }

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
    CHECK(!f.window.modeSwitcher()->isEnabled());
    QApplication::processEvents();
    auto* sw = f.window.modeSwitcher();
    CHECK(sw->isVisible());
    CHECK(sw->parentWidget() == f.window.menuBar());
    CHECK(std::abs(sw->geometry().center().x() - f.window.menuBar()->width() / 2) <= 2);
    CHECK(f.window.openAction()->isEnabled());
    CHECK(f.window.welcomePage()->openButton()->isVisible());
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
    CHECK(f.window.windowTitle().contains(QStringLiteral("2026")));
    CHECK(f.window.setupAction()->isEnabled());
    CHECK(f.window.closeAction()->isEnabled());
    CHECK(f.window.modeSwitcher()->runEnabled());
    CHECK(QFileInfo::exists(project.path() + "/.wpiemu/robot.json"));
    CHECK(f.env.services->recentProjects().lastProject() == project.path());
    // the robot launches as soon as the editor shows, armed and detecting devices
    CHECK(f.env.demo->status().active());
    CHECK_FALSE(f.env.demo->status().simulating);
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

#ifdef WPIEMU_APP_HAS_RUNVIEW
TEST_CASE("shell: the run page shows the demo's camera and the View menu lists the camera toggles") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.show();
    f.window.openProjectPath(project.path());
    f.controller.setMode(AppMode::Run);
    auto* run = f.window.findChild<wpiemu::app::runview::RunPage*>();
    REQUIRE(run != nullptr);
    REQUIRE(waitFor([&] {
        run->pollNow();
        return run->cameraHud().rowCount() == 1 &&
               run->cameraHud().card(0).state == wpiemu::app::runview::CameraState::Ok;
    }));
    CHECK(run->cameraHud().card(0).label == QStringLiteral("Front camera"));
    CHECK(run->scene().cameraModel().rowCount() == 1);
    CHECK(run->scene().tagModel().rowCount() > 0);   // the game's AprilTags

    QStringList viewMenu;
    for (QAction* top : f.window.menuBar()->actions())
        if (top->menu() != nullptr && top->text().contains(QStringLiteral("View")))
            for (QAction* a : top->menu()->actions()) viewMenu << a->text();
    CHECK(viewMenu.contains(QStringLiteral("Cameras")));
    CHECK(viewMenu.contains(QStringLiteral("AprilTags")));
}

TEST_CASE("shell: Ctrl Shift R restarts from Run without opening setup") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.show();
    f.window.openProjectPath(project.path());
    f.controller.setMode(AppMode::Run);
    auto* run = f.window.findChild<wpiemu::app::runview::RunPage*>();
    REQUIRE(run != nullptr);
    f.window.activateWindow();
    run->setFocus();
    QApplication::processEvents();
    const auto generation = f.env.demo->status().runGeneration;
    const auto wizardRuns = f.wizardRuns;
    CHECK(f.window.setupAction()->shortcut().isEmpty());
    QTest::keyClick(run, Qt::Key_R, Qt::ControlModifier | Qt::ShiftModifier);
    QApplication::processEvents();
    CHECK(f.env.demo->status().runGeneration == generation + 1);
    CHECK(f.wizardRuns == wizardRuns);
    f.controller.setMode(AppMode::Editor);
    QApplication::processEvents();
    const auto afterRestart = f.env.demo->status().runGeneration;
    QTest::keyClick(&f.window, Qt::Key_R, Qt::ControlModifier | Qt::ShiftModifier);
    CHECK(f.env.demo->status().runGeneration == afterRestart);
}
#endif

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

    f.window.modeSwitcher()->requestMode(AppMode::Run);
    CHECK(f.controller.mode() == AppMode::Run);
    CHECK(f.window.currentPage() == MainWindow::PageId::Run);
    CHECK(f.window.modeSwitcher()->mode() == AppMode::Run);

    f.window.modeSwitcher()->requestMode(AppMode::Editor);
    CHECK(f.window.currentPage() == MainWindow::PageId::Editor);

    f.window.close();   // closing the window stops a running robot
    CHECK(!f.env.demo->status().active());
}

TEST_CASE("the app has no toolbar; project and simulation actions live in the menus") {
    ShellFixture f;
    f.window.show();
    CHECK(f.window.findChildren<QToolBar*>().isEmpty());
    QList<QAction*> menuActions;
    for (QAction* top : f.window.menuBar()->actions())
        if (top->menu() != nullptr) menuActions += top->menu()->actions();
    CHECK(menuActions.contains(f.window.openAction()));
    CHECK(menuActions.contains(f.window.setupAction()));
    CHECK(menuActions.size() > 8);   // the View menu also lists the run page's overlay toggles
}

TEST_CASE("the simulation starts by itself when the run tab opens") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());

    f.window.modeSwitcher()->requestMode(AppMode::Run);
    CHECK(f.controller.mode() == AppMode::Run);
    REQUIRE(waitFor([&] { return f.env.demo->status().simulating; }));
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

#ifdef WPIEMU_APP_HAS_EDITOR
namespace {

editor::EditorPage& editorPage(ShellFixture& f) {
    auto* page = f.window.findChild<editor::EditorPage*>();
    REQUIRE(page != nullptr);
    page->setAutosaveDelayMs(0);
    return *page;
}

void renameRobot(editor::EditorPage& page, const std::string& name) {
    REQUIRE(page.controller().document().edit(QStringLiteral("Rename robot"), [&](sdk::RobotConfig& config) {
        config.name = name;
        return true;
    }));
    REQUIRE(page.hasUnsavedChanges());
}

// A directory at the authoritative file path fails even when tests run as an administrator.
struct BlockAppDataSave {
    QString path;
    explicit BlockAppDataSave(const QString& file) : path(file) {
        REQUIRE(QFile::rename(path, path + ".held"));
        REQUIRE(QDir().mkpath(path));
    }
    void restore() {
        if (path.isEmpty()) return;
        REQUIRE(QDir().rmdir(path));
        REQUIRE(QFile::rename(path + ".held", path));
        path.clear();
    }
    ~BlockAppDataSave() {
        if (!path.isEmpty()) {
            QDir().rmdir(path);
            QFile::rename(path + ".held", path);
        }
    }
};

class InventoryRuntime final : public IRuntimeHost {
public:
    HostStatus status() const override { return status_; }
    HostScene scene() const override { return scene_; }
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override { return snapshot_; }
    void setProject(const QString&, std::shared_ptr<const sdk::RobotConfig> config, const sdk::IGameModule*) override {
        scene_.config = std::move(config);
    }
    void launch() override { ++status_.runGeneration; emit statusChanged(); }
    void startSimulation() override {}
    void stopSimulation() override {}
    void stop() override {}
    void restart() override { launch(); }
    void resetSimulation() override {}
    void setPaused(bool) override {}
    void stepTicks(int) override {}
    void setTimeScale(double) override {}
    void setDriverStation(const sdk::proto::DsState&) override {}
    void setRobotPose(const sdk::Pose3&) override {}
    void publish(std::vector<sdk::DiscoveredDevice> devices) {
        sdk::SimSnapshotData data;
        data.devices = std::move(devices);
        snapshot_ = std::make_shared<const sdk::SimSnapshot>(std::move(data));
        emit snapshotPublished();
    }

private:
    HostStatus status_;
    HostScene scene_;
    std::shared_ptr<const sdk::ISimSnapshot> snapshot_;
};

}  // namespace

TEST_CASE("shell composition: immediate close open and setup flush the old editor document") {
    ShellFixture f;
    f.acceptWizard();
    TestProject first;
    TestProject second;
    f.env.addProject(first);
    f.env.addProject(second);
    f.window.openProjectPath(first.path());
    auto& page = editorPage(f);
    renameRobot(page, "Pending old project");

    SUBCASE("window close") { CHECK(f.window.close()); }
    SUBCASE("project close") { f.controller.closeProject(); }
    SUBCASE("project open") {
        f.window.openProjectPath(second.path());
        CHECK(f.env.services->session().path() == second.path());
    }
    SUBCASE("setup replacement") { f.window.setupAction()->trigger(); }

    const auto stored = f.env.services->configs().load(first.path());
    REQUIRE(stored.ok());
    CHECK(stored.value().config.name == "Pending old project");
    CHECK_FALSE(page.hasUnsavedChanges());
    REQUIRE(f.env.services->session().open(first.path()).ok());
    CHECK(f.env.services->session().config()->name == "Pending old project");
}

TEST_CASE("shell composition: authoritative save failure blocks every destructive transition and can retry") {
    ShellFixture f;
    f.acceptWizard();
    TestProject first;
    TestProject second;
    f.env.addProject(first);
    f.env.addProject(second);
    f.window.openProjectPath(first.path());
    auto& page = editorPage(f);
    renameRobot(page, "Keep these edits");
    const auto previous = f.env.services->session().config();
    BlockAppDataSave blocked(f.env.services->configs().appDataPath(first.path()));
    const int wizardRuns = f.wizardRuns;

    SUBCASE("window close") { CHECK_FALSE(f.window.close()); }
    SUBCASE("project close") { f.window.closeAction()->trigger(); }
    SUBCASE("project open") { f.window.openProjectPath(second.path()); }
    SUBCASE("setup replacement") { f.window.setupAction()->trigger(); }
    SUBCASE("direct session open") { CHECK_FALSE(f.env.services->session().open(second.path()).ok()); }
    SUBCASE("direct session close") { CHECK_FALSE(f.env.services->session().close().ok()); }
    SUBCASE("wizard finish") {
        WizardViewModel model(*f.env.services);
        model.setProjectPath(second.path());
        const auto result = model.finish();
        CHECK_FALSE(result.ok);
        CHECK(result.message.contains("retry Save"));
    }

    CHECK(f.env.services->session().path() == first.path());
    CHECK(f.env.services->session().config() == previous);
    CHECK(f.env.demo->status().active());
    CHECK(f.wizardRuns == wizardRuns);
    CHECK(page.hasUnsavedChanges());
    CHECK(page.controller().saveStatusText().contains("Save failed"));
    blocked.restore();
    f.window.saveAction()->trigger();
    CHECK_FALSE(page.hasUnsavedChanges());
    CHECK(f.env.services->session().config()->name == "Keep these edits");
}

TEST_CASE("shell composition: shell and editor Save share success state and export warnings stay nonfatal") {
    ShellFixture f;
    f.acceptWizard();
    TestProject project;
    f.env.addProject(project);
    f.window.openProjectPath(project.path());
    auto& page = editorPage(f);
    QSignalSpy saved(&page.controller(), &editor::EditorController::autosaveRequested);
    renameRobot(page, "Shell Save");
    f.window.saveAction()->trigger();
    CHECK_FALSE(page.hasUnsavedChanges());
    CHECK(saved.count() == 1);
    renameRobot(page, "Editor Save");
    page.saveAction()->trigger();
    CHECK_FALSE(page.hasUnsavedChanges());
    CHECK(saved.count() == 2);

    const QString exported = f.env.services->configs().exportPath(project.path());
    REQUIRE(QFile::remove(exported));
    REQUIRE(QDir().rmdir(QFileInfo(exported).absolutePath()));
    writeFile(QFileInfo(exported).absolutePath(), "block export only");
    bool warned = false;
    QObject::connect(&page.controller(), &editor::EditorController::logLine, &page,
                     [&](sdk::Severity severity, const QString&) { warned |= severity == sdk::Severity::Warning; });
    renameRobot(page, "Authoritative success");
    page.saveAction()->trigger();
    CHECK(warned);
    CHECK_FALSE(page.hasUnsavedChanges());
    CHECK(page.controller().saveStatusText().contains("Saved "));
    const auto stored = f.env.services->configs().load(project.path());
    REQUIRE(stored.ok());
    CHECK(stored.value().config.name == "Authoritative success");
}

TEST_CASE("editor adapter: complete inventories and new runs reconcile physical liveness without losing mappings") {
    TestProject project;
    AppServicesOptions options;
    options.store = std::make_unique<MemoryKeyValueStore>();
    options.appDataRoot = std::filesystem::path((project.path() + "/appdata").toStdU16String());
    auto runtime = std::make_unique<InventoryRuntime>();
    auto& host = *runtime;
    options.runtime = std::move(runtime);
    AppServices services(std::move(options));
    REQUIRE(services.session().open(project.path()).ok());
    auto config = buildRobotConfig(RobotBasics{}, services.buildContext(services.session().inspection()));
    auto devices = DemoRuntimeHost::demoDevicesFor(config);
    REQUIRE(devices.size() >= 3);
    REQUIRE_FALSE(config.drivetrain.spec.wheels.empty());
    config.drivetrain.mappings.push_back(
        {{std::string(sdk::slot_owner::kDrivetrain), config.drivetrain.spec.wheels.front().name + ".drive"},
         devices[0].descriptor.key()});
    REQUIRE(services.session().commit(config, ProjectPrefs{}).ok());
    EditorBackendAdapter backend(services);
    editor::EditorController controller(backend);
    controller.setAutosaveDelayMs(0);
    editor::DetectedDevicesModel model(controller.devices(), controller.document());
    QObject::connect(&controller, &editor::EditorController::devicesUpdated, &model, [&] { model.refresh(); });
    auto x = devices[0];
    auto y = devices[1];
    auto z = devices[2];
    z.id = x.id;   // numeric runtime ids are not physical identity
    const auto mappings = controller.document().config().allMappings();
    REQUIRE_FALSE(mappings.empty());
    host.launch();
    host.publish({x, y});
    REQUIRE(controller.devices().find(x.descriptor.key()) != nullptr);
    CHECK(controller.devices().find(x.descriptor.key())->live);
    host.publish({y});   // omission in a complete same-run snapshot means gone
    CHECK_FALSE(controller.devices().find(x.descriptor.key())->live);
    CHECK(controller.devices().find(y.descriptor.key())->live);
    host.restart();
    CHECK_FALSE(controller.devices().find(y.descriptor.key())->live);
    host.publish({y, z});
    CHECK(backend.devices().size() == 3);
    CHECK(controller.devices().all().size() == 3);
    CHECK_FALSE(controller.devices().find(x.descriptor.key())->live);
    CHECK(controller.devices().find(z.descriptor.key())->live);
    bool gone = false;
    for (int row = 0; row < model.rowCount(); ++row)
        gone |= model.data(model.index(row, editor::DetectedDevicesModel::ColStatus)).toString() == "gone";
    CHECK(gone);
    const auto after = controller.document().config().allMappings();
    REQUIRE(after.size() == mappings.size());
    for (std::size_t i = 0; i < mappings.size(); ++i) {
        CHECK(after[i].device == mappings[i].device);
        CHECK(after[i].slot == mappings[i].slot);
    }
}
#endif

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
