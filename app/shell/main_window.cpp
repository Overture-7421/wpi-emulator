#include "shell/main_window.h"

#include <QApplication>
#include <QCloseEvent>
#include <QDragEnterEvent>
#include <QDir>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QMenuBar>
#include <QMimeData>
#include <QStatusBar>
#include <QUrl>

#include "common/ui_helpers.h"
#include "shell/about_dialog.h"
#include "shell/settings_dialog.h"
#include "wizard/project_drop_zone.h"
#include "wizard/wizard_dialog.h"

namespace wpiemu::app {

MainWindow::MainWindow(AppServices& services, ShellController& controller, const UiResources& ui, ShellPages pages, QWidget* parent)
    : QMainWindow(parent),
      services_(services),
      controller_(controller),
      ui_(ui),
      pages_(std::move(pages)),
      stack_(new QStackedWidget),
      welcome_(new WelcomePage(services.recentProjects(), ui)) {
    wizardRunner_ = [](WizardDialog& dialog) { return dialog.exec(); };
    folderPicker_ = [this] {
        return QFileDialog::getExistingDirectory(this, tr("Open WPILib robot project"), services_.recentProjects().lastProject());
    };

    setObjectName(QStringLiteral("MainWindow"));
    setAcceptDrops(true);
    setWindowIcon(ui_.logo());
    resize(1360, 860);
    setMinimumSize(900, 600);

    stack_->addWidget(welcome_);
    stack_->addWidget(pages_.editor);
    stack_->addWidget(pages_.run);
    setCentralWidget(stack_);

    buildActions();
    buildMenus();
    buildToolbar();
    buildStatusBar();

    connect(welcome_, &WelcomePage::openRequested, this, &MainWindow::browseForProject);
    connect(welcome_, &WelcomePage::projectChosen, this, &MainWindow::openProjectPath);
    connect(modeSwitcher_, &ModeSwitcher::modeRequested, &controller_, &ShellController::setMode);
    connect(&controller_, &ShellController::modeChanged, this, [this] { refresh(); });
    connect(&controller_, &ShellController::projectChanged, this, [this] { refresh(); });
    connect(&controller_, &ShellController::runtimeChanged, this, [this] { refresh(); });
    connect(&controller_, &ShellController::notice, this, &MainWindow::showMessage);
    connect(&controller_, &ShellController::errorOccurred, this, &MainWindow::showMessage);
    connect(&services_.recentProjects(), &RecentProjects::changed, this, &MainWindow::rebuildRecentMenu);
    connect(&services_.settings(), &AppSettings::changed, this, [this] { refresh(); });
    connect(&ui_, &UiResources::themeChanged, this, &MainWindow::refreshIcons);

    if (const QByteArray geometry = services_.store().value(QStringLiteral("window/geometry")).toByteArray(); !geometry.isEmpty())
        restoreGeometry(geometry);

    rebuildRecentMenu();
    refreshIcons();
    refresh();
}

void MainWindow::buildActions() {
    openAction_ = new QAction(tr("&Open project…"), this);
    openAction_->setShortcut(QKeySequence::Open);
    connect(openAction_, &QAction::triggered, this, &MainWindow::browseForProject);

    setupAction_ = new QAction(tr("Set up &robot…"), this);
    setupAction_->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+R")));
    setupAction_->setToolTip(tr("Run the setup wizard again for this project"));
    connect(setupAction_, &QAction::triggered, this, [this] { runSetupWizard(services_.session().path()); });

    saveAction_ = new QAction(tr("&Save"), this);
    saveAction_->setShortcut(QKeySequence::Save);
    connect(saveAction_, &QAction::triggered, this, [this] {
        if (pages_.saveEditor) pages_.saveEditor();
    });

    closeAction_ = new QAction(tr("&Close project"), this);
    closeAction_->setShortcut(QKeySequence::Close);
    connect(closeAction_, &QAction::triggered, &controller_, &ShellController::closeProject);

    settingsAction_ = new QAction(tr("&Settings…"), this);
    settingsAction_->setShortcut(QKeySequence::Preferences);
    settingsAction_->setMenuRole(QAction::PreferencesRole);
    connect(settingsAction_, &QAction::triggered, this, &MainWindow::showSettings);

    quitAction_ = new QAction(tr("&Quit"), this);
    quitAction_->setShortcut(QKeySequence::Quit);
    quitAction_->setMenuRole(QAction::QuitRole);
    connect(quitAction_, &QAction::triggered, this, &QWidget::close);

    editorAction_ = new QAction(tr("&Editor"), this);
    editorAction_->setShortcut(QKeySequence(QStringLiteral("Ctrl+1")));
    editorAction_->setCheckable(true);
    connect(editorAction_, &QAction::triggered, this, [this] { controller_.setMode(AppMode::Editor); });

    runAction_ = new QAction(tr("&Run"), this);
    runAction_->setShortcut(QKeySequence(QStringLiteral("Ctrl+2")));
    runAction_->setCheckable(true);
    connect(runAction_, &QAction::triggered, this, [this] { controller_.setMode(AppMode::Run); });

    detectAction_ = new QAction(tr("&Detect devices"), this);
    detectAction_->setShortcut(QKeySequence(QStringLiteral("Ctrl+D")));
    detectAction_->setToolTip(tr("Run the robot code once to find its motors and sensors"));
    connect(detectAction_, &QAction::triggered, &controller_, &ShellController::startDiscovery);

    startStopAction_ = new QAction(tr("&Start robot"), this);
    startStopAction_->setShortcut(QKeySequence(QStringLiteral("F5")));
    connect(startStopAction_, &QAction::triggered, this, &MainWindow::toggleRobot);

    aboutAction_ = new QAction(tr("&About WPI Emulator"), this);
    aboutAction_->setMenuRole(QAction::AboutRole);
    connect(aboutAction_, &QAction::triggered, this, &MainWindow::showAbout);
    aboutQtAction_ = new QAction(tr("About &Qt"), this);
    aboutQtAction_->setMenuRole(QAction::AboutQtRole);
    connect(aboutQtAction_, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::buildMenus() {
    QMenu* file = menuBar()->addMenu(tr("&File"));
    file->addAction(openAction_);
    recentMenu_ = file->addMenu(tr("Open &recent"));
    file->addAction(setupAction_);
    file->addSeparator();
    file->addAction(saveAction_);
    file->addAction(closeAction_);
    file->addSeparator();
    file->addAction(settingsAction_);
    file->addAction(quitAction_);

    QMenu* view = menuBar()->addMenu(tr("&View"));
    view->addAction(editorAction_);
    view->addAction(runAction_);

    QMenu* robot = menuBar()->addMenu(tr("&Robot"));
    robot->addAction(detectAction_);
    robot->addAction(startStopAction_);

    QMenu* help = menuBar()->addMenu(tr("&Help"));
    help->addAction(aboutAction_);
    help->addAction(aboutQtAction_);
}

void MainWindow::buildToolbar() {
    toolbar_ = addToolBar(tr("Main"));
    toolbar_->setMovable(false);
    toolbar_->setFloatable(false);
    toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toolbar_->setContextMenuPolicy(Qt::PreventContextMenu);
    toolbar_->addAction(openAction_);
    toolbar_->addAction(setupAction_);

    const auto spacer = [] {
        auto* s = new QWidget;
        s->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        return s;
    };
    toolbar_->addWidget(spacer());
    modeSwitcher_ = new ModeSwitcher(ui_);
    toolbar_->addWidget(modeSwitcher_);
    toolbar_->addWidget(spacer());
    toolbar_->addAction(detectAction_);
    toolbar_->addAction(startStopAction_);
    toolbar_->addSeparator();
    toolbar_->addAction(settingsAction_);
}

void MainWindow::buildStatusBar() {
    projectChip_ = new QLabel;
    projectChip_->setProperty("chip", QStringLiteral("true"));
    gameChip_ = new QLabel;
    gameChip_->setProperty("chip", QStringLiteral("true"));
    runtimeLabel_ = new QLabel;
    demoChip_ = new QLabel(tr("Demo mode"));
    demoChip_->setProperty("chip", QStringLiteral("warning"));
    demoChip_->setToolTip(tr("Simulated devices and motion; no robot code is launched."));
    demoChip_->setVisible(services_.demo());
    statusBar()->setSizeGripEnabled(false);
    statusBar()->addWidget(projectChip_);
    statusBar()->addWidget(gameChip_);
    statusBar()->addPermanentWidget(runtimeLabel_);
    statusBar()->addPermanentWidget(demoChip_);
}

void MainWindow::refreshIcons() {
    openAction_->setIcon(ui_.icon(QStringLiteral("folder-open")));
    setupAction_->setIcon(ui_.icon(QStringLiteral("robot")));
    closeAction_->setIcon(ui_.icon(QStringLiteral("close-project")));
    settingsAction_->setIcon(ui_.icon(QStringLiteral("settings")));
    detectAction_->setIcon(ui_.icon(QStringLiteral("search")));
    editorAction_->setIcon(ui_.icon(QStringLiteral("edit-3d")));
    runAction_->setIcon(ui_.icon(QStringLiteral("run")));
    aboutAction_->setIcon(ui_.icon(QStringLiteral("info")));
    refresh();
}

void MainWindow::rebuildRecentMenu() {
    recentMenu_->clear();
    for (const QString& path : services_.recentProjects().paths()) {
        QAction* a = recentMenu_->addAction(QStringLiteral("%1  —  %2").arg(QFileInfo(path).fileName(), QDir::toNativeSeparators(path)));
        connect(a, &QAction::triggered, this, [this, path] { openProjectPath(path); });
    }
    if (recentMenu_->isEmpty()) recentMenu_->addAction(tr("No recent projects"))->setEnabled(false);
    else {
        recentMenu_->addSeparator();
        connect(recentMenu_->addAction(tr("Clear list")), &QAction::triggered, &services_.recentProjects(), &RecentProjects::clear);
    }
}

void MainWindow::refresh() {
    const bool ready = controller_.hasProject() && controller_.canRun();
    const bool inRun = controller_.mode() == AppMode::Run && ready;
    if (!ready) stack_->setCurrentIndex(static_cast<int>(PageId::Welcome));
    else stack_->setCurrentIndex(static_cast<int>(inRun ? PageId::Run : PageId::Editor));

    modeSwitcher_->setEnabled(ready);
    modeSwitcher_->setRunEnabled(ready);
    modeSwitcher_->setMode(inRun ? AppMode::Run : AppMode::Editor);
    editorAction_->setChecked(!inRun);
    runAction_->setChecked(inRun);
    editorAction_->setEnabled(ready);
    runAction_->setEnabled(ready);

    setupAction_->setEnabled(controller_.hasProject());
    closeAction_->setEnabled(controller_.hasProject());
    saveAction_->setEnabled(ready && !inRun && static_cast<bool>(pages_.saveEditor));
    detectAction_->setEnabled(ready);

    const HostStatus rs = services_.runtime().status();
    const bool active = rs.active();
    startStopAction_->setEnabled(ready);
    startStopAction_->setText(active ? tr("&Stop robot") : tr("&Start robot"));
    startStopAction_->setIcon(ui_.icon(active ? QStringLiteral("stop") : QStringLiteral("play")));
    startStopAction_->setShortcut(QKeySequence(active ? QStringLiteral("Shift+F5") : QStringLiteral("F5")));

    setWindowTitle(controller_.windowTitle());
    projectChip_->setText(controller_.hasProject() ? controller_.projectName() : tr("No project"));
    gameChip_->setVisible(controller_.hasProject());
    gameChip_->setText(controller_.gameLabel());
    gameChip_->setProperty("chip", controller_.gameModuleAvailable() ? QStringLiteral("success") : QStringLiteral("warning"));
    gameChip_->setToolTip(controller_.gameModuleAvailable() ? tr("Game module for this season is active")
                                                            : tr("No game module for this season: running with core features only"));
    repolish(gameChip_);
    runtimeLabel_->setText(controller_.runtimeLabel());
}

void MainWindow::showMessage(const QString& text) {
    lastMessage_ = text;
    statusBar()->showMessage(text, 10000);
}

void MainWindow::browseForProject() {
    const QString dir = folderPicker_();
    if (!dir.isEmpty()) openProjectPath(dir);
}

void MainWindow::toggleRobot() {
    if (services_.runtime().status().active()) controller_.stopRobot();
    else controller_.startRobot();
}

void MainWindow::openProjectPath(const QString& path) {
    if (busyOpening_ || path.isEmpty()) return;
    busyOpening_ = true;
    const OpenOutcome outcome = controller_.openProject(path);
    if (outcome != OpenOutcome::Opened) runSetupWizard(outcome == OpenOutcome::Invalid ? path : services_.session().path());
    busyOpening_ = false;
    refresh();
}

void MainWindow::runSetupWizard(const QString& projectPath) {
    WizardDialog dialog(services_, ui_, this);
    if (!projectPath.isEmpty()) dialog.setProjectPath(projectPath);
    const int code = wizardRunner_(dialog);
    if (code == QDialog::Accepted) {
        if (!dialog.wizardResult().message.isEmpty()) showMessage(dialog.wizardResult().message);
        controller_.setupCompleted(dialog.wizardResult().detectDevices);
    } else if (services_.session().needsSetup()) {
        controller_.closeProject();   // cancelled before a model existed: back to the welcome page
    }
    refresh();
}

void MainWindow::showSettings() {
    SettingsDialog dialog(services_.settings(), services_.probe(), ui_, this);
    dialog.exec();
}

void MainWindow::showAbout() {
    AboutDialog dialog(services_.plugins(), ui_, this);
    dialog.exec();
}

void MainWindow::closeEvent(QCloseEvent* event) {
    services_.store().setValue(QStringLiteral("window/geometry"), saveGeometry());
    services_.store().sync();
    services_.runtime().stop();
    event->accept();
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event) {
    if (!ProjectDropZone::pathFromMimeData(event->mimeData()).isEmpty()) event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent* event) {
    const QString path = ProjectDropZone::pathFromMimeData(event->mimeData());
    if (path.isEmpty()) return;
    event->acceptProposedAction();
    openProjectPath(path);
}

}  // namespace wpiemu::app
