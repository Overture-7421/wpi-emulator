#include "client/client_window.h"

#include <QAction>
#include <QApplication>
#include <QKeySequence>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTimer>
#include <QToolButton>

#include "wpiemu/net/module.h"
#include "wpiemu/sdk/version.h"

#ifndef WPIEMU_APP_VERSION
#define WPIEMU_APP_VERSION "0.0.0"
#endif

namespace wpiemu::app::client {

ClientWindow::ClientWindow(ClientWindowOptions options, QWidget* parent) : QMainWindow(parent) {
    setObjectName(QStringLiteral("client.window"));
    setWindowTitle(tr("WPI Emulator Client"));
    resize(1280, 800);

    auto services = ClientServices::create(options.clock);
    if (!services) qFatal("wpiemu_client: %s", services.error().message.c_str());
    services_ = std::move(services).value();

    if (options.settingsFile.isEmpty()) {
        settings_ = std::make_unique<QSettings>();
        store_ = std::make_unique<QSettingsKeyValueStore>();
    } else {
        settings_ = std::make_unique<QSettings>(options.settingsFile, QSettings::IniFormat);
        store_ = std::make_unique<QSettingsKeyValueStore>(options.settingsFile);
    }
    if (options.useGamepads) gamepads_ = runview::createDefaultJoystickProvider();
    joysticks_ = std::make_unique<runview::JoystickManager>(gamepads_.get(), settings_.get());
    input_ = std::make_unique<JoystickClientInput>(*joysticks_);
    if (options.session)
        controller_ = std::make_unique<ClientController>(*services_, *store_, input_.get(), std::move(options.session));
    else
        controller_ = std::make_unique<ClientController>(*services_, *store_, input_.get());

    stack_ = new QStackedWidget(this);
    join_ = new JoinPage(*controller_, stack_);
    drive_ = new DrivePage(*controller_, joysticks_.get(), stack_);
    stack_->addWidget(join_);
    stack_->addWidget(drive_);
    setCentralWidget(stack_);

    auto* file = menuBar()->addMenu(tr("&File"));
    auto* quit = file->addAction(tr("&Quit"));
    quit->setShortcut(QKeySequence::Quit);
    connect(quit, &QAction::triggered, this, &QWidget::close);
    auto* view = menuBar()->addMenu(tr("&View"));
    auto* stats = view->addAction(tr("Connection &stats"));
    stats->setCheckable(true);
    stats->setChecked(drive_->diagnosticsVisible());
    connect(stats, &QAction::toggled, drive_, &DrivePage::setDiagnosticsVisible);
    connect(drive_->diagnosticsButton(), &QToolButton::toggled, stats, &QAction::setChecked);
    auto* help = menuBar()->addMenu(tr("&Help"));
    auto* about = help->addAction(tr("&About"));
    connect(about, &QAction::triggered, this, [this] {
        QMessageBox::about(this, tr("About WPI Emulator Client"),
                           tr("<b>WPI Emulator Client</b> %1<br>Joins a practice match hosted from WPI Emulator. It describes your "
                              "robot to the host and drives it; the host runs the simulation.<br>SDK %2.%3, %4")
                               .arg(QStringLiteral(WPIEMU_APP_VERSION))
                               .arg(sdk::kSdkVersionMajor)
                               .arg(sdk::kSdkVersionMinor)
                               .arg(QString::fromLatin1(net::moduleName())));
    });
    statusBar()->showMessage(tr("Version %1").arg(QStringLiteral(WPIEMU_APP_VERSION)));

    connect(controller_.get(), &ClientController::presentationChanged, this, &ClientWindow::showScreen);
    timer_ = new QTimer(this);
    timer_->setInterval(options.tickIntervalMs);
    connect(timer_, &QTimer::timeout, this, [this] { controller_->tick(); });
    timer_->start();
    showScreen();
}

ClientWindow::~ClientWindow() {
    if (timer_ != nullptr) timer_->stop();
    delete takeCentralWidget();   // the pages refer to the controller and the joysticks, which are destroyed before the base class
    controller_->leave();
}

bool ClientWindow::showingDrive() const { return stack_->currentWidget() == drive_; }

void ClientWindow::showScreen() {
    const bool drive = controller_->presentation().screen == Screen::Drive;
    stack_->setCurrentWidget(drive ? static_cast<QWidget*>(drive_) : static_cast<QWidget*>(join_));
}

}  // namespace wpiemu::app::client
