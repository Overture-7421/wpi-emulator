#include "client/drive_page.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <algorithm>
#include <QEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

#include "client/client_text.h"
#include "ui/qt_text.h"

namespace wpiemu::app::client {
namespace {

bool isModifierKey(int key) {
    return key == Qt::Key_Shift || key == Qt::Key_Control || key == Qt::Key_Alt || key == Qt::Key_Meta || key == Qt::Key_AltGr;
}

bool typingInto(const QWidget* w) {
    return w != nullptr && (qobject_cast<const QLineEdit*>(w) != nullptr || w->inherits("QAbstractSpinBox"));
}

}  // namespace

DrivePage::DrivePage(ClientController& controller, runview::JoystickManager* joysticks, QWidget* parent)
    : QWidget(parent), controller_(controller), joysticks_(joysticks) {
    build();
}

DrivePage::~DrivePage() {
    if (qApp != nullptr) qApp->removeEventFilter(this);
    // The QML context points at members that are destroyed before the child widgets: drop the view first.
    delete viewport_;
    viewport_ = nullptr;
}

void DrivePage::build() {
    setObjectName(QStringLiteral("client.drivePage"));
    theme_.setPalette(palette());
    clock_.start();

    viewport_ = new runview::RunViewport(scene_, hud_, theme_, options_, this);
    viewport_->setMinimumSize(480, 320);
    banner_ = new ClientBanner(viewport_);
    diagnostics_ = new DiagnosticsOverlay(viewport_);
    zones_ = new ZonePanel(controller_, this);

    // Camera: field overview, the driver's point of view (following the robot), or follow on/off.
    cameraMenu_ = new QMenu(tr("Camera view"), this);
    auto* group = new QActionGroup(cameraMenu_);
    homeCamera_ = cameraMenu_->addAction(tr("Field overview"));
    homeCamera_->setObjectName(QStringLiteral("client.camera.home"));
    homeCamera_->setCheckable(true);
    homeCamera_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
    homeCamera_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    driverCamera_ = cameraMenu_->addAction(tr("Driver view"));
    driverCamera_->setObjectName(QStringLiteral("client.camera.driver"));
    driverCamera_->setCheckable(true);
    driverCamera_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_9));
    driverCamera_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    group->addAction(homeCamera_);
    group->addAction(driverCamera_);
    cameraMenu_->addSeparator();
    followCamera_ = cameraMenu_->addAction(tr("Follow my robot"));
    followCamera_->setObjectName(QStringLiteral("client.camera.follow"));
    followCamera_->setCheckable(true);
    addAction(homeCamera_);
    addAction(driverCamera_);
    viewport_->addCameraControl(cameraMenu_);
    connect(homeCamera_, &QAction::triggered, &scene_, &runview::SceneModel::resetCamera);
    connect(viewport_, &runview::RunViewport::homeRequested, &scene_, &runview::SceneModel::resetCamera);
    connect(driverCamera_, &QAction::triggered, &scene_, &runview::SceneModel::showDriverView);
    connect(followCamera_, &QAction::toggled, &scene_, &runview::SceneModel::setFollowRobot);
    const auto syncCamera = [this] {
        const bool driver = scene_.cameraView().kind == runview::CameraView::Kind::DriverPov;
        const QSignalBlocker a(homeCamera_), b(driverCamera_), c(followCamera_);
        homeCamera_->setChecked(!driver);
        driverCamera_->setChecked(driver);
        followCamera_->setChecked(scene_.followRobot());
    };
    connect(&scene_, &runview::SceneModel::cameraViewChanged, this, syncCamera);
    connect(&scene_, &runview::SceneModel::followRobotChanged, this, syncCamera);
    syncCamera();

    who_ = new QLabel(this);
    who_->setObjectName(QStringLiteral("client.who"));
    QFont f = who_->font();
    f.setBold(true);
    who_->setFont(f);
    state_ = new QLabel(this);
    state_->setObjectName(QStringLiteral("client.stateChip"));
    state_->setContentsMargins(12, 0, 0, 0);
    diagnosticsButton_ = new QToolButton(this);
    diagnosticsButton_->setObjectName(QStringLiteral("client.diagnosticsButton"));
    diagnosticsButton_->setText(tr("Connection stats"));
    diagnosticsButton_->setCheckable(true);
    diagnosticsButton_->setShortcut(QKeySequence(Qt::Key_F3));
    diagnosticsButton_->setToolTip(tr("Ping, packet loss, prediction error, snapshot age and buffer delay (F3)"));
    leave_ = new QPushButton(tr("Leave match"), this);
    leave_->setObjectName(QStringLiteral("client.leave"));

    auto* top = new QHBoxLayout;
    top->setContentsMargins(10, 6, 10, 6);
    top->addWidget(who_);
    top->addWidget(state_);
    top->addStretch(1);
    top->addWidget(diagnosticsButton_);
    top->addWidget(leave_);
    auto* middle = new QHBoxLayout;
    middle->setContentsMargins(0, 0, 0, 0);
    middle->setSpacing(0);
    middle->addWidget(viewport_, 1);
    middle->addWidget(zones_);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addLayout(top);
    layout->addLayout(middle, 1);

    viewport_->installEventFilter(this);
    connect(diagnosticsButton_, &QToolButton::toggled, this, [this](bool on) {
        diagnostics_->setVisible(on);
        if (on) {
            diagnostics_->setView(controller_.view().get());
            diagnostics_->raise();
            placeOverlays();
        }
        ClientPrefs p = controller_.prefs();
        p.showDiagnostics = on;
        controller_.setPrefs(p);
    });
    connect(leave_, &QPushButton::clicked, &controller_, &ClientController::leave);
    connect(banner_, &ClientBanner::leaveRequested, &controller_, &ClientController::leave);
    connect(banner_, &ClientBanner::reconnectRequested, &controller_, &ClientController::reconnect);
    connect(&controller_, &ClientController::sceneChanged, this, &DrivePage::refreshScene);
    connect(&controller_, &ClientController::viewChanged, this, &DrivePage::refreshView);
    connect(&controller_, &ClientController::presentationChanged, this, &DrivePage::refreshPresentation);

    diagnosticsButton_->setChecked(controller_.prefs().showDiagnostics);
    qApp->installEventFilter(this);
    refreshScene();
    refreshPresentation();
}

void DrivePage::refreshScene() {
    if (controller_.sceneRevision() != sceneRevision_) {
        sceneRevision_ = controller_.sceneRevision();
        const HostScene& info = controller_.scene();
        scene_.setSceneInfo(info);
        hud_.setDescriptors(info.hud);
        zones_->rebuild();
    }
    if (controller_.rosterRevision() != rosterRevision_) {
        rosterRevision_ = controller_.rosterRevision();
        scene_.setMatchRoster(controller_.roster());
    }
    if (const auto view = controller_.view(); view && view->hasOwn && !cameraPlaced_) {
        cameraPlaced_ = true;
        scene_.setDriverStation(sdk::proto::allianceOf(view->own.station), sdk::proto::stationNumberOf(view->own.station));
        scene_.showDriverView();
        scene_.setFollowRobot(true);
    } else if (!view) {
        cameraPlaced_ = false;
    }
}

void DrivePage::refreshView() {
    const auto snapshot = controller_.snapshot();
    const auto view = controller_.view();
    const qint64 nowNs = clock_.nsecsElapsed();
    const double dt = lastCameraNs_ == 0 ? 0.0 : static_cast<double>(nowNs - lastCameraNs_) / 1e9;
    lastCameraNs_ = nowNs;

    scene_.update(snapshot.get());
    scene_.advanceCamera(dt);
    hud_.update(snapshot.get());
    zones_->refresh();
    if (diagnostics_->isVisible()) diagnostics_->setView(view.get());

    const bool timed = view && view->hud.matchTimeS >= 0.f;
    viewport_->setMatchClock(timed ? runview::formatClock(static_cast<double>(view->hud.matchTimeS)) : QString());
    if (view && view->hasOwn)
        who_->setText(QStringLiteral("%1 · %2").arg(QString::fromStdString(view->own.name), ui::stationText(view->own.station)));
}

void DrivePage::refreshPresentation() {
    const Presentation& p = controller_.presentation();
    banner_->setPresentation(p);
    state_->setText(p.screen == Screen::Drive ? stateWord(controller_.sessionState().status.state) : QString());
    zones_->setEnabled(p.controlsEnabled);
    placeOverlays();
}

void DrivePage::placeOverlays() {
    if (viewport_ == nullptr) return;
    banner_->adjustSize();
    banner_->move((viewport_->width() - banner_->width()) / 2, 64);
    diagnostics_->adjustSize();
    diagnostics_->move(10, std::max(10, (viewport_->height() - diagnostics_->height()) / 2));
    if (banner_->isVisible()) banner_->raise();
    if (diagnostics_->isVisible()) diagnostics_->raise();
}

bool DrivePage::handleKey(int qtKey, bool down) {
    if (joysticks_ == nullptr || isModifierKey(qtKey)) return false;
    return joysticks_->handleKey(qtKey, down);
}

bool DrivePage::keysReach() const { return window() != nullptr && window()->isActiveWindow(); }

bool DrivePage::eventFilter(QObject* watched, QEvent* event) {
    switch (event->type()) {
        case QEvent::Resize:
            if (watched == viewport_) placeOverlays();
            break;
        case QEvent::KeyPress:
        case QEvent::KeyRelease: {
            const auto* key = static_cast<QKeyEvent*>(event);
            if (key->isAutoRepeat() || !isVisible() || !keysReach()) break;
            if (typingInto(QApplication::focusWidget())) break;
            if (handleKey(key->key(), event->type() == QEvent::KeyPress)) return true;
            break;
        }
        case QEvent::ApplicationDeactivate:
        case QEvent::WindowDeactivate:
            if (joysticks_ != nullptr) joysticks_->releaseAllKeys();
            break;
        default: break;
    }
    return QWidget::eventFilter(watched, event);
}

}  // namespace wpiemu::app::client
