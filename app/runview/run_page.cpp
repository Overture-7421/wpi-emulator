#include "run_page.h"

#include <QAbstractSpinBox>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QComboBox>
#include <QDateTime>
#include <QEvent>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QSplitter>
#include <QStyle>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>

namespace wpiemu::app::runview {
namespace {

constexpr int kPollMs = 16;         // ~60 Hz
constexpr int kDsPublishMs = 20;   // like a real DS: 50 Hz
constexpr int kRobotLoopUs = 20000;

bool isModifier(int key) {
    return key == Qt::Key_Shift || key == Qt::Key_Control || key == Qt::Key_Alt || key == Qt::Key_Meta || key == Qt::Key_AltGr ||
           key == Qt::Key_CapsLock;
}

bool typingInto(const QWidget* w) {
    return w != nullptr && (w->testAttribute(Qt::WA_InputMethodEnabled) || qobject_cast<const QAbstractSpinBox*>(w) != nullptr);
}

}  // namespace

RunPage::RunPage(IRunBackend& backend, QWidget* parent) : RunPage(backend, RunPageOptions{}, parent) {}

RunPage::RunPage(IRunBackend& backend, RunPageOptions options, QWidget* parent)
    : QWidget(parent),
      backend_(backend),
      options_(options),
      ownedSettings_(options.settings == nullptr ? std::make_unique<QSettings>() : nullptr),
      ownedProvider_(options.gamepads == nullptr && options.useDefaultGamepadBackend ? createDefaultJoystickProvider() : nullptr),
      settings_(options.settings != nullptr ? options.settings : ownedSettings_.get()),
      ds_(settings_),
      joysticks_(options.gamepads != nullptr ? options.gamepads : ownedProvider_.get(), settings_) {
    registerRunViewMetaTypes();
    setObjectName(QStringLiteral("runview.page"));
    build();
}

RunPage::~RunPage() {
    if (qApp != nullptr) qApp->removeEventFilter(this);
    // The QML context points at members that are destroyed before the child widgets: drop the view first.
    delete viewport_;
    viewport_ = nullptr;
}

void RunPage::build() {
    theme_.setPalette(palette());

    buildActions();

    viewport_ = new RunViewport(scene_, hud_, mechanisms_, cameraHud_, theme_, viewOptions_, this);

    // Restart (with its robot-code option) and speed float at the bottom-right of the 3D view, beside Home view.
    viewport_->addRestartControl(restartAction_, restartMenu_);
    viewport_->addSpeedControl(speedCombo_);
    console_ = new ConsolePanel(this);
    dsPanel_ = new DriverStationPanel(ds_, joysticks_, this);

    // The Driver Station sits bottom-left, beside the console; the 3D view takes the full width above them.
    auto* bottom = new QSplitter(Qt::Horizontal, this);
    bottom->addWidget(dsPanel_);
    bottom->addWidget(console_);
    bottom->setStretchFactor(0, 0);
    bottom->setStretchFactor(1, 1);
    bottom->setChildrenCollapsible(false);
    dsPanel_->setMinimumWidth(500);
    bottom->setSizes({520, 800});
    bottom->setMinimumHeight(200);

    verticalSplit_ = new QSplitter(Qt::Vertical, this);
    verticalSplit_->addWidget(viewport_);
    verticalSplit_->addWidget(bottom);
    verticalSplit_->setStretchFactor(0, 4);
    verticalSplit_->setStretchFactor(1, 0);
    verticalSplit_->setChildrenCollapsible(false);
    verticalSplit_->setSizes({600, 210});
    viewport_->setMinimumSize(320, 240);
    console_->setMinimumHeight(110);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(verticalSplit_, 1);

    connect(&backend_, &IRunBackend::statusChanged, this, &RunPage::applyStatus);
    connect(&backend_, &IRunBackend::sceneInfoChanged, this, &RunPage::onSceneInfoChanged);
    connect(&backend_, &IRunBackend::logEntry, console_, &ConsolePanel::append);
    connect(&ds_, &DriverStationModel::stateReady, &backend_, &IRunBackend::setDriverStation);
    connect(viewport_->overlay(), &StateOverlay::restartRequested, this, [this] {
        ds_.disable();
        backend_.restartRobot();
    });
    connect(dsPanel_, &DriverStationPanel::matchClockChanged, viewport_, &RunViewport::setMatchClock);
    connect(viewport_, &RunViewport::homeRequested, this, [this] { scene_.resetCamera(); });
    connect(viewport_->overlay(), &StateOverlay::showConsoleRequested, this, [this] { setConsoleVisible(true); });
    connect(&scene_, &SceneModel::zoneLabelsChanged, this, &RunPage::rebuildZoneMenu);
    // The vision problems the HUD shows also go to the console once, where the level filter finds them.
    connect(&cameraHud_.noticeModel(), &VisionNoticeModel::noticeRaised, this, [this](const QString& title, const QString& detail, int severity) {
        const sdk::LogLevel level = severity == static_cast<int>(sdk::Severity::Error)     ? sdk::LogLevel::Error
                                    : severity == static_cast<int>(sdk::Severity::Warning) ? sdk::LogLevel::Warn
                                                                                           : sdk::LogLevel::Info;
        console_->append({QDateTime::currentMSecsSinceEpoch(), level, LogStream::Emulator, tr("vision"),
                          detail.isEmpty() ? title : tr("%1: %2").arg(title, detail)});
    });
    connect(&scene_, &SceneModel::robotPoseRequested, this, [this](double xM, double yM, double zM, double yawRad) {
        backend_.setRobotPose(sdk::toPose3(sdk::StartPose{static_cast<float>(xM), static_cast<float>(yM), static_cast<float>(yawRad)}, static_cast<float>(zM)));
    });
    connect(&joysticks_, &JoystickManager::gamepadConnected, this, [this](const JoystickSourceInfo& info, int slot) {
        console_->append({QDateTime::currentMSecsSinceEpoch(), sdk::LogLevel::Info, LogStream::Emulator, tr("gamepad"),
                          tr("%1 connected, assigned to joystick port %2").arg(info.name).arg(slot)});
    });

    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(kPollMs);
    connect(pollTimer_, &QTimer::timeout, this, [this] { pollNow(); });
    pollTimer_->start();
    dsTimer_ = new QTimer(this);
    dsTimer_->setInterval(kDsPublishMs);
    connect(dsTimer_, &QTimer::timeout, this, &RunPage::publishDriverStation);
    dsTimer_->start();

    onSceneInfoChanged();
    applyStatus();
    setConsoleVisible(true);
}

void RunPage::buildActions() {    restartAction_ = new QAction(reloadIcon(), tr("Restart"), this);
    restartAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    restartAction_->setToolTip(tr("Put the robot and game pieces back to their starting state; the robot code keeps running (Ctrl+R). Use the arrow to also restart the robot code."));
    auto* restartMenu = new QMenu(this);
    restartMenu_ = restartMenu;
    connect(restartMenu->addAction(tr("Restart physics only")), &QAction::triggered, restartAction_, &QAction::trigger);
    restartRobotAction_ = restartMenu->addAction(tr("Restart robot code"));
    restartRobotAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_R));
    restartAction_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    restartRobotAction_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    addAction(restartAction_);
    addAction(restartRobotAction_);
    restartRobotAction_->setToolTip(tr("Reboot the robot code and reset the physics and game state (Ctrl+Shift+R)"));

    speedCombo_ = new QComboBox(this);
    speedCombo_->setObjectName(QStringLiteral("runview.speed"));
    speedCombo_->setToolTip(tr("Simulation speed relative to real time"));
    for (double s : {0.1, 0.25, 0.5, 1.0, 2.0, 4.0}) speedCombo_->addItem(QStringLiteral("%1×").arg(s), s);
    speedCombo_->setCurrentIndex(3);
    speedCombo_->setFixedSize(58, 34);

    // The Home view button is an icon inside the 3D view; the action only carries the shortcut.
    QAction* resetView = new QAction(tr("Home view"), this);
    resetView->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
    resetView->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    addAction(resetView);

    // Overlay and console toggles are exposed through viewActions() so the shell can list them in its View menu. The overlay toggles are
    // remembered in the settings, like the Driver Station choices.
    const auto overlayToggle = [this](const QString& text, const char* key, bool fallback, const QString& tip, std::function<void(bool)> apply) {
        auto* action = new QAction(text, this);
        action->setCheckable(true);
        if (!tip.isEmpty()) action->setToolTip(tip);
        const QString path = QStringLiteral("runview/view/") + QLatin1String(key);
        const bool on = settings_->value(path, fallback).toBool();
        action->setChecked(on);
        apply(on);
        connect(action, &QAction::toggled, this, [this, path, apply = std::move(apply)](bool checked) {
            apply(checked);
            settings_->setValue(path, checked);
        });
        return action;
    };
    hudAction_ = overlayToggle(tr("Game HUD"), "hud", true, {}, [this](bool on) { viewOptions_.setShowHud(on); });
    mechAction_ = overlayToggle(tr("Mechanism values"), "mechanisms", true, {}, [this](bool on) { viewOptions_.setShowMechanisms(on); });
    zonesAction_ = overlayToggle(tr("Zones (intake, shooter, ...)"), "zones", true, {}, [this](bool on) { scene_.setZonesVisible(on); });
    zoneMenu_ = new QMenu(tr("Individual zones"), this);
    zoneMenu_->setEnabled(false);
    camerasAction_ = overlayToggle(tr("Cameras"), "cameras", true, tr("Camera bodies, their view frustums and the camera cards"),
                                   [this](bool on) { scene_.setCamerasVisible(on); });
    tagsAction_ = overlayToggle(tr("AprilTags"), "aprilTags", true, tr("The field's AprilTags, highlighted while a camera sees them"),
                                [this](bool on) { scene_.setTagsVisible(on); });

    consoleAction_ = new QAction(tr("Console"), this);
    consoleAction_->setCheckable(true);
    consoleAction_->setChecked(true);
    consoleAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_J));
    consoleAction_->setToolTip(tr("Show or hide the console (Ctrl+J)"));
    connect(restartAction_, &QAction::triggered, this, [this] {
        ds_.onSimReset();
        backend_.resetSimulation();
    });
    connect(restartRobotAction_, &QAction::triggered, this, [this] {
        ds_.disable();
        ds_.onSimReset();
        backend_.restartRobot();
        backend_.startSimulation();
    });
    connect(speedCombo_, &QComboBox::activated, this, [this](int i) { backend_.setTimeScale(speedCombo_->itemData(i).toDouble()); });
    connect(resetView, &QAction::triggered, this, [this] { scene_.resetCamera(); });
    connect(consoleAction_, &QAction::toggled, this, [this](bool on) { setConsoleVisible(on); });
}

QList<QAction*> RunPage::viewActions() const {
    return {hudAction_, mechAction_, zonesAction_, zoneMenu_->menuAction(), camerasAction_, tagsAction_, consoleAction_};
}

void RunPage::setConsoleVisible(bool visible) {
    console_->setVisible(visible);
    if (consoleAction_ != nullptr && consoleAction_->isChecked() != visible) {
        const QSignalBlocker block(consoleAction_);
        consoleAction_->setChecked(visible);
    }
}

bool RunPage::consoleVisible() const { return !console_->isHidden(); }

void RunPage::rebuildZoneMenu() {
    zoneMenu_->clear();
    const QStringList labels = scene_.zoneLabels();
    zoneMenu_->setEnabled(!labels.isEmpty());
    for (const QString& label : labels) {
        QAction* a = zoneMenu_->addAction(label);
        a->setCheckable(true);
        a->setChecked(scene_.zoneLabelShown(label));
        connect(a, &QAction::toggled, this, [this, label](bool on) { scene_.setZoneLabelShown(label, on); });
    }
}

void RunPage::onSceneInfoChanged() {
    const RunSceneInfo info = backend_.sceneInfo();
    scene_.setSceneInfo(info);
    hud_.setDescriptors(info.hud);
    mechanisms_.setConfig(info.config);
    cameraHud_.setConfig(info.config);
    rebuildZoneMenu();
}

void RunPage::applyStatus() {
    const RunStatus status = backend_.status();
    const auto snapshot = backend_.latestSnapshot();
    const StatusPresentation p = presentStatus(status, snapshot != nullptr && snapshot->robotConnected());
    viewport_->overlay()->setPresentation(p);
    refreshControls(snapshot.get());

    const bool available = status.process == RobotProcessState::Running && status.link == LinkState::Connected && status.simulating;
    ds_.setRobotAvailable(available);
    dsPanel_->setRobotConnected(available);

    if (p.tone == Tone::Error && !consoleAutoShown_) {
        consoleAutoShown_ = true;
        setConsoleVisible(true);
    } else if (p.tone != Tone::Error) {
        consoleAutoShown_ = false;
    }
}

void RunPage::refreshControls(const sdk::ISimSnapshot* snapshot) {
    const bool paused = snapshot != nullptr && snapshot->paused();
    const RunControlState c = deriveControlState(backend_.status(), paused);
    restartAction_->setEnabled(c.simulating);
    restartRobotAction_->setEnabled(c.canRestart);
    speedCombo_->setEnabled(c.simulating);
    scene_.setPlacementEnabled(c.simulating);   // the robot can be dragged on the field while the simulation runs
}

void RunPage::pollNow() {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    const auto snapshot = backend_.latestSnapshot();
    const sdk::ISimSnapshot* s = snapshot.get();
    if (s != nullptr) ds_.onSimTime(s->simTimeUs());

    scene_.update(s);
    hud_.update(s);
    mechanisms_.update(s);
    cameraHud_.update(s);
    refreshControls(s);

    if (stats_.update(s, now) || now - lastLabelMs_ > 250) {
        lastLabelMs_ = now;
        refreshStatsLabels();
    }
}

void RunPage::refreshStatsLabels() { dsPanel_->setStats(stats_.values()); }

void RunPage::publishDriverStation() {
    const auto& js = joysticks_.sample();
    ds_.publish(js, QDateTime::currentMSecsSinceEpoch());
}

bool RunPage::handleKey(int qtKey, bool down) {
    if (isModifier(qtKey)) return false;
    // Enter disables and Space latches the e-stop, like the real DS: they must reach the shortcuts, not the joystick keys.
    if (down && (qtKey == Qt::Key_Space || qtKey == Qt::Key_Return || qtKey == Qt::Key_Enter) && !joysticks_.learning()) return false;
    return joysticks_.handleKey(qtKey, down);
}

bool RunPage::eventFilter(QObject* watched, QEvent* event) {
    switch (event->type()) {
        case QEvent::ShortcutOverride:
            if (joysticks_.learning() && isVisible()) {
                event->accept();
                return true;
            }
            break;
        case QEvent::KeyPress:
        case QEvent::KeyRelease: {
            const auto* key = static_cast<QKeyEvent*>(event);
            if (key->isAutoRepeat() || !isVisible() || !window()->isActiveWindow()) break;
            const bool down = event->type() == QEvent::KeyPress;
            if (down && !joysticks_.learning() && typingInto(QApplication::focusWidget())) break;
            if (handleKey(key->key(), down)) return true;
            break;
        }
        case QEvent::ApplicationDeactivate:
        case QEvent::WindowDeactivate: joysticks_.releaseAllKeys(); break;
        default: break;
    }
    return QWidget::eventFilter(watched, event);
}

void RunPage::changeEvent(QEvent* e) {
    QWidget::changeEvent(e);
    if (e->type() == QEvent::PaletteChange || e->type() == QEvent::ApplicationPaletteChange) theme_.setPalette(palette());
}

void RunPage::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    qApp->installEventFilter(this);
    theme_.setPalette(palette());
}

void RunPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    if (qApp != nullptr) qApp->removeEventFilter(this);
    joysticks_.releaseAllKeys();
}

}  // namespace wpiemu::app::runview
