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
#include <QSignalBlocker>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>

#include "joystick_editor.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview {
namespace {

constexpr int kPollMs = 7;          // ~144 Hz cap
constexpr int kDsPublishMs = 20;   // like a real DS: 50 Hz

bool isModifier(int key) {
    return key == Qt::Key_Shift || key == Qt::Key_Control || key == Qt::Key_Alt || key == Qt::Key_Meta || key == Qt::Key_AltGr ||
           key == Qt::Key_CapsLock;
}

bool typingInto(const QWidget* w) {
    return w != nullptr && (w->testAttribute(Qt::WA_InputMethodEnabled) || qobject_cast<const QAbstractSpinBox*>(w) != nullptr);
}

}  // namespace

RunPage::RunPage(IRuntimeHost& host, QWidget* parent) : RunPage(host, RunPageOptions{}, parent) {}

RunPage::RunPage(IRuntimeHost& host, RunPageOptions options, QWidget* parent)
    : QWidget(parent),
      host_(host),
      options_(options),
      ownedSettings_(options.settings == nullptr ? std::make_unique<QSettings>() : nullptr),
      ownedProvider_(options.gamepads == nullptr && options.useDefaultGamepadBackend ? createDefaultJoystickProvider() : nullptr),
      ownedSfx_(options.sfx == nullptr && options.useDefaultSfxBackend ? createDefaultSfxOutput() : nullptr),
      settings_(options.settings != nullptr ? options.settings : ownedSettings_.get()),
      ds_(settings_),
      joysticks_(options.gamepads != nullptr ? options.gamepads : ownedProvider_.get(), settings_),
      sfx_(options.sfx != nullptr ? options.sfx : ownedSfx_.get()) {
    setObjectName(QStringLiteral("runview.page"));
    sfx_.setActive(false);   // until the page is shown
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

    viewport_ = new RunViewport(scene_, hud_, theme_, viewOptions_, this);

    stateWindow_ = new StateWindow(theme_, this);
    stateWindow_->restoreGeometry(settings_->value(QStringLiteral("runview/stateWindow/geometry")).toByteArray());
    connect(stateWindow_, &StateWindow::closed, this, [this] {
        saveStateWindowGeometry();
        const QSignalBlocker block(stateAction_);
        stateAction_->setChecked(false);
    });

    // State, restart (with its robot-code option) and speed float at the bottom-right of the 3D view, beside Home view.
    viewport_->addStateControl(stateAction_);
    startButton_ = viewport_->addStartPositionControl(startMenu_);
    allianceButton_ = viewport_->addAllianceControl();
    connect(allianceButton_, &QToolButton::clicked, this, [this] {
        ds_.setAlliance(ds_.alliance() == sdk::Alliance::Red ? sdk::Alliance::Blue : sdk::Alliance::Red);
    });
    connect(&ds_, &DriverStationModel::changed, this, &RunPage::updateAllianceButton);
    connect(&ds_, &DriverStationModel::changed, this, [this] {
        scene_.setDriverStation(ds_.alliance(), ds_.station());
        updateDriverPovLabel();
    });
    connect(&scene_, &SceneModel::cameraViewChanged, this, [this] {
        syncCameraActions();
        settings_->setValue(QStringLiteral("runview/camera/view"),
                            QString::fromStdString(cameraViewKey(scene_.cameraView())));
    });
    connect(&scene_, &SceneModel::followRobotChanged, this, [this] {
        settings_->setValue(QStringLiteral("runview/camera/follow"), scene_.followRobot());
    });
    updateAllianceButton();
    viewport_->addRestartControl(restartAction_, restartMenu_);
    viewport_->addSpeedControl(speedCombo_);
    viewport_->addCameraControl(cameraMenu_);
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

    connect(&host_, &IRuntimeHost::statusChanged, this, &RunPage::applyStatus);
    connect(&host_, &IRuntimeHost::sceneChanged, this, &RunPage::onSceneChanged);
    connect(&host_, &IRuntimeHost::startPositionChanged, this, &RunPage::rebuildStartMenu);
    connect(&host_, &IRuntimeHost::logLine, console_, &ConsolePanel::append);
    connect(&ds_, &DriverStationModel::stateReady, &host_, &IRuntimeHost::setDriverStation);
    connect(&ds_, &DriverStationModel::sfxRequested, &sfx_, &SfxPlayer::play);
    connect(&host_, &IRuntimeHost::soundCue, &sfx_, &SfxPlayer::play);
    connect(&sfx_, &SfxPlayer::problem, this, [this](const QString& message) {
        console_->append(emulatorLogLine(sdk::LogLevel::Warn, tr("sound"), message));
    });
    connect(viewport_->overlay(), &StateOverlay::restartRequested, this, [this] {
        ds_.disable();
        host_.restart();
    });
    connect(dsPanel_, &DriverStationPanel::matchClockChanged, viewport_, &RunViewport::setMatchClock);
    connect(viewport_, &RunViewport::homeRequested, this, [this] { scene_.resetCamera(); });
    connect(viewport_->overlay(), &StateOverlay::showConsoleRequested, this, [this] { setConsoleVisible(true); });
    connect(viewport_->overlay(), &StateOverlay::problemActionRequested, this, &RunPage::problemActionRequested);
    connect(&scene_, &SceneModel::zoneLabelsChanged, this, &RunPage::rebuildZoneMenu);
    connect(&joysticks_, &JoystickManager::gamepadConnected, this, [this](const JoystickSourceInfo& info, int slot) {
        console_->append(emulatorLogLine(sdk::LogLevel::Info, tr("gamepad"),
                                         tr("%1 connected, assigned to joystick port %2").arg(info.name).arg(slot)));
    });

    pollTimer_ = new QTimer(this);
    cameraClock_.start();
    pollTimer_->setInterval(kPollMs);
    connect(pollTimer_, &QTimer::timeout, this, [this] { pollNow(); });
    pollTimer_->start();
    dsTimer_ = new QTimer(this);
    dsTimer_->setInterval(kDsPublishMs);
    connect(dsTimer_, &QTimer::timeout, this, &RunPage::publishDriverStation);
    dsTimer_->start();

    onSceneChanged();
    applyStatus();
    setConsoleVisible(true);
}

void RunPage::buildActions() {    restartAction_ = new QAction(reloadIcon(), tr("Restart"), this);
    restartAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    restartAction_->setToolTip(tr("Put the robot (at the selected start position) and game pieces back to their starting state; the robot code keeps running (Ctrl+R). Use the arrow to also restart the robot code."));
    startMenu_ = new QMenu(this);
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

    cameraMenu_ = new QMenu(tr("Camera view"), this);
    cameraMenu_->setObjectName(QStringLiteral("runview.cameraMenu"));
    cameraActionGroup_ = new QActionGroup(cameraMenu_);
    cameraActionGroup_->setExclusive(true);
    homeCameraAction_ = cameraMenu_->addAction(tr("Field overview"));
    homeCameraAction_->setObjectName(QStringLiteral("runview.camera.home"));
    homeCameraAction_->setCheckable(true);
    homeCameraAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
    homeCameraAction_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    homeCameraAction_->setToolTip(tr("Show the whole field (Ctrl+0)"));
    cameraActionGroup_->addAction(homeCameraAction_);
    driverCameraAction_ = cameraMenu_->addAction(QString());
    driverCameraAction_->setObjectName(QStringLiteral("runview.camera.driver"));
    driverCameraAction_->setCheckable(true);
    driverCameraAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_9));
    driverCameraAction_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    driverCameraAction_->setToolTip(tr("Set the 3D camera viewpoint at your Driver Station; this is not a simulated robot camera (Ctrl+9)."));
    cameraActionGroup_->addAction(driverCameraAction_);
    cameraMenu_->addSection(tr("Blue alliance"));
    for (int number = 1; number <= 3; ++number) {
        QAction* action = cameraMenu_->addAction(tr("Station %1").arg(number));
        action->setObjectName(QStringLiteral("runview.camera.blue%1").arg(number));
        action->setData(QStringLiteral("blue%1").arg(number));
        action->setCheckable(true);
        cameraActionGroup_->addAction(action);
        stationCameraActions_.push_back(action);
    }
    cameraMenu_->addSection(tr("Red alliance"));
    for (int number = 1; number <= 3; ++number) {
        QAction* action = cameraMenu_->addAction(tr("Station %1").arg(number));
        action->setObjectName(QStringLiteral("runview.camera.red%1").arg(number));
        action->setData(QStringLiteral("red%1").arg(number));
        action->setCheckable(true);
        cameraActionGroup_->addAction(action);
        stationCameraActions_.push_back(action);
    }
    cameraMenu_->addSeparator();
    followCameraAction_ = cameraMenu_->addAction(tr("Follow robot"));
    followCameraAction_->setObjectName(QStringLiteral("runview.camera.follow"));
    followCameraAction_->setCheckable(true);
    followCameraAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_F));
    followCameraAction_->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    initialCameraView_ = parseCameraViewKey(
                             settings_->value(QStringLiteral("runview/camera/view"), QStringLiteral("home"))
                                 .toString().toStdString())
                             .value_or(CameraView{});
    const bool follow = settings_->value(QStringLiteral("runview/camera/follow"), true).toBool();
    followCameraAction_->setChecked(follow);
    scene_.setFollowRobot(follow);
    addAction(homeCameraAction_);
    addAction(driverCameraAction_);
    addAction(followCameraAction_);
    updateDriverPovLabel();
    connect(homeCameraAction_, &QAction::triggered, this, [this] { scene_.resetCamera(); });
    connect(driverCameraAction_, &QAction::triggered, this, [this] { scene_.showDriverView(); });
    for (QAction* action : stationCameraActions_) {
        connect(action, &QAction::triggered, this, [this, action] {
            const auto view = parseCameraViewKey(action->data().toString().toStdString());
            if (view) scene_.showStationView(view->alliance, view->number);
        });
    }
    connect(followCameraAction_, &QAction::toggled, &scene_, &SceneModel::setFollowRobot);
    connect(&scene_, &SceneModel::followRobotChanged, this, [this] {
        const QSignalBlocker blocker(followCameraAction_);
        followCameraAction_->setChecked(scene_.followRobot());
    });
    syncCameraActions();

    // Overlay and console toggles are exposed through viewActions() so the shell can list them in its View menu.
    hudAction_ = new QAction(tr("Game HUD"), this);
    hudAction_->setCheckable(true);
    hudAction_->setChecked(viewOptions_.showHud());
    stateAction_ = new QAction(tr("Robot state"), this);
    stateAction_->setCheckable(true);
    stateAction_->setToolTip(tr("Open a window with the live state of mechanisms, sensors, zones and cameras"));
    zonesAction_ = new QAction(tr("Zones (intake, shooter, ...)"), this);
    zonesAction_->setCheckable(true);
    zonesAction_->setChecked(scene_.zonesVisible());
    zoneMenu_ = new QMenu(tr("Individual zones"), this);
    zoneMenu_->setEnabled(false);
    tagsAction_ = new QAction(tr("AprilTags"), this);
    tagsAction_->setCheckable(true);
    tagsAction_->setChecked(scene_.tagsVisible());
    tagsAction_->setToolTip(tr("Show the field AprilTags; tags seen by a camera are highlighted"));

    consoleAction_ = new QAction(tr("Console"), this);
    consoleAction_->setCheckable(true);
    consoleAction_->setChecked(true);
    consoleAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_J));
    consoleAction_->setToolTip(tr("Show or hide the console (Ctrl+J)"));
    connect(restartAction_, &QAction::triggered, this, [this] {
        ds_.onSimReset();
        host_.resetSimulation();
    });
    connect(restartRobotAction_, &QAction::triggered, this, [this] {
        ds_.disable();
        ds_.onSimReset();
        host_.restart();
        host_.startSimulation();
    });
    connect(speedCombo_, &QComboBox::activated, this, [this](int i) { host_.setTimeScale(speedCombo_->itemData(i).toDouble()); });
    connect(hudAction_, &QAction::toggled, &viewOptions_, &ViewportOptions::setShowHud);
    connect(stateAction_, &QAction::toggled, this, [this](bool on) { setStateWindowVisible(on); });
    connect(zonesAction_, &QAction::toggled, &scene_, &SceneModel::setZonesVisible);
    connect(tagsAction_, &QAction::toggled, &scene_, &SceneModel::setTagsVisible);
    connect(consoleAction_, &QAction::toggled, this, [this](bool on) { setConsoleVisible(on); });
}

QList<QAction*> RunPage::viewActions() const {
    return {hudAction_, stateAction_, zonesAction_, zoneMenu_->menuAction(), cameraMenu_->menuAction(), tagsAction_,
            consoleAction_};
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

void RunPage::rebuildStartMenu() {
    // Called from the menu's own actions (through startPositionChanged): never delete an action that is emitting.
    for (QAction* a : startMenu_->actions()) {
        startMenu_->removeAction(a);
        a->deleteLater();
    }
    for (QActionGroup* g : startMenu_->findChildren<QActionGroup*>(Qt::FindDirectChildrenOnly)) g->deleteLater();
    const QString current = host_.startPosition();
    auto* group = new QActionGroup(startMenu_);
    group->setExclusive(true);
    const auto addEntry = [&](const QString& text, const QString& id) {
        QAction* a = startMenu_->addAction(text);
        a->setCheckable(true);
        a->setChecked(id == current);
        group->addAction(a);
        connect(a, &QAction::triggered, this, [this, id] { selectStartPosition(id); });
    };
    addEntry(tr("Default (robot setup)"), QString());
    QString currentLabel = tr("Default");
    for (const sdk::Alliance alliance : {sdk::Alliance::Blue, sdk::Alliance::Red}) {
        bool any = false;
        for (const sdk::StartPosition& p : startPositions_) {
            if (p.alliance != alliance) continue;
            if (!any) startMenu_->addSection(alliance == sdk::Alliance::Blue ? tr("Blue alliance") : tr("Red alliance"));
            any = true;
            const QString id = QString::fromStdString(p.id);
            const QString label = p.label.empty() ? id : QString::fromStdString(p.label);
            addEntry(label, id);
            if (id == current) currentLabel = label;
        }
    }
    startButton_->setToolTip(startPositions_.empty()
                                 ? tr("No start positions yet: add them in the editor under Robot > Start positions")
                                 : tr("Start position: %1. Choose where the robot is placed on restart.").arg(currentLabel));
}

void RunPage::updateAllianceButton() {
    const bool red = ds_.alliance() == sdk::Alliance::Red;
    allianceButton_->setText(red ? tr("RED") : tr("BLUE"));
    allianceButton_->setToolTip(tr("Running as the %1 alliance. Click to switch to %2.").arg(red ? tr("Red") : tr("Blue"), red ? tr("Blue") : tr("Red")));
    allianceButton_->setAccessibleName(tr("Alliance: %1").arg(red ? tr("Red") : tr("Blue")));
    allianceButton_->setStyleSheet(QStringLiteral(
        "QToolButton { color: white; font-weight: 700; border-radius: 8px; border: 1px solid rgba(255,255,255,90); background: %1; }"
        "QToolButton:hover { border-color: white; }")
                                       .arg(red ? QStringLiteral("#c93a3f") : QStringLiteral("#3a6fd8")));
}

void RunPage::selectStartPosition(const QString& id) {
    const sdk::StartPosition* position = nullptr;
    for (const sdk::StartPosition& p : startPositions_)
        if (QString::fromStdString(p.id) == id) position = &p;
    host_.setStartPosition(position != nullptr ? id : QString());
    if (position != nullptr) {
        ds_.setAlliance(position->alliance);
        console_->append(emulatorLogLine(sdk::LogLevel::Info, tr("start position"),
                                         tr("%1: x %2 m, y %3 m, heading %4°")
                                             .arg(position->label.empty() ? id : QString::fromStdString(position->label))
                                             .arg(static_cast<double>(position->pose.xM), 0, 'f', 2)
                                             .arg(static_cast<double>(position->pose.yM), 0, 'f', 2)
                                             .arg(sdk::radToDeg(static_cast<double>(position->pose.headingRad)), 0, 'f', 1)));
    }
    ds_.onSimReset();
    host_.resetSimulation();
}

void RunPage::applyProjectDefaults(const HostScene& scene) {
    if (scene.projectPath.isEmpty()) {
        // No project: forget what was applied so reopening the same project applies its default again.
        defaultsProject_.clear();
        defaultsStation_.reset();
        return;
    }
    if (scene.projectPath == defaultsProject_ && defaultsStation_ == scene.defaultStation) return;
    defaultsProject_ = scene.projectPath;
    defaultsStation_ = scene.defaultStation;
    ds_.setAlliance(sdk::proto::allianceOf(scene.defaultStation));
    ds_.setStation(sdk::proto::stationNumberOf(scene.defaultStation));
}

void RunPage::onSceneChanged() {
    const HostScene info = host_.scene();
    applyProjectDefaults(info);
    startPositions_ = info.config ? info.config->startPositions : std::vector<sdk::StartPosition>{};
    scene_.setSceneInfo(info);
    if (!cameraViewRestored_) {
        scene_.restoreView(initialCameraView_);
        cameraViewRestored_ = true;
    }
    sfx_.setCatalog(info.gameId, info.soundCues);
    hud_.setDescriptors(info.hud);
    stateWindow_->model()->setScene(info);
    rebuildZoneMenu();
    rebuildStartMenu();
}

void RunPage::applyStatus() {
    const HostStatus status = host_.status();
    const auto snapshot = host_.latestSnapshot();
    const StatusPresentation p = presentStatus(status, snapshot != nullptr && snapshot->robotConnected());
    viewport_->overlay()->setPresentation(p);
    refreshControls(snapshot.get());

    const bool available = status.live() && status.simulating;
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
    const RunControlState c = deriveControlState(host_.status(), paused);
    restartAction_->setEnabled(c.simulating);
    restartRobotAction_->setEnabled(c.canRestart);
    startButton_->setEnabled(c.simulating && !startPositions_.empty());
    speedCombo_->setEnabled(c.simulating);
}

void RunPage::pollNow() {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (!cameraClock_.isValid()) cameraClock_.start();
    const qint64 cameraNowNs = cameraClock_.nsecsElapsed();
    const double cameraDt = lastCameraNs_ == 0
                                ? 0.0
                                : static_cast<double>(cameraNowNs - lastCameraNs_) / 1'000'000'000.0;
    lastCameraNs_ = cameraNowNs;

    const auto snapshot = host_.latestSnapshot();
    const sdk::ISimSnapshot* s = snapshot.get();
    if (s != nullptr) ds_.onSimTime(s->simTimeUs());

    scene_.update(s);
    scene_.advanceCamera(cameraDt);
    hud_.update(s);
    if (stateWindow_->isVisible()) stateWindow_->showSnapshot(s);
    refreshControls(s);

    if (stats_.update(s, now) || now - lastLabelMs_ > 250) {
        lastLabelMs_ = now;
        refreshStatsLabels();
    }
}

void RunPage::updateDriverPovLabel() {
    const QString alliance = ds_.alliance() == sdk::Alliance::Blue ? tr("Blue") : tr("Red");
    driverCameraAction_->setText(tr("Driver POV (%1 %2)").arg(alliance).arg(ds_.station()));
}

void RunPage::syncCameraActions() {
    const CameraView& view = scene_.cameraView();
    const auto setChecked = [](QAction* action, bool checked) {
        const QSignalBlocker blocker(action);
        action->setChecked(checked);
    };
    setChecked(homeCameraAction_, view.kind == CameraView::Kind::Home);
    setChecked(driverCameraAction_, view.kind == CameraView::Kind::DriverPov);
    for (QAction* action : stationCameraActions_) {
        const auto station = parseCameraViewKey(action->data().toString().toStdString());
        setChecked(action, station && view.kind == CameraView::Kind::Station &&
                               station->alliance == view.alliance && station->number == view.number);
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

// The Driver Station settings popup is a separate window: it only receives keys while a binding is being learned.
bool RunPage::keysReachJoystick() const {
    if (window()->isActiveWindow()) return true;
    return joysticks_.learning() && QApplication::activeWindow() == dsPanel_->joystickEditor()->window();
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
            if (key->isAutoRepeat() || !isVisible() || !keysReachJoystick()) break;
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
    sfx_.setActive(true);
    qApp->installEventFilter(this);
    theme_.setPalette(palette());
    if (stateAction_ != nullptr && stateAction_->isChecked() && stateWindow_->isHidden()) stateWindow_->show();
}

void RunPage::hideEvent(QHideEvent* e) {
    QWidget::hideEvent(e);
    sfx_.setActive(false);   // leaving Run silences it; switching windows does not
    if (qApp != nullptr) qApp->removeEventFilter(this);
    joysticks_.releaseAllKeys();
    // The shell's stacked widget does not hide child windows; the action stays checked so showEvent restores it.
    if (stateWindow_ != nullptr && stateWindow_->isVisible()) {
        saveStateWindowGeometry();
        stateWindow_->hide();
    }
}

void RunPage::setSoundEffects(bool enabled, int volumePercent) {
    sfx_.setEnabled(enabled);
    sfx_.setVolumePercent(volumePercent);
}

void RunPage::saveStateWindowGeometry() {
    settings_->setValue(QStringLiteral("runview/stateWindow/geometry"), stateWindow_->saveGeometry());
}

void RunPage::setStateWindowVisible(bool visible) {
    if (visible) {
        stateWindow_->showSnapshot(host_.latestSnapshot().get());
        stateWindow_->show();
        stateWindow_->raise();
        stateWindow_->activateWindow();
    } else {
        saveStateWindowGeometry();
        stateWindow_->hide();
    }
}

}  // namespace wpiemu::app::runview
