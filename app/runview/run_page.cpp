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

namespace wpiemu::app::runview {
namespace {

constexpr int kPollMs = 16;        // ~60 Hz
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

    buildToolBar();

    statusChip_ = new StatusChip(this);
    statusChip_->setObjectName(QStringLiteral("runview.statusChip"));
    realDsChip_ = new StatusChip(this);
    realDsChip_->setObjectName(QStringLiteral("runview.realDsChip"));
    simTimeLabel_ = new QLabel(this);
    rateLabel_ = new QLabel(this);
    batteryLabel_ = new QLabel(this);
    extraLabel_ = new QLabel(this);
    const QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    for (QLabel* l : {simTimeLabel_, rateLabel_, batteryLabel_, extraLabel_}) {
        l->setFont(mono);
        l->setEnabled(false);
    }
    simTimeLabel_->setToolTip(tr("Simulated time"));
    rateLabel_->setToolTip(tr("Simulation ticks per second and speed relative to real time"));
    batteryLabel_->setToolTip(tr("Simulated battery voltage"));
    extraLabel_->setToolTip(tr("Physics step time and entity count"));

    auto* strip = new QWidget(this);
    auto* stripLayout = new QHBoxLayout(strip);
    stripLayout->setContentsMargins(10, 4, 10, 4);
    stripLayout->setSpacing(10);
    stripLayout->addWidget(statusChip_);
    stripLayout->addWidget(realDsChip_);
    stripLayout->addStretch(1);
    stripLayout->addWidget(simTimeLabel_);
    stripLayout->addWidget(rateLabel_);
    stripLayout->addWidget(batteryLabel_);
    stripLayout->addWidget(extraLabel_);

    viewport_ = new RunViewport(scene_, hud_, mechanisms_, theme_, viewOptions_, this);
    console_ = new ConsolePanel(this);
    dsPanel_ = new DriverStationPanel(ds_, joysticks_, this);

    verticalSplit_ = new QSplitter(Qt::Vertical, this);
    verticalSplit_->addWidget(viewport_);
    verticalSplit_->addWidget(console_);
    verticalSplit_->setStretchFactor(0, 4);
    verticalSplit_->setStretchFactor(1, 1);
    verticalSplit_->setChildrenCollapsible(false);
    verticalSplit_->setSizes({600, 170});
    viewport_->setMinimumSize(320, 240);
    console_->setMinimumHeight(110);

    auto* horizontal = new QSplitter(Qt::Horizontal, this);
    horizontal->addWidget(verticalSplit_);
    horizontal->addWidget(dsPanel_);
    horizontal->setStretchFactor(0, 1);
    horizontal->setStretchFactor(1, 0);
    horizontal->setChildrenCollapsible(false);
    dsPanel_->setMinimumWidth(380);
    horizontal->setSizes({900, 400});

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(toolBar_);
    layout->addWidget(strip);
    layout->addWidget(horizontal, 1);

    connect(&backend_, &IRunBackend::statusChanged, this, &RunPage::applyStatus);
    connect(&backend_, &IRunBackend::sceneInfoChanged, this, &RunPage::onSceneInfoChanged);
    connect(&backend_, &IRunBackend::realDsStateChanged, this, [this] {
        realDsChip_->setStatus(realDsTone(backend_.realDsState()), realDsText(backend_.realDsState()));
    });
    connect(&backend_, &IRunBackend::logEntry, console_, &ConsolePanel::append);
    connect(&ds_, &DriverStationModel::stateReady, &backend_, &IRunBackend::setDriverStation);
    connect(viewport_->overlay(), &StateOverlay::startRequested, &backend_, &IRunBackend::startRobot);
    connect(viewport_->overlay(), &StateOverlay::restartRequested, this, [this] {
        ds_.disable();
        backend_.restartRobot();
    });
    connect(viewport_->overlay(), &StateOverlay::showConsoleRequested, this, [this] { setConsoleVisible(true); });
    connect(&scene_, &SceneModel::zoneLabelsChanged, this, &RunPage::rebuildZoneMenu);
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

    realDsChip_->setStatus(realDsTone(backend_.realDsState()), realDsText(backend_.realDsState()));
    onSceneInfoChanged();
    applyStatus();
    setConsoleVisible(true);
}

void RunPage::buildToolBar() {
    toolBar_ = new QToolBar(tr("Run controls"), this);
    toolBar_->setObjectName(QStringLiteral("runview.toolbar"));
    toolBar_->setMovable(false);
    toolBar_->setFloatable(false);
    toolBar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toolBar_->setIconSize(QSize(18, 18));

    QStyle* st = style();
    startAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_MediaPlay), tr("Start"));
    startAction_->setShortcut(QKeySequence(Qt::Key_F5));
    startAction_->setToolTip(tr("Launch the robot program in simulation (F5)"));
    stopAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_MediaStop), tr("Stop"));
    stopAction_->setShortcut(QKeySequence(Qt::SHIFT | Qt::Key_F5));
    stopAction_->setToolTip(tr("Stop the robot program (Shift+F5)"));
    restartAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_BrowserReload), tr("Restart"));
    restartAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    restartAction_->setToolTip(tr("Stop and start the robot program again (Ctrl+R)"));
    resetAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_DialogResetButton), tr("Reset sim"));
    resetAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_R));
    resetAction_->setToolTip(tr("Put the robot and game pieces back to their starting state, keeping the program running (Ctrl+Shift+R)"));
    toolBar_->addSeparator();

    pauseAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_MediaPause), tr("Pause"));
    pauseAction_->setCheckable(true);
    pauseAction_->setShortcut(QKeySequence(Qt::Key_F8));
    pauseAction_->setToolTip(tr("Pause / resume the simulation clock (F8)"));
    stepAction_ = toolBar_->addAction(st->standardIcon(QStyle::SP_MediaSeekForward), tr("Step"));
    stepAction_->setShortcut(QKeySequence(Qt::Key_F10));
    stepAction_->setToolTip(tr("Advance one robot loop (20 ms) while paused (F10)"));

    toolBar_->addWidget(new QLabel(tr(" Speed "), this));
    speedCombo_ = new QComboBox(this);
    speedCombo_->setObjectName(QStringLiteral("runview.speed"));
    speedCombo_->setToolTip(tr("Simulation speed relative to real time"));
    for (double s : {0.1, 0.25, 0.5, 1.0, 2.0, 4.0}) speedCombo_->addItem(QStringLiteral("%1×").arg(s), s);
    speedCombo_->setCurrentIndex(3);
    toolBar_->addWidget(speedCombo_);
    toolBar_->addSeparator();

    cameraGroup_ = new QActionGroup(this);
    const std::pair<QString, QString> cameras[] = {{tr("Orbit"), tr("Free orbit: drag to rotate, right-drag to pan, wheel to zoom (Ctrl+1)")},
                                                   {tr("Follow"), tr("Chase camera behind the robot (Ctrl+2)")},
                                                   {tr("Top-down"), tr("Top-down field view (Ctrl+3)")}};
    for (int i = 0; i < 3; ++i) {
        QAction* a = toolBar_->addAction(cameras[i].first);
        a->setCheckable(true);
        a->setToolTip(cameras[i].second);
        a->setShortcut(QKeySequence(Qt::CTRL | static_cast<Qt::Key>(Qt::Key_1 + i)));
        cameraGroup_->addAction(a);
        a->setData(i);
        if (i == 0) a->setChecked(true);
    }
    QAction* resetView = toolBar_->addAction(tr("Reset view"));
    resetView->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
    resetView->setToolTip(tr("Restore the default camera (Ctrl+0, or double-click the view)"));
    toolBar_->addSeparator();

    auto* viewButton = new QToolButton(this);
    viewButton->setText(tr("Overlays"));
    viewButton->setToolTip(tr("Choose which overlays are drawn on the 3D view"));
    viewButton->setPopupMode(QToolButton::InstantPopup);
    viewButton->setStyleSheet(QStringLiteral("QToolButton { padding-right: 16px; }"));
    auto* viewMenu = new QMenu(viewButton);
    QAction* hudAction = viewMenu->addAction(tr("Game HUD"));
    hudAction->setCheckable(true);
    hudAction->setChecked(viewOptions_.showHud());
    QAction* mechAction = viewMenu->addAction(tr("Mechanism values"));
    mechAction->setCheckable(true);
    mechAction->setChecked(viewOptions_.showMechanisms());
    QAction* zonesAction = viewMenu->addAction(tr("Zones (intake, shooter, ...)"));
    zonesAction->setCheckable(true);
    zonesAction->setChecked(scene_.zonesVisible());
    zoneMenu_ = viewMenu->addMenu(tr("Individual zones"));
    zoneMenu_->setEnabled(false);
    viewButton->setMenu(viewMenu);
    toolBar_->addWidget(viewButton);

    consoleAction_ = toolBar_->addAction(tr("Console"));
    consoleAction_->setCheckable(true);
    consoleAction_->setChecked(true);
    consoleAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_J));
    consoleAction_->setToolTip(tr("Show or hide the console (Ctrl+J)"));

    connect(startAction_, &QAction::triggered, this, [this] { backend_.startRobot(); });
    connect(stopAction_, &QAction::triggered, this, [this] {
        ds_.disable();
        backend_.stopRobot();
    });
    connect(restartAction_, &QAction::triggered, this, [this] {
        ds_.disable();
        backend_.restartRobot();
    });
    connect(resetAction_, &QAction::triggered, this, [this] {
        ds_.onSimReset();
        backend_.resetSimulation();
    });
    connect(pauseAction_, &QAction::toggled, this, [this](bool paused) {
        if (!syncingPause_) backend_.setPaused(paused);
    });
    connect(stepAction_, &QAction::triggered, this, [this] {
        const auto cfg = backend_.sceneInfo().config;
        const std::int64_t stepUs = cfg ? std::max<std::int64_t>(1, cfg->sim.stepUs) : 5000;
        backend_.stepTicks(static_cast<int>(std::max<std::int64_t>(1, kRobotLoopUs / stepUs)));
    });
    connect(speedCombo_, &QComboBox::activated, this, [this](int i) { backend_.setTimeScale(speedCombo_->itemData(i).toDouble()); });
    connect(cameraGroup_, &QActionGroup::triggered, this, [this](QAction* a) { scene_.setCameraMode(a->data().toInt()); });
    connect(resetView, &QAction::triggered, this, [this] { scene_.resetCamera(); });
    connect(hudAction, &QAction::toggled, &viewOptions_, &ViewportOptions::setShowHud);
    connect(mechAction, &QAction::toggled, &viewOptions_, &ViewportOptions::setShowMechanisms);
    connect(zonesAction, &QAction::toggled, &scene_, &SceneModel::setZonesVisible);
    connect(consoleAction_, &QAction::toggled, this, [this](bool on) { setConsoleVisible(on); });
}

void RunPage::setCameraMode(int mode) {
    mode = std::clamp(mode, 0, 2);
    scene_.setCameraMode(mode);
    for (QAction* a : cameraGroup_->actions())
        if (a->data().toInt() == mode) a->setChecked(true);
}

void RunPage::setConsoleVisible(bool visible) {
    console_->setVisible(visible);
    if (consoleAction_ != nullptr && consoleAction_->isChecked() != visible) {
        const QSignalBlocker block(consoleAction_);
        consoleAction_->setChecked(visible);
    }
    if (visible && verticalSplit_ != nullptr) {
        const QList<int> sizes = verticalSplit_->sizes();
        if (sizes.size() == 2 && sizes[1] < 80) verticalSplit_->setSizes({std::max(300, sizes[0] - 170), 170});
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
    rebuildZoneMenu();
}

void RunPage::applyStatus() {
    const RunStatus status = backend_.status();
    const auto snapshot = backend_.latestSnapshot();
    const StatusPresentation p = presentStatus(status, snapshot != nullptr && snapshot->robotConnected());
    statusChip_->setStatus(p.tone, p.chipText, p.detail);
    viewport_->overlay()->setPresentation(p);
    refreshControls(snapshot.get());

    const bool available = status.process == RobotProcessState::Running && status.link == LinkState::Connected;
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
    startAction_->setEnabled(c.canStart);
    stopAction_->setEnabled(c.canStop);
    restartAction_->setEnabled(c.canRestart);
    resetAction_->setEnabled(c.canReset);
    pauseAction_->setEnabled(c.canPause);
    stepAction_->setEnabled(c.canStep);
    speedCombo_->setEnabled(c.canPause);
    if (pauseAction_->isChecked() != paused) {
        syncingPause_ = true;
        pauseAction_->setChecked(paused);
        syncingPause_ = false;
    }
    pauseAction_->setText(paused ? tr("Resume") : tr("Pause"));
}

void RunPage::pollNow() {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const double dt = lastPollMs_ == 0 ? 0.0 : std::clamp(static_cast<double>(now - lastPollMs_) / 1000.0, 0.0, 0.25);
    lastPollMs_ = now;

    const auto snapshot = backend_.latestSnapshot();
    const sdk::ISimSnapshot* s = snapshot.get();
    if (s != nullptr) ds_.onSimTime(s->simTimeUs());

    scene_.update(s, dt);
    hud_.update(s);
    mechanisms_.update(s);
    refreshControls(s);

    const StatusPresentation p = presentStatus(backend_.status(), s != nullptr && s->robotConnected());
    if (p.chipText != statusChip_->accessibleName()) statusChip_->setStatus(p.tone, p.chipText, p.detail);

    if (stats_.update(s, now) || now - lastLabelMs_ > 250) {
        lastLabelMs_ = now;
        refreshStatsLabels();
    }
}

void RunPage::refreshStatsLabels() {
    const SimStatsValues& v = stats_.values();
    if (!v.hasData) {
        simTimeLabel_->setText(QStringLiteral("—"));
        rateLabel_->clear();
        batteryLabel_->clear();
        extraLabel_->clear();
        return;
    }
    simTimeLabel_->setText(tr("sim %1%2").arg(formatSimTime(v.simTimeS), v.paused ? tr(" (paused)") : QString()));
    rateLabel_->setText(tr("%1 Hz · %2×").arg(v.tickRateHz, 0, 'f', 0).arg(v.realtimeFactor, 0, 'f', 2));
    batteryLabel_->setText(tr("%1 V").arg(v.batteryV, 0, 'f', 1));
    const QColor c = v.batteryV < 7.0 ? toneColor(Tone::Error) : (v.batteryV < 9.0 ? toneColor(Tone::Warning) : palette().color(QPalette::Text));
    QPalette pal = batteryLabel_->palette();
    pal.setColor(QPalette::Disabled, QPalette::WindowText, c);
    batteryLabel_->setPalette(pal);
    extraLabel_->setText(v.entities > 0 ? tr("%1 ms · %n entities", nullptr, v.entities).arg(v.physicsStepMs, 0, 'f', 2)
                                        : tr("%1 ms").arg(v.physicsStepMs, 0, 'f', 2));
}

void RunPage::publishDriverStation() {
    const auto& js = joysticks_.sample();
    ds_.publish(js, QDateTime::currentMSecsSinceEpoch());
}

bool RunPage::handleKey(int qtKey, bool down) {
    if (isModifier(qtKey)) return false;
    if (down && qtKey == Qt::Key_Space && !joysticks_.learning()) return false;   // Space is the disable shortcut
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
