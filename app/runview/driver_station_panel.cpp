#include "driver_station_panel.h"

#include <QAction>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QRadioButton>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "hud_model.h"
#include "joystick_editor.h"
#include "status_chip.h"
#include "ui/widget_support.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::runview {
namespace {

QIcon cogIcon(const QColor& color) {
    const qreal dpr = 2.0;
    QPixmap pm(QSize(20, 20) * dpr);
    pm.setDevicePixelRatio(dpr);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.translate(10, 10);
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    QPainterPath gear;
    constexpr int kTeeth = 8;
    for (int i = 0; i < kTeeth * 2; ++i) {
        // Alternate outer/inner radius with a short flat on each tooth.
        const double r = (i % 2 == 0) ? 9.0 : 6.6;
        const double a0 = (i - 0.35) * sdk::kPi / kTeeth, a1 = (i + 0.35) * sdk::kPi / kTeeth;
        const QPointF p0(r * std::cos(a0), r * std::sin(a0)), p1(r * std::cos(a1), r * std::sin(a1));
        if (i == 0) gear.moveTo(p0); else gear.lineTo(p0);
        gear.lineTo(p1);
    }
    gear.closeSubpath();
    QPainterPath hole;
    hole.addEllipse(QPointF(0, 0), 2.9, 2.9);
    p.drawPath(gear.subtracted(hole));
    return QIcon(pm);
}

QString bigButtonStyle(const QColor& c) {
    return QStringLiteral(
               "QPushButton { background: rgba(%1,%2,%3,30); border: 2px solid rgb(%1,%2,%3); border-radius: 8px; font-weight: bold;"
               " font-size: 15px; padding: 6px; }"
               "QPushButton:hover { background: rgba(%1,%2,%3,70); }"
               "QPushButton:checked { background: rgb(%1,%2,%3); color: white; }"
               "QPushButton:disabled { border-color: rgba(%1,%2,%3,70); background: transparent; }")
        .arg(c.red())
        .arg(c.green())
        .arg(c.blue());
}

}  // namespace

DriverStationPanel::DriverStationPanel(DriverStationModel& model, JoystickManager& joysticks, QWidget* parent)
    : QWidget(parent), model_(model), joysticks_(joysticks) {
    setObjectName(QStringLiteral("runview.driverStation"));

    auto* title = new QLabel(tr("Driver Station"), this);
    QFont bold = title->font();
    bold.setBold(true);
    bold.setPointSizeF(bold.pointSizeF() * 1.15);
    title->setFont(bold);
    robotChip_ = new StatusChip(this);
    joystickChip_ = new StatusChip(this);

    enable_ = new QPushButton(tr("Enable"), this);
    enable_->setCheckable(true);
    enable_->setMinimumHeight(56);
    enable_->setStyleSheet(bigButtonStyle(toneColor(Tone::Success)));
    eStop_ = new QPushButton(tr("EMERGENCY STOP"), this);
    eStop_->setMinimumHeight(34);
    eStop_->setStyleSheet(QStringLiteral("QPushButton { background: #b3261e; color: white; font-weight: bold; border-radius: 6px; }"
                                         "QPushButton:hover { background: #d3342b; }"
                                         "QPushButton:disabled { background: #5c2a27; color: #aaa; }"));

    // Mode selector: Auto / Teleop / Practice / Test. Practice runs auto -> gap -> teleop on Enable.
    auto* modeGroup = new QButtonGroup(this);
    modeGroup->setExclusive(true);
    struct ModeInfo {
        DsMode mode;
        const char* name;
        const char* tip;
    };
    const ModeInfo modes[4] = {
        {DsMode::Auto, QT_TR_NOOP("Auto"), QT_TR_NOOP("Autonomous: Enable runs until you disable")},
        {DsMode::Teleop, QT_TR_NOOP("Teleop"), QT_TR_NOOP("Teleoperated: Enable runs until you disable")},
        {DsMode::Practice, QT_TR_NOOP("Practice"), QT_TR_NOOP("Enable runs autonomous, a short gap, then teleop on the match clock")},
        {DsMode::Test, QT_TR_NOOP("Test"), QT_TR_NOOP("Test mode: Enable runs until you disable")},
    };
    auto* modeRow = new QHBoxLayout;
    modeRow->setSpacing(4);
    for (int i = 0; i < 4; ++i) {
        modeButtons_[i] = new QPushButton(tr(modes[i].name), this);
        modeButtons_[i]->setCheckable(true);
        modeButtons_[i]->setMinimumHeight(30);
        // The app theme has no checked state for push buttons, so the selected mode needs its own highlight.
        // During a Practice match the segment that is running right now is marked "live" (green).
        modeButtons_[i]->setStyleSheet(QStringLiteral("QPushButton:checked { background: palette(highlight); color: palette(highlighted-text); font-weight: 600; }"
                                                      "QPushButton[live=\"true\"] { background: #2e9e5b; color: white; font-weight: 600; border-color: #2e9e5b; }"));
        modeButtons_[i]->setToolTip(tr(modes[i].tip));
        modeGroup->addButton(modeButtons_[i], static_cast<int>(modes[i].mode));
        modeRow->addWidget(modeButtons_[i], 1);
    }

    settingsButton_ = new QToolButton(this);
    settingsButton_->setObjectName(QStringLiteral("runview.dsSettingsButton"));
    settingsButton_->setIcon(cogIcon(palette().color(QPalette::WindowText)));
    settingsButton_->setIconSize(QSize(20, 20));
    settingsButton_->setAutoRaise(true);
    settingsButton_->setCursor(Qt::PointingHandCursor);
    settingsButton_->setToolTip(tr("Driver Station settings: alliance, station, game message, Practice timings and joysticks"));
    settingsButton_->setAccessibleName(tr("Driver Station settings"));

    enable_->setMinimumHeight(48);
    eStop_->setMinimumHeight(48);
    auto* bigRow = new QHBoxLayout;
    bigRow->addWidget(enable_, 3);
    bigRow->addWidget(eStop_, 2);

    auto* head = new QHBoxLayout;
    head->addWidget(title);
    head->addStretch(1);
    head->setSpacing(6);
    head->addWidget(robotChip_);
    head->addWidget(joystickChip_);
    head->addWidget(settingsButton_);

    // ---- Setup tab ----
    redAlliance_ = new QRadioButton(tr("Red"), this);
    blueAlliance_ = new QRadioButton(tr("Blue"), this);
    redAlliance_->setStyleSheet(QStringLiteral("QRadioButton { color: #e5484d; font-weight: bold; }"));
    blueAlliance_->setStyleSheet(QStringLiteral("QRadioButton { color: #4f8cff; font-weight: bold; }"));
    station_ = new QComboBox(this);
    for (int s = 1; s <= 3; ++s) station_->addItem(tr("Station %1").arg(s), s);
    auto* allianceRow = new QHBoxLayout;
    allianceRow->addWidget(redAlliance_);
    allianceRow->addWidget(blueAlliance_);
    allianceRow->addWidget(station_, 1);

    message_ = new QLineEdit(this);
    message_->setPlaceholderText(tr("Game-specific message (e.g. field data)"));
    message_->setClearButtonEnabled(true);
    message_->setMaxLength(64);

    auto makeSeconds = [this](const SecondsRange& range) {
        auto* box = new QDoubleSpinBox(this);
        box->setRange(range.min, range.max);
        box->setDecimals(0);
        box->setSuffix(tr(" s"));
        return box;
    };
    autoSeconds_ = makeSeconds(kAutoSecondsRange);
    gapSeconds_ = makeSeconds(kGapSecondsRange);
    teleopSeconds_ = makeSeconds(kTeleopSecondsRange);

    auto* setupTab = new QWidget(this);
    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignLeft);
    form->addRow(tr("Alliance"), allianceRow);
    form->addRow(tr("Game message"), message_);

    auto* practiceBox = new QGroupBox(tr("Practice match"), this);
    practiceBox->setObjectName(QStringLiteral("runview.dsPracticeGroup"));
    auto* practiceForm = new QFormLayout(practiceBox);
    practiceForm->setLabelAlignment(Qt::AlignLeft);
    practiceForm->addRow(tr("Autonomous"), autoSeconds_);
    practiceForm->addRow(tr("Gap"), gapSeconds_);
    practiceForm->addRow(tr("Teleop"), teleopSeconds_);
    auto* setupLayout = new QVBoxLayout(setupTab);
    setupLayout->addLayout(form);
    setupLayout->addWidget(practiceBox);
    setupLayout->addStretch(1);

    editor_ = new JoystickEditor(joysticks_, this);
    settingsDialog_ = new QDialog(this);
    settingsDialog_->setObjectName(QStringLiteral("runview.dsSettings"));
    settingsDialog_->setWindowTitle(tr("Driver Station settings"));
    tabs_ = new QTabWidget(settingsDialog_);
    tabs_->addTab(setupTab, tr("Setup"));
    tabs_->addTab(editor_, tr("Joysticks"));
    auto* dialogLayout = new QVBoxLayout(settingsDialog_);
    dialogLayout->addWidget(tabs_);
    settingsDialog_->resize(560, 600);
    connect(settingsButton_, &QToolButton::clicked, this, [this] { showSettings(0); });

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 6, 10, 6);
    layout->setSpacing(6);
    layout->addLayout(head);
    layout->addLayout(modeRow);
    layout->addLayout(bigRow);

    // Simulation stats: a distinct, framed group below the Enable / E-STOP row.
    auto* statsFrame = new QFrame(this);
    statsFrame->setObjectName(QStringLiteral("runview.dsStats"));
    statsFrame->setProperty("statsGroup", true);
    statsFrame->setStyleSheet(QStringLiteral("QFrame[statsGroup=\"true\"] { background: palette(base); border: 1px solid palette(mid); border-radius: 6px; }"
                                             "QFrame[statsGroup=\"true\"] QLabel { border: none; background: transparent; }"));
    auto* statsRow = new QHBoxLayout(statsFrame);
    statsRow->setContentsMargins(10, 4, 10, 4);
    statsRow->setSpacing(6);
    struct StatInfo {
        const char* caption;
        const char* tip;
    };
    const StatInfo stats[6] = {
        {QT_TR_NOOP("Sim time"), QT_TR_NOOP("Simulated time")},
        {QT_TR_NOOP("Tick rate"), QT_TR_NOOP("Simulation ticks per second")},
        {QT_TR_NOOP("Speed"), QT_TR_NOOP("Simulation speed relative to real time")},
        {QT_TR_NOOP("Battery"), QT_TR_NOOP("Simulated battery voltage")},
        {QT_TR_NOOP("Physics"), QT_TR_NOOP("Physics step time")},
        {QT_TR_NOOP("Entities"), QT_TR_NOOP("Instanced entities (game pieces)")},
    };
    const QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    for (int i = 0; i < 6; ++i) {
        auto* cell = new QVBoxLayout;
        cell->setSpacing(0);
        auto* caption = ui::captionLabel(tr(stats[i].caption), statsFrame);
        statValues_[i] = new QLabel(QStringLiteral("—"), statsFrame);
        statValues_[i]->setFont(mono);
        cell->addWidget(caption);
        cell->addWidget(statValues_[i]);
        statsRow->addLayout(cell, 1);
        caption->setToolTip(tr(stats[i].tip));
        statValues_[i]->setToolTip(tr(stats[i].tip));
    }
    layout->addWidget(statsFrame);
    layout->addStretch(1);

    disableAction_ = new QAction(tr("Disable robot"), this);
    disableAction_->setShortcuts({QKeySequence(Qt::Key_Return), QKeySequence(Qt::Key_Enter)});
    disableAction_->setShortcutContext(Qt::WindowShortcut);
    eStopAction_ = new QAction(tr("Emergency stop"), this);
    eStopAction_->setShortcut(QKeySequence(Qt::Key_Space));
    eStopAction_->setShortcutContext(Qt::WindowShortcut);
    addAction(disableAction_);
    addAction(eStopAction_);

    // ---- wiring (widgets -> model) ----
    connect(enable_, &QPushButton::clicked, this, [this] {
        if (model_.enabled()) model_.disable();
        else model_.setEnabled(true);
        refresh();
    });
    connect(disableAction_, &QAction::triggered, this, [this] { model_.disable(); });
    connect(eStopAction_, &QAction::triggered, this, [this] { model_.eStop(); });
    connect(eStop_, &QPushButton::clicked, this, [this] {
        if (model_.eStopped()) model_.clearEStop();
        else model_.eStop();
    });
    connect(modeGroup, &QButtonGroup::idClicked, this, [this](int id) {
        if (!updating_) model_.setDsMode(static_cast<DsMode>(id));
        refresh();
    });
    connect(redAlliance_, &QRadioButton::toggled, this, [this](bool on) {
        if (on && !updating_) model_.setAlliance(sdk::Alliance::Red);
    });
    connect(blueAlliance_, &QRadioButton::toggled, this, [this](bool on) {
        if (on && !updating_) model_.setAlliance(sdk::Alliance::Blue);
    });
    connect(station_, &QComboBox::activated, this, [this](int i) { model_.setStation(station_->itemData(i).toInt()); });
    connect(message_, &QLineEdit::textEdited, this, [this](const QString& t) { model_.setGameMessage(t); });
    auto matchSettingsChanged = [this] {
        if (updating_) return;
        MatchSettings s = model_.matchSettings();
        s.autoSeconds = autoSeconds_->value();
        s.gapSeconds = gapSeconds_->value();
        s.teleopSeconds = teleopSeconds_->value();
        model_.setMatchSettings(s);
    };
    connect(autoSeconds_, &QDoubleSpinBox::editingFinished, this, matchSettingsChanged);
    connect(gapSeconds_, &QDoubleSpinBox::editingFinished, this, matchSettingsChanged);
    connect(teleopSeconds_, &QDoubleSpinBox::editingFinished, this, matchSettingsChanged);

    connect(&model_, &DriverStationModel::changed, this, &DriverStationPanel::refresh);
    connect(&joysticks_, &JoystickManager::slotChanged, this, [this] { refresh(); });
    connect(&joysticks_, &JoystickManager::sourcesChanged, this, [this] { refresh(); });

    clockTimer_ = new QTimer(this);
    clockTimer_->setInterval(100);
    connect(clockTimer_, &QTimer::timeout, this, &DriverStationPanel::refreshClock);
    clockTimer_->start();

    refresh();
}

void DriverStationPanel::setStats(const SimStatsValues& v) {
    if (!v.hasData) {
        statValues_[0]->setText(QStringLiteral("—"));
        for (int i = 1; i < 6; ++i) statValues_[i]->clear();
        return;
    }
    statValues_[0]->setText(v.paused ? tr("%1 (paused)").arg(formatSimTime(v.simTimeS)) : formatSimTime(v.simTimeS));
    statValues_[1]->setText(tr("%1 Hz").arg(v.tickRateHz, 0, 'f', 0));
    statValues_[2]->setText(tr("%1×").arg(v.realtimeFactor, 0, 'f', 2));
    statValues_[3]->setText(tr("%1 V").arg(v.batteryV, 0, 'f', 1));
    const QColor c = v.batteryV < 7.0 ? toneColor(Tone::Error) : (v.batteryV < 9.0 ? toneColor(Tone::Warning) : palette().color(QPalette::WindowText));
    statValues_[3]->setStyleSheet(QStringLiteral("color: %1;").arg(c.name()));
    statValues_[4]->setText(tr("%1 ms").arg(v.physicsStepMs, 0, 'f', 2));
    const QString physicsTip = tr("Physics step time");
    const QString threadsTip = v.physicsWorkers > 1 ? tr("%1 (%2 threads)").arg(physicsTip).arg(v.physicsWorkers) : physicsTip;
    statValues_[4]->setToolTip(threadsTip);
    statValues_[5]->setText(QString::number(v.entities));
}

void DriverStationPanel::setRobotConnected(bool connected) {
    robotChip_->setStatus(connected ? Tone::Success : Tone::Neutral,     connected ? tr("Simulating") : tr("No simulation"),
                              connected ? tr("The robot program is running in the simulation") : tr("Waiting for the robot to connect"));
    robotChip_->setVisible(!connected);
    refresh();
}

void DriverStationPanel::refresh() {
    updating_ = true;
    const bool enabled = model_.enabled();
    enable_->setChecked(enabled);
    enable_->setText(tr("Enabled"));
    enable_->setToolTip(enabled ? tr("Disable the robot immediately") : tr("Enable the robot in the selected mode"));
    enable_->setEnabled(model_.canEnable() || enabled);
    eStop_->setText(model_.eStopped() ? tr("E-STOPPED") : tr("EMERGENCY STOP"));
    eStop_->setToolTip(model_.eStopped() ? tr("Clear the emergency stop so the robot can be enabled again")
                                         : tr("Latch an emergency stop (Space). Cleared by clearing it here or restarting the robot."));
    eStop_->setEnabled(model_.robotAvailable() || model_.eStopped());

    modeButtons_[std::clamp(static_cast<int>(model_.dsMode()), 0, 3)]->setChecked(true);
    redAlliance_->setChecked(model_.alliance() == sdk::Alliance::Red);
    blueAlliance_->setChecked(model_.alliance() == sdk::Alliance::Blue);
    station_->setCurrentIndex(model_.station() - 1);
    if (!message_->hasFocus()) message_->setText(model_.gameMessage());
    const MatchSettings& ms = model_.matchSettings();
    autoSeconds_->setValue(ms.autoSeconds);
    teleopSeconds_->setValue(ms.teleopSeconds);
    gapSeconds_->setValue(ms.gapSeconds);

    int connected = 0;
    for (int i = 0; i < joy::kSlotCount; ++i)
        if (joysticks_.slotConnected(i) && !joysticks_.slot(i).sourceId.isEmpty()) ++connected;
    joystickChip_->setStatus(connected > 0 ? Tone::Success : Tone::Neutral, tr("Joysticks: %1").arg(connected),
                             tr("Joystick ports with a connected device"));
    if (!model_.robotAvailable()) {
        robotChip_->setStatus(Tone::Neutral, tr("No simulation"), tr("Waiting for the robot to connect"));
        robotChip_->setVisible(true);
    }
    updating_ = false;
    refreshClock();
}

void DriverStationPanel::showSettings(int tab) {
    tabs_->setCurrentIndex(std::clamp(tab, 0, 1));
    settingsDialog_->show();
    settingsDialog_->raise();
    settingsDialog_->activateWindow();
}

void DriverStationPanel::refreshLiveMode() {
    const MatchSegment* seg = model_.matchActive() ? model_.match().current() : nullptr;
    for (int i = 0; i < 4; ++i) {
        const bool live = seg != nullptr && seg->enabled &&
                          ((i == static_cast<int>(DsMode::Auto) && seg->mode == sdk::proto::RobotMode::Autonomous) ||
                           (i == static_cast<int>(DsMode::Teleop) && seg->mode == sdk::proto::RobotMode::Teleop));
        if (modeButtons_[i]->property("live").toBool() == live) continue;
        modeButtons_[i]->setProperty("live", live);
        modeButtons_[i]->style()->unpolish(modeButtons_[i]);
        modeButtons_[i]->style()->polish(modeButtons_[i]);
    }
}

void DriverStationPanel::refreshClock() {
    refreshLiveMode();
    const QString text = model_.matchActive()
                             ? QStringLiteral("%1  %2").arg(model_.matchPhaseText(), formatClock(model_.match().remainingS()))
                             : QString();
    if (text == lastClock_) return;
    lastClock_ = text;
    emit matchClockChanged(text);
}

}  // namespace wpiemu::app::runview
