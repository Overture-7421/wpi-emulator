#include "driver_station_panel.h"

#include <QAction>
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

#include "hud_model.h"
#include "joystick_editor.h"
#include "status_chip.h"

namespace wpiemu::app::runview {
namespace {

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
    disable_ = new QPushButton(tr("Disable\n(Space)"), this);
    disable_->setCheckable(true);
    disable_->setMinimumHeight(56);
    disable_->setStyleSheet(bigButtonStyle(toneColor(Tone::Error)));
    disable_->setToolTip(tr("Disable the robot immediately (Space)"));
    eStop_ = new QPushButton(tr("EMERGENCY STOP"), this);
    eStop_->setMinimumHeight(34);
    eStop_->setStyleSheet(QStringLiteral("QPushButton { background: #b3261e; color: white; font-weight: bold; border-radius: 6px; }"
                                         "QPushButton:hover { background: #d3342b; }"
                                         "QPushButton:disabled { background: #5c2a27; color: #aaa; }"));

    auto* modeGroup = new QButtonGroup(this);
    const QString modeNames[3] = {tr("Teleoperated"), tr("Autonomous"), tr("Test")};
    auto* modeRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        modeButtons_[i] = new QRadioButton(modeNames[i], this);
        modeGroup->addButton(modeButtons_[i], i);
        modeRow->addWidget(modeButtons_[i]);
    }
    modeRow->addStretch(1);

    auto* bigRow = new QHBoxLayout;
    bigRow->addWidget(enable_, 1);
    bigRow->addWidget(disable_, 1);

    auto* head = new QHBoxLayout;
    head->addWidget(title);
    head->addStretch(1);
    head->addWidget(robotChip_);
    head->addWidget(joystickChip_);

    hint_ = new QLabel(this);
    hint_->setWordWrap(true);
    hint_->setEnabled(false);

    // ---- Match tab ----
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

    clock_ = new QLabel(QStringLiteral("—"), this);
    QFont big = clock_->font();
    big.setPointSizeF(big.pointSizeF() * 2.2);
    big.setBold(true);
    clock_->setFont(big);
    phase_ = new QLabel(this);
    phase_->setEnabled(false);
    startMatch_ = new QPushButton(tr("Start match"), this);
    startMatch_->setToolTip(tr("Enable the robot and run the match clock"));
    stopMatch_ = new QPushButton(tr("Stop"), this);
    sequence_ = new QCheckBox(tr("Run autonomous, then teleop automatically"), this);
    sequence_->setToolTip(tr("Off: only the selected mode runs for its period"));
    autoSeconds_ = new QDoubleSpinBox(this);
    autoSeconds_->setRange(1, 600);
    autoSeconds_->setDecimals(0);
    autoSeconds_->setSuffix(tr(" s"));
    teleopSeconds_ = new QDoubleSpinBox(this);
    teleopSeconds_->setRange(1, 1200);
    teleopSeconds_->setDecimals(0);
    teleopSeconds_->setSuffix(tr(" s"));

    auto* matchButtons = new QHBoxLayout;
    matchButtons->addWidget(startMatch_, 1);
    matchButtons->addWidget(stopMatch_);
    auto* clockRow = new QHBoxLayout;
    clockRow->addWidget(clock_);
    clockRow->addWidget(phase_, 1, Qt::AlignVCenter);

    auto* matchTab = new QWidget(this);
    auto* form = new QFormLayout;
    form->setLabelAlignment(Qt::AlignLeft);
    form->addRow(tr("Alliance"), allianceRow);
    form->addRow(tr("Game message"), message_);
    form->addRow(tr("Autonomous"), autoSeconds_);
    form->addRow(tr("Teleop"), teleopSeconds_);
    auto* matchLayout = new QVBoxLayout(matchTab);
    matchLayout->addLayout(form);
    matchLayout->addWidget(sequence_);
    matchLayout->addLayout(clockRow);
    matchLayout->addLayout(matchButtons);
    matchLayout->addStretch(1);

    editor_ = new JoystickEditor(joysticks_, this);
    tabs_ = new QTabWidget(this);
    tabs_->addTab(matchTab, tr("Match"));
    tabs_->addTab(editor_, tr("Joysticks"));

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(8);
    layout->addLayout(head);
    layout->addLayout(bigRow);
    layout->addWidget(eStop_);
    layout->addWidget(hint_);
    layout->addLayout(modeRow);
    layout->addWidget(tabs_, 1);

    disableAction_ = new QAction(tr("Disable robot"), this);
    disableAction_->setShortcut(QKeySequence(Qt::Key_Space));
    disableAction_->setShortcutContext(Qt::WindowShortcut);
    eStopAction_ = new QAction(tr("Emergency stop"), this);
    eStopAction_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Space));
    eStopAction_->setShortcutContext(Qt::WindowShortcut);
    addAction(disableAction_);
    addAction(eStopAction_);

    // ---- wiring (widgets -> model) ----
    connect(enable_, &QPushButton::clicked, this, [this] { model_.setEnabled(true); refresh(); });
    connect(disable_, &QPushButton::clicked, this, [this] { model_.disable(); refresh(); });
    connect(disableAction_, &QAction::triggered, this, [this] { model_.disable(); });
    connect(eStopAction_, &QAction::triggered, this, [this] { model_.eStop(); });
    connect(eStop_, &QPushButton::clicked, this, [this] {
        if (model_.eStopped()) model_.clearEStop();
        else model_.eStop();
    });
    connect(modeGroup, &QButtonGroup::idClicked, this, [this](int id) {
        if (!updating_) model_.setMode(static_cast<sdk::proto::RobotMode>(id));
    });
    connect(redAlliance_, &QRadioButton::toggled, this, [this](bool on) {
        if (on && !updating_) model_.setAlliance(DriverStationModel::Alliance::Red);
    });
    connect(blueAlliance_, &QRadioButton::toggled, this, [this](bool on) {
        if (on && !updating_) model_.setAlliance(DriverStationModel::Alliance::Blue);
    });
    connect(station_, &QComboBox::activated, this, [this](int i) { model_.setStation(station_->itemData(i).toInt()); });
    connect(message_, &QLineEdit::textEdited, this, [this](const QString& t) { model_.setGameMessage(t); });
    connect(startMatch_, &QPushButton::clicked, this, [this] { model_.startMatch(); });
    connect(stopMatch_, &QPushButton::clicked, this, [this] { model_.stopMatch(); });
    auto matchSettingsChanged = [this] {
        if (updating_) return;
        MatchSettings s = model_.matchSettings();
        s.autoSeconds = autoSeconds_->value();
        s.teleopSeconds = teleopSeconds_->value();
        s.sequence = sequence_->isChecked();
        model_.setMatchSettings(s);
    };
    connect(autoSeconds_, &QDoubleSpinBox::editingFinished, this, matchSettingsChanged);
    connect(teleopSeconds_, &QDoubleSpinBox::editingFinished, this, matchSettingsChanged);
    connect(sequence_, &QCheckBox::toggled, this, matchSettingsChanged);

    connect(&model_, &DriverStationModel::changed, this, &DriverStationPanel::refresh);
    connect(&joysticks_, &JoystickManager::slotChanged, this, [this] { refresh(); });
    connect(&joysticks_, &JoystickManager::sourcesChanged, this, [this] { refresh(); });

    clockTimer_ = new QTimer(this);
    clockTimer_->setInterval(100);
    connect(clockTimer_, &QTimer::timeout, this, &DriverStationPanel::refreshClock);
    clockTimer_->start();

    refresh();
}

void DriverStationPanel::setRobotConnected(bool connected) {
    robotChip_->setStatus(connected ? Tone::Success : Tone::Neutral, connected ? tr("Robot code") : tr("No robot"),
                          connected ? tr("The robot program is connected") : tr("Start the robot to enable it"));
    refresh();
}

void DriverStationPanel::refresh() {
    updating_ = true;
    const bool enabled = model_.enabled();
    enable_->setChecked(enabled);
    disable_->setChecked(!enabled);
    enable_->setEnabled(model_.canEnable());
    eStop_->setText(model_.eStopped() ? tr("E-STOPPED — click to clear") : tr("EMERGENCY STOP"));
    eStop_->setToolTip(model_.eStopped() ? tr("Clear the emergency stop so the robot can be enabled again")
                                         : tr("Latch an emergency stop (Ctrl+Space). Cleared by clearing it here or restarting the robot."));
    eStop_->setEnabled(model_.robotAvailable() || model_.eStopped());

    modeButtons_[std::clamp(static_cast<int>(model_.mode()), 0, 2)]->setChecked(true);
    redAlliance_->setChecked(model_.alliance() == DriverStationModel::Alliance::Red);
    blueAlliance_->setChecked(model_.alliance() == DriverStationModel::Alliance::Blue);
    station_->setCurrentIndex(model_.station() - 1);
    if (!message_->hasFocus()) message_->setText(model_.gameMessage());
    const MatchSettings& ms = model_.matchSettings();
    autoSeconds_->setValue(ms.autoSeconds);
    teleopSeconds_->setValue(ms.teleopSeconds);
    sequence_->setChecked(ms.sequence);
    startMatch_->setEnabled(model_.canEnable());
    stopMatch_->setEnabled(model_.matchActive());

    if (!model_.robotAvailable()) hint_->setText(tr("Start the robot (and wait for it to connect) to enable it."));
    else if (model_.eStopped()) hint_->setText(tr("Emergency stop is latched. Clear it to enable the robot."));
    else hint_->setText({});
    hint_->setVisible(!hint_->text().isEmpty());

    int connected = 0;
    for (int i = 0; i < joy::kSlotCount; ++i)
        if (joysticks_.slotConnected(i) && !joysticks_.slot(i).sourceId.isEmpty()) ++connected;
    joystickChip_->setStatus(connected > 0 ? Tone::Success : Tone::Neutral, tr("Joysticks: %1").arg(connected),
                             tr("Joystick ports with a connected device"));
    if (!model_.robotAvailable()) robotChip_->setStatus(Tone::Neutral, tr("No robot"), tr("Start the robot to enable it"));
    updating_ = false;
    refreshClock();
}

void DriverStationPanel::refreshClock() {
    if (model_.matchActive()) {
        clock_->setText(formatClock(model_.match().remainingS()));
        phase_->setText(model_.matchPhaseText());
    } else {
        clock_->setText(QStringLiteral("—"));
        phase_->setText(model_.enabled() ? tr("Enabled (no match clock)") : tr("Disabled"));
    }
}

}  // namespace wpiemu::app::runview
