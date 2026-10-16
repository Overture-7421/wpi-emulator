#pragma once

#include <QWidget>

#include "driver_station_model.h"
#include "joystick_manager.h"
#include "sim_stats.h"

class QAction;
class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QTabWidget;
class QDialog;
class QTimer;
class QToolButton;

namespace wpiemu::app::runview {

class JoystickEditor;
class StatusChip;

// Built-in Driver Station: mode (Auto / Teleop / Practice / Test), enable / disable / e-stop, match clock,
// alliance and the joystick editor.
class DriverStationPanel : public QWidget {
    Q_OBJECT
public:
    DriverStationPanel(DriverStationModel& model, JoystickManager& joysticks, QWidget* parent = nullptr);

    // Enter disables the robot immediately and Space latches the emergency stop (like the real DS).
    QAction* disableAction() const { return disableAction_; }
    QAction* eStopAction() const { return eStopAction_; }
    JoystickEditor* joystickEditor() const { return editor_; }
    QPushButton* enableButton() const { return enable_; }
    QPushButton* eStopButton() const { return eStop_; }
    QPushButton* modeButton(DsMode mode) const { return modeButtons_[static_cast<int>(mode)]; }

    // Opens the Setup (0) or Joysticks (1) popup.
    void showSettings(int tab);
    QDialog* settingsDialog() const { return settingsDialog_; }
    void setRobotConnected(bool connected);
    void setStats(const SimStatsValues& values);   // sim time, rates, battery and physics load for the stats group
    void refresh();   // model -> widgets; also driven by the model's changed() signal

signals:
    // Match phase and remaining time ("Teleop  2:15"); empty when no Practice match is running.
    void matchClockChanged(const QString& text);

private:
    void refreshClock();
    void refreshLiveMode();

    DriverStationModel& model_;
    JoystickManager& joysticks_;
    QPushButton* enable_;
    QPushButton* eStop_;
    QPushButton* modeButtons_[4];
    QRadioButton* redAlliance_;
    QRadioButton* blueAlliance_;
    QComboBox* station_;
    QDoubleSpinBox* autoSeconds_;
    QDoubleSpinBox* gapSeconds_;
    QDoubleSpinBox* teleopSeconds_;
    QLineEdit* message_;
    QString lastClock_;
    QLabel* statValues_[6];
    StatusChip* robotChip_;
    StatusChip* joystickChip_;
    QTabWidget* tabs_;
    QDialog* settingsDialog_;
    QToolButton* settingsButton_;
    JoystickEditor* editor_;
    QAction* disableAction_;
    QAction* eStopAction_;
    QTimer* clockTimer_;
    bool updating_ = false;
};

}  // namespace wpiemu::app::runview
