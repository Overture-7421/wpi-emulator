#pragma once

#include <QWidget>

#include "driver_station_model.h"
#include "joystick_manager.h"

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
class QTimer;

namespace wpiemu::app::runview {

class JoystickEditor;
class StatusChip;

// Built-in Driver Station: enable / disable / e-stop, mode, alliance, match timer and the joystick editor.
class DriverStationPanel : public QWidget {
    Q_OBJECT
public:
    DriverStationPanel(DriverStationModel& model, JoystickManager& joysticks, QWidget* parent = nullptr);

    // Space disables the robot immediately (like the real DS); Ctrl+Space latches the emergency stop.
    QAction* disableAction() const { return disableAction_; }
    QAction* eStopAction() const { return eStopAction_; }
    JoystickEditor* joystickEditor() const { return editor_; }
    QPushButton* enableButton() const { return enable_; }
    QPushButton* disableButton() const { return disable_; }
    QPushButton* eStopButton() const { return eStop_; }
    QPushButton* startMatchButton() const { return startMatch_; }

    void setRobotConnected(bool connected);
    void refresh();   // model -> widgets; also driven by the model's changed() signal

private:
    void refreshClock();

    DriverStationModel& model_;
    JoystickManager& joysticks_;
    QPushButton* enable_;
    QPushButton* disable_;
    QPushButton* eStop_;
    QRadioButton* modeButtons_[3];
    QRadioButton* redAlliance_;
    QRadioButton* blueAlliance_;
    QComboBox* station_;
    QPushButton* startMatch_;
    QPushButton* stopMatch_;
    QCheckBox* sequence_;
    QDoubleSpinBox* autoSeconds_;
    QDoubleSpinBox* teleopSeconds_;
    QLabel* clock_;
    QLabel* phase_;
    QLineEdit* message_;
    QLabel* hint_;
    StatusChip* robotChip_;
    StatusChip* joystickChip_;
    QTabWidget* tabs_;
    JoystickEditor* editor_;
    QAction* disableAction_;
    QAction* eStopAction_;
    QTimer* clockTimer_;
    bool updating_ = false;
};

}  // namespace wpiemu::app::runview
