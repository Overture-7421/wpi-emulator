#pragma once

// The join screen: the invite, the robot file, a display name and, under Advanced, a direct address and the relay. It only
// edits a JoinFormInput and shows what the controller says back (field errors, the robot report, the connection state).

#include <QWidget>

#include "client/client_controller.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QToolButton;

namespace wpiemu::app::client {

class JoinPage : public QWidget {
    Q_OBJECT
public:
    explicit JoinPage(ClientController& controller, QWidget* parent = nullptr);

    // What the fields hold right now.
    JoinFormInput input() const;
    void setInvite(const QString& text);

    QLineEdit* inviteEdit() const { return invite_; }
    QLineEdit* nameEdit() const { return name_; }
    QLineEdit* robotEdit() const { return robot_; }
    QLineEdit* directEdit() const { return direct_; }
    QComboBox* relayCombo() const { return relayChoice_; }
    QLineEdit* customRelayEdit() const { return customRelay_; }
    QPushButton* joinButton() const { return join_; }
    QPushButton* cancelButton() const { return cancel_; }
    QToolButton* advancedToggle() const { return advancedToggle_; }
    QLabel* statusLabel() const { return status_; }
    QLabel* robotReport() const { return robotReport_; }
    QLabel* fieldError(JoinField field) const;

private slots:
    void browse();
    void submit();
    void onRobotEdited();
    void refreshRobot();
    void refreshState();
    void refreshErrors();
    void refreshRelay();
    void storeControls();

private:
    void build();
    void showErrors(const std::vector<JoinFieldError>& errors);

    ClientController& controller_;
    QLineEdit* invite_ = nullptr;
    QLineEdit* name_ = nullptr;
    QLineEdit* robot_ = nullptr;
    QPushButton* browse_ = nullptr;
    QLabel* robotReport_ = nullptr;
    QLineEdit* direct_ = nullptr;
    QToolButton* advancedToggle_ = nullptr;
    QWidget* advanced_ = nullptr;
    QComboBox* relayChoice_ = nullptr;
    QLineEdit* customRelay_ = nullptr;
    QLabel* relayAddress_ = nullptr;
    QDoubleSpinBox* deadband_ = nullptr;
    QDoubleSpinBox* linearScale_ = nullptr;
    QCheckBox* fieldRelative_ = nullptr;
    QLabel* status_ = nullptr;
    QProgressBar* busy_ = nullptr;
    QPushButton* join_ = nullptr;
    QPushButton* cancel_ = nullptr;
    QLabel* errors_[5] = {};
    bool showErrors_ = false;
};

}  // namespace wpiemu::app::client
