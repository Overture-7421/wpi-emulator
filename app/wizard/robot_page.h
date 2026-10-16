#pragma once

#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QToolButton>

#include "common/ui_helpers.h"
#include "wizard/robot_schematic_view.h"
#include "wizard/wizard_page.h"
#include "wizard/wizard_view_model.h"

namespace wpiemu::app {

// Step 3: the few numbers that define the robot, with a live schematic. Widgets only translate
// between display units and the view-model's SI RobotBasics.
class RobotPage : public WizardPage {
    Q_OBJECT
public:
    RobotPage(WizardViewModel& model, const UiResources& ui, QWidget* parent = nullptr);

    QString title() const override { return tr("Describe your robot"); }
    QString subtitle() const override { return tr("Rough numbers are fine: everything can be refined in the editor later."); }
    QWidget* initialFocus() const override { return nameEdit_; }
    void onEnter() override { loadFromModel(); }

    RobotSchematicView* schematic() const { return schematic_; }
    Banner* banner() const { return banner_; }
    QLineEdit* nameEdit() const { return nameEdit_; }
    QDoubleSpinBox* lengthSpin() const { return length_; }
    QDoubleSpinBox* widthSpin() const { return width_; }
    QDoubleSpinBox* massSpin() const { return mass_; }
    QDoubleSpinBox* wheelDiameterSpin() const { return wheelDiameter_; }
    QSpinBox* wheelsPerSideSpin() const { return wheelsPerSide_; }
    QComboBox* driveMotorCombo() const { return driveMotor_; }
    QComboBox* steerMotorCombo() const { return steerMotor_; }
    QToolButton* drivetrainButton(const std::string& kind) const;

private:
    void loadFromModel();
    void pushToModel();
    void applyUnits();
    void updateVisibility();
    QDoubleSpinBox* makeSpin(double decimals);

    WizardViewModel& model_;
    const UiResources& ui_;
    bool loading_ = false;

    QLineEdit* nameEdit_;
    QButtonGroup* kindGroup_;
    QToolButton* swerveButton_;
    QToolButton* tankButton_;
    QToolButton* mecanumButton_;
    QDoubleSpinBox* length_;
    QDoubleSpinBox* width_;
    QDoubleSpinBox* mass_;
    QDoubleSpinBox* wheelDiameter_;
    QSpinBox* wheelsPerSide_;
    QLabel* wheelCountLabel_;
    QComboBox* driveMotor_;
    QSpinBox* motorsPerWheel_;
    QDoubleSpinBox* driveRatio_;
    QComboBox* steerMotor_;
    QDoubleSpinBox* steerRatio_;
    QDoubleSpinBox* bumper_;
    QToolButton* redButton_;
    QToolButton* blueButton_;
    QComboBox* station_;
    QFormLayout* motorForm_;
    QFormLayout* dimensionForm_;
    Banner* banner_;
    RobotSchematicView* schematic_;
};

}  // namespace wpiemu::app
