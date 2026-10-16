#pragma once

#include <QWidget>

#include "common/robot_basics.h"
#include "common/ui_resources.h"
#include "common/unit_system.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app {

// Top-down schematic of the robot being described (frame, bumpers, wheels, driving direction and
// dimensions). It draws the very RobotConfig the wizard will save, so what you see is what you get.
class RobotSchematicView : public QWidget {
    Q_OBJECT
public:
    explicit RobotSchematicView(const UiResources& ui, QWidget* parent = nullptr);

    void setBasics(const RobotBasics& basics, const UnitFormatter& units);
    const sdk::RobotConfig& model() const { return model_; }

    QSize sizeHint() const override { return {380, 440}; }
    QSize minimumSizeHint() const override { return {240, 280}; }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    const UiResources& ui_;
    RobotBasics basics_;
    UnitFormatter units_;
    sdk::RobotConfig model_;
};

}  // namespace wpiemu::app
