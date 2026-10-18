#pragma once

#include <QString>

#include <string>
#include <vector>

#include "common/project_inspector.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/game/game_module.h"

namespace wpiemu::app {

enum class Alliance { Red, Blue };

struct MotorChoice {
    QString key;     // stable id persisted in the wizard state
    QString label;   // shown in the UI
    sdk::MotorSpec spec;
};

// Motors offered in the wizard (same data as WPILib's DCMotor catalog).
const std::vector<MotorChoice>& motorCatalog();
const MotorChoice* findMotor(const QString& key);
// Best catalog match for a spec (by name), nullptr when the spec is custom.
const MotorChoice* findMotorBySpec(const sdk::MotorSpec& spec);

// The handful of numbers a team knows off the top of their head. Everything is SI; the UI converts.
struct RobotBasics {
    QString name = QStringLiteral("Robot");
    std::string drivetrain = sdk::drivetrain_kind::kSwerve;
    double frameLengthM = 0.6858;       // 27 in, frame perimeter without bumpers (along +X)
    double frameWidthM = 0.6858;        // along +Y
    double massKg = 54.0;               // robot with battery and bumpers (~119 lb)
    int wheelsPerSide = 3;              // tank only; swerve and mecanum always have 4 wheels
    double wheelDiameterM = 0.1016;     // 4 in
    QString driveMotor = QStringLiteral("kraken_x60");
    int motorsPerWheel = 1;
    QString steerMotor = QStringLiteral("neo550");   // swerve only
    double driveGearRatio = 6.75;       // motor : wheel
    double steerGearRatio = 12.8;       // swerve only
    double bumperThicknessM = 0.0826;   // 3.25 in
    Alliance alliance = Alliance::Red;
    int station = 2;                    // driver station 1..3

    int wheelCount() const;
    bool isSwerve() const { return drivetrain == sdk::drivetrain_kind::kSwerve; }
    bool isTank() const { return drivetrain == sdk::drivetrain_kind::kTank; }
    bool isMecanum() const { return drivetrain == sdk::drivetrain_kind::kMecanum; }

    bool operator==(const RobotBasics&) const = default;
};

// Sensible starting values when the user switches the drivetrain type.
RobotBasics defaultBasicsFor(const std::string& drivetrainKind);

// Range checks with plain-language messages. Errors block the wizard, warnings do not.
std::vector<sdk::Issue> validateBasics(const RobotBasics& basics);
bool hasErrors(const std::vector<sdk::Issue>& issues);

struct RobotBuildContext {
    QString projectPath;
    QString language;
    int wpilibYear = 0;
    QString gradleRioVersion;
    std::int64_t stepUs = 5000;
    const sdk::IGameModule* gameModule = nullptr;   // null = core-only mode
};

// Turns the basics into a complete RobotConfig: chassis, wheels with motors, bumpers, battery and
// sim settings, plus the selected game plugin's default section. Mappings stay empty: they are
// filled in by device discovery.
sdk::RobotConfig buildRobotConfig(const RobotBasics& basics, const RobotBuildContext& context);

// Inverse used to re-open the wizard on an existing config (unknown pieces keep defaults).
RobotBasics basicsFromConfig(const sdk::RobotConfig& config);

// Re-running the wizard on a project that already has a model must not throw the editor's work away:
// the basics replace the chassis size/mass, bumpers and the drivetrain wheels; mechanisms, zones, the
// vision settings (cameras, AprilTag layout, PhotonLib UUID override), the start pose, robot mappings,
// battery, plugin sections and any hand-added chassis primitives are kept, and drivetrain mappings
// survive for wheels that still exist. With no existing model this is buildRobotConfig().
sdk::RobotConfig updateRobotConfig(const sdk::RobotConfig* existing, const RobotBasics& basics, const RobotBuildContext& context);

}  // namespace wpiemu::app
