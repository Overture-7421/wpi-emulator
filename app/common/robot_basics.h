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

struct MotorChoice {
    QString key;     // the SDK motor table key
    QString label;   // shown in the UI
    sdk::MotorSpec spec;
};

// Every motor of the SDK table (sdk::motors::presets()), in table order.
const std::vector<MotorChoice>& motorCatalog();
const MotorChoice* findMotor(const QString& key);
// Best catalog match for a spec (by name), nullptr when the spec is custom.
const MotorChoice* findMotorBySpec(const sdk::MotorSpec& spec);

// What the wizard offers and accepts for each value: the spin-box limits and the ranges validateBasics() reports. They
// are presentation limits, always inside what core's ConfigValidator calls physical (positive, finite, motors per wheel
// up to sdk::kMaxMotorsPerAxis); test_robot_basics.cpp checks that every accepted value builds a model core accepts.
namespace basics_limits {

template <class T>
struct Range {
    T min;
    T max;
    constexpr bool excludes(T value) const { return value < min || value > max; }
};

inline constexpr Range<double> kFrameM{0.25, 1.5};              // each side, without bumpers
inline constexpr Range<double> kMassKg{5.0, 120.0};
inline constexpr Range<double> kWheelDiameterM{0.04, 0.25};
inline constexpr Range<double> kDriveRatio{0.5, 100.0};
inline constexpr Range<double> kSteerRatio{0.5, 200.0};
inline constexpr Range<double> kBumperM{0.0, 0.15};
inline constexpr Range<int> kWheelsPerSide{2, 4};
inline constexpr Range<int> kMotorsPerWheel{1, 4};
inline constexpr Range<int> kStation{1, 3};

}  // namespace basics_limits

// The handful of numbers a team knows off the top of their head. Everything is SI; the UI converts.
struct RobotBasics {
    QString name = QStringLiteral("Robot");
    std::string drivetrain = sdk::drivetrain_kind::kSwerve;
    double frameLengthM = 0.6858;       // 27 in, frame perimeter without bumpers (along +X)
    double frameWidthM = 0.6858;        // along +Y
    double massKg = 54.0;               // robot with battery and bumpers (~119 lb)
    int wheelsPerSide = 3;              // tank only; swerve and mecanum always have 4 wheels
    double wheelDiameterM = 2.0 * sdk::kDefaultWheelRadiusM;     // 4 in
    QString driveMotor = QStringLiteral("kraken_x60");
    int motorsPerWheel = 1;
    QString steerMotor = QStringLiteral("neo_550");  // swerve only
    double driveGearRatio = 6.75;       // motor : wheel
    double steerGearRatio = 12.8;       // swerve only
    double bumperThicknessM = 0.0826;   // 3.25 in
    sdk::Alliance alliance = sdk::Alliance::Red;
    int station = 2;                    // driver station 1..3

    int wheelCount() const;
    bool isSwerve() const { return drivetrain == sdk::drivetrain_kind::kSwerve; }
    bool isTank() const { return drivetrain == sdk::drivetrain_kind::kTank; }
    bool isMecanum() const { return drivetrain == sdk::drivetrain_kind::kMecanum; }

    bool operator==(const RobotBasics&) const = default;
};

// Sensible starting values when the user switches the drivetrain type.
RobotBasics defaultBasicsFor(const std::string& drivetrainKind);

// Colour of the bumper primitives of a robot on this alliance; basicsFromConfig() reads the alliance back from it.
const char* bumperColorFor(sdk::Alliance alliance);

// Range checks with plain-language messages. Errors block the wizard, warnings do not.
std::vector<sdk::Issue> validateBasics(const RobotBasics& basics);
bool hasErrors(const std::vector<sdk::Issue>& issues);

struct RobotBuildContext {
    QString projectPath;
    QString language;
    int wpilibYear = 0;
    QString gradleRioVersion;
    std::int64_t stepUs = sdk::kDefaultStepUs;
    const sdk::IGameModule* gameModule = nullptr;   // null = core-only mode
};

// Turns the basics into a complete RobotConfig: chassis, wheels with motors, bumpers, battery and
// sim settings, plus the selected game plugin's default section. Mappings stay empty: they are
// filled in by device discovery.
sdk::RobotConfig buildRobotConfig(const RobotBasics& basics, const RobotBuildContext& context);

// Inverse used to re-open the wizard on an existing config (unknown pieces keep defaults).
RobotBasics basicsFromConfig(const sdk::RobotConfig& config);

// Re-running the wizard on a project that already has a model must not throw the editor's work away:
// the basics replace the chassis size/mass, bumpers and the drivetrain wheels; mechanisms, robot
// mappings, battery, plugin sections and any hand-added chassis primitives are kept, and drivetrain
// mappings survive for wheels that still exist. With no existing model this is buildRobotConfig().
sdk::RobotConfig updateRobotConfig(const sdk::RobotConfig* existing, const RobotBasics& basics, const RobotBuildContext& context);

}  // namespace wpiemu::app
