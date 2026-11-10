#pragma once

// The RobotConfig of samples/swerve_robot/java (Swerve.java and RobotContainer.java: track 11.125 in x 10.875 in, the CAN ids
// of its module list) for the opt-in end-to-end tests that run that project: the single-robot run and the practice match.
#include <utility>

#include "test_support.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::app::host::testing {

struct SwerveModuleIds {
    const char* name;
    float x, y;
    int driveId, steerId, encoderId;
};
inline constexpr SwerveModuleIds kModules[] = {
    {"fl", 0.2826f, 0.2762f, 4, 3, 12},
    {"fr", 0.2826f, -0.2762f, 8, 7, 11},
    {"bl", -0.2826f, 0.2762f, 2, 1, 9},
    {"br", -0.2826f, -0.2762f, 6, 5, 10},
};
inline constexpr int kPigeonId = 13;
inline constexpr int kIntakeRollerId = 32;
inline constexpr int kIntakePivotId = 29;
inline constexpr int kIntakePivotEncoderId = 30;

inline sdk::RobotConfig swerveSampleBaseConfig() {
    sdk::RobotConfig c;
    c.name = "Swerve sample e2e";
    c.project.language = "java";
    c.project.wpilibYear = 2026;
    c.project.gradleRioVersion = "2026.2.1";
    return c;
}

// Minimal but complete model: swerve chassis, IMU, one roller mechanism, every mapping to a device that discovery announced.
inline sdk::RobotConfig swerveSampleMappedConfig() {
    sdk::RobotConfig c = swerveSampleBaseConfig();
    c.chassis.sizeM = {0.775f, 0.80f, 0.15f};
    c.chassis.massKg = 61.235f;
    c.chassis.comHeightM = 0.20f;
    c.drivetrain.spec.kind = "swerve";
    c.drivetrain.spec.tireModel = "brush";
    for (const SwerveModuleIds& m : kModules) {
        sdk::WheelSpec w;
        w.name = m.name;
        w.kind = sdk::WheelKind::Swerve;
        w.steered = true;
        w.positionM = {m.x, m.y, 0.0508f};
        w.radiusM = 0.0508f;
        w.driveGearRatio = 7.03f;
        w.steerGearRatio = 287.0f / 11.0f;
        w.steerMotor = sdk::motors::falcon500();
        c.drivetrain.spec.wheels.push_back(w);
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, w.name + ".drive"}, phoenixKey("talonfx", m.driveId)});
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, sdk::steerSlotName(w.name)}, phoenixKey("talonfx", m.steerId)});
        c.drivetrain.mappings.push_back({{sdk::slot_owner::kDrivetrain, sdk::steerEncoderSlotName(w.name)},
                                         phoenixKey("cancoder", m.encoderId)});
    }
    c.robotMappings.push_back({{sdk::slot_owner::kRobot, "imu"}, phoenixKey("pigeon2", kPigeonId)});

    sdk::MechanismConfig roller;
    roller.id = "intake_roller";
    roller.type = "mechanism";
    roller.label = "Intake roller";
    roller.mount.position = {0.45f, 0.f, 0.15f};
    roller.mappings.push_back({{roller.id, "motor"}, phoenixKey("talonfx", kIntakeRollerId)});
    c.mechanisms.push_back(std::move(roller));

    // The intake pivot: teleopInit runs a Motion Magic move to 114 deg through a fused CANcoder, which closes the loop through
    // the emulator (Phoenix firmware sim -> rotor feedback from the pivot plant -> CANcoder position).
    sdk::MechanismConfig pivot;
    pivot.id = "intake_pivot";
    pivot.type = "mechanism";
    pivot.properties["motion"] = "arm";
    pivot.label = "Intake pivot";
    pivot.mount.position = {0.35f, 0.f, 0.20f};
    pivot.properties["gearRatio"] = 40.0;
    pivot.properties["minPosition"] = 0.0;
    pivot.properties["maxPosition"] = 2.3;
    pivot.properties["startPosition"] = 0.087;
    pivot.properties["simulateGravity"] = false;
    pivot.properties["momentOfInertiaKgM2"] = 0.4;
    pivot.properties["statorCurrentLimitA"] = 75.0;
    pivot.properties["neutralMode"] = "brake";
    pivot.mappings.push_back({{pivot.id, "motor"}, phoenixKey("talonfx", kIntakePivotId), true});
    pivot.mappings.push_back({{pivot.id, "encoder"}, phoenixKey("cancoder", kIntakePivotEncoderId)});
    c.mechanisms.push_back(std::move(pivot));
    return c;
}

}  // namespace wpiemu::app::host::testing
