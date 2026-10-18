// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/length.h>

#include "Vision.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousPeriodic() override;
  void TeleopPeriodic() override;

 private:
  static constexpr int kLeftMotorId = 1;
  static constexpr int kRightMotorId = 2;
  static constexpr int kPigeonId = 10;
  static constexpr units::meter_t kTrackWidth = 0.60_m;
  static constexpr units::meter_t kWheelRadius = 0.0762_m;
  static constexpr double kGearRatio = 8.0;
  static constexpr double kAutoSpeed = 0.3;

  units::meter_t WheelDistance(ctre::phoenix6::hardware::TalonFX& motor);

  ctre::phoenix6::hardware::TalonFX m_left{kLeftMotorId};
  ctre::phoenix6::hardware::TalonFX m_right{kRightMotorId};
  ctre::phoenix6::hardware::Pigeon2 m_gyro{kPigeonId};
  frc::DifferentialDrive m_drive{[this](double output) { m_left.Set(output); },
                                 [this](double output) { m_right.Set(output); }};
  frc::XboxController m_controller{0};
  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  Vision m_vision{m_kinematics};
};
