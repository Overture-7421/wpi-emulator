// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <units/angle.h>

#include <numbers>

Robot::Robot() {
  ctre::phoenix6::configs::TalonFXConfiguration config;
  config.Feedback.SensorToMechanismRatio = kGearRatio;
  m_left.GetConfigurator().Apply(config);
  m_right.GetConfigurator().Apply(config);
}

units::meter_t Robot::WheelDistance(ctre::phoenix6::hardware::TalonFX& motor) {
  return units::meter_t{motor.GetPosition().GetValue().value() * 2.0 * std::numbers::pi *
                        kWheelRadius.value()};
}

void Robot::RobotPeriodic() {
  m_vision.Update(m_gyro.GetRotation2d(), WheelDistance(m_left), WheelDistance(m_right));
}

void Robot::AutonomousPeriodic() {
  m_drive.ArcadeDrive(kAutoSpeed, 0.0, false);
}

void Robot::TeleopPeriodic() {
  m_drive.ArcadeDrive(-m_controller.GetLeftY(), -m_controller.GetRightX());
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
