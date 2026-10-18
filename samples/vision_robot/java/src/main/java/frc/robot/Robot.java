// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  private static final int kLeftMotorId = 1;
  private static final int kRightMotorId = 2;
  private static final int kPigeonId = 10;
  private static final double kTrackWidthMeters = 0.60;
  private static final double kWheelRadiusMeters = 0.0762;
  private static final double kGearRatio = 8.0;
  private static final double kAutoSpeed = 0.3;

  private final TalonFX m_left = new TalonFX(kLeftMotorId);
  private final TalonFX m_right = new TalonFX(kRightMotorId);
  private final Pigeon2 m_gyro = new Pigeon2(kPigeonId);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_left::set, m_right::set);
  private final XboxController m_controller = new XboxController(0);
  private final Vision m_vision =
      new Vision(new DifferentialDriveKinematics(kTrackWidthMeters));

  public Robot() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = kGearRatio;
    m_left.getConfigurator().apply(config);
    m_right.getConfigurator().apply(config);
  }

  private double wheelMeters(TalonFX motor) {
    return motor.getPosition().getValueAsDouble() * 2.0 * Math.PI * kWheelRadiusMeters;
  }

  @Override
  public void robotPeriodic() {
    m_vision.update(m_gyro.getRotation2d(), wheelMeters(m_left), wheelMeters(m_right));
  }

  @Override
  public void autonomousPeriodic() {
    m_drive.arcadeDrive(kAutoSpeed, 0.0, false);
  }

  @Override
  public void teleopPeriodic() {
    m_drive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }
}
