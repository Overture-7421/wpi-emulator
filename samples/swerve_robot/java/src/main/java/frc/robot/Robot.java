// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  // Kept slow so the sample is safe in a small space and a short run ends well short of the field elements.
  private static final double kTeleopMaxSpeedMps = 0.4;
  private static final double kTeleopMaxTurnRadPerSec = Math.PI;
  private static final double kDeadband = 0.1;

  private final XboxController m_controller = new XboxController(0);
  private final Swerve m_swerve = new Swerve();
  private final Intake m_intake = new Intake();

  @Override
  public void teleopInit() {
    m_intake.deploy();
  }

  @Override
  public void teleopPeriodic() {
    // Pushing a stick forward or left reads negative; field +x is forward and +y is left.
    double x = -MathUtil.applyDeadband(m_controller.getLeftY(), kDeadband) * kTeleopMaxSpeedMps;
    double y = -MathUtil.applyDeadband(m_controller.getLeftX(), kDeadband) * kTeleopMaxSpeedMps;
    double omega =
        -MathUtil.applyDeadband(m_controller.getRightX(), kDeadband) * kTeleopMaxTurnRadPerSec;
    m_swerve.drive(x, y, omega);
  }

  @Override
  public void teleopExit() {
    m_intake.stop();
  }
}
