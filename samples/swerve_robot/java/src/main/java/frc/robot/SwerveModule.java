// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** One swerve module: a TalonFX that drives the wheel and a TalonFX that steers it, read through a CANcoder. */
public class SwerveModule {
  private static final double kSteerGearRatio = 287.0 / 11.0;
  private static final double kDriveVoltsPerMps = 2.61;
  private static final double kSteerKp = 40.0;
  private static final double kSteerKs = 0.15;
  private static final double kDriveSupplyLimitA = 40.0;

  private final TalonFX m_drive;
  private final TalonFX m_steer;
  private final CANcoder m_encoder;
  private final VoltageOut m_driveRequest = new VoltageOut(0.0);
  private final PositionVoltage m_steerRequest = new PositionVoltage(0.0);

  public SwerveModule(int driveId, int steerId, int encoderId) {
    m_drive = new TalonFX(driveId);
    m_steer = new TalonFX(steerId);
    // The steer motor reads the CANcoder through its fused feedback, so the module only needs to own it.
    m_encoder = new CANcoder(encoderId);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyLimitA;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_drive.getConfigurator().apply(driveConfig);

    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerConfig.Slot0.kP = kSteerKp;
    steerConfig.Slot0.kS = kSteerKs;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    steerConfig.Feedback.RotorToSensorRatio = kSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    m_steer.getConfigurator().apply(steerConfig);
  }

  /** The wheel's heading relative to the robot. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_steer.getPosition().getValueAsDouble());
  }

  /** Steers to the state's angle (turning the short way around) and drives at its speed. */
  public void set(SwerveModuleState state) {
    state.optimize(getAngle());
    m_steer.setControl(m_steerRequest.withPosition(state.angle.getRotations()));
    m_drive.setControl(m_driveRequest.withOutput(kDriveVoltsPerMps * state.speedMetersPerSecond));
  }
}
