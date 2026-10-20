// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

/** An intake: a roller, and a pivot that swings it out with Motion Magic through a fused CANcoder. */
public class Intake {
  private static final int kRollerId = 32;
  private static final int kPivotId = 29;
  private static final int kPivotEncoderId = 30;

  private static final double kRollerVolts = 8.0;
  private static final Angle kDeployedAngle = Degrees.of(114.0);
  private static final double kPivotRotorToSensorRatio = 40.0;

  private final TalonFX m_roller = new TalonFX(kRollerId);
  private final TalonFX m_pivot = new TalonFX(kPivotId);
  private final CANcoder m_pivotEncoder = new CANcoder(kPivotEncoderId);
  private final VoltageOut m_rollerRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0.0);

  public Intake() {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 100.0;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_roller.getConfigurator().apply(rollerConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_pivotEncoder.getConfigurator().apply(encoderConfig);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 75.0;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.Slot0.kP = 200.0;
    pivotConfig.Slot0.kV = 4.8;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 2.29; // rotations per second
    pivotConfig.MotionMagic.MotionMagicAcceleration = 11.1; // rotations per second squared
    pivotConfig.MotionMagic.MotionMagicJerk = 0.0;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.FeedbackRemoteSensorID = kPivotEncoderId;
    pivotConfig.Feedback.RotorToSensorRatio = kPivotRotorToSensorRatio;
    m_pivot.getConfigurator().apply(pivotConfig);
  }

  /** Swings the intake out and starts the roller. */
  public void deploy() {
    m_pivot.setControl(m_pivotRequest.withPosition(kDeployedAngle));
    m_roller.setControl(m_rollerRequest.withOutput(kRollerVolts));
  }

  /** Stops the roller; the pivot holds its position. */
  public void stop() {
    m_roller.setControl(m_rollerRequest.withOutput(0.0));
  }
}
