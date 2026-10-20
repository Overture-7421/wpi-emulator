// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** A four-module swerve drivetrain driven by field-relative speeds and the Pigeon 2 heading. */
public class Swerve {
  /** Fastest a module can drive, in meters per second, at about 12 V. */
  public static final double kMaxSpeedMps = 4.541;

  private static final int kPigeonId = 13;
  // Module offsets from the robot's center: 11.125 in forward and 10.875 in to the side.
  private static final double kTrackXMeters = 0.2826;
  private static final double kTrackYMeters = 0.2762;

  // Front left, front right, back left, back right: drive, steer and CANcoder CAN IDs.
  private final SwerveModule[] m_modules = {
    new SwerveModule(4, 3, 12),
    new SwerveModule(8, 7, 11),
    new SwerveModule(2, 1, 9),
    new SwerveModule(6, 5, 10),
  };
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          new Translation2d(kTrackXMeters, kTrackYMeters),
          new Translation2d(kTrackXMeters, -kTrackYMeters),
          new Translation2d(-kTrackXMeters, kTrackYMeters),
          new Translation2d(-kTrackXMeters, -kTrackYMeters));
  private final Pigeon2 m_gyro = new Pigeon2(kPigeonId);

  /** Drives relative to the field: +x forward, +y left, counterclockwise-positive rotation. */
  public void drive(double xMps, double yMps, double omegaRadPerSec) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xMps, yMps, omegaRadPerSec, m_gyro.getRotation2d());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMps);
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].set(states[i]);
    }
  }
}
