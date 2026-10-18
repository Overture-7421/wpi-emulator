// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <networktables/StructTopic.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/array.h>

#include <optional>

/** Fuses a Limelight and a PhotonVision camera into one drivetrain pose estimate. */
class Vision {
 public:
  static constexpr const char* kLimelightName = "limelight-front";
  static constexpr const char* kPhotonName = "photon-rear";

  // Camera mounts in the WPILib robot frame: +X forward, +Y left, +Z up, origin on the floor.
  static const frc::Transform3d kRobotToLimelight;
  static const frc::Transform3d kRobotToPhoton;

  explicit Vision(frc::DifferentialDriveKinematics& kinematics);

  frc::Pose2d GetEstimatedPose() const { return m_estimator.GetEstimatedPosition(); }

  /** Call every loop with the current gyro heading and wheel distances. */
  void Update(const frc::Rotation2d& gyro, units::meter_t left, units::meter_t right);

 private:
  void UpdateLimelight();
  void UpdatePhoton();
  void Fuse(const frc::Pose2d& pose, units::second_t timestamp,
            const wpi::array<double, 3>& stdDevs, bool headingTrusted);
  bool LimelightConnected() const;
  void PublishAndPrint(units::second_t now);

  frc::DifferentialDrivePoseEstimator m_estimator;
  photon::PhotonCamera m_photon{kPhotonName};
  photon::PhotonPoseEstimator m_photonEstimator;
  nt::StructPublisher<frc::Pose2d> m_posePublisher;

  frc::Rotation2d m_gyro;
  units::meter_t m_left{0};
  units::meter_t m_right{0};
  bool m_seeded = false;

  units::second_t m_lastLimelightMt1Timestamp{-1};
  units::second_t m_lastLimelightMt2Timestamp{-1};
  photon::PhotonPipelineResult m_photonResult;
  std::optional<frc::Pose2d> m_photonPose;
  units::second_t m_lastPrint;
};
