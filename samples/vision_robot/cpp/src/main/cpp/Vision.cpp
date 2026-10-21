// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Vision.h"

#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>

#include <cmath>
#include <cstdio>
#include <numbers>

#include "LimelightHelpers.h"

namespace {
constexpr units::second_t kPrintPeriod = 0.5_s;
constexpr int64_t kLimelightHeartbeatTimeoutUs = 1'000'000;
const wpi::array<double, 3> kMultiTagStdDevs{0.3, 0.3, 0.5};
const wpi::array<double, 3> kSingleTagStdDevs{0.7, 0.7, 1.0e9};
}  // namespace

const frc::Transform3d Vision::kRobotToLimelight{
    frc::Translation3d{0.30_m, 0.0_m, 0.50_m}, frc::Rotation3d{0_rad, 0_rad, 0_rad}};
const frc::Transform3d Vision::kRobotToPhoton{
    frc::Translation3d{-0.30_m, 0.0_m, 0.50_m},
    frc::Rotation3d{0_rad, 0_rad, units::radian_t{std::numbers::pi}}};

Vision::Vision(frc::DifferentialDriveKinematics& kinematics)
    : m_estimator{kinematics, frc::Rotation2d{}, 0_m, 0_m, frc::Pose2d{}},
      m_photonEstimator{
          frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded),
          kRobotToPhoton},
      m_posePublisher{nt::NetworkTableInstance::GetDefault()
                          .GetStructTopic<frc::Pose2d>("/SmartDashboard/Vision/estimatedPose")
                          .Publish()},
      m_lastPrint{frc::Timer::GetFPGATimestamp()} {
  const auto& mount = kRobotToLimelight;
  LimelightHelpers::setCameraPose_RobotSpace(
      kLimelightName, mount.X().value(), mount.Y().value(), mount.Z().value(),
      units::degree_t{mount.Rotation().X()}.value(), units::degree_t{mount.Rotation().Y()}.value(),
      units::degree_t{mount.Rotation().Z()}.value());
}

void Vision::Update(const frc::Rotation2d& gyro, units::meter_t left, units::meter_t right) {
  m_gyro = gyro;
  m_left = left;
  m_right = right;
  m_estimator.Update(gyro, left, right);

  UpdateLimelight();
  UpdatePhoton();
  m_posePublisher.Set(GetEstimatedPose());

  const units::second_t now = frc::Timer::GetFPGATimestamp();
  if (now - m_lastPrint >= kPrintPeriod) {
    m_lastPrint = now;
    PublishAndPrint(now);
  }
}

void Vision::UpdateLimelight() {
  LimelightHelpers::SetRobotOrientation(kLimelightName,
                                        GetEstimatedPose().Rotation().Degrees().value(), 0, 0, 0,
                                        0, 0);

  const auto mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelightName);
  if (mt1.tagCount >= 2 && mt1.timestampSeconds != m_lastLimelightMt1Timestamp) {
    m_lastLimelightMt1Timestamp = mt1.timestampSeconds;
    Fuse(mt1.pose, mt1.timestampSeconds, kMultiTagStdDevs, true);
  }

  const auto mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
  if (mt2.tagCount > 0 && mt2.timestampSeconds != m_lastLimelightMt2Timestamp) {
    m_lastLimelightMt2Timestamp = mt2.timestampSeconds;
    Fuse(mt2.pose, mt2.timestampSeconds, kSingleTagStdDevs, false);
  }
}

void Vision::UpdatePhoton() {
  for (const auto& result : m_photon.GetAllUnreadResults()) {
    m_photonResult = result;
    auto estimate = m_photonEstimator.EstimateCoprocMultiTagPose(result);
    const bool multiTag = estimate.has_value();
    if (!estimate) {
      estimate = m_photonEstimator.EstimateLowestAmbiguityPose(result);
    }

    m_photonPose.reset();
    if (estimate) {
      m_photonPose = estimate->estimatedPose.ToPose2d();
      Fuse(*m_photonPose, estimate->timestamp, multiTag ? kMultiTagStdDevs : kSingleTagStdDevs,
           multiTag);
    }
  }
}

// The first measurement that is trustworthy in heading places the estimator on the field.
void Vision::Fuse(const frc::Pose2d& pose, units::second_t timestamp,
                  const wpi::array<double, 3>& stdDevs, bool headingTrusted) {
  if (!m_seeded) {
    if (headingTrusted) {
      m_estimator.ResetPosition(m_gyro, m_left, m_right, pose);
      m_seeded = true;
    }
    return;
  }
  m_estimator.AddVisionMeasurement(pose, timestamp, stdDevs);
}

bool Vision::LimelightConnected() const {
  auto heartbeat = nt::NetworkTableInstance::GetDefault().GetTable(kLimelightName)->GetEntry("hb");
  return heartbeat.Exists() &&
         static_cast<int64_t>(frc::RobotController::GetFPGATime()) - heartbeat.GetLastChange() <
             kLimelightHeartbeatTimeoutUs;
}

void Vision::PublishAndPrint(units::second_t now) {
  const bool tv = LimelightHelpers::getTV(kLimelightName);
  const auto mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelightName);
  const double latencyMs = LimelightHelpers::getLatency_Pipeline(kLimelightName) +
                           LimelightHelpers::getLatency_Capture(kLimelightName);
  const bool llConnected = LimelightConnected();
  const frc::Pose2d llPose = mt1.pose;

  std::printf(
      "VISION limelight tv=%d tags=%d botposeX=%.3f botposeY=%.3f botposeYawDeg=%.2f "
      "latencyMs=%.1f connected=%s\n",
      tv ? 1 : 0, mt1.tagCount, llPose.X().value(), llPose.Y().value(),
      llPose.Rotation().Degrees().value(), latencyMs, llConnected ? "true" : "false");

  const frc::Pose2d photonPose = m_photonPose.value_or(frc::Pose2d{});
  const bool photonTargets = m_photonResult.HasTargets();
  const auto photonTags = m_photonResult.GetTargets().size();
  const double resultAgeMs = (now - m_photonResult.GetTimestamp()).value() * 1000.0;
  const bool photonConnected = m_photon.IsConnected();

  std::printf(
      "VISION photon hasTargets=%s tags=%zu poseX=%.3f poseY=%.3f poseYawDeg=%.2f "
      "resultAgeMs=%.1f connected=%s\n",
      photonTargets ? "true" : "false", photonTags, photonPose.X().value(),
      photonPose.Y().value(), photonPose.Rotation().Degrees().value(), resultAgeMs,
      photonConnected ? "true" : "false");
  std::fflush(stdout);

  using frc::SmartDashboard;
  SmartDashboard::PutBoolean("Vision/limelight/tv", tv);
  SmartDashboard::PutNumber("Vision/limelight/tags", mt1.tagCount);
  SmartDashboard::PutNumber("Vision/limelight/botposeX", llPose.X().value());
  SmartDashboard::PutNumber("Vision/limelight/botposeY", llPose.Y().value());
  SmartDashboard::PutNumber("Vision/limelight/botposeYawDeg", llPose.Rotation().Degrees().value());
  SmartDashboard::PutNumber("Vision/limelight/latencyMs", latencyMs);
  SmartDashboard::PutBoolean("Vision/limelight/connected", llConnected);

  SmartDashboard::PutBoolean("Vision/photon/hasTargets", photonTargets);
  SmartDashboard::PutNumber("Vision/photon/tags", static_cast<double>(photonTags));
  SmartDashboard::PutNumber("Vision/photon/poseX", photonPose.X().value());
  SmartDashboard::PutNumber("Vision/photon/poseY", photonPose.Y().value());
  SmartDashboard::PutNumber("Vision/photon/poseYawDeg", photonPose.Rotation().Degrees().value());
  SmartDashboard::PutNumber("Vision/photon/resultAgeMs", resultAgeMs);
  SmartDashboard::PutBoolean("Vision/photon/connected", photonConnected);

  const frc::Pose2d fused = GetEstimatedPose();
  std::printf(
      "VISION drivetrain leftMeters=%.3f rightMeters=%.3f fusedX=%.3f fusedY=%.3f "
      "fusedYawDeg=%.2f\n",
      m_left.value(), m_right.value(), fused.X().value(), fused.Y().value(),
      fused.Rotation().Degrees().value());
  std::fflush(stdout);
  SmartDashboard::PutNumber("Vision/fused/x", fused.X().value());
  SmartDashboard::PutNumber("Vision/fused/y", fused.Y().value());
  SmartDashboard::PutNumber("Vision/fused/yawDeg", fused.Rotation().Degrees().value());
}
