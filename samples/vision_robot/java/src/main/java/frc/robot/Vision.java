// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Fuses a Limelight and a PhotonVision camera into one drivetrain pose estimate. */
public class Vision {
  public static final String kLimelightName = "limelight-front";
  public static final String kPhotonName = "photon-rear";

  // Camera mounts in the WPILib robot frame: +X forward, +Y left, +Z up, origin on the floor.
  public static final Transform3d kRobotToLimelight =
      new Transform3d(0.30, 0.0, 0.50, new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d kRobotToPhoton =
      new Transform3d(-0.30, 0.0, 0.50, new Rotation3d(0.0, 0.0, Math.PI));

  private static final double kPrintPeriodSeconds = 0.5;
  private static final double kLimelightHeartbeatTimeoutUs = 1.0e6;
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.5);
  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.7, 0.7, 1.0e9);

  private final DifferentialDrivePoseEstimator m_estimator;
  private final PhotonCamera m_photon = new PhotonCamera(kPhotonName);
  private final PhotonPoseEstimator m_photonEstimator;
  private final StructPublisher<Pose2d> m_posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("/SmartDashboard/Vision/estimatedPose", Pose2d.struct)
          .publish();

  private Rotation2d m_gyro = new Rotation2d();
  private double m_leftMeters;
  private double m_rightMeters;
  private boolean m_seeded;

  private double m_lastLimelightMt1Timestamp = -1.0;
  private double m_lastLimelightMt2Timestamp = -1.0;
  private PhotonPipelineResult m_photonResult = new PhotonPipelineResult();
  private Optional<Pose2d> m_photonPose = Optional.empty();
  private double m_lastPrintSeconds;

  public Vision(DifferentialDriveKinematics kinematics) {
    m_estimator =
        new DifferentialDrivePoseEstimator(kinematics, m_gyro, 0.0, 0.0, new Pose2d());
    m_photonEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), kRobotToPhoton);

    LimelightHelpers.setCameraPose_RobotSpace(
        kLimelightName,
        kRobotToLimelight.getX(),
        kRobotToLimelight.getY(),
        kRobotToLimelight.getZ(),
        Math.toDegrees(kRobotToLimelight.getRotation().getX()),
        Math.toDegrees(kRobotToLimelight.getRotation().getY()),
        Math.toDegrees(kRobotToLimelight.getRotation().getZ()));
    m_lastPrintSeconds = Timer.getFPGATimestamp();
  }

  public Pose2d getEstimatedPose() {
    return m_estimator.getEstimatedPosition();
  }

  /** Call every loop with the current gyro heading and wheel distances. */
  public void update(Rotation2d gyro, double leftMeters, double rightMeters) {
    m_gyro = gyro;
    m_leftMeters = leftMeters;
    m_rightMeters = rightMeters;
    m_estimator.update(gyro, leftMeters, rightMeters);

    updateLimelight();
    updatePhoton();
    m_posePublisher.set(getEstimatedPose());

    double now = Timer.getFPGATimestamp();
    if (now - m_lastPrintSeconds >= kPrintPeriodSeconds) {
      m_lastPrintSeconds = now;
      publishAndPrint(now);
    }
  }

  private void updateLimelight() {
    LimelightHelpers.SetRobotOrientation(
        kLimelightName, getEstimatedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
    if (mt1 != null && mt1.tagCount >= 2 && mt1.timestampSeconds != m_lastLimelightMt1Timestamp) {
      m_lastLimelightMt1Timestamp = mt1.timestampSeconds;
      fuse(mt1.pose, mt1.timestampSeconds, kMultiTagStdDevs, true);
    }

    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
    if (mt2 != null && mt2.tagCount > 0 && mt2.timestampSeconds != m_lastLimelightMt2Timestamp) {
      m_lastLimelightMt2Timestamp = mt2.timestampSeconds;
      fuse(mt2.pose, mt2.timestampSeconds, kSingleTagStdDevs, false);
    }
  }

  private void updatePhoton() {
    for (PhotonPipelineResult result : m_photon.getAllUnreadResults()) {
      m_photonResult = result;
      Optional<EstimatedRobotPose> estimate = m_photonEstimator.estimateCoprocMultiTagPose(result);
      boolean multiTag = estimate.isPresent();
      if (estimate.isEmpty()) {
        estimate = m_photonEstimator.estimateLowestAmbiguityPose(result);
      }

      m_photonPose = estimate.map(e -> e.estimatedPose.toPose2d());
      estimate.ifPresent(
          e ->
              fuse(
                  e.estimatedPose.toPose2d(),
                  e.timestampSeconds,
                  multiTag ? kMultiTagStdDevs : kSingleTagStdDevs,
                  multiTag));
    }
  }

  // The first measurement that is trustworthy in heading places the estimator on the field.
  private void fuse(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, boolean headingTrusted) {
    if (!m_seeded) {
      if (headingTrusted) {
        m_estimator.resetPosition(m_gyro, m_leftMeters, m_rightMeters, pose);
        m_seeded = true;
      }
      return;
    }
    m_estimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }

  private boolean limelightConnected() {
    var heartbeat = NetworkTableInstance.getDefault().getTable(kLimelightName).getEntry("hb");
    return heartbeat.exists()
        && RobotController.getFPGATime() - heartbeat.getLastChange() < kLimelightHeartbeatTimeoutUs;
  }

  private void publishAndPrint(double now) {
    boolean tv = LimelightHelpers.getTV(kLimelightName);
    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
    double latencyMs =
        LimelightHelpers.getLatency_Pipeline(kLimelightName)
            + LimelightHelpers.getLatency_Capture(kLimelightName);
    boolean llConnected = limelightConnected();
    Pose2d llPose = mt1.pose;

    System.out.printf(
        java.util.Locale.ROOT,
        "VISION limelight tv=%d tags=%d botposeX=%.3f botposeY=%.3f botposeYawDeg=%.2f"
            + " latencyMs=%.1f connected=%b%n",
        tv ? 1 : 0,
        mt1.tagCount,
        llPose.getX(),
        llPose.getY(),
        llPose.getRotation().getDegrees(),
        latencyMs,
        llConnected);

    Pose2d photonPose = m_photonPose.orElse(new Pose2d());
    boolean photonTargets = m_photonResult.hasTargets();
    int photonTags = m_photonResult.getTargets().size();
    double resultAgeMs = (now - m_photonResult.getTimestampSeconds()) * 1000.0;
    boolean photonConnected = m_photon.isConnected();

    System.out.printf(
        java.util.Locale.ROOT,
        "VISION photon hasTargets=%b tags=%d poseX=%.3f poseY=%.3f poseYawDeg=%.2f"
            + " resultAgeMs=%.1f connected=%b%n",
        photonTargets,
        photonTags,
        photonPose.getX(),
        photonPose.getY(),
        photonPose.getRotation().getDegrees(),
        resultAgeMs,
        photonConnected);
    System.out.flush();

    SmartDashboard.putBoolean("Vision/limelight/tv", tv);
    SmartDashboard.putNumber("Vision/limelight/tags", mt1.tagCount);
    SmartDashboard.putNumber("Vision/limelight/botposeX", llPose.getX());
    SmartDashboard.putNumber("Vision/limelight/botposeY", llPose.getY());
    SmartDashboard.putNumber("Vision/limelight/botposeYawDeg", llPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/limelight/latencyMs", latencyMs);
    SmartDashboard.putBoolean("Vision/limelight/connected", llConnected);

    SmartDashboard.putBoolean("Vision/photon/hasTargets", photonTargets);
    SmartDashboard.putNumber("Vision/photon/tags", photonTags);
    SmartDashboard.putNumber("Vision/photon/poseX", photonPose.getX());
    SmartDashboard.putNumber("Vision/photon/poseY", photonPose.getY());
    SmartDashboard.putNumber("Vision/photon/poseYawDeg", photonPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/photon/resultAgeMs", resultAgeMs);
    SmartDashboard.putBoolean("Vision/photon/connected", photonConnected);

    Pose2d fused = getEstimatedPose();
    System.out.printf(
        java.util.Locale.ROOT,
        "VISION drivetrain leftMeters=%.3f rightMeters=%.3f fusedX=%.3f fusedY=%.3f"
            + " fusedYawDeg=%.2f%n",
        m_leftMeters,
        m_rightMeters,
        fused.getX(),
        fused.getY(),
        fused.getRotation().getDegrees());
    SmartDashboard.putNumber("Vision/fused/x", fused.getX());
    SmartDashboard.putNumber("Vision/fused/y", fused.getY());
    SmartDashboard.putNumber("Vision/fused/yawDeg", fused.getRotation().getDegrees());
  }
}
