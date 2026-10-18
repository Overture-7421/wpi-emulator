// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Vision sample constants. The emulator's end-to-end tests depend on these exact values.
  // Transforms are robot to camera in the WPILib frame: +X forward, +Y left, +Z up, with pitch
  // positive nose down.
  private static final String kPhotonCameraName = "photon-front";
  private static final Transform3d kRobotToPhotonCamera =
      new Transform3d(
          new Translation3d(0.30, 0.00, 0.50), new Rotation3d(0.0, Math.toRadians(-20.0), 0.0));

  private static final String kLimelightName = "limelight-rear";
  // The angles stay in degrees so they reach the Limelight exactly, without a rotation round trip.
  private static final double kLimelightRollDeg = 0.0;
  private static final double kLimelightPitchDeg = -25.0;
  private static final double kLimelightYawDeg = 180.0;
  private static final Transform3d kRobotToLimelight =
      new Transform3d(
          new Translation3d(-0.30, 0.00, 0.45),
          new Rotation3d(
              Math.toRadians(kLimelightRollDeg),
              Math.toRadians(kLimelightPitchDeg),
              Math.toRadians(kLimelightYawDeg)));

  private static final int kPigeonCanId = 0;
  private static final int kJoystickPort = 0;
  private static final int kPrintEveryLoops = 5;
  private static final double kYawOffsetDegPerAxis = 90.0;
  private static final double[] kNoValues = new double[0];

  private final Pigeon2 m_pigeon = new Pigeon2(kPigeonCanId, CANBus.roboRIO());
  private final Joystick m_joystick = new Joystick(kJoystickPort);
  private final AprilTagFieldLayout m_fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private final PhotonCamera m_photon = new PhotonCamera(kPhotonCameraName);
  private final PhotonPoseEstimator m_photonEstimator =
      new PhotonPoseEstimator(m_fieldLayout, kRobotToPhotonCamera);
  private final Mirror m_mirror = new Mirror();

  private int m_loop;

  // PhotonVision: newest result seen so far, the estimate made from it and how it was made.
  private PhotonPipelineResult m_photonLatest;
  private Optional<EstimatedRobotPose> m_photonEstimate = Optional.empty();
  private String m_photonStrategy = "-";
  private int m_photonResultsSincePrint;

  // Limelight: values read in the latest loop.
  private boolean m_llTv;
  private int m_llTid;
  private double m_llTx;
  private double m_llTy;
  private double m_llTa;
  private int m_llFiducials;
  private PoseEstimate m_llMt1 = new PoseEstimate();
  private PoseEstimate m_llMt2 = new PoseEstimate();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Joystick 0 is only there when a driver station provides it.
    DriverStation.silenceJoystickConnectionWarning(true);

    // Limelight robot space is +X forward, +Y right, +Z up, so Y and the angles change sign.
    LimelightHelpers.setCameraPose_RobotSpace(
        kLimelightName,
        kRobotToLimelight.getX(),
        -kRobotToLimelight.getY(),
        kRobotToLimelight.getZ(),
        -kLimelightRollDeg,
        -kLimelightPitchDeg,
        -kLimelightYawDeg);
    LimelightHelpers.SetIMUMode(kLimelightName, 1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_loop++;
    double yawDeg = m_pigeon.getYaw().getValueAsDouble();

    applyOperatorInputs();
    drainPhotonResults();
    // Axis 0 adds a heading error to what the Limelight is told, to emulate a wrong gyro.
    readLimelight(yawDeg + m_joystick.getRawAxis(0) * kYawOffsetDegPerAxis);

    if (m_loop % kPrintEveryLoops == 0) {
      reportTelemetry(yawDeg);
    }
  }

  /** Buttons 1-4 of joystick 0 drive the camera settings; releasing a button restores them. */
  private void applyOperatorInputs() {
    m_photon.setDriverMode(m_joystick.getRawButton(1));
    m_photon.setPipelineIndex(m_joystick.getRawButton(2) ? 1 : 0);
    if (m_joystick.getRawButton(3)) {
      LimelightHelpers.setLEDMode_ForceOn(kLimelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
    }
    LimelightHelpers.setPipelineIndex(kLimelightName, m_joystick.getRawButton(4) ? 1 : 0);
  }

  /** Reads every PhotonVision result that arrived since the last loop. */
  private void drainPhotonResults() {
    List<PhotonPipelineResult> results = m_photon.getAllUnreadResults();
    m_photonResultsSincePrint += results.size();
    for (PhotonPipelineResult result : results) {
      Optional<EstimatedRobotPose> estimate = m_photonEstimator.estimateCoprocMultiTagPose(result);
      String strategy = "multi";
      if (estimate.isEmpty()) {
        estimate = m_photonEstimator.estimateLowestAmbiguityPose(result);
        strategy = "ambiguity";
      }
      m_photonLatest = result;
      m_photonEstimate = estimate;
      m_photonStrategy = estimate.isPresent() ? strategy : "-";
    }
  }

  private void readLimelight(double yawDeg) {
    LimelightHelpers.SetRobotOrientation(kLimelightName, yawDeg, 0, 0, 0, 0, 0);
    m_llTv = LimelightHelpers.getTV(kLimelightName);
    m_llTid = (int) LimelightHelpers.getFiducialID(kLimelightName);
    m_llTx = LimelightHelpers.getTX(kLimelightName);
    m_llTy = LimelightHelpers.getTY(kLimelightName);
    m_llTa = LimelightHelpers.getTA(kLimelightName);
    m_llMt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
    m_llMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
    m_llFiducials = LimelightHelpers.getRawFiducials(kLimelightName).length;
  }

  private void reportTelemetry(double yawDeg) {
    List<PhotonTrackedTarget> targets =
        m_photonLatest == null ? List.of() : m_photonLatest.getTargets();
    Telemetry telemetry =
        new Telemetry(
            m_loop,
            Timer.getFPGATimestamp(),
            yawDeg,
            m_photon.isConnected(),
            m_photonResultsSincePrint,
            m_photonLatest == null ? -1 : targets.size(),
            targets.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray(),
            m_photonLatest == null ? Double.NaN : m_photonLatest.getTimestampSeconds(),
            m_photonStrategy,
            m_photonEstimate.map(e -> poseValues(e.estimatedPose.toPose2d())).orElse(kNoValues),
            m_llTv,
            m_llTid,
            m_llTx,
            m_llTy,
            m_llTa,
            m_llFiducials,
            estimateValues(m_llMt1),
            estimateValues(m_llMt2));
    m_photonResultsSincePrint = 0;

    String line = telemetry.line();
    System.out.println(line);
    m_mirror.publish(telemetry, line);
  }

  private static double[] poseValues(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }

  /** A Limelight estimate with no tags is not an estimate. */
  private static double[] estimateValues(PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount == 0) {
      return kNoValues;
    }
    return new double[] {
      estimate.pose.getX(),
      estimate.pose.getY(),
      estimate.pose.getRotation().getDegrees(),
      estimate.tagCount,
      estimate.timestampSeconds
    };
  }

  /**
   * One telemetry sample. Values that do not exist are -1 (counts), NaN (numbers) or empty (arrays)
   * and print as "-".
   */
  private record Telemetry(
      int loop,
      double fpgaSeconds,
      double yawDeg,
      boolean phConnected,
      int phResults,
      int phTargets,
      int[] phIds,
      double phTimestampSeconds,
      String phStrategy,
      double[] phPose,
      boolean llTv,
      int llTid,
      double llTx,
      double llTy,
      double llTa,
      int llFiducials,
      double[] llMt1,
      double[] llMt2) {

    String line() {
      return String.format(
          Locale.ROOT,
          "VISION|loop=%d|fpga=%.3f|yaw=%.2f|ph.connected=%d|ph.results=%d|ph.targets=%s"
              + "|ph.ids=%s|ph.ts=%s|ph.strategy=%s|ph.pose=%s|ll.tv=%d|ll.tid=%d|ll.tx=%.2f"
              + "|ll.ty=%.2f|ll.ta=%.3f|ll.fids=%d|ll.mt1=%s|ll.mt2=%s",
          loop,
          fpgaSeconds,
          yawDeg,
          phConnected ? 1 : 0,
          phResults,
          phTargets < 0 ? "-" : Integer.toString(phTargets),
          phIds.length == 0
              ? "-"
              : Arrays.stream(phIds).mapToObj(Integer::toString).collect(Collectors.joining(",")),
          Double.isNaN(phTimestampSeconds)
              ? "-"
              : String.format(Locale.ROOT, "%.3f", phTimestampSeconds),
          phStrategy,
          phPose.length == 0
              ? "-"
              : String.format(Locale.ROOT, "%.3f,%.3f,%.2f", phPose[0], phPose[1], phPose[2]),
          llTv ? 1 : 0,
          llTid,
          llTx,
          llTy,
          llTa,
          llFiducials,
          estimateText(llMt1),
          estimateText(llMt2));
    }

    private static String estimateText(double[] e) {
      return e.length == 0
          ? "-"
          : String.format(
              Locale.ROOT, "%.3f,%.3f,%.2f,%d,%.3f", e[0], e[1], e[2], (int) e[3], e[4]);
    }
  }

  /** The same values as the printed line, published under /VisionSample/. */
  private static final class Mirror {
    private final StringPublisher m_line;
    private final IntegerPublisher m_loop;
    private final DoublePublisher m_fpga;
    private final DoublePublisher m_yaw;
    private final BooleanPublisher m_phConnected;
    private final IntegerPublisher m_phResults;
    private final IntegerPublisher m_phTargets;
    private final IntegerArrayPublisher m_phIds;
    private final DoublePublisher m_phTs;
    private final StringPublisher m_phStrategy;
    private final DoubleArrayPublisher m_phPose;
    private final BooleanPublisher m_llTv;
    private final IntegerPublisher m_llTid;
    private final DoublePublisher m_llTx;
    private final DoublePublisher m_llTy;
    private final DoublePublisher m_llTa;
    private final IntegerPublisher m_llFids;
    private final DoubleArrayPublisher m_llMt1;
    private final DoubleArrayPublisher m_llMt2;

    Mirror() {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("VisionSample");
      m_line = table.getStringTopic("line").publish();
      m_loop = table.getIntegerTopic("loop").publish();
      m_fpga = table.getDoubleTopic("fpga").publish();
      m_yaw = table.getDoubleTopic("yaw").publish();
      m_phConnected = table.getBooleanTopic("ph/connected").publish();
      m_phResults = table.getIntegerTopic("ph/results").publish();
      m_phTargets = table.getIntegerTopic("ph/targets").publish();
      m_phIds = table.getIntegerArrayTopic("ph/ids").publish();
      m_phTs = table.getDoubleTopic("ph/ts").publish();
      m_phStrategy = table.getStringTopic("ph/strategy").publish();
      m_phPose = table.getDoubleArrayTopic("ph/pose").publish();
      m_llTv = table.getBooleanTopic("ll/tv").publish();
      m_llTid = table.getIntegerTopic("ll/tid").publish();
      m_llTx = table.getDoubleTopic("ll/tx").publish();
      m_llTy = table.getDoubleTopic("ll/ty").publish();
      m_llTa = table.getDoubleTopic("ll/ta").publish();
      m_llFids = table.getIntegerTopic("ll/fids").publish();
      m_llMt1 = table.getDoubleArrayTopic("ll/mt1").publish();
      m_llMt2 = table.getDoubleArrayTopic("ll/mt2").publish();
    }

    void publish(Telemetry t, String line) {
      m_line.set(line);
      m_loop.set(t.loop());
      m_fpga.set(t.fpgaSeconds());
      m_yaw.set(t.yawDeg());
      m_phConnected.set(t.phConnected());
      m_phResults.set(t.phResults());
      m_phTargets.set(t.phTargets());
      m_phIds.set(Arrays.stream(t.phIds()).asLongStream().toArray());
      m_phTs.set(t.phTimestampSeconds());
      m_phStrategy.set(t.phStrategy());
      m_phPose.set(t.phPose());
      m_llTv.set(t.llTv());
      m_llTid.set(t.llTid());
      m_llTx.set(t.llTx());
      m_llTy.set(t.llTy());
      m_llTa.set(t.llTa());
      m_llFids.set(t.llFiducials());
      m_llMt1.set(t.llMt1());
      m_llMt2.set(t.llMt2());
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
