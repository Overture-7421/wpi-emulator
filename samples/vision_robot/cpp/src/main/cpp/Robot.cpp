// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>

#include <fmt/format.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/print.h>

namespace {

// Vision sample constants. The emulator's end-to-end tests depend on these exact values.
// Transforms are robot to camera in the WPILib frame: +X forward, +Y left, +Z up, with pitch
// positive nose down.
constexpr const char* kPhotonCameraName = "photon-front";
const frc::Transform3d kRobotToPhotonCamera{
    frc::Translation3d{units::meter_t{0.30}, units::meter_t{0.00}, units::meter_t{0.50}},
    frc::Rotation3d{units::degree_t{0.0}, units::degree_t{-20.0}, units::degree_t{0.0}}};

constexpr const char* kLimelightName = "limelight-rear";
// The angles stay in degrees so they reach the Limelight exactly, without a rotation round trip.
constexpr double kLimelightRollDeg = 0.0;
constexpr double kLimelightPitchDeg = -25.0;
constexpr double kLimelightYawDeg = 180.0;
const frc::Transform3d kRobotToLimelight{
    frc::Translation3d{units::meter_t{-0.30}, units::meter_t{0.00}, units::meter_t{0.45}},
    frc::Rotation3d{units::degree_t{kLimelightRollDeg}, units::degree_t{kLimelightPitchDeg},
                    units::degree_t{kLimelightYawDeg}}};

constexpr int kPigeonCanId = 0;
constexpr int kJoystickPort = 0;
constexpr int kPrintEveryLoops = 5;
constexpr double kYawOffsetDegPerAxis = 90.0;

std::vector<double> PoseValues(const frc::Pose2d& pose) {
  return {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
}

// A Limelight estimate with no tags is not an estimate.
std::vector<double> EstimateValues(const LimelightHelpers::PoseEstimate& estimate) {
  if (estimate.tagCount == 0) {
    return {};
  }
  return {estimate.pose.X().value(), estimate.pose.Y().value(),
          estimate.pose.Rotation().Degrees().value(), static_cast<double>(estimate.tagCount),
          estimate.timestampSeconds.value()};
}

std::string PoseText(const std::vector<double>& p) {
  return p.empty() ? "-" : fmt::format("{:.3f},{:.3f},{:.2f}", p[0], p[1], p[2]);
}

std::string EstimateText(const std::vector<double>& e) {
  return e.empty() ? "-"
                   : fmt::format("{:.3f},{:.3f},{:.2f},{},{:.3f}", e[0], e[1], e[2],
                                 static_cast<int>(e[3]), e[4]);
}

}  // namespace

std::string Robot::Telemetry::Line() const {
  std::string ids;
  for (int id : phIds) {
    if (!ids.empty()) {
      ids += ',';
    }
    ids += std::to_string(id);
  }
  return fmt::format(
      "VISION|loop={}|fpga={:.3f}|yaw={:.2f}|ph.connected={}|ph.results={}|ph.targets={}"
      "|ph.ids={}|ph.ts={}|ph.strategy={}|ph.pose={}|ll.tv={}|ll.tid={}|ll.tx={:.2f}"
      "|ll.ty={:.2f}|ll.ta={:.3f}|ll.fids={}|ll.mt1={}|ll.mt2={}",
      loop, fpgaSeconds, yawDeg, phConnected ? 1 : 0, phResults,
      phTargets < 0 ? "-" : std::to_string(phTargets), ids.empty() ? "-" : ids,
      std::isnan(phTimestampSeconds) ? "-" : fmt::format("{:.3f}", phTimestampSeconds),
      phStrategy, PoseText(phPose), llTv ? 1 : 0, llTid, llTx, llTy, llTa, llFiducials,
      EstimateText(llMt1), EstimateText(llMt2));
}

Robot::Mirror::Mirror(const std::shared_ptr<nt::NetworkTable>& table)
    : m_line{table->GetStringTopic("line").Publish()},
      m_loop{table->GetIntegerTopic("loop").Publish()},
      m_fpga{table->GetDoubleTopic("fpga").Publish()},
      m_yaw{table->GetDoubleTopic("yaw").Publish()},
      m_phConnected{table->GetBooleanTopic("ph/connected").Publish()},
      m_phResults{table->GetIntegerTopic("ph/results").Publish()},
      m_phTargets{table->GetIntegerTopic("ph/targets").Publish()},
      m_phIds{table->GetIntegerArrayTopic("ph/ids").Publish()},
      m_phTs{table->GetDoubleTopic("ph/ts").Publish()},
      m_phStrategy{table->GetStringTopic("ph/strategy").Publish()},
      m_phPose{table->GetDoubleArrayTopic("ph/pose").Publish()},
      m_llTv{table->GetBooleanTopic("ll/tv").Publish()},
      m_llTid{table->GetIntegerTopic("ll/tid").Publish()},
      m_llTx{table->GetDoubleTopic("ll/tx").Publish()},
      m_llTy{table->GetDoubleTopic("ll/ty").Publish()},
      m_llTa{table->GetDoubleTopic("ll/ta").Publish()},
      m_llFids{table->GetIntegerTopic("ll/fids").Publish()},
      m_llMt1{table->GetDoubleArrayTopic("ll/mt1").Publish()},
      m_llMt2{table->GetDoubleArrayTopic("ll/mt2").Publish()} {}

void Robot::Mirror::Publish(const Telemetry& t, const std::string& line) {
  m_line.Set(line);
  m_loop.Set(t.loop);
  m_fpga.Set(t.fpgaSeconds);
  m_yaw.Set(t.yawDeg);
  m_phConnected.Set(t.phConnected);
  m_phResults.Set(t.phResults);
  m_phTargets.Set(t.phTargets);
  m_phIds.Set(std::vector<int64_t>(t.phIds.begin(), t.phIds.end()));
  m_phTs.Set(t.phTimestampSeconds);
  m_phStrategy.Set(t.phStrategy);
  m_phPose.Set(t.phPose);
  m_llTv.Set(t.llTv);
  m_llTid.Set(t.llTid);
  m_llTx.Set(t.llTx);
  m_llTy.Set(t.llTy);
  m_llTa.Set(t.llTa);
  m_llFids.Set(t.llFiducials);
  m_llMt1.Set(t.llMt1);
  m_llMt2.Set(t.llMt2);
}

Robot::Robot()
    : m_pigeon{kPigeonCanId, ctre::phoenix6::CANBus::RoboRIO()},
      m_joystick{kJoystickPort},
      m_photon{kPhotonCameraName},
      m_photonEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded),
                        kRobotToPhotonCamera},
      m_mirror{nt::NetworkTableInstance::GetDefault().GetTable("VisionSample")} {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Joystick 0 is only there when a driver station provides it.
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Limelight robot space is +X forward, +Y right, +Z up, so Y and the angles change sign.
  LimelightHelpers::setCameraPose_RobotSpace(
      kLimelightName, kRobotToLimelight.X().value(), -kRobotToLimelight.Y().value(),
      kRobotToLimelight.Z().value(), -kLimelightRollDeg, -kLimelightPitchDeg, -kLimelightYawDeg);
  LimelightHelpers::SetIMUMode(kLimelightName, 1);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  ++m_loop;
  const double yawDeg = m_pigeon.GetYaw().GetValueAsDouble();

  ApplyOperatorInputs();
  DrainPhotonResults();
  // Axis 0 adds a heading error to what the Limelight is told, to emulate a wrong gyro.
  ReadLimelight(yawDeg + m_joystick.GetRawAxis(0) * kYawOffsetDegPerAxis);

  if (m_loop % kPrintEveryLoops == 0) {
    ReportTelemetry(yawDeg);
  }
}

// Buttons 1-4 of joystick 0 drive the camera settings; releasing a button restores them.
void Robot::ApplyOperatorInputs() {
  m_photon.SetDriverMode(m_joystick.GetRawButton(1));
  m_photon.SetPipelineIndex(m_joystick.GetRawButton(2) ? 1 : 0);
  if (m_joystick.GetRawButton(3)) {
    LimelightHelpers::setLEDMode_ForceOn(kLimelightName);
  } else {
    LimelightHelpers::setLEDMode_ForceOff(kLimelightName);
  }
  LimelightHelpers::setPipelineIndex(kLimelightName, m_joystick.GetRawButton(4) ? 1 : 0);
}

// Reads every PhotonVision result that arrived since the last loop.
void Robot::DrainPhotonResults() {
  const std::vector<photon::PhotonPipelineResult> results = m_photon.GetAllUnreadResults();
  m_photonResultsSincePrint += static_cast<int>(results.size());
  for (const photon::PhotonPipelineResult& result : results) {
    std::optional<photon::EstimatedRobotPose> estimate =
        m_photonEstimator.EstimateCoprocMultiTagPose(result);
    std::string strategy = "multi";
    if (!estimate) {
      estimate = m_photonEstimator.EstimateLowestAmbiguityPose(result);
      strategy = "ambiguity";
    }

    m_photonTargets = static_cast<int>(result.GetTargets().size());
    m_photonIds.clear();
    for (const photon::PhotonTrackedTarget& target : result.GetTargets()) {
      m_photonIds.push_back(target.GetFiducialId());
    }
    m_photonTimestampSeconds = result.GetTimestamp().value();
    if (estimate) {
      m_photonStrategy = strategy;
      m_photonPose = PoseValues(estimate->estimatedPose.ToPose2d());
    } else {
      m_photonStrategy = "-";
      m_photonPose.clear();
    }
  }
}

void Robot::ReadLimelight(double yawDeg) {
  LimelightHelpers::SetRobotOrientation(kLimelightName, yawDeg, 0, 0, 0, 0, 0);
  m_llTv = LimelightHelpers::getTV(kLimelightName);
  m_llTid = static_cast<int>(LimelightHelpers::getFiducialID(kLimelightName));
  m_llTx = LimelightHelpers::getTX(kLimelightName);
  m_llTy = LimelightHelpers::getTY(kLimelightName);
  m_llTa = LimelightHelpers::getTA(kLimelightName);
  m_llMt1 = EstimateValues(LimelightHelpers::getBotPoseEstimate_wpiBlue(kLimelightName));
  m_llMt2 = EstimateValues(LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName));
  m_llFiducials = static_cast<int>(LimelightHelpers::getRawFiducials(kLimelightName).size());
}

void Robot::ReportTelemetry(double yawDeg) {
  Telemetry t;
  t.loop = m_loop;
  t.fpgaSeconds = frc::Timer::GetFPGATimestamp().value();
  t.yawDeg = yawDeg;
  t.phConnected = m_photon.IsConnected();
  t.phResults = m_photonResultsSincePrint;
  t.phTargets = m_photonTargets;
  t.phIds = m_photonIds;
  t.phTimestampSeconds = m_photonTimestampSeconds;
  t.phStrategy = m_photonStrategy;
  t.phPose = m_photonPose;
  t.llTv = m_llTv;
  t.llTid = m_llTid;
  t.llTx = m_llTx;
  t.llTy = m_llTy;
  t.llTa = m_llTa;
  t.llFiducials = m_llFiducials;
  t.llMt1 = m_llMt1;
  t.llMt2 = m_llMt2;
  m_photonResultsSincePrint = 0;

  const std::string line = t.Line();
  std::puts(line.c_str());
  std::fflush(stdout);
  m_mirror.Publish(t, line);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  wpi::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
