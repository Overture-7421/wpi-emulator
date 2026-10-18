// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include "LimelightHelpers.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  // One telemetry sample. Values that do not exist are -1 (counts), NaN (numbers) or empty
  // (vectors) and print as "-".
  struct Telemetry {
    int loop = 0;
    double fpgaSeconds = 0.0;
    double yawDeg = 0.0;
    bool phConnected = false;
    int phResults = 0;
    int phTargets = -1;
    std::vector<int> phIds;
    double phTimestampSeconds = std::numeric_limits<double>::quiet_NaN();
    std::string phStrategy = "-";
    std::vector<double> phPose;  // x, y, yawDeg
    bool llTv = false;
    int llTid = 0;
    double llTx = 0.0;
    double llTy = 0.0;
    double llTa = 0.0;
    int llFiducials = 0;
    std::vector<double> llMt1;  // x, y, yawDeg, tagCount, timestampSeconds
    std::vector<double> llMt2;

    std::string Line() const;
  };

  // The same values as the printed line, published under /VisionSample/.
  class Mirror {
   public:
    explicit Mirror(const std::shared_ptr<nt::NetworkTable>& table);
    void Publish(const Telemetry& t, const std::string& line);

   private:
    nt::StringPublisher m_line;
    nt::IntegerPublisher m_loop;
    nt::DoublePublisher m_fpga;
    nt::DoublePublisher m_yaw;
    nt::BooleanPublisher m_phConnected;
    nt::IntegerPublisher m_phResults;
    nt::IntegerPublisher m_phTargets;
    nt::IntegerArrayPublisher m_phIds;
    nt::DoublePublisher m_phTs;
    nt::StringPublisher m_phStrategy;
    nt::DoubleArrayPublisher m_phPose;
    nt::BooleanPublisher m_llTv;
    nt::IntegerPublisher m_llTid;
    nt::DoublePublisher m_llTx;
    nt::DoublePublisher m_llTy;
    nt::DoublePublisher m_llTa;
    nt::IntegerPublisher m_llFids;
    nt::DoubleArrayPublisher m_llMt1;
    nt::DoubleArrayPublisher m_llMt2;
  };

  void ApplyOperatorInputs();
  void DrainPhotonResults();
  void ReadLimelight(double yawDeg);
  void ReportTelemetry(double yawDeg);

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  ctre::phoenix6::hardware::Pigeon2 m_pigeon;
  frc::Joystick m_joystick;
  photon::PhotonCamera m_photon;
  photon::PhotonPoseEstimator m_photonEstimator;
  Mirror m_mirror;

  int m_loop = 0;

  // PhotonVision: newest result seen so far, the estimate made from it and how it was made.
  int m_photonResultsSincePrint = 0;
  int m_photonTargets = -1;
  std::vector<int> m_photonIds;
  double m_photonTimestampSeconds = std::numeric_limits<double>::quiet_NaN();
  std::string m_photonStrategy = "-";
  std::vector<double> m_photonPose;

  // Limelight: values read in the latest loop.
  bool m_llTv = false;
  int m_llTid = 0;
  double m_llTx = 0.0;
  double m_llTy = 0.0;
  double m_llTa = 0.0;
  int m_llFiducials = 0;
  std::vector<double> m_llMt1;
  std::vector<double> m_llMt2;
};
