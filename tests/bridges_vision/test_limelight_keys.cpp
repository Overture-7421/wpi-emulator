#include <doctest/doctest.h>

#include <algorithm>
#include <set>
#include <string>
#include <string_view>

#include "wpiemu/bridges/vision/limelight_keys.h"

// The pin of the Limelight key table: every name and index below is what the vendored LimelightHelpers
// (samples/vision_robot, both languages) reads or writes. The numbers are asserted here on purpose, the vendor file is
// never read at run time; the comments point at the helper function that fixes each one.

namespace {

namespace ll = wpiemu::bridges::vision::limelight;

TEST_CASE("limelight keys: the botpose array layout is the one LimelightHelpers.getBotPoseEstimate parses") {
    // getBotPoseEstimate: toPose2D(poseArray) reads x, y and the yaw at index 5 (it needs at least 6 values);
    // latency = poseArray[6], tagCount = [7], tagSpan = [8], tagDist = [9], tagArea = [10]; the per-tag part starts at
    // 11 with 7 values per fiducial (valsPerFiducial) and is accepted only when size == 11 + 7 * tagCount.
    CHECK(ll::kBotposeX == 0);
    CHECK(ll::kBotposeY == 1);
    CHECK(ll::kBotposeZ == 2);
    CHECK(ll::kBotposeRollDeg == 3);
    CHECK(ll::kBotposePitchDeg == 4);
    CHECK(ll::kBotposeYawDeg == 5);
    CHECK(ll::kBotposeLatencyMs == 6);
    CHECK(ll::kBotposeTagCount == 7);
    CHECK(ll::kBotposeTagSpanM == 8);
    CHECK(ll::kBotposeAvgTagDistM == 9);
    CHECK(ll::kBotposeAvgTagAreaPct == 10);
    CHECK(ll::kBotposeHeaderSize == 11);
    CHECK(ll::kBotposeTagStride == 7);
}

TEST_CASE("limelight keys: the per-tag block is the one getBotPoseEstimate and getRawFiducials parse") {
    // RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity) at baseIndex + 0..6; getRawFiducials uses
    // valsPerEntry = 7 and returns nothing when the size is not a multiple of it.
    CHECK(ll::kTagId == 0);
    CHECK(ll::kTagTxnc == 1);
    CHECK(ll::kTagTync == 2);
    CHECK(ll::kTagTa == 3);
    CHECK(ll::kTagDistToCameraM == 4);
    CHECK(ll::kTagDistToRobotM == 5);
    CHECK(ll::kTagAmbiguity == 6);
    CHECK(ll::kRawFiducialsStride == 7);
}

TEST_CASE("limelight keys: pose6 and orientation layouts match what the helpers write") {
    // setCameraPose_RobotSpace: entries = [forward, side, up, roll, pitch, yaw] -> camerapose_robotspace_set; toPose3D
    // reads the same six values for camerapose_robotspace.
    CHECK(ll::kPose6Forward == 0);
    CHECK(ll::kPose6Right == 1);
    CHECK(ll::kPose6Up == 2);
    CHECK(ll::kPose6RollDeg == 3);
    CHECK(ll::kPose6PitchDeg == 4);
    CHECK(ll::kPose6YawDeg == 5);
    CHECK(ll::kPose6Size == 6);
    // SetRobotOrientation: entries = [yaw, yawRate, pitch, pitchRate, roll, rollRate] -> robot_orientation_set.
    CHECK(ll::kOrientationYawDeg == 0);
    CHECK(ll::kOrientationYawRateDegPerS == 1);
    CHECK(ll::kOrientationPitchDeg == 2);
    CHECK(ll::kOrientationPitchRateDegPerS == 3);
    CHECK(ll::kOrientationRollDeg == 4);
    CHECK(ll::kOrientationRollRateDegPerS == 5);
    CHECK(ll::kOrientationSize == 6);
    // hw: [cpu temp, cpu usage, ram usage, fps] (complete NetworkTables API table)
    CHECK(ll::kHwFps == 3);
    CHECK(ll::kHwSize == 4);
}

TEST_CASE("limelight keys: the keys the helpers read are spelled exactly") {
    CHECK(ll::kTv == "tv");          // getTV: getDouble == 1.0
    CHECK(ll::kTx == "tx");
    CHECK(ll::kTy == "ty");
    CHECK(ll::kTxnc == "txnc");
    CHECK(ll::kTync == "tync");
    CHECK(ll::kTa == "ta");
    CHECK(ll::kTl == "tl");
    CHECK(ll::kCl == "cl");
    CHECK(ll::kTid == "tid");        // getFiducialID
    CHECK(ll::kHb == "hb");          // getHeartbeat
    CHECK(ll::kGetPipe == "getpipe");
    CHECK(ll::kGetPipeType == "getpipetype");
    CHECK(ll::kBotpose == "botpose");
    CHECK(ll::kBotposeWpiBlue == "botpose_wpiblue");           // getBotPoseEstimate_wpiBlue
    CHECK(ll::kBotposeWpiRed == "botpose_wpired");             // getBotPoseEstimate_wpiRed
    CHECK(ll::kBotposeOrb == "botpose_orb");                   // LimelightResults json key
    CHECK(ll::kBotposeOrbWpiBlue == "botpose_orb_wpiblue");    // getBotPoseEstimate_wpiBlue_MegaTag2
    CHECK(ll::kBotposeOrbWpiRed == "botpose_orb_wpired");      // getBotPoseEstimate_wpiRed_MegaTag2
    CHECK(ll::kRawFiducials == "rawfiducials");                // getRawFiducials
    CHECK(ll::kCameraPoseRobotSpace == "camerapose_robotspace");  // getCameraPose3d_RobotSpace
    CHECK(ll::kHw == "hw");
}

TEST_CASE("limelight keys: the keys the helpers write are all on the robot-written list") {
    CHECK(ll::kPipeline == "pipeline");                        // setPipelineIndex
    CHECK(ll::kPriorityId == "priorityid");                    // setPriorityTagID
    CHECK(ll::kLedMode == "ledMode");                          // setLEDMode_*
    CHECK(ll::kStream == "stream");                            // setStreamMode_*
    CHECK(ll::kCrop == "crop");                                // setCropWindow
    CHECK(ll::kKeystoneSet == "keystone_set");                 // setKeystone
    CHECK(ll::kFiducialOffsetSet == "fiducial_offset_set");    // setFiducial3DOffset
    CHECK(ll::kRobotOrientationSet == "robot_orientation_set");  // SetRobotOrientation
    CHECK(ll::kImuModeSet == "imumode_set");                   // SetIMUMode
    CHECK(ll::kImuAssistAlphaSet == "imuassistalpha_set");     // SetIMUAssistAlpha
    CHECK(ll::kThrottleSet == "throttle_set");                 // SetThrottle
    CHECK(ll::kFiducialIdFiltersSet == "fiducial_id_filters_set");        // SetFiducialIDFiltersOverride
    CHECK(ll::kFiducialDownscaleSet == "fiducial_downscale_set");         // SetFiducialDownscalingOverride
    CHECK(ll::kCameraPoseRobotSpaceSet == "camerapose_robotspace_set");   // setCameraPose_RobotSpace
    CHECK(ll::kLlRobot == "llrobot");                          // setPythonScriptData
    CHECK(ll::kSnapshot == "snapshot");                        // triggerSnapshot
    CHECK(ll::kRewindEnableSet == "rewind_enable_set");        // setRewindEnabled
    CHECK(ll::kCaptureRewind == "capture_rewind");             // triggerRewindCapture
    CHECK(ll::kCamMode == "camMode");                          // older API, still on the device

    for (const std::string_view key :
         {"pipeline", "priorityid", "ledMode", "stream", "crop", "keystone_set", "fiducial_offset_set",
          "robot_orientation_set", "imumode_set", "imuassistalpha_set", "throttle_set", "fiducial_id_filters_set",
          "fiducial_downscale_set", "camerapose_robotspace_set", "llrobot", "snapshot", "rewind_enable_set",
          "capture_rewind", "camMode"})
        CHECK_MESSAGE(ll::isRobotWrittenKey(key), key);
    CHECK(ll::kRobotWrittenKeys.size() == 19);
}

TEST_CASE("limelight keys: what the device publishes and what the robot writes never overlap") {
    std::set<std::string_view> published(ll::kPerFrameKeys.begin(), ll::kPerFrameKeys.end());
    published.insert(ll::kHw);
    CHECK(published.size() == ll::kPerFrameKeys.size() + 1);  // no duplicates
    for (const std::string_view key : ll::kRobotWrittenKeys) CHECK_MESSAGE(published.count(key) == 0, key);
    for (const std::string_view key : ll::kForeignProbeKeys) {
        const auto& all = ll::kPerFrameKeys;
        const bool perFrame = std::find(all.begin(), all.end(), key) != all.end();
        CHECK_MESSAGE(perFrame, key);
    }
    CHECK_FALSE(ll::isRobotWrittenKey("tv"));
    CHECK_FALSE(ll::isRobotWrittenKey("botpose_wpiblue"));
    CHECK_FALSE(ll::isRobotWrittenKey("camerapose_robotspace"));  // the echo is ours, only the _set key is the robot's
}

TEST_CASE("limelight keys: table and topic helpers") {
    CHECK(ll::tableFor("") == "limelight");  // sanitizeName
    CHECK(ll::tableFor("limelight-rear") == "limelight-rear");
    CHECK(ll::topicOf("limelight-rear", "tv") == "/limelight-rear/tv");
    CHECK(ll::kTopicPrefix == "/limelight");

    CHECK(ll::isLimelightTable("limelight"));
    CHECK(ll::isLimelightTable("limelight-rear"));
    CHECK_FALSE(ll::isLimelightTable("limelight-"));
    CHECK_FALSE(ll::isLimelightTable("limelightfoo"));
    CHECK_FALSE(ll::isLimelightTable("photonvision"));

    std::string_view table;
    std::string_view key;
    REQUIRE(ll::splitTopic("/limelight-rear/pipeline", table, key));
    CHECK(table == "limelight-rear");
    CHECK(key == "pipeline");
    CHECK_FALSE(ll::splitTopic("limelight/pipeline", table, key));
    CHECK_FALSE(ll::splitTopic("/limelight", table, key));
    CHECK_FALSE(ll::splitTopic("/limelight/", table, key));
    CHECK_FALSE(ll::splitTopic("//pipeline", table, key));
    CHECK_FALSE(ll::splitTopic("/limelight/a/b", table, key));
}

TEST_CASE("limelight keys: the field and value constants") {
    CHECK(ll::kWpiFieldLengthM == doctest::Approx(16.541));
    CHECK(ll::kWpiFieldWidthM == doctest::Approx(8.069));
    CHECK(ll::kNoTagId == -1.0);
    CHECK(ll::kHeartbeatWrap == 2'000'000'000);
    CHECK(ll::kPipelineTypeFiducial == "pipe_fiducial");
}

}  // namespace
