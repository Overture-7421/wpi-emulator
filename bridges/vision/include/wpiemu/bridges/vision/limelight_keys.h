#pragma once

// THE Limelight NetworkTables contract, in one place (plan section 13, "one key table"). Every topic name, array layout
// and index the Limelight publisher uses is a named constant below; no other file holds a topic string or an array
// index literal. A Limelight OS or LimelightHelpers update is therefore an edit of this header plus its pin test
// (tests/bridges_vision/test_limelight_keys.cpp).
//
// Facts only, taken from the public contract (no vendor code or firmware is reproduced):
//   [NT]  https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
//   [CS]  https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
//   [MT2] https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
//   [LH]  the unmodified LimelightHelpers vendored by the sample robots (samples/vision_robot/{java,cpp}, MIT): the
//         exact keys and indices robot code reads and writes. The acceptance fixture, never a source of code.
//   [ADR] docs/decisions/0017-vision-virtual-coprocessors.md and the nt-inprocess spike of 2026-10-17, where the real
//         LimelightHelpers (Java and C++) read what this publisher wrote.
//
// Scalars are NT doubles (what the device publishes and LH reads), arrays are double[]; angles are degrees, distances
// metres, latencies milliseconds.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace wpiemu::bridges::vision::limelight {

// ---- tables ----------------------------------------------------------------------------------------------------
// A device publishes into the NT table named after it: "limelight", or "limelight-<nickname>" once renamed [NT].
inline constexpr std::string_view kTopicPrefix = "/limelight";  // IVisionVendor::topicPrefix()
inline constexpr std::string_view kDefaultTable = "limelight";
inline constexpr std::string_view kNamedTablePrefix = "limelight-";

constexpr bool isLimelightTable(std::string_view table) {
    return table == kDefaultTable || (table.size() > kNamedTablePrefix.size() &&
                                      table.substr(0, kNamedTablePrefix.size()) == kNamedTablePrefix);
}
// LH maps the empty name to "limelight" (LimelightHelpers sanitizeName); so does the bridge.
inline std::string tableFor(std::string_view ntName) { return std::string(ntName.empty() ? kDefaultTable : ntName); }
// "/<table>/<key>"
inline std::string topicOf(std::string_view table, std::string_view key) {
    std::string topic;
    topic.reserve(table.size() + key.size() + 2);
    topic += '/';
    topic += table;
    topic += '/';
    topic += key;
    return topic;
}
// Splits exactly "/<table>/<key>" (one level, both parts non-empty); false for any other topic.
constexpr bool splitTopic(std::string_view topic, std::string_view& table, std::string_view& key) {
    if (topic.size() < 4 || topic.front() != '/') return false;
    const std::string_view rest = topic.substr(1);
    const std::size_t slash = rest.find('/');
    if (slash == std::string_view::npos || slash == 0 || slash + 1 >= rest.size()) return false;
    const std::string_view leaf = rest.substr(slash + 1);
    if (leaf.find('/') != std::string_view::npos) return false;
    table = rest.substr(0, slash);
    key = leaf;
    return true;
}

// ---- keys the device publishes, once per processed frame -------------------------------------------------------
// All of them are published with keepDuplicates and the frame's own timestamp: ideal-mode arrays repeat frame after
// frame, ntcore would suppress them and LH's timestamp would freeze (spike pitfall 6).
inline constexpr std::string_view kTv = "tv";      // 1.0 valid target, else 0.0 (a double: LH tests getDouble == 1.0)
inline constexpr std::string_view kTx = "tx";      // deg, positive = target right of the crosshair [NT]
inline constexpr std::string_view kTy = "ty";      // deg, positive = up
inline constexpr std::string_view kTxnc = "txnc";  // deg from the principal pixel (the crosshair is centred: = tx)
inline constexpr std::string_view kTync = "tync";
inline constexpr std::string_view kTa = "ta";    // target area, % of the image
inline constexpr std::string_view kTl = "tl";    // pipeline latency, ms
inline constexpr std::string_view kCl = "cl";    // capture latency, ms; tl + cl is the total latency LH subtracts
inline constexpr std::string_view kTid = "tid";  // id of the primary tag, kNoTagId when none
inline constexpr std::string_view kHb = "hb";    // heartbeat: +1 per processed frame, wraps at kHeartbeatWrap
inline constexpr std::string_view kGetPipe = "getpipe";          // active pipeline index (follows the robot's request)
inline constexpr std::string_view kGetPipeType = "getpipetype";  // a string
inline constexpr std::string_view kBotpose = "botpose";                 // MegaTag1, field-centre origin [CS]
inline constexpr std::string_view kBotposeWpiBlue = "botpose_wpiblue";  // MegaTag1, WPILib blue origin
inline constexpr std::string_view kBotposeWpiRed = "botpose_wpired";    // MegaTag1, WPILib red origin (rotated 180 deg)
inline constexpr std::string_view kBotposeOrb = "botpose_orb";          // MegaTag2, field-centre origin
inline constexpr std::string_view kBotposeOrbWpiBlue = "botpose_orb_wpiblue";
inline constexpr std::string_view kBotposeOrbWpiRed = "botpose_orb_wpired";
inline constexpr std::string_view kRawFiducials = "rawfiducials";                   // kRawFiducialsStride per tag
inline constexpr std::string_view kCameraPoseRobotSpace = "camerapose_robotspace";  // follows camerapose_robotspace_set

// Written once, at start: hardware metrics with nominal values (the emulator has no hardware to measure).
inline constexpr std::string_view kHw = "hw";  // [cpu temp C, cpu usage %, ram usage %, fps]

inline constexpr std::array<std::string_view, 20> kPerFrameKeys = {
    kTv,
    kTx,
    kTy,
    kTxnc,
    kTync,
    kTa,
    kTl,
    kCl,
    kTid,
    kHb,
    kGetPipe,
    kGetPipeType,
    kBotpose,
    kBotposeWpiBlue,
    kBotposeWpiRed,
    kBotposeOrb,
    kBotposeOrbWpiBlue,
    kBotposeOrbWpiRed,
    kRawFiducials,
    kCameraPoseRobotSpace,
};

// A publisher of any of these that exists before ours is a robot-side simulation of the camera (a PhotonCameraSim-style
// class): the camera is then left alone (camera_status::kForeignPublisher).
inline constexpr std::array<std::string_view, 2> kForeignProbeKeys = {kTv, kBotposeWpiBlue};

// ---- keys the ROBOT writes (through LH or raw NetworkTables) ----------------------------------------------------
// The device reads these. A table holding any of them, created by the robot, belongs to a robot that uses the camera.
// Discovery never looks at the per-frame keys above: the bridge's own publications must not discover themselves.
// Each comment gives the value and the LH function that writes the key.
inline constexpr std::string_view kPipeline = "pipeline";  // 0..9; setPipelineIndex
inline constexpr std::string_view kLedMode = "ledMode";    // 0 pipeline, 1 off, 2 blink, 3 on; setLEDMode_*
inline constexpr std::string_view kCamMode = "camMode";    // 0 vision, 1 driver camera; older API [NT]
inline constexpr std::string_view kStream = "stream";      // 0..2; setStreamMode_*
inline constexpr std::string_view kSnapshot = "snapshot";  // a counter; triggerSnapshot
inline constexpr std::string_view kCrop = "crop";          // double[4] x0 x1 y0 y1; setCropWindow
inline constexpr std::string_view kKeystoneSet = "keystone_set";  // double[2]; setKeystone
inline constexpr std::string_view kPriorityId = "priorityid";     // tag id for tx/ty, -1 none; setPriorityTagID
inline constexpr std::string_view kRobotOrientationSet = "robot_orientation_set";  // double[6], kOrientation*
                                                                                    // SetRobotOrientation
inline constexpr std::string_view kImuModeSet = "imumode_set";                // 0..4; SetIMUMode
inline constexpr std::string_view kImuAssistAlphaSet = "imuassistalpha_set";  // SetIMUAssistAlpha
inline constexpr std::string_view kThrottleSet = "throttle_set";  // frames skipped between two processed; SetThrottle
inline constexpr std::string_view kFiducialIdFiltersSet = "fiducial_id_filters_set";  // valid ids, empty = all
                                                                                       // SetFiducialIDFiltersOverride
inline constexpr std::string_view kFiducialOffsetSet = "fiducial_offset_set";        // double[3]; setFiducial3DOffset
inline constexpr std::string_view kFiducialDownscaleSet = "fiducial_downscale_set";  // SetFiducialDownscalingOverride
inline constexpr std::string_view kCameraPoseRobotSpaceSet = "camerapose_robotspace_set";  // double[6], kPose6*
                                                                                            // setCameraPose_RobotSpace
inline constexpr std::string_view kLlRobot = "llrobot";                    // data for python pipelines
inline constexpr std::string_view kRewindEnableSet = "rewind_enable_set";  // setRewindEnabled
inline constexpr std::string_view kCaptureRewind = "capture_rewind";       // double[2]; triggerRewindCapture

inline constexpr std::array<std::string_view, 19> kRobotWrittenKeys = {
    kPipeline,
    kLedMode,
    kCamMode,
    kStream,
    kSnapshot,
    kCrop,
    kKeystoneSet,
    kPriorityId,
    kRobotOrientationSet,
    kImuModeSet,
    kImuAssistAlphaSet,
    kThrottleSet,
    kFiducialIdFiltersSet,
    kFiducialOffsetSet,
    kFiducialDownscaleSet,
    kCameraPoseRobotSpaceSet,
    kLlRobot,
    kRewindEnableSet,
    kCaptureRewind,
};

constexpr bool isRobotWrittenKey(std::string_view key) {
    for (const std::string_view known : kRobotWrittenKeys)
        if (known == key) return true;
    return false;
}

// ---- array layouts ---------------------------------------------------------------------------------------------
// botpose, botpose_wpiblue, botpose_wpired, botpose_orb, botpose_orb_wpiblue, botpose_orb_wpired [NT, LH
// getBotPoseEstimate]:
//   [x y z rollDeg pitchDeg yawDeg latencyMs tagCount tagSpanM avgTagDistM avgTagAreaPct], then kBotposeTagStride
//   doubles per tag. LH takes the per-tag part only when size == kBotposeHeaderSize + kBotposeTagStride * tagCount,
//   reads the pose from the first six values (it needs at least six) and treats an EMPTY array as "no data".
inline constexpr std::size_t kBotposeX = 0;
inline constexpr std::size_t kBotposeY = 1;
inline constexpr std::size_t kBotposeZ = 2;
inline constexpr std::size_t kBotposeRollDeg = 3;
inline constexpr std::size_t kBotposePitchDeg = 4;
inline constexpr std::size_t kBotposeYawDeg = 5;
inline constexpr std::size_t kBotposeLatencyMs = 6;  // tl + cl
inline constexpr std::size_t kBotposeTagCount = 7;
inline constexpr std::size_t kBotposeTagSpanM = 8;     // largest distance between two of the tags
inline constexpr std::size_t kBotposeAvgTagDistM = 9;  // mean distance from the camera to the tags
inline constexpr std::size_t kBotposeAvgTagAreaPct = 10;
inline constexpr std::size_t kBotposeHeaderSize = 11;
inline constexpr std::size_t kBotposeTagStride = 7;

// One tag: inside the botpose arrays (offset kBotposeHeaderSize + i * kBotposeTagStride) and in rawfiducials (offset
// i * kRawFiducialsStride) [NT, LH getRawFiducials]. rawfiducials is a plain concatenation whose size must be a
// multiple of the stride, otherwise LH returns nothing.
inline constexpr std::size_t kTagId = 0;
inline constexpr std::size_t kTagTxnc = 1;
inline constexpr std::size_t kTagTync = 2;
inline constexpr std::size_t kTagTa = 3;
inline constexpr std::size_t kTagDistToCameraM = 4;
inline constexpr std::size_t kTagDistToRobotM = 5;
inline constexpr std::size_t kTagAmbiguity = 6;
inline constexpr std::size_t kRawFiducialsStride = 7;

// camerapose_robotspace and camerapose_robotspace_set, in Limelight robot space (X forward, Y RIGHT, Z up, origin on
// the floor under the robot) [NT, CS, LH setCameraPose_RobotSpace]. The angle signs are fixed by the sample robots,
// which send a WPILib robot-to-camera transform as [x, -y, z, -roll, -pitch, -yaw] (samples/vision_robot/README.md).
inline constexpr std::size_t kPose6Forward = 0;
inline constexpr std::size_t kPose6Right = 1;
inline constexpr std::size_t kPose6Up = 2;
inline constexpr std::size_t kPose6RollDeg = 3;
inline constexpr std::size_t kPose6PitchDeg = 4;
inline constexpr std::size_t kPose6YawDeg = 5;
inline constexpr std::size_t kPose6Size = 6;

// robot_orientation_set [NT, MT2, LH SetRobotOrientation]: yaw is counter-clockwise positive, 0 = facing the red wall
// (WPILib blue frame).
inline constexpr std::size_t kOrientationYawDeg = 0;
inline constexpr std::size_t kOrientationYawRateDegPerS = 1;
inline constexpr std::size_t kOrientationPitchDeg = 2;
inline constexpr std::size_t kOrientationPitchRateDegPerS = 3;
inline constexpr std::size_t kOrientationRollDeg = 4;
inline constexpr std::size_t kOrientationRollRateDegPerS = 5;
inline constexpr std::size_t kOrientationSize = 6;  // a shorter array is accepted (yaw only is common), the rest is 0

// hw [NT]
inline constexpr std::size_t kHwCpuTempC = 0;
inline constexpr std::size_t kHwCpuUsagePct = 1;
inline constexpr std::size_t kHwRamUsagePct = 2;
inline constexpr std::size_t kHwFps = 3;
inline constexpr std::size_t kHwSize = 4;
// Nominal health of a healthy device; fps is the camera's frame rate.
inline constexpr double kHwNominalCpuTempC = 45.0;
inline constexpr double kHwNominalCpuUsagePct = 15.0;
inline constexpr double kHwNominalRamUsagePct = 30.0;

// ---- values ----------------------------------------------------------------------------------------------------
inline constexpr double kTvValid = 1.0;
inline constexpr double kTvNone = 0.0;
inline constexpr double kNoTagId = -1.0;  // tid without a target
inline constexpr int kPriorityIdNone = -1;
inline constexpr int kDefaultPipeline = 0;
inline constexpr int kCamModeDriver = 1;  // the camMode value that disables vision processing
inline constexpr std::string_view kPipelineTypeFiducial = "pipe_fiducial";  // getpipetype of an AprilTag pipeline
inline constexpr std::int64_t kHeartbeatWrap = 2'000'000'000;              // hb "resets at 2 billion" [NT]

// ---- field frames ----------------------------------------------------------------------------------------------
// The WPILib blue-origin field (the 2026 layout's "field" block, game_2026 plugin). The red-origin and field-centre
// flavours of every pose derive from it. The bridge only receives frames (no layout), so the size is a constant.
inline constexpr double kWpiFieldLengthM = 16.541;
inline constexpr double kWpiFieldWidthM = 8.069;

}  // namespace wpiemu::bridges::vision::limelight
