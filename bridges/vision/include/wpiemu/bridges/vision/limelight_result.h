#pragma once

// What one processed camera frame looks like on the Limelight's NetworkTables entries: a pure function from the
// vendor-neutral VisionFrame (emulator truth, camera frame X forward / Y left / Z up) plus what the robot asked of the
// camera to the values of the per-frame keys (limelight_keys.h). No NetworkTables here: the publisher writes the
// result, the tests inspect it.
//
// What the device reports, and which tags it uses
//   * tv / tx / ty / txnc / tync / ta / tid describe the PRIMARY target: the largest tag in the image (a frame lists
//     its tags by area), or the priority tag when the robot set one (priorityid; no valid target while that tag is
//     out of view).
//   * rawfiducials lists every detected tag, unfiltered, in the order of the frame.
//   * Localization (botpose*, MegaTag1 and MegaTag2) only uses the tags the robot allows with
//     fiducial_id_filters_set (all when it set none).
//       - MegaTag1 takes the camera pose from the frame's multi-tag solution when it is valid and every one of its
//         tags is allowed, otherwise from the largest allowed tag alone (the robot process has no PnP solver). Its
//         arrays list the tags it used.
//       - MegaTag2 solves the position from the largest allowed tag with the orientation the robot supplied
//         (limelight_math.h). Like the device it reports every allowed tag in its statistics and per-tag entries.
//   * Both estimates place the robot with the camera pose the device BELIEVES (settings.assumedRobotToCamera) while
//     the observations come from the physical camera in the frame: a wrong assumed mount skews them as on a real robot.
//   * A frame that detects nothing, and a driver-camera frame (vision processing is off), is still a result: tv 0,
//     tid -1, zeros for the pose, tag count 0 and the frame's latency in the arrays (11 values, an empty per-tag
//     part), an empty rawfiducials. A real device shows an all-zero botpose then; the latency entry is our assumption.
//   * Latency: cl = frame.captureLatencyUs (clamped to the latency), tl = the rest of frame.latencyUs, so the latency
//     in the arrays is tl + cl.
//   * A degenerate solve (non-finite pose) is "no estimate": tag count 0, never NaN.

#include <span>
#include <vector>

#include "wpiemu/bridges/vision/limelight_math.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::bridges::vision::limelight {

struct ResultSettings {
    Pose3d assumedRobotToCamera;    // the mount the device believes in (camerapose_robotspace_set, else the default)
    RobotOrientation orientation;   // the robot's orientation at the capture time (robot_orientation_set), MegaTag2
    std::span<const int> idFilter;  // fiducial_id_filters_set: tags allowed to localize; empty = every tag
    int priorityId = kPriorityIdNone;  // priorityid
    bool driverMode = false;           // camMode = driver: vision processing is off
};

struct LimelightResult {
    bool hasTarget = false;  // tv
    double txDeg = 0.0;      // tx and txnc (the crosshair is at the principal pixel)
    double tyDeg = 0.0;      // ty and tync
    double taPct = 0.0;
    double tid = kNoTagId;
    double tlMs = 0.0;
    double clMs = 0.0;
    // Each one is kBotposeHeaderSize + kBotposeTagStride * tagCount doubles.
    std::vector<double> botpose;            // MegaTag1, field-centre origin
    std::vector<double> botposeWpiBlue;     // MegaTag1, WPILib blue origin
    std::vector<double> botposeWpiRed;      // MegaTag1, WPILib red origin
    std::vector<double> botposeOrb;         // MegaTag2, field-centre origin
    std::vector<double> botposeOrbWpiBlue;  // MegaTag2, WPILib blue origin
    std::vector<double> botposeOrbWpiRed;   // MegaTag2, WPILib red origin
    std::vector<double> rawFiducials;       // kRawFiducialsStride doubles per detected tag
};

LimelightResult computeResult(const sdk::VisionFrame& frame, const ResultSettings& settings);

}  // namespace wpiemu::bridges::vision::limelight
