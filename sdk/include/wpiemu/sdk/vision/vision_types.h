#pragma once

// Vendor-neutral vision data exchanged between the emulator and the robot process (ADR 0017).
//
// The emulator (core/vision) computes a VisionFrame per processed camera frame from geometric truth
// only; the vision bridge in the robot process turns it into the vendor's NetworkTables entries.
// Everything a robot writes to a camera (pipeline, LED, driver mode, ...) stays in the bridge and
// comes back only as CameraRobotState, which is informational (discovery, UI, warnings) and never
// feeds back into the measurements.
//
// Frames follow camera_config.h: the camera frame is X forward / Y left / Z up, image pixels start
// at the top-left corner with y down. Tag corners are always listed BL, BR, TR, TL as seen from the
// tag front (counter-clockwise on screen, starting bottom-left), which is PhotonVision's
// detectedCorners order. Angles: yaw is positive for a target to the RIGHT of the optical axis,
// pitch positive for UP.
//
// Timing: the extension applies SensorFrame N before it steps HAL time, so a capture at sim time
// T(N0) rides SensorFrame N = N0 + 1 + k with VisionFrame::latencyUs = k * step (= T(N-1) - T(N0));
// the extension publishes at its own NT_Now() and stamps the capture as publish - latencyUs.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/vision/camera_config.h"

namespace wpiemu::sdk {

struct PixelPoint {
    double x = 0.0;
    double y = 0.0;

    bool operator==(const PixelPoint&) const = default;
};

struct FiducialObservation {
    int id = 0;
    std::array<PixelPoint, 4> cornersPx{};  // BL, BR, TR, TL; noisy only in realistic mode
    PixelPoint centerPx;
    double yawRad = 0.0;
    double pitchRad = 0.0;
    double skewRad = 0.0;
    double areaPct = 0.0;  // 100 * polygonArea(corners) / (width * height)
    double distanceM = 0.0;
    Pose3 fieldToTag;       // from the layout
    Pose3 cameraToTagBest;  // tag pose in the camera frame (best = the lower reprojection error)
    Pose3 cameraToTagAlt;
    double ambiguity = 0.0;  // reprojErrBest / reprojErrAlt in [0,1]; 0 = unambiguous
    double reprojErrBestPx = 0.0;
    double reprojErrAltPx = 0.0;

    bool operator==(const FiducialObservation&) const = default;
};

struct MultiTagSolution {
    bool valid = false;
    Pose3 fieldToCamera;
    double reprojErrPx = 0.0;
    std::vector<int> tagIds;

    bool operator==(const MultiTagSolution&) const = default;
};

// One processed camera frame, emulator -> robot process (SensorFrame::vision).
struct VisionFrame {
    std::string cameraId;
    CameraVendor vendor = CameraVendor::PhotonVision;
    std::string ntName;              // CameraConfig::networkName()
    std::uint64_t sequence = 0;      // per camera, starts at 1, never resets
    std::int64_t captureTimeUs = 0;  // sim time of the capture (informational)
    std::int64_t latencyUs = 0;      // EFFECTIVE capture-to-publish latency, a multiple of the step
    std::int64_t captureLatencyUs = 0;  // Limelight "cl" share (<= latencyUs)
    Pose3 robotToCamera;                // PHYSICAL camera pose in the chassis frame at capture
    // Sorted by areaPct descending; empty = nothing detected.
    std::vector<FiducialObservation> observations;
    MultiTagSolution multiTag;  // valid only when multiTag && >= 2 tags

    bool operator==(const VisionFrame&) const = default;
};

// One enabled camera as the robot process needs to know it (HelloAck::vision).
struct CameraSetup {
    std::string cameraId;
    CameraVendor vendor = CameraVendor::PhotonVision;
    std::string ntName;
    int widthPx = 0;
    int heightPx = 0;
    double fxPx = 0.0;
    double fyPx = 0.0;
    double cxPx = 0.0;
    double cyPx = 0.0;
    std::array<double, 5> distortion{};
    double frameRateHz = 30.0;
    std::int64_t latencyUs = 0;
    // Limelight, until the robot writes camerapose_robotspace_set.
    Pose3 defaultAssumedRobotToCamera;
    // "" => the bridge default (newest known); non-empty must have a bridge profile or the camera
    // is refused.
    std::string photonMessageUuid;

    bool operator==(const CameraSetup&) const = default;
};

struct VisionSetup {
    std::vector<CameraSetup> cameras;

    bool empty() const { return cameras.empty(); }
    bool operator==(const VisionSetup&) const = default;
};

// What the robot process observed about one camera (ActuatorFrame::vision): informational only.
struct CameraRobotState {
    std::string ntName;  // as seen on NetworkTables
    CameraVendor vendor = CameraVendor::PhotonVision;
    std::string cameraId;  // the configured camera it matched; "" = an unconfigured detection
    bool present = true;   // robot code uses this camera
    std::optional<std::int32_t> pipelineIndex;
    std::optional<std::int32_t> ledMode;
    std::optional<std::int32_t> imuMode;
    std::optional<bool> driverMode;
    std::optional<double> throttle;
    std::optional<double> fpsLimit;
    // yaw, yawRate, pitch, pitchRate, roll, rollRate (deg, deg/s)
    std::optional<std::array<double, 6>> robotOrientationDeg;
    // Limelight robot space as written: fwd, side, up, roll, pitch, yaw (m, deg)
    std::optional<std::array<double, 6>> cameraPoseOverride;
    bool publishing = false;
    // "" = ok | "ntcore_unavailable" | "photon_uuid_unsupported" | "foreign_publisher" | "disabled"
    std::string status;
    std::uint64_t framesPublished = 0;

    bool operator==(const CameraRobotState&) const = default;
};

}  // namespace wpiemu::sdk
