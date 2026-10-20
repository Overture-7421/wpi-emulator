#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "nt_sink.h"
#include "vision_math.h"
#include "wpiemu/bridges/vision/vision_bridges.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace wpiemu::bridges::vision {

// Limelight (LLOS 2026 / LimelightHelpers v1.14) NetworkTables presentation of simulated AprilTag cameras.
// Table: "limelight" (empty CameraConfig::name) or "limelight-<name>".
//
// Published per frame (timestamp = backend.nowUs(), the publish time; tl + cl = publish - capture in ms, so
// FPGATimestamp - (tl+cl)/1000 is the capture time, which is how LimelightHelpers derives it):
//   tv tid tx ty ta txnc tync tl cl t2d tcornxy json
//   botpose botpose_wpiblue botpose_wpired botpose_orb botpose_orb_wpiblue botpose_orb_wpired
//       (11 + 7 * tagCount: x y z roll pitch yaw latencyMs tagCount tagSpan avgDist avgArea, then per tag
//        id txnc tync ta distToCamera distToRobot ambiguity; empty when no tag / no robot pose)
//   rawfiducials (7 per tag) stddevs (12) camerapose_robotspace botpose_targetspace camerapose_targetspace
//   targetpose_cameraspace targetpose_robotspace (6 each)
// Every few ticks of simulated time: hb. Mirrored from the robot: pipeline -> getpipe (all pipelines are
// AprilTag pipelines). ledMode and robot_orientation_set are accepted and ignored: LEDs do not influence
// fiducial detection and the simulated MegaTag2 result is the ground-truth pose estimate anyway.
class LimelightBridge final : public sdk::IVisionBridge {
public:
    LimelightBridge(std::shared_ptr<INtBackend> backend, VisionBridgeOptions options);

    std::string_view name() const override { return "vision.limelight"; }
    sdk::Status start(sdk::ILogger& logger) override;
    void configure(const sdk::VisionSetup& setup) override;
    void poll(sdk::SimTimeUs simTimeUs) override;
    void apply(std::span<const sdk::CameraFrame> frames, sdk::SimTimeUs simTimeUs) override;
    void reset() override;
    void stop() noexcept override;

private:
    struct Cam {
        sdk::CameraConfig config;
        std::string root;  // "/limelight-front"
        bool initialized = false;
        double pipeline = 0;
        double heartbeat = 0;
        sdk::SimTimeUs nextHeartbeatUs = 0;
        sdk::Pose3 robotToCamera;  // last captured, for camerapose_robotspace
        bool hasRobotToCamera = false;
    };

    Cam* camera(std::uint32_t index);
    std::optional<sdk::Pose3> poseInOrigin(const sdk::Pose3& blue, detail::FieldOrigin origin);
    bool ensureInitialized(Cam& cam, std::int64_t ntNow);
    // synthetic: an empty no-target frame (initial state / reset), not a camera capture.
    void publishFrame(Cam& cam, const sdk::CameraFrame& frame, std::int64_t ntNow, bool synthetic);

    detail::NtSink sink_;
    VisionBridgeOptions options_;
    double fieldLengthM_ = 0.0;   // from VisionSetup; valid only while fieldSizeKnown_
    double fieldWidthM_ = 0.0;
    bool fieldSizeKnown_ = false;
    bool fieldWarningLogged_ = false;
    std::vector<std::optional<Cam>> cams_;
    bool started_ = false;
};

}  // namespace wpiemu::bridges::vision
