#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "nt_sink.h"
#include "wpiemu/bridges/vision/vision_bridges.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace wpiemu::bridges::vision {

// PhotonVision coprocessor (PhotonLib) NetworkTables presentation of simulated AprilTag cameras.
// Root table "photonvision" (version, ledModeState), camera table "photonvision/<CameraConfig::name>".
//
// Per camera: rawBytes (raw topic, PhotonPipelineResult photonstruct, see photon_encoding.h), pipelineIndexState,
// driverMode, fpsLimit, heartbeat, cameraIntrinsics (3x3 row-major), cameraDistortion (8 zeros) and the legacy
// scalar topics (hasTarget, latencyMillis, fps, targetYaw/Pitch/Area/Skew, targetPixelsX/Y).
// Mirrored from the robot: pipelineIndexRequest -> pipelineIndexState, driverModeRequest -> driverMode,
// fpsLimitRequest -> fpsLimit, ledModeRequest -> ledModeState. In driver mode empty results are published.
//
// Timestamps: results are stamped with backend.nowUs() (publish time); the metadata carries
// publish = nowUs and capture = nowUs - (publishTimeUs - captureTimeUs), which is exactly what PhotonLib
// uses to derive the capture time in the robot's time base.
//
// Versions: the type string embeds the message-definition hash, which the robot's PhotonLib compares and
// throws on mismatch. Only versions whose definitions were verified are served; for any other
// VisionSetup::photonVersion an error is logged once and no PhotonVision camera is published.
class PhotonBridge final : public sdk::IVisionBridge {
public:
    PhotonBridge(std::shared_ptr<INtBackend> backend, VisionBridgeOptions options);

    std::string_view name() const override { return "vision.photonvision"; }
    sdk::Status start(sdk::ILogger& logger) override;
    void configure(const sdk::VisionSetup& setup) override;
    void poll(sdk::SimTimeUs simTimeUs) override;
    void apply(std::span<const sdk::CameraFrame> frames, sdk::SimTimeUs simTimeUs) override;
    void reset() override;
    void stop() noexcept override;

private:
    struct Cam {
        sdk::CameraConfig config;
        std::string root;  // "/photonvision/front"
        bool initialized = false;
        std::int64_t pipelineIndex = 0;
        bool driverMode = false;
        std::int64_t fpsLimit = -1;
        std::int64_t heartbeat = 0;
        sdk::SimTimeUs nextHeartbeatUs = 0;
        std::int64_t sequence = 0;  // for synthetic (empty) results
    };

    Cam* camera(std::uint32_t index);
    bool ensureInitialized(Cam& cam, std::int64_t ntNow);
    void publishFrame(Cam& cam, const sdk::CameraFrame* frame, std::int64_t ntNow);

    detail::NtSink sink_;
    VisionBridgeOptions options_;
    std::vector<std::optional<Cam>> cams_;
    std::string version_;
    bool enabled_ = true;
    std::string warnedVersion_;
    bool rootInitialized_ = false;
    std::int64_t ledMode_ = -1;
    bool started_ = false;
};

}  // namespace wpiemu::bridges::vision
