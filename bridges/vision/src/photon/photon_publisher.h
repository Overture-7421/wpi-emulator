#pragma once

// The PhotonVision camera publisher and the state its cameras share (private to src/photon: tests reach it through
// makePhotonVendor()).

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/bridges/vision/camera_publisher.h"
#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/bridges/vision/photon_codec.h"

namespace wpiemu::bridges::vision {

// What the PhotonVision cameras of one vendor object share:
//  * the topics PhotonVision keeps at the ROOT of /photonvision/ (not per camera): the coprocessor's version text
//    and the LED mode. They exist once however many cameras there are, and the robot's LED request is read once;
//  * a small memory per camera table, so that a camera created again for the same table (the bridge rebuilds its
//    cameras when a reconnect brings another setup) continues its heartbeat and timestamps instead of restarting them.
class PhotonSharedState {
public:
    struct CameraMemory {
        std::int64_t heartbeat = 0;
        std::int64_t lastPublishUs = 0;
    };

    // Reads the robot's LED requests (call it from every camera's start(), the first call subscribes).
    void subscribe(INtApi& nt);
    // Creates /photonvision/version and /photonvision/ledModeState once, on behalf of the first camera that may
    // publish. `versionText` must not be empty (PhotonLib's C++ camera treats an empty version as "no coprocessor").
    void publish(INtApi& nt, std::string_view versionText);
    // Drains the LED requests and echoes the newest into ledModeState (when that topic exists).
    void poll(INtApi& nt);

    std::optional<std::int32_t> ledMode() const { return ledMode_; }

    CameraMemory recall(const std::string& table) const;
    void remember(const std::string& table, const CameraMemory& memory) { memory_[table] = memory; }

private:
    // The handles belong to one INtApi; a different one (tests reuse a vendor object) starts over.
    void adopt(INtApi& nt);

    INtApi* nt_ = nullptr;
    NtPublisher version_;
    NtPublisher ledModeState_;
    NtSubscriber ledModeRequest_;
    std::optional<std::int32_t> ledMode_;
    std::vector<NtValue> inbox_;
    std::map<std::string, CameraMemory> memory_;
};

// One PhotonVision camera: /photonvision/<ntName>/... Inert (publishes nothing) when its message profile is
// unusable or another publisher already owns its result topic; it still reads the robot's requests.
class PhotonPublisher final : public ICameraPublisher {
public:
    // `codec` is null when the camera has no usable profile; `unsupportedReason` then says why.
    PhotonPublisher(INtApi& nt, sdk::ILogger& log, const sdk::CameraSetup& setup,
                    std::shared_ptr<const PhotonCodec> codec, std::string unsupportedReason,
                    std::shared_ptr<PhotonSharedState> shared);

    const sdk::CameraSetup& setup() const override { return setup_; }
    void start() override;
    void poll() override;
    void publish(const sdk::VisionFrame& frame) override;
    sdk::CameraRobotState robotState() const override;

private:
    struct Topics {
        NtPublisher result, heartbeat, latencyMillis, fps, hasTarget;
        NtPublisher targetPitch, targetYaw, targetArea, targetSkew, targetPixelsX, targetPixelsY, targetPose;
        NtPublisher pipelineIndexState, driverMode, fpsLimit, cameraIntrinsics, cameraDistortion;
    };

    void refuse(std::string_view status, const std::string& why);
    bool createTopics();
    void publishStaticValues(std::int64_t timeUs);
    void pollRequests();
    bool publishing() const { return started_ && !inert_; }

    INtApi& nt_;
    sdk::ILogger& log_;
    sdk::CameraSetup setup_;
    std::shared_ptr<const PhotonCodec> codec_;
    std::string unsupportedReason_;
    std::shared_ptr<PhotonSharedState> shared_;

    std::string table_;  // "/photonvision/<ntName>/"
    bool started_ = false;
    bool inert_ = false;
    std::string status_;

    Topics topics_;
    NtSubscriber pipelineRequest_;
    NtSubscriber driverModeRequest_;
    NtSubscriber fpsLimitRequest_;
    std::string requestTopics_[3];

    bool present_ = false;
    std::optional<std::int32_t> pipelineIndex_;
    std::optional<bool> driverMode_;
    std::optional<double> fpsLimit_;

    std::int64_t lastPublishUs_ = 0;
    std::int64_t heartbeat_ = 0;
    std::uint64_t framesPublished_ = 0;
    bool encodeFailureLogged_ = false;

    std::vector<std::uint8_t> packet_;
    std::vector<std::uint8_t> poseBytes_;
    std::vector<NtValue> inbox_;
};

}  // namespace wpiemu::bridges::vision
