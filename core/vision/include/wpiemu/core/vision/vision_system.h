#pragma once

// The vision engine (ADR 0017): what every simulated camera sees, as vendor-neutral VisionFrames.
//
// A pure function of truth (the chassis pose, mechanism link poses, the tag layout, line of sight
// in the physics world), the camera configuration, the sim clock and SimContext::rng(). It never
// reads anything the robot wrote: what the robot reports about its cameras (RobotCameraStates) is
// shown next to the cameras in the snapshot, never used for measuring. All the vendor behaviour
// (topics, pipelines, MegaTag, ...) lives in the robot-process bridge.
//
// Timing (vision spec, section D). T(N) is SensorFrame::simTimeUs of tick N (time after step N).
//  * Capture: at tick N0, after the physics/game step, a camera whose accumulator is due captures
//    at T(N0). The accumulator adds the step each tick and subtracts one frame period when due,
//    so the long-run rate is exact; at most one capture per tick and per camera.
//  * Latency: L = latencyUs (+ uniform jitter in realistic mode), k = ceil(L / step). The frame
//    rides SensorFrame N0 + 1 + k and VisionFrame::latencyUs = k * step = T(N-1) - T(N0): the
//    extension applies a SensorFrame before it steps HAL time, so its clock reads T(N-1) when it
//    publishes. Jitter never reorders the frames of one camera: a frame is published no earlier
//    than its predecessor (its effective latency grows to match, so latencyUs stays exact).
//  * Randomness, only when the parameter is non-zero and in camera config order: per capture
//    latency jitter (1 draw), dropout (1 draw), corner noise (2 draws per corner, Box-Muller). An
//    ideal camera never touches the generator.
//
// Threading: sim thread only, like everything that reads the physics world.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "wpiemu/core/vision/camera_model.h"
#include "wpiemu/core/vision/robot_camera_states.h"
#include "wpiemu/core/vision/visibility.h"
#include "wpiemu/core/vision/vision_math.h"
#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/fiducial_layout.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::core::vision {

// Limits of the IPC codec (ipc/binary_codec.h: kMaxVisionFrames, kMaxVisionObservations,
// kMaxVisionCameras). Core must not include ipc, so they are mirrored here and a test pins them to
// the codec's values.
inline constexpr std::size_t kMaxFramesPerSensorFrame = 1024;
inline constexpr std::size_t kMaxObservationsPerFrame = 1024;
inline constexpr std::size_t kMaxEnabledCameras = 256;

// The simulated world the cameras observe. Everything outlives the call.
struct VisionWorldView {
    const sdk::IPhysicsWorld& world;  // chassis pose and line of sight (layer::Field)
    sdk::BodyId chassisBody;
    // Link poses for mechanism-mounted cameras. Null, or a link that does not resolve: the camera
    // falls back to the chassis root (like a mechanism with a broken attachment).
    const sdk::IMechanismDirectory* mechanisms = nullptr;
};

class VisionSystem {
public:
    // `startMechanisms` is only used to pose mechanism-mounted cameras at session start (the
    // default assumed mount of setup()). Errors: a non-positive step, more than kMaxEnabledCameras
    // enabled cameras, a duplicate or empty camera id, or camera parameters that cannot be
    // simulated (the config validator rejects them earlier; this is the safety net).
    static sdk::Result<std::unique_ptr<VisionSystem>> create(
        const sdk::VisionConfig& config, sdk::FiducialLayout layout, sdk::SimTimeUs stepUs,
        const sdk::IMechanismDirectory* startMechanisms = nullptr);

    ~VisionSystem();
    VisionSystem(const VisionSystem&) = delete;
    VisionSystem& operator=(const VisionSystem&) = delete;

    // One call per tick N, after the physics and game step. `simTimeAfterStepUs` is T(N). Places
    // every camera from the live chassis / link poses, captures the ones that are due and queues
    // their frames. Safe to call with a teleported chassis: the next capture uses the new pose and
    // queued frames still publish.
    void step(const VisionWorldView& view, sdk::Rng& rng, sdk::SimTimeUs simTimeAfterStepUs);

    // Appends the frames due in the SensorFrame whose simTimeUs is `sensorFrameSimTimeUs` (publish
    // time = sensorFrameSimTimeUs - step), oldest capture first, at most
    // kMaxFramesPerSensorFrame in `out` in total; the rest stay queued for the next tick. A frame
    // that had to wait for a later tick reports the latency it really has, so that the capture
    // time the robot derives from it (publish time - latency) stays true.
    void collectDue(sdk::SimTimeUs sensorFrameSimTimeUs, std::vector<sdk::VisionFrame>& out);

    // Session reset / new epoch / reconnect: drops queued frames, restarts every capture schedule
    // and forgets the latest capture. Sequence numbers and the frame counters continue.
    void reset();

    // HelloAck::vision: one entry per ENABLED camera. photonMessageUuid is the config override,
    // else `defaultPhotonUuid`, else "".
    sdk::VisionSetup setup(std::string_view defaultPhotonUuid) const;

    // Cameras (placed from the live `view`) and the layout's tags. What the robot reported about
    // each camera (CameraSample::robotConnected / status) is read from `robotStates`; it is shown,
    // never used for measuring. The detected cameras themselves are not contributed here: they exist
    // whether or not an engine does (see RobotCameraStates).
    void contribute(sdk::SnapshotBuilder& out, const VisionWorldView& view,
                    const RobotCameraStates& robotStates) const;

    sdk::SimTimeUs stepUs() const { return stepUs_; }
    const sdk::FiducialLayout& layout() const { return layout_; }
    std::size_t cameraCount() const { return cameras_.size(); }
    std::size_t enabledCameraCount() const;

private:
    struct PendingFrame {
        sdk::SimTimeUs publishAtUs = 0;
        sdk::VisionFrame frame;
    };
    struct Camera {
        sdk::CameraConfig config;
        CameraModel model;
        VisibilityLimits limits;
        std::int64_t rateMicroHz = 0;
        std::int64_t accumulator = 0;  // frame schedule, in micro-Hz * us (one frame = 1e12)
        std::uint64_t sequence = 0;    // frames captured so far; never reset
        std::uint64_t published = 0;
        std::optional<sdk::SimTimeUs> lastPublishAtUs;  // of the newest queued frame
        std::deque<PendingFrame> queue;
        sdk::Pose3 startRobotToCamera;  // physical pose in the chassis frame at session start
        std::vector<int> visibleTagIds;  // latest capture
        double lastLatencyMs = 0.0;      // of the latest published frame
    };
    struct TagRecord {
        sdk::FiducialTag tag;
        Pose3d pose;
        double sizeM = 0.0;  // the tag's own size, else the layout's
    };
    struct Placement {
        Pose3d chassisToCamera;
        bool attachmentResolved = true;
    };

    VisionSystem() = default;

    Placement placeCamera(const sdk::CameraConfig& config,
                          const sdk::IMechanismDirectory* mechanisms) const;
    void capture(Camera& cam, const Pose3d& fieldToCamera, const Pose3d& chassisToCamera,
                 const VisionWorldView& view, sdk::Rng& rng, sdk::SimTimeUs captureTimeUs);
    void detect(const Camera& cam, const Pose3d& fieldToCamera, const sdk::IPhysicsWorld& world,
                sdk::Rng& rng, sdk::VisionFrame& frame) const;

    sdk::VisionConfig config_;
    sdk::FiducialLayout layout_;
    sdk::SimTimeUs stepUs_ = 0;
    std::vector<Camera> cameras_;
    std::vector<TagRecord> tags_;
};

}  // namespace wpiemu::core::vision
