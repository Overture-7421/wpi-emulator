#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/vision/apriltag.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::core::runtime {

// Simulated vision coprocessors (docs/runtime.md "Vision"). Owned by SimulationSession; stepped once per tick after the
// physics step. Each camera captures at its own frame rate (first capture on the first step, then every round(1e6/fps)
// us, always at a step boundary), decides which AprilTags it sees with a simple model (in front, inside the FOV, within
// range, facing the camera, incidence limit; no occlusion or distortion), and delivers the frame `latencyMs` after
// capture. Frames are always produced, even when no tag is seen. Deterministic: noise comes from the caller's Rng.
// Threading: sim thread only.
class VisionSystem {
public:
    // The robot frame reported to robot code is the WPILib one: origin on the floor below the hull centre, yaw = chassis
    // yaw. It is derived from the chassis spec (body origin == COM, ADR 0003).
    VisionSystem(const sdk::RobotConfig& config, sdk::AprilTagLayout layout);

    bool empty() const { return cameras_.empty(); }
    // What the extension needs before the first frame (photonVersion is filled in by the runner).
    sdk::VisionSetup setup() const;

    // One capture opportunity at sim time `simTimeUs` (a step boundary, after the physics step). `mechanisms` may be null
    // (cameras attached to a mechanism link then fall back to the chassis frame, like zones).
    void step(sdk::SimTimeUs simTimeUs, const sdk::Pose3& chassisPose, sdk::IMechanismDirectory* mechanisms, sdk::Rng& rng);
    // Frames whose publish time has been reached, in publish order.
    std::vector<sdk::CameraFrame> collectDue(sdk::SimTimeUs simTimeUs);
    // CameraState of the last captured frame of each camera that has captured one.
    void publish(sdk::SnapshotBuilder& builder) const;
    // New epoch: queues, sequences and capture times start over.
    void reset();

private:
    struct CameraRuntime {
        sdk::CameraConfig config;
        sdk::SimTimeUs periodUs = 0;
        sdk::SimTimeUs latencyUs = 0;
        sdk::SimTimeUs nextCaptureUs = 0;
        std::uint64_t sequence = 0;
        bool captured = false;
        sdk::Pose3 lastPose;
        std::vector<int> lastTagIds;
    };

    sdk::CameraFrame capture(std::size_t index, sdk::SimTimeUs simTimeUs, const sdk::Pose3& chassisPose,
                             const sdk::Pose3& robotFrame, sdk::IMechanismDirectory* mechanisms, sdk::Rng& rng);

    std::vector<CameraRuntime> cameras_;
    sdk::AprilTagLayout layout_;
    sdk::Vec3 robotOriginInChassis_;
    std::deque<sdk::CameraFrame> pending_;  // ordered by publishTimeUs, then capture order
};

}  // namespace wpiemu::core::runtime
