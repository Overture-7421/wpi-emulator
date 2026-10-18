#pragma once

// What the robot process reports about the cameras it uses (ADR 0017): CameraRobotState entries from
// ActuatorFrame::vision while lock-step ticks run and from VisionReport everywhere else (discovery-only
// sessions, before BeginLockstep, free-running phases).
//
// Informational only: nothing here changes what the vision engine measures. And nothing here depends
// on the engine existing: a project without a configured camera, or without a resolvable AprilTag
// layout, still learns which PhotonVision / Limelight cameras its robot code uses (the editor's "add
// from detected camera" and the vision_camera_unconfigured hint need exactly that).
//
// Entries are upserted by (vendor, ntName): the first report of a connection lists every camera, later
// ones only what changed. The state describes the robot process, not the simulation, so a session
// reset keeps it; the owner clears it when the connection is lost or replaced.
//
// Threading: sim thread only, like the session that owns it.

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::core::vision {

class RobotCameraStates {
public:
    // Distinct (vendor, ntName) entries kept: a safety bound well above the 256 cameras one message
    // can carry (ipc kMaxVisionCameras). A new camera beyond it is ignored.
    static constexpr std::size_t kMaxEntries = 1024;

    // Replaces the entry of the same (vendor, ntName), else appends.
    void upsert(std::span<const sdk::CameraRobotState> states);
    // The robot process went away (disconnect, replaced connection, new Hello).
    void clear() { states_.clear(); }

    bool empty() const { return states_.empty(); }
    std::span<const sdk::CameraRobotState> states() const { return states_; }

    // The state the robot reported for the configured camera `cameraId` (CameraRobotState::cameraId,
    // the camera the robot process matched to its HelloAck setup); null when there is none.
    const sdk::CameraRobotState* forCamera(std::string_view cameraId) const;

    // The issue-list entries the states imply for `config`:
    //  * vision_camera_unconfigured (Warning): the robot uses a camera that is not simulated. It is
    //    absent from the configuration (also when no camera is configured at all), disabled there,
    //    or enabled but not simulated in this run (for instance without a layout); the message
    //    says which.
    //  * vision_camera_status (Error for ntcore_unavailable and photon_uuid_unsupported, else
    //    Warning): a configured camera the robot process could not bring up or leaves alone.
    // A camera the robot does not use (present == false) and a healthy one raise nothing.
    std::vector<sdk::HubWarning> warnings(const sdk::VisionConfig& config) const;

private:
    std::vector<sdk::CameraRobotState> states_;
};

}  // namespace wpiemu::core::vision
