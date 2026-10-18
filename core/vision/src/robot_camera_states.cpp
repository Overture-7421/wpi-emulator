#include "wpiemu/core/vision/robot_camera_states.h"

#include <algorithm>
#include <string>
#include <utility>

#include "wpiemu/sdk/config/validation_codes.h"

namespace wpiemu::core::vision {

using namespace wpiemu::sdk;

namespace {

const char* vendorName(CameraVendor vendor) {
    return vendor == CameraVendor::Limelight ? "Limelight" : "PhotonVision";
}

// What the robot process reports (CameraRobotState::status) about a camera it could not bring up
// or leaves alone, as a severity and an explanation.
std::pair<Severity, std::string> describeStatus(const std::string& status) {
    if (status == "ntcore_unavailable")
        return {Severity::Error,
                "the robot process has no NetworkTables library to publish through, so the "
                "camera publishes nothing"};
    if (status == "photon_uuid_unsupported")
        return {Severity::Error,
                "the project's PhotonLib speaks a PhotonVision message definition the emulator "
                "has no profile for, so the camera publishes nothing (update the emulator, or set "
                "vision.photonMessageUuid once the definition is verified)"};
    if (status == "foreign_publisher")
        return {Severity::Warning,
                "something in the robot already publishes this camera (a robot-side "
                "PhotonCameraSim?), so the emulator leaves it alone"};
    if (status == "disabled") return {Severity::Warning, "the camera is disabled"};
    return {Severity::Warning, "the robot process reports the status '" + status + "'"};
}

}  // namespace

void RobotCameraStates::upsert(std::span<const CameraRobotState> states) {
    for (const CameraRobotState& state : states) {
        const auto same = [&state](const CameraRobotState& e) {
            return e.vendor == state.vendor && e.ntName == state.ntName;
        };
        if (const auto it = std::find_if(states_.begin(), states_.end(), same); it != states_.end())
            *it = state;
        else if (states_.size() < kMaxEntries)
            states_.push_back(state);
    }
}

const CameraRobotState* RobotCameraStates::forCamera(std::string_view cameraId) const {
    if (cameraId.empty()) return nullptr;  // an empty id is "no simulated camera", not a camera
    for (const CameraRobotState& s : states_)
        if (s.cameraId == cameraId) return &s;
    return nullptr;
}

std::vector<HubWarning> RobotCameraStates::warnings(const VisionConfig& config) const {
    std::vector<HubWarning> out;
    for (const CameraRobotState& s : states_) {
        if (s.cameraId.empty()) {
            // The robot process matched it to no simulated camera.
            if (!s.present) continue;
            bool enabledHere = false, disabledHere = false;
            for (const CameraConfig& c : config.cameras) {
                if (c.vendor != s.vendor || c.networkName() != s.ntName) continue;
                (c.enabled ? enabledHere : disabledHere) = true;
            }
            const char* reason =
                enabledHere    ? "in the vision configuration but not simulated in this run"
                : disabledHere ? "disabled in the vision configuration"
                               : "not in the vision configuration (add it from the detected "
                                 "cameras)";
            out.push_back({Severity::Warning, DeviceId{},
                           validation_code::kVisionCameraUnconfigured,
                           std::string("the robot uses the ") + vendorName(s.vendor) +
                               " camera '" + s.ntName + "', which is " + reason});
        } else if (!s.status.empty()) {
            auto [severity, what] = describeStatus(s.status);
            out.push_back({severity, DeviceId{}, validation_code::kVisionCameraStatus,
                           "camera '" + s.cameraId + "' (" + s.ntName + "): " + what});
        }
    }
    return out;
}

}  // namespace wpiemu::core::vision
