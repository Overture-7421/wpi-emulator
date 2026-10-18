#pragma once

// The cameras the robot code uses that the project does not list (ADR 0017), and what the editor can offer for them. The robot process
// reports a camera by the NetworkTables name its code gives it, so a reported camera belongs to a configured one when vendor and name
// match. Ids are never compared: the robot process knows them only from the run it was started with.

#include <string>
#include <vector>

#include <QString>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::app::editor {

struct UnconfiguredCamera {
    sdk::CameraRobotState seen;
    // Ids of the configured cameras of the same vendor the robot code does not use. When the name in the project is a typo or the code
    // was renamed, one of them is this camera.
    std::vector<std::string> renameCandidates;
};

// What `seen` (the cameras the robot reports) uses that no camera of `config` has the vendor and name of, in the order reported. Cameras
// the robot stopped using are left out.
std::vector<UnconfiguredCamera> unconfiguredCameras(const sdk::RobotConfig& config, const std::vector<sdk::CameraRobotState>& seen);

// One short line for a reported camera that is not configured: that the code uses it, or the problem the robot process reports.
QString detectedCameraStatusText(const sdk::CameraRobotState& state);

}  // namespace wpiemu::app::editor
