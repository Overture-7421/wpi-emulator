#include "model/detected_cameras.h"

#include <algorithm>

#include "model/tr.h"

namespace wpiemu::app::editor {

std::vector<UnconfiguredCamera> unconfiguredCameras(const sdk::RobotConfig& config, const std::vector<sdk::CameraRobotState>& seen) {
    const auto configured = [&](const sdk::CameraRobotState& s) {
        return std::any_of(config.vision.cameras.begin(), config.vision.cameras.end(),
                           [&](const sdk::CameraConfig& c) { return c.vendor == s.vendor && c.networkName() == s.ntName; });
    };
    const auto used = [&](const sdk::CameraConfig& c) {
        return std::any_of(seen.begin(), seen.end(),
                           [&](const sdk::CameraRobotState& s) { return s.present && s.vendor == c.vendor && s.ntName == c.networkName(); });
    };

    std::vector<UnconfiguredCamera> out;
    for (const sdk::CameraRobotState& s : seen) {
        if (!s.present || configured(s)) continue;
        UnconfiguredCamera row;
        row.seen = s;
        for (const sdk::CameraConfig& c : config.vision.cameras)
            if (c.vendor == s.vendor && !used(c)) row.renameCandidates.push_back(c.id);
        out.push_back(std::move(row));
    }
    return out;
}

QString detectedCameraStatusText(const sdk::CameraRobotState& state) {
    if (state.status == "ntcore_unavailable") return ETR("NetworkTables is not available in the robot process");
    if (state.status == "photon_uuid_unsupported") return ETR("PhotonLib message format not supported");
    if (state.status == "foreign_publisher") return ETR("published by the robot code itself");
    if (!state.status.empty()) return ETR("status: %1").arg(QString::fromStdString(state.status));
    return ETR("not in the project");
}

}  // namespace wpiemu::app::editor
