#pragma once

#include <memory>
#include <string_view>

#include "wpiemu/bridges/vision/nt_backend.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace wpiemu::bridges::vision {

std::string_view bridgeFamily();  // "vision"

struct VisionBridgeOptions {
    // Period (simulated time) of the Limelight "hb" / PhotonVision "heartbeat" counters.
    sdk::SimTimeUs heartbeatPeriodUs = 100'000;
};

// Registers `vision.limelight` and `vision.photonvision` against `backend` (shared by both bridges). Returns
// InvalidArgument when `backend` is null and AlreadyExists when a name is taken.
sdk::Status registerVisionBridges(sdk::VisionBridgeRegistry& registry, std::shared_ptr<INtBackend> backend,
                                  VisionBridgeOptions options = {});

// Registration without ntcore: the bridges exist but never publish (the backend is never available). Used by the
// built-in aggregator when the ntcore binding was not compiled in.
sdk::Status registerVisionBridges(sdk::VisionBridgeRegistry& registry);

}  // namespace wpiemu::bridges::vision
