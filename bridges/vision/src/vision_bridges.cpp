#include "wpiemu/bridges/vision/vision_bridges.h"

#include <memory>
#include <utility>

namespace wpiemu::bridges::vision {

sdk::Status registerVisionBridges(sdk::DeviceBridgeRegistry& registry) {
    return registerVisionBridges(registry, VisionBridgeOptions{});
}

sdk::Status registerVisionBridges(sdk::DeviceBridgeRegistry& registry, VisionBridgeOptions options) {
    // The registry may build the bridge more than once: the options (shared NT seam and vendors) are copied into each.
    return registry.registerBridge(
        "vision",
        sdk::BridgeInfo{"vision",
                        "Virtual PhotonVision and Limelight AprilTag cameras published on the robot's NetworkTables",
                        "vision"},
        [options = std::move(options)] { return std::make_unique<VisionBridge>(options); });
}

}  // namespace wpiemu::bridges::vision
