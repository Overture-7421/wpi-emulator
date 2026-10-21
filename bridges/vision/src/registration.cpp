#include <memory>
#include <utility>

#include "limelight_bridge.h"
#include "photon_bridge.h"
#include "wpiemu/bridges/vision/vision_bridges.h"

namespace wpiemu::bridges::vision {

namespace {

// Stand-in used when ntcore support was not compiled in: the bridges exist but never publish.
class UnavailableNtBackend final : public INtBackend {
public:
    bool available() override { return false; }
    std::int64_t nowUs() override { return 0; }
    sdk::Status publish(std::string_view, const NtValue&, std::int64_t, std::string_view) override {
        return sdk::makeError(sdk::ErrorCode::Unsupported, "ntcore support is not available in this build");
    }
    sdk::Status setTopicProperty(std::string_view, std::string_view, std::string_view) override {
        return sdk::makeError(sdk::ErrorCode::Unsupported, "ntcore support is not available in this build");
    }
    std::optional<NtValue> read(std::string_view) override { return std::nullopt; }
    void unpublish(std::string_view) override {}
};

}  // namespace

std::string_view bridgeFamily() { return "vision"; }

sdk::Status registerVisionBridges(sdk::VisionBridgeRegistry& registry, std::shared_ptr<INtBackend> backend,
                                  VisionBridgeOptions options) {
    if (!backend) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "registerVisionBridges: null INtBackend");

    sdk::Status s = registry.add(
        "vision.limelight",
        sdk::BridgeInfo{"vision", "Limelight AprilTag camera (LimelightHelpers NetworkTables)", "limelight"},
        [backend, options]() -> std::unique_ptr<sdk::IVisionBridge> {
            return std::make_unique<LimelightBridge>(backend, options);
        });
    if (!s.ok()) return s;
    return registry.add(
        "vision.photonvision",
        sdk::BridgeInfo{"vision", "PhotonVision AprilTag camera (PhotonLib NetworkTables)", "photonvision"},
        [backend, options]() -> std::unique_ptr<sdk::IVisionBridge> {
            return std::make_unique<PhotonBridge>(backend, options);
        });
}

sdk::Status registerVisionBridges(sdk::VisionBridgeRegistry& registry) {
    return registerVisionBridges(registry, std::make_shared<UnavailableNtBackend>());
}

}  // namespace wpiemu::bridges::vision
