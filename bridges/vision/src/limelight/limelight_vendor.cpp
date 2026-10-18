#include "wpiemu/bridges/vision/limelight_vendor.h"

namespace wpiemu::bridges::vision {

namespace {

// PLACEHOLDER owned by the limelight-publisher work stream, which replaces this whole file: no cameras yet.
class LimelightVendorPlaceholder final : public IVisionVendor {
public:
    sdk::CameraVendor vendor() const override { return sdk::CameraVendor::Limelight; }
    std::string_view topicPrefix() const override { return "/limelight"; }
    std::unique_ptr<ICameraPublisher> createPublisher(INtApi&, sdk::ILogger&, const sdk::CameraSetup&) const override {
        return nullptr;
    }
    void discoverRobotCameras(std::span<const NtTopicInfo>, std::vector<std::string>&) const override {}
};

}  // namespace

std::unique_ptr<IVisionVendor> makeLimelightVendor() { return std::make_unique<LimelightVendorPlaceholder>(); }

}  // namespace wpiemu::bridges::vision
