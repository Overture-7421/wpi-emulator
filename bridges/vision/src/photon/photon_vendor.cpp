#include "wpiemu/bridges/vision/photon_vendor.h"

namespace wpiemu::bridges::vision {

namespace {

// PLACEHOLDER owned by the photon-publisher work stream, which replaces this whole file: no cameras yet.
class PhotonVendorPlaceholder final : public IVisionVendor {
public:
    sdk::CameraVendor vendor() const override { return sdk::CameraVendor::PhotonVision; }
    std::string_view topicPrefix() const override { return "/photonvision/"; }
    std::unique_ptr<ICameraPublisher> createPublisher(INtApi&, sdk::ILogger&, const sdk::CameraSetup&) const override {
        return nullptr;
    }
    void discoverRobotCameras(std::span<const NtTopicInfo>, std::vector<std::string>&) const override {}
};

}  // namespace

std::unique_ptr<IVisionVendor> makePhotonVendor() { return std::make_unique<PhotonVendorPlaceholder>(); }

}  // namespace wpiemu::bridges::vision
