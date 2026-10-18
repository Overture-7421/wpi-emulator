#include "wpiemu/bridges/vision/limelight_vendor.h"

#include <algorithm>

#include "limelight_publisher.h"
#include "wpiemu/bridges/vision/limelight_keys.h"

namespace wpiemu::bridges::vision {

namespace {

class LimelightVendor final : public IVisionVendor {
public:
    sdk::CameraVendor vendor() const override { return sdk::CameraVendor::Limelight; }
    std::string_view topicPrefix() const override { return limelight::kTopicPrefix; }

    std::unique_ptr<ICameraPublisher> createPublisher(INtApi& nt, sdk::ILogger& log,
                                                      const sdk::CameraSetup& setup) const override {
        return std::make_unique<limelight::LimelightPublisher>(nt, log, setup);
    }

    // A device table ("limelight", "limelight-<name>") belongs to a robot that uses the camera as soon as the ROBOT
    // created one of the keys it is known to write (limelight_keys.h). Our own per-frame topics never count.
    void discoverRobotCameras(std::span<const NtTopicInfo> topics, std::vector<std::string>& ntNames) const override {
        for (const NtTopicInfo& info : topics) {
            std::string_view table;
            std::string_view key;
            if (!limelight::splitTopic(info.name, table, key) || !limelight::isLimelightTable(table) ||
                !limelight::isRobotWrittenKey(key))
                continue;
            if (std::find(ntNames.begin(), ntNames.end(), table) == ntNames.end()) ntNames.emplace_back(table);
        }
    }
};

}  // namespace

std::unique_ptr<IVisionVendor> makeLimelightVendor() { return std::make_unique<LimelightVendor>(); }

}  // namespace wpiemu::bridges::vision
