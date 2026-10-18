#include "wpiemu/bridges/vision/photon_vendor.h"

#include <algorithm>
#include <array>
#include <map>
#include <string>

#include "photon_publisher.h"
#include "wpiemu/bridges/vision/photon_mapper.h"

namespace wpiemu::bridges::vision {

namespace {

constexpr std::string_view kPrefix = "/photonvision/";

// The topics a PhotonLib PhotonCamera creates for the robot to WRITE to (PhotonCamera.java / PhotonCamera.cpp,
// v2026.3.4); the root ledModeRequest is shared by all cameras and says nothing about one of them. Nothing the
// bridge publishes ends in one of these.
constexpr std::array<std::string_view, 3> kRequestLeaves = {"pipelineIndexRequest", "driverModeRequest",
                                                            "fpsLimitRequest"};

class PhotonVendor final : public IVisionVendor {
public:
    sdk::CameraVendor vendor() const override { return sdk::CameraVendor::PhotonVision; }
    std::string_view topicPrefix() const override { return kPrefix; }

    std::unique_ptr<ICameraPublisher> createPublisher(INtApi& nt, sdk::ILogger& log,
                                                      const sdk::CameraSetup& setup) const override {
        if (setup.vendor != sdk::CameraVendor::PhotonVision) return nullptr;
        const Resolved& resolved = resolve(setup.photonMessageUuid);
        return std::make_unique<PhotonPublisher>(nt, log, setup, resolved.codec, resolved.reason, shared_);
    }

    void discoverRobotCameras(std::span<const NtTopicInfo> topics, std::vector<std::string>& ntNames) const override {
        for (const NtTopicInfo& topic : topics) {
            const std::string_view name = topic.name;
            if (!name.starts_with(kPrefix)) continue;
            // "<camera>/<leaf>": PhotonLib uses the camera nickname verbatim as the table name, so everything up to
            // the last '/' is the camera (a nickname may hold spaces, punctuation or even a '/').
            const std::string_view rest = name.substr(kPrefix.size());
            const std::size_t slash = rest.rfind('/');
            if (slash == std::string_view::npos || slash == 0) continue;
            if (std::find(kRequestLeaves.begin(), kRequestLeaves.end(), rest.substr(slash + 1)) ==
                kRequestLeaves.end()) {
                continue;
            }
            const std::string camera(rest.substr(0, slash));
            if (std::find(ntNames.begin(), ntNames.end(), camera) == ntNames.end()) ntNames.push_back(camera);
        }
    }

private:
    // The outcome of loading a message profile; shared by every camera that asks for the same UUID.
    struct Resolved {
        std::shared_ptr<const PhotonCodec> codec;  // null when unusable
        std::string reason;
    };

    // "" asks for the newest known profile. A UUID without a profile, a profile whose texts do not hash to its
    // UUIDs, or one with a field the mapper cannot fill, resolves to no codec and the reason: the camera is refused.
    const Resolved& resolve(std::string_view requested) const {
        if (const auto it = resolved_.find(requested); it != resolved_.end()) return it->second;

        Resolved resolved;
        const PhotonProfile* profile = requested.empty() ? &newestPhotonProfile() : findPhotonProfile(requested);
        if (profile == nullptr) {
            std::string known;
            for (const PhotonProfile& candidate : knownPhotonProfiles()) {
                known += (known.empty() ? "" : ", ") + std::string(candidate.uuid);
            }
            resolved.reason = "no PhotonVision message definition has UUID '" + std::string(requested) +
                              "' (known: " + known +
                              "); the project's PhotonLib is newer or older than this emulator supports";
        } else {
            sdk::Result<PhotonCodec> codec = PhotonCodec::create(*profile);
            if (!codec.ok()) {
                resolved.reason = codec.error().message;
            } else if (const sdk::Status mapped = verifyPhotonMapping(codec.value()); !mapped.ok()) {
                resolved.reason = mapped.error().message;
            } else {
                resolved.codec = std::make_shared<const PhotonCodec>(std::move(codec).value());
            }
        }
        return resolved_.emplace(std::string(requested), std::move(resolved)).first->second;
    }

    mutable std::map<std::string, Resolved, std::less<>> resolved_;
    std::shared_ptr<PhotonSharedState> shared_ = std::make_shared<PhotonSharedState>();
};

}  // namespace

std::unique_ptr<IVisionVendor> makePhotonVendor() {
    return std::make_unique<PhotonVendor>();
}

}  // namespace wpiemu::bridges::vision
