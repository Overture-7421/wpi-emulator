#include "wpiemu/bridges/vision/photon_profile.h"

namespace wpiemu::bridges::vision {

const PhotonProfile* findPhotonProfile(std::string_view uuid) {
    if (uuid.empty()) return nullptr;
    for (const PhotonProfile& profile : knownPhotonProfiles()) {
        if (profile.uuid == uuid) return &profile;
    }
    return nullptr;
}

const PhotonProfile& newestPhotonProfile() {
    return knownPhotonProfiles().back();
}

std::string photonTypeString(std::string_view messageName, std::string_view uuid) {
    std::string text = "photonstruct:";
    text += messageName;
    text += ':';
    text += uuid;
    return text;
}

}  // namespace wpiemu::bridges::vision
