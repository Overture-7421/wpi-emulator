#include "wpiemu/bridges/vision/photon_profile.h"

namespace wpiemu::bridges::vision {

namespace {

// A message UUID is hexadecimal text: a hand-edited robot.json may spell it in capitals.
bool sameUuid(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    const auto lower = [](char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; };
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lower(a[i]) != lower(b[i])) return false;
    return true;
}

}  // namespace

const PhotonProfile* findPhotonProfile(std::string_view uuid) {
    if (uuid.empty()) return nullptr;
    for (const PhotonProfile& profile : knownPhotonProfiles()) {
        if (sameUuid(profile.uuid, uuid)) return &profile;
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
