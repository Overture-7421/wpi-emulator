#pragma once

// PhotonVision message profiles: DATA, not code (ADR 0017, plan section 13).
//
// A profile is what the bridge needs to speak one generation of PhotonVision's result message (the
// "photonstruct" family carried by /photonvision/<camera>/rawBytes):
//   * the UUID of PhotonPipelineResult. It is the `message_uuid` property of the topic AND the tail of its type
//     string "photonstruct:PhotonPipelineResult:<uuid>". PhotonLib compares it with its own at its first version
//     check and THROWS on a difference (Java UnsupportedOperationException, C++ std::runtime_error), so the bridge
//     never guesses: an unknown UUID means the camera is refused (camera_status::kPhotonUuidUnsupported).
//   * the schema text of every message the result contains. The UUID of a message is the MD5 of its text
//     (PhotonVision photon-serde README and PacketSerde.getInterfaceUUID, v2026.3.4); a nested photon message
//     appears in its parent's text as "<Name>:<uuid>", so a change anywhere below changes the root UUID. WPILib
//     structs (Transform3d, ...) are not part of a profile: their layout is fixed by WPILib.
//   * the text published at /photonvision/version (PhotonLib's C++ camera treats an empty version as "no
//     coprocessor", the Java one only prints it in a mismatch message).
// The generic codec (photon_codec.h) walks these texts, so a new PhotonVision message version is a new entry in
// photon_profile_data.cpp (plus a mapper field in photon_mapper.cpp when the schema gained a field), never a new
// encoder.
//
// Adding a profile (docs: tools/gen_photon_fixtures.py header): run the generator against the new PhotonLib
// release, which runs the REAL serde and writes tests/bridges_vision/fixtures/photon/<uuid>/ (the schema texts and
// golden bytes); copy the printed schema texts into photon_profile_data.cpp, extend the version rule in
// sdk/vision/photon_profiles.h, and run the vision tests: a profile without golden fixtures, or whose texts differ
// from the real library's, fails them.

#include <span>
#include <string>
#include <string_view>

namespace wpiemu::bridges::vision {

struct PhotonMessageDef {
    std::string_view name;    // "PhotonTrackedTarget"
    std::string_view uuid;    // md5(schema), 32 lower-case hex digits
    std::string_view schema;  // exactly the text PhotonLib reports for the message
};

struct PhotonProfile {
    std::string_view uuid;     // UUID of PhotonPipelineResult
    std::string_view version;  // the PhotonVision release the texts came from, e.g. "v2026.3.4"
    std::span<const PhotonMessageDef> messages;
};

// The message published on /photonvision/<camera>/rawBytes.
inline constexpr std::string_view kPhotonResultMessage = "PhotonPipelineResult";

// Every profile this build can write, oldest first. The last one is what a camera without a configured UUID uses.
std::span<const PhotonProfile> knownPhotonProfiles();
// nullptr when no profile has this UUID (also for "").
const PhotonProfile* findPhotonProfile(std::string_view uuid);
const PhotonProfile& newestPhotonProfile();

// "photonstruct:<name>:<uuid>", the NetworkTables type string of a photon message topic.
std::string photonTypeString(std::string_view messageName, std::string_view uuid);

}  // namespace wpiemu::bridges::vision
