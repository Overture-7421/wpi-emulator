#include "wpiemu/bridges/vision/photon_profile.h"
#include "wpiemu/sdk/vision/photon_profiles.h"

// PROFILE DATA. Every schema text below is copied from the PhotonLib release named in the profile (printed by
// tools/gen_photon_fixtures.py from the real serde and committed in tests/bridges_vision/fixtures/photon/<uuid>/
// schemas.txt, which a test compares with this file). Never edit a text by hand: its MD5 is the message UUID, and
// PhotonLib throws when the UUID the robot sees differs from its own.
//
// Grammar of a text (PhotonVision photon-serde, messages.yaml / README, v2026.3.4): fields end with ';',
// "<type> <name>", a nested photon message is "<Name>:<uuid>", "name[?]" is a variable length list (at most 127
// items), "optional <type> <name>" is a bool followed by the payload when it is true.

namespace wpiemu::bridges::vision {

namespace {

// PhotonLib v2025.3.2, v2026.1.1 and v2026.3.4 all carry these messages (verified identical).
constexpr PhotonMessageDef kMessagesV2025_3[] = {
    {"PhotonPipelineResult", sdk::kPhotonMessageUuidV2025_3,
     "PhotonPipelineMetadata:ac0a45f686457856fb30af77699ea356 metadata;"
     "PhotonTrackedTarget:cc6dbb5c5c1e0fa808108019b20863f1 targets[?];"
     "optional MultiTargetPNPResult:541096947e9f3ca2d3f425ff7b04aa7b multitagResult;"},
    {"PhotonPipelineMetadata", "ac0a45f686457856fb30af77699ea356",
     "int64 sequenceID;int64 captureTimestampMicros;int64 publishTimestampMicros;int64 timeSinceLastPong;"},
    {"PhotonTrackedTarget", "cc6dbb5c5c1e0fa808108019b20863f1",
     "float64 yaw;float64 pitch;float64 area;float64 skew;int32 fiducialId;int32 objDetectId;float32 objDetectConf;"
     "Transform3d bestCameraToTarget;Transform3d altCameraToTarget;float64 poseAmbiguity;"
     "TargetCorner:16f6ac0dedc8eaccb951f4895d9e18b6 minAreaRectCorners[?];"
     "TargetCorner:16f6ac0dedc8eaccb951f4895d9e18b6 detectedCorners[?];"},
    {"MultiTargetPNPResult", "541096947e9f3ca2d3f425ff7b04aa7b",
     "PnpResult:ae4d655c0a3104d88df4f5db144c1e86 estimatedPose;int16 fiducialIDsUsed[?];"},
    {"PnpResult", "ae4d655c0a3104d88df4f5db144c1e86",
     "Transform3d best;Transform3d alt;float64 bestReprojErr;float64 altReprojErr;float64 ambiguity;"},
    {"TargetCorner", "16f6ac0dedc8eaccb951f4895d9e18b6", "float64 x;float64 y;"},
};

constexpr PhotonProfile kProfiles[] = {
    {sdk::kPhotonMessageUuidV2025_3, "v2026.3.4", kMessagesV2025_3},
};

}  // namespace

std::span<const PhotonProfile> knownPhotonProfiles() {
    return kProfiles;
}

}  // namespace wpiemu::bridges::vision
