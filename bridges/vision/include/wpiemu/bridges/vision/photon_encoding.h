#pragma once

// PhotonLib wire format of PhotonPipelineResult ("photonstruct", photon-serde), little-endian packed structs:
//
//   PhotonPipelineResult   := PhotonPipelineMetadata, list<PhotonTrackedTarget>, optional<MultiTargetPNPResult>
//   PhotonPipelineMetadata := int64 sequenceID, captureTimestampMicros, publishTimestampMicros, timeSinceLastPong
//   PhotonTrackedTarget    := f64 yaw, pitch, area, skew; i32 fiducialId, objDetectId; f32 objDetectConf;
//                             Transform3d best, alt; f64 poseAmbiguity; list<TargetCorner> minAreaRect, detected
//   Transform3d            := f64 x, y, z, qw, qx, qy, qz
//   MultiTargetPNPResult   := PnpResult(Transform3d best, alt; f64 bestReprojErr, altReprojErr, ambiguity),
//                             list<i16> fiducialIDsUsed
//   list<T>                := u8 count, T...      optional<T> := u8 present, T?
//
// Verified against photon-serde at v2026.3.4 (the message definition hashes are identical from v2025.3.2 to
// v2026.7). The reader of the topic compares the type string, so it must match the robot's PhotonLib exactly.

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace wpiemu::bridges::vision {

struct PhotonTransform3d {
    double x = 0, y = 0, z = 0;                       // meters
    double qw = 1, qx = 0, qy = 0, qz = 0;            // rotation quaternion
};

struct PhotonCorner {
    double x = 0, y = 0;
};

struct PhotonTarget {
    double yawDeg = 0, pitchDeg = 0, areaPercent = 0, skewDeg = 0;
    std::int32_t fiducialId = -1;
    std::int32_t objDetectId = -1;
    float objDetectConf = -1.f;
    PhotonTransform3d bestCameraToTarget;
    PhotonTransform3d altCameraToTarget;
    double poseAmbiguity = 0;
    std::vector<PhotonCorner> minAreaRectCorners;
    std::vector<PhotonCorner> detectedCorners;
};

struct PhotonMultiTag {
    PhotonTransform3d best;                            // field -> camera
    PhotonTransform3d alt;
    double bestReprojErr = 0, altReprojErr = 0, ambiguity = 0;
    std::vector<std::int16_t> fiducialIdsUsed;
};

struct PhotonResult {
    std::int64_t sequenceId = 0;
    std::int64_t captureTimestampMicros = 0;
    std::int64_t publishTimestampMicros = 0;
    std::int64_t timeSinceLastPongMicros = 0;
    std::vector<PhotonTarget> targets;                 // at most 127 are encoded
    std::optional<PhotonMultiTag> multiTag;
};

// The photonlib versions whose message definitions were verified (see above), e.g. "v2026.3.4".
// A leading 'v' is optional.
bool isVerifiedPhotonVersion(std::string_view version);
inline constexpr std::string_view kDefaultPhotonVersion = "v2026.3.4";

// "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4" (PhotonPipelineResult.photonStruct.getTypeString()).
std::string_view photonPipelineResultTypeString();

// "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4" -> the interface UUID. PhotonLib publishes it
// as the JSON string property "message_uuid" of rawBytes and compares it in its version check.
std::string_view photonPipelineResultUuid();

std::vector<std::uint8_t> encodePhotonPipelineResult(const PhotonResult& result);

}  // namespace wpiemu::bridges::vision
