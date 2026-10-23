#pragma once

// PhotonLib wire format of PhotonPipelineResult ("photonstruct", photon-serde), little-endian packed structs:
//
//   PhotonPipelineResult   := PhotonPipelineMetadata, list<PhotonTrackedTarget>, optional<MultiTargetPNPResult>
//   PhotonPipelineMetadata := int64 sequenceID, captureTimestamp, publishTimestamp, timeSinceLastPong
//   PhotonTrackedTarget    := f64 yaw, pitch, area, skew; i32 fiducialId, objDetectId; f32 objDetectConf;
//                             Transform3d best, alt; f64 poseAmbiguity; list<TargetCorner> minAreaRect, detected
//   Transform3d            := f64 x, y, z, qw, qx, qy, qz
//   MultiTargetPNPResult   := PnpResult(Transform3d best, alt; f64 bestReprojErr, altReprojErr, ambiguity),
//                             list<i16> fiducialIDsUsed
//   list<T>                := u8 count, T...      optional<T> := u8 present, T?
//
// Two wire schemas exist: "photon-us" (microsecond metadata; the message definition hashes are identical across the
// verified 2025-2026 releases) and "photon-ns" (nanosecond metadata, a different result UUID, unchanged targets/multitag).
// Which PhotonLib versions were verified against which schema is data: vendors.photonvision of each season pack
// (seasons/<year>/season.json). The reader of the topic compares the type string, so it must match the robot's PhotonLib.

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::bridges::vision {

enum class PhotonSchema { Microseconds, Nanoseconds };

// The manifest name of a schema ("photon-us", "photon-ns").
std::string_view photonSchemaName(PhotonSchema schema);

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
    // Bridge-facing time remains microseconds; the selected schema determines wire units.
    std::int64_t sequenceId = 0;
    std::int64_t captureTimestampMicros = 0;
    std::int64_t publishTimestampMicros = 0;
    std::int64_t timeSinceLastPongMicros = 0;
    std::vector<PhotonTarget> targets;                 // at most 127 are encoded
    std::optional<PhotonMultiTag> multiTag;
};

// One verified PhotonLib version (a pattern: exact, or a trailing '*') and the schema it speaks.
struct PhotonVersionRule {
    std::string_view version;
    PhotonSchema schema;
};
// Generated from the season packs at configure time (bridges/vision/CMakeLists.txt): every verified version of every
// season, the version assumed for a project that declares none, and the schema of unverified versions. The last two
// come from the season this build targets.
const std::vector<PhotonVersionRule>& photonVersionRules();
std::string_view photonDefaultVersion();
PhotonSchema photonDefaultSchema();

// True for a verified photonlib version, e.g. "v2026.3.4"; a leading 'v' is optional for release versions.
bool isVerifiedPhotonVersion(std::string_view version);
// The schema of a verified version; an unverified version is served with the build's default schema (best effort).
PhotonSchema photonSchemaForVersion(std::string_view version);

// "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4" (PhotonPipelineResult.photonStruct.getTypeString()).
std::string_view photonPipelineResultTypeString(PhotonSchema schema = PhotonSchema::Microseconds);

// "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4" -> the interface UUID. PhotonLib publishes it
// as the JSON string property "message_uuid" of rawBytes and compares it in its version check.
std::string_view photonPipelineResultUuid(PhotonSchema schema = PhotonSchema::Microseconds);

sdk::Result<std::vector<std::uint8_t>> encodePhotonPipelineResult(
    const PhotonResult& result, PhotonSchema schema = PhotonSchema::Microseconds);

}  // namespace wpiemu::bridges::vision
