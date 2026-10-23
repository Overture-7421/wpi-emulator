#include "wpiemu/bridges/vision/photon_encoding.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

#include "wpiemu/sdk/core/byte_order.h"

namespace wpiemu::bridges::vision {

namespace {

constexpr std::string_view kTypeString = "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4";
constexpr std::string_view kTypeString2027 = "photonstruct:PhotonPipelineResult:fd353b11f3a818e9c5b457c7dc5a8f5c";

// Little-endian packet writer matching org.photonvision.common.dataflow.structures.Packet.
using PacketWriter = sdk::LittleEndianWriter<std::uint8_t>;

constexpr std::size_t kMaxListLen = 127;  // Packet.MAX_ARRAY_LEN: the count is a signed byte

void writeTransform(PacketWriter& w, const PhotonTransform3d& t) {
    w.f64(t.x);
    w.f64(t.y);
    w.f64(t.z);
    w.f64(t.qw);
    w.f64(t.qx);
    w.f64(t.qy);
    w.f64(t.qz);
}

void writeCorners(PacketWriter& w, const std::vector<PhotonCorner>& corners) {
    const std::size_t n = std::min(corners.size(), kMaxListLen);
    w.u8(static_cast<std::uint8_t>(n));
    for (std::size_t i = 0; i < n; ++i) {
        w.f64(corners[i].x);
        w.f64(corners[i].y);
    }
}

void writeTarget(PacketWriter& w, const PhotonTarget& t) {
    w.f64(t.yawDeg);
    w.f64(t.pitchDeg);
    w.f64(t.areaPercent);
    w.f64(t.skewDeg);
    w.i32(t.fiducialId);
    w.i32(t.objDetectId);
    w.f32(t.objDetectConf);
    writeTransform(w, t.bestCameraToTarget);
    writeTransform(w, t.altCameraToTarget);
    w.f64(t.poseAmbiguity);
    writeCorners(w, t.minAreaRectCorners);
    writeCorners(w, t.detectedCorners);
}

}  // namespace

bool isVerifiedPhotonVersion(std::string_view version) {
    if (version == kPhoton2027Version) return true;
    if (!version.empty() && version.front() == 'v') version.remove_prefix(1);
    static constexpr std::array<std::string_view, 9> kVerified = {
        "2025.3.2", "2026.1.1", "2026.2.1", "2026.2.2", "2026.3.1", "2026.3.2", "2026.3.3", "2026.3.4", "2026.7"};
    return std::find(kVerified.begin(), kVerified.end(), version) != kVerified.end();
}

PhotonSchema photonSchemaForVersion(std::string_view version) {
    return version == kPhoton2027Version ? PhotonSchema::Nanoseconds2027 : PhotonSchema::Microseconds2026;
}

std::string_view photonPipelineResultTypeString(PhotonSchema schema) {
    return schema == PhotonSchema::Nanoseconds2027 ? kTypeString2027 : kTypeString;
}

std::string_view photonPipelineResultUuid(PhotonSchema schema) {
    const auto type = photonPipelineResultTypeString(schema);
    return type.substr(type.rfind(':') + 1);
}

sdk::Result<std::vector<std::uint8_t>> encodePhotonPipelineResult(const PhotonResult& r, PhotonSchema schema) {
    const std::int64_t scale = schema == PhotonSchema::Nanoseconds2027 ? 1000 : 1;
    for (auto time : {r.captureTimestampMicros, r.publishTimestampMicros, r.timeSinceLastPongMicros})
        if (time > std::numeric_limits<std::int64_t>::max() / scale ||
            time < std::numeric_limits<std::int64_t>::min() / scale)
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "PhotonVision timestamp exceeds the wire time range");
    std::vector<std::uint8_t> out;
    PacketWriter w(out);
    w.i64(r.sequenceId);
    w.i64(r.captureTimestampMicros * scale);
    w.i64(r.publishTimestampMicros * scale);
    w.i64(r.timeSinceLastPongMicros * scale);

    const std::size_t n = std::min(r.targets.size(), kMaxListLen);
    w.u8(static_cast<std::uint8_t>(n));
    for (std::size_t i = 0; i < n; ++i) writeTarget(w, r.targets[i]);

    w.boolean(r.multiTag.has_value());
    if (r.multiTag) {
        const PhotonMultiTag& m = *r.multiTag;
        writeTransform(w, m.best);
        writeTransform(w, m.alt);
        w.f64(m.bestReprojErr);
        w.f64(m.altReprojErr);
        w.f64(m.ambiguity);
        const std::size_t ids = std::min(m.fiducialIdsUsed.size(), kMaxListLen);
        w.u8(static_cast<std::uint8_t>(ids));
        for (std::size_t i = 0; i < ids; ++i) w.i16(m.fiducialIdsUsed[i]);
    }
    return out;
}

}  // namespace wpiemu::bridges::vision
