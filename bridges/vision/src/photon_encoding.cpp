#include "wpiemu/bridges/vision/photon_encoding.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <utility>

namespace wpiemu::bridges::vision {

namespace {

constexpr std::string_view kTypeString = "photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4";

// Little-endian packet writer matching org.photonvision.common.dataflow.structures.Packet.
class PacketWriter {
public:
    void u8(std::uint8_t v) { out_.push_back(v); }
    void boolean(bool v) { u8(v ? 1 : 0); }
    void i16(std::int16_t v) { le(static_cast<std::uint16_t>(v), 2); }
    void i32(std::int32_t v) { le(static_cast<std::uint32_t>(v), 4); }
    void i64(std::int64_t v) { le(static_cast<std::uint64_t>(v), 8); }
    void f32(float v) { le(std::bit_cast<std::uint32_t>(v), 4); }
    void f64(double v) { le(std::bit_cast<std::uint64_t>(v), 8); }
    std::vector<std::uint8_t> take() { return std::move(out_); }

private:
    void le(std::uint64_t v, int bytes) {
        for (int i = 0; i < bytes; ++i) u8(static_cast<std::uint8_t>((v >> (8 * i)) & 0xff));
    }
    std::vector<std::uint8_t> out_;
};

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
    if (!version.empty() && version.front() == 'v') version.remove_prefix(1);
    static constexpr std::array<std::string_view, 9> kVerified = {
        "2025.3.2", "2026.1.1", "2026.2.1", "2026.2.2", "2026.3.1", "2026.3.2", "2026.3.3", "2026.3.4", "2026.7"};
    return std::find(kVerified.begin(), kVerified.end(), version) != kVerified.end();
}

std::string_view photonPipelineResultTypeString() { return kTypeString; }

std::string_view photonPipelineResultUuid() { return kTypeString.substr(kTypeString.rfind(':') + 1); }

std::vector<std::uint8_t> encodePhotonPipelineResult(const PhotonResult& r) {
    PacketWriter w;
    w.i64(r.sequenceId);
    w.i64(r.captureTimestampMicros);
    w.i64(r.publishTimestampMicros);
    w.i64(r.timeSinceLastPongMicros);

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
    return w.take();
}

}  // namespace wpiemu::bridges::vision
