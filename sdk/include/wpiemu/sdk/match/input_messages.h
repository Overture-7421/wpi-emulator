#pragma once

// Client -> host input of a practice match: the COMMANDED wheel setpoints (not measured state), zone activation flags and
// optional zone pose overrides, stamped with the host tick they should apply on. Sent unreliably at ~kDefaultInputRateHz;
// each packet repeats the last kInputRedundancy inputs so a lost datagram costs nothing. Plain structs; see
// match_protocol.h.
//
// The host ignores non-finite values (sanitizedWheelSpeed), rejects a wheel count that differs from the accepted spec,
// masks zoneFlags with the accepted zone count and rate-limits (kMaxInputRateHz). Silence for kInputTimeoutUs zeroes the
// setpoints; disabled, e-stopped or paused states ignore inputs.
//
// Zone poses. A remote robot simulates no mechanisms, so its client may MOVE its zones (a fake extending intake, a turret):
// `zonePoses` lists, in the CHASSIS frame, where the zones in it should sit. The list belongs to the input and is
// self-contained: the host applies it on the input's target tick together with the wheels and the flags, and every zone
// WITHOUT an entry in the applied input sits at its spec pose. A client that wants a pose held keeps sending it in every
// input. The host clamps each pose to the robot's envelope (remote_spec_limits.h, clampRemoteZonePose).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/match/zone_pose_wire.h"

namespace wpiemu::sdk::match {

struct WheelInput {
    float wheelSpeedRadPerSec = 0.f;  // target wheel spin speed (non-finite is treated as 0 by the host)
    // Target steer angle, quantizeAngle()/dequantizeAngle(); ignored by drivetrains without steered wheels.
    std::int16_t steerAngleQ = 0;

    bool operator==(const WheelInput&) const = default;
};

// Zone flags: bit i = zone i of the accepted spec is active.
constexpr std::uint32_t zoneFlagBit(std::size_t zoneIndex) {
    return zoneIndex < kMaxZonesPerRobot ? (std::uint32_t{1} << zoneIndex) : 0u;
}
constexpr std::uint32_t zoneFlagsMask(std::size_t zoneCount) {
    return zoneCount >= kMaxZonesPerRobot ? (std::uint32_t{1} << kMaxZonesPerRobot) - 1u
                                          : (std::uint32_t{1} << zoneCount) - 1u;
}

// The wheel speed the host applies: non-finite -> 0, then clamped to +-maxAbsRadPerSec.
inline float sanitizedWheelSpeed(float radPerSec, float maxAbsRadPerSec) {
    if (!std::isfinite(radPerSec)) return 0.f;
    return std::clamp(radPerSec, -maxAbsRadPerSec, maxAbsRadPerSec);
}

struct MatchInput {
    std::uint32_t inputSeq = kNoInputSeq;  // strictly increasing from 1; wraps (compare with seqNewer)
    std::uint64_t targetTick = 0;          // host tick this input should apply on
    std::vector<WheelInput> wheels;        // <= kMaxWheelsPerRobot, one per wheel of the accepted spec
    std::uint32_t zoneFlags = 0;
    // Zone pose overrides, see the file comment. Empty = every zone at its spec pose.
    std::vector<ZonePoseInput> zonePoses;  // <= kMaxZonesPerRobot, strictly increasing zoneIndex

    bool operator==(const MatchInput&) const = default;
};

// Sets the override of zone `index` in `input`, keeping zonePoses ordered and replacing an earlier entry of that zone.
// False (and nothing changed) when `index` >= kMaxZonesPerRobot.
inline bool setZonePose(MatchInput& input, std::size_t index, const Pose3& pose) {
    if (index >= kMaxZonesPerRobot) return false;
    const ZonePoseInput z = makeZonePoseInput(index, pose);
    const auto at = std::lower_bound(input.zonePoses.begin(), input.zonePoses.end(), z.zoneIndex,
                                     [](const ZonePoseInput& p, std::uint8_t i) { return p.zoneIndex < i; });
    if (at != input.zonePoses.end() && at->zoneIndex == z.zoneIndex) *at = z;
    else input.zonePoses.insert(at, z);
    return true;
}

// The datagram: identity once, then the last kInputRedundancy inputs, OLDEST FIRST (the newest is back()). Identity lives
// here and not in each MatchInput (the host classifies the packet once with proto::classifyFrame).
struct MatchInputPacket {
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t runEpoch = proto::kUnspecified;
    std::vector<MatchInput> inputs;  // <= kInputRedundancy

    bool operator==(const MatchInputPacket&) const = default;
};

// Adds the newest input, dropping the oldest ones beyond kInputRedundancy.
inline void appendInput(MatchInputPacket& packet, MatchInput input) {
    packet.inputs.push_back(std::move(input));
    if (packet.inputs.size() > kInputRedundancy)
        packet.inputs.erase(packet.inputs.begin(),
                            packet.inputs.begin() + static_cast<std::ptrdiff_t>(packet.inputs.size() - kInputRedundancy));
}

inline bool fitsLimits(const MatchInput& input) {
    return input.wheels.size() <= kMaxWheelsPerRobot && zonePosesWellFormed(input.zonePoses);
}
inline bool fitsLimits(const MatchInputPacket& packet) {
    return packet.inputs.size() <= kInputRedundancy &&
           std::all_of(packet.inputs.begin(), packet.inputs.end(), [](const MatchInput& i) { return fitsLimits(i); });
}

}  // namespace wpiemu::sdk::match
