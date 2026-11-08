#pragma once

// Client -> host input of a practice match: the COMMANDED wheel setpoints (not measured state) and zone activation flags,
// stamped with the host tick they should apply on. Sent unreliably at ~kDefaultInputRateHz; each packet repeats the last
// kInputRedundancy inputs so a lost datagram costs nothing. Plain structs; see match_protocol.h.
//
// The host ignores non-finite values (sanitizedWheelSpeed), rejects a wheel count that differs from the accepted spec,
// masks zoneFlags with the accepted zone count and rate-limits (kMaxInputRateHz). Silence for kInputTimeoutUs zeroes the
// setpoints; disabled, e-stopped or paused states ignore inputs.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"

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

    bool operator==(const MatchInput&) const = default;
};

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

inline bool fitsLimits(const MatchInput& input) { return input.wheels.size() <= kMaxWheelsPerRobot; }
inline bool fitsLimits(const MatchInputPacket& packet) {
    return packet.inputs.size() <= kInputRedundancy &&
           std::all_of(packet.inputs.begin(), packet.inputs.end(), [](const MatchInput& i) { return fitsLimits(i); });
}

}  // namespace wpiemu::sdk::match
