#pragma once

// The ONE dead-reckoning model of match entities (fuel and other numerous things). The host uses it to decide which entity
// updates a client needs (core/match/entity_delta.h: an upsert is only sent when the client's prediction would be wrong) and
// the client uses it to extrapolate every entity between updates, so both sides compute exactly the same thing. Standard
// library only; integer arithmetic only, so the result is identical on every platform and compiler.
//
// An update (EntityState) is the state of an entity at the sim time of the snapshot that carried it (`stampUs`, a
// MatchSnapshot::simTimeUs; the receiver remembers it per entity). The state `dtUs` microseconds later is
// predictEntity(state, dtUs), ALWAYS evaluated from the last update (never from an earlier prediction: the rounding makes the
// function approximately but not exactly composable). Units: centimetres, cm/s, microseconds.
//
// Model:
//   * x and y move at constant velocity (the host corrects the drift of a rolling entity with a new update);
//   * z follows a parabola under gravity while `ballistic` is set (the host sets it for an entity in free flight and clears
//     it for an entity that rests or rolls: no gravity then, vz is ignored);
//   * a ballistic entity that would sink below the floor (z < kEntityFloorCm) lands: z = floor, vz = 0, ballistic cleared;
//   * dtUs is clamped to [0, kEntityPredictHorizonUs]: no extrapolation into the past, and the entity holds still beyond
//     the horizon (the host re-confirms every entity well within it, see EntityPolicy);
//   * positions saturate at the i16 range of the wire.
//
// Smooth display of a corrected path (renderEntityCm): when a new update arrives the prediction of the OLD update and of the
// NEW one differ by the error the host tolerated; showing the new one at once pops. The client keeps the last two updates of
// an entity and, from the stamp of the newer one, blends from the old extrapolation to the new over kEntityBlendUs.

#include <algorithm>
#include <cstdint>
#include <limits>

#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::sdk::match {

inline constexpr std::int64_t kEntityGravityCmps2 = 981;
inline constexpr std::int16_t kEntityFloorCm = 0;
inline constexpr std::int64_t kEntityPredictHorizonUs = 2'000'000;
inline constexpr std::int64_t kEntityBlendUs = 100'000;

namespace detail {

constexpr std::int64_t divRoundAway(std::int64_t numerator, std::int64_t denominator) {
    const std::int64_t half = denominator / 2;
    return numerator >= 0 ? (numerator + half) / denominator : -((-numerator + half) / denominator);
}

constexpr std::int16_t saturateI16(std::int64_t v) {
    return static_cast<std::int16_t>(std::clamp<std::int64_t>(v, std::numeric_limits<std::int16_t>::min(),
                                                                std::numeric_limits<std::int16_t>::max()));
}

}  // namespace detail

constexpr EntityState predictEntity(const EntityState& s, std::int64_t dtUs) {
    const std::int64_t dt = std::clamp<std::int64_t>(dtUs, 0, kEntityPredictHorizonUs);
    if (dt == 0) return s;
    EntityState p = s;
    p.xCm = detail::saturateI16(s.xCm + detail::divRoundAway(static_cast<std::int64_t>(s.vxCms) * dt, 1'000'000));
    p.yCm = detail::saturateI16(s.yCm + detail::divRoundAway(static_cast<std::int64_t>(s.vyCms) * dt, 1'000'000));
    if (s.ballistic) {
        // z + vz*t - g*t^2/2 with t = dt / 1e6 s, as one rounded division.
        const std::int64_t numerator = 2 * static_cast<std::int64_t>(s.vzCms) * dt * 1'000'000 - kEntityGravityCmps2 * dt * dt;
        const std::int64_t z = s.zCm + detail::divRoundAway(numerator, 2'000'000'000'000);
        p.vzCms = detail::saturateI16(s.vzCms - detail::divRoundAway(kEntityGravityCmps2 * dt, 1'000'000));
        if (z < kEntityFloorCm) {
            p.zCm = kEntityFloorCm;
            p.vzCms = 0;
            p.ballistic = false;
        } else {
            p.zCm = detail::saturateI16(z);
        }
    }
    return p;
}

struct EntityPositionCm {
    double x = 0;
    double y = 0;
    double z = 0;
};

// Where to DRAW an entity at sim time `renderUs`. `current` was stamped `currentStampUs` (the newest update that applies at
// renderUs); `previous` (may be null) is the update before it, stamped `previousStampUs`. At the stamp of `current` the
// result is the extrapolation of `previous` and it moves to the extrapolation of `current` over kEntityBlendUs. Not part of
// the deterministic contract (floating point, display only).
inline EntityPositionCm renderEntityCm(const EntityState* previous, std::int64_t previousStampUs, const EntityState& current,
                                       std::int64_t currentStampUs, std::int64_t renderUs) {
    const EntityState now = predictEntity(current, renderUs - currentStampUs);
    EntityPositionCm out{static_cast<double>(now.xCm), static_cast<double>(now.yCm), static_cast<double>(now.zCm)};
    if (previous == nullptr) return out;
    const std::int64_t since = renderUs - currentStampUs;
    if (since >= kEntityBlendUs) return out;
    const EntityState old = predictEntity(*previous, renderUs - previousStampUs);
    const double w = since <= 0 ? 0.0 : static_cast<double>(since) / static_cast<double>(kEntityBlendUs);
    out.x = static_cast<double>(old.xCm) + (out.x - static_cast<double>(old.xCm)) * w;
    out.y = static_cast<double>(old.yCm) + (out.y - static_cast<double>(old.yCm)) * w;
    out.z = static_cast<double>(old.zCm) + (out.z - static_cast<double>(old.zCm)) * w;
    return out;
}

}  // namespace wpiemu::sdk::match
