#pragma once

// Host -> client state of a practice match, plus the client's acknowledgement. Sent unreliably at the snapshot rate (60 Hz
// on a LAN, 30 Hz on a WAN); each snapshot is built for ONE client (its ack fields and its entity baseline). Plain
// structs; fixed-point schemes are in quantize.h; see match_protocol.h for identity and limits.
//
// Snapshots are latest-wins: a receiver drops a snapshot whose snapshotSeq is not newer (seqNewer) than the last applied.
//
// Entity deltas (fuel and other numerous things). Per client the host numbers its snapshots with snapshotSeq (from 1)
// and remembers, per entity group, the entity state of the newest snapshot the client acknowledged
// (MatchSnapshotAck::snapshotSeq). An EntityDelta carries `baselineId` = that acknowledged snapshotSeq, or kNoBaseline
// for the full state; the upserts and removals are relative to the entity state the client holds for that snapshotSeq.
// Entities not mentioned are unchanged since the baseline (the host may omit moving entities to fit
// kSnapshotBudgetBytes; they then catch up in a later snapshot). The client keeps the states of its recent snapshots keyed
// by snapshotSeq, applies the delta on the one named by baselineId, stores the result under the new snapshotSeq and
// acknowledges it. If the client no longer holds the baseline it acks nothing newer and the host falls back to full state.

#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/quantize.h"
#include "wpiemu/sdk/match/zone_pose_wire.h"

namespace wpiemu::sdk::match {

// A small named number for the HUD. `keyId` is defined by the game plugin (both sides run the same plugin, see
// Join::gamePluginId); this protocol only carries it.
struct HudValue {
    std::uint16_t keyId = 0;
    float value = 0.f;

    bool operator==(const HudValue&) const = default;
};

// The host's match state, as the host Driver Station shows it. Remote robots follow it.
struct MatchInfo {
    MatchState state = MatchState::Lobby;
    bool enabled = false;
    proto::RobotMode mode = proto::RobotMode::Teleop;
    bool eStop = false;
    float matchTimeS = -1.f;  // seconds remaining in the period (DsState convention); -1 = no timed match
    std::string phaseKey;     // optional translation key of the match phase, <= kMaxNameBytes

    bool operator==(const MatchInfo&) const = default;
};

// Steer and spin angle of one wheel, both quantizeAngle() values (a full turn over i16).
struct WheelWireState {
    std::int16_t steerAngleQ = 0;
    std::int16_t spinAngleQ = 0;

    bool operator==(const WheelWireState&) const = default;
};

// One robot. Field frame (right-handed, +X toward the far wall, +Y left). Position is quantizeMilliI32 millimetres,
// yaw quantizeAngle, WORLD-frame linear velocity quantizeMilliI16 (mm/s), angular velocity quantizeMilliI16 (mrad/s).
//
// `zonePoses` (an appended tail of the length-prefixed record; absent in older records) lists the zones that are NOT at
// their rest pose, in the CHASSIS frame, so every client sees a moved intake or turret. A zone's rest pose is known to the
// receiver from the roster appearance (the robot's accepted spec), so only zones whose effective pose differs from it are
// listed (selectSnapshotZonePoses in zone_pose_wire.h): at most kMaxSnapshotZonePosesPerRobot, active zones first and then
// the lowest indices when more qualify, strictly increasing zoneIndex. The receiver applies each entry as an override of
// that zone's rest pose; a zone with no entry is at rest. This holds for the host's robot too (its zones move with its
// mechanisms). It is the latest state, not a delta: every snapshot lists them all.
struct RobotWireState {
    ParticipantId participantId = kHostParticipantId;
    std::int32_t xMm = 0;
    std::int32_t yMm = 0;
    std::int16_t yawQ = 0;
    std::int16_t vxMmps = 0;
    std::int16_t vyMmps = 0;
    std::int16_t omegaMradps = 0;
    std::vector<WheelWireState> wheels;  // <= kMaxWheelsPerRobot (empty for the host's robot if it does not publish wheels)
    std::uint32_t zoneFlags = 0;         // bit i = zone i active
    std::vector<HudValue> hud;           // <= kMaxHudValuesPerRobot
    std::vector<ZonePoseInput> zonePoses;  // <= kMaxSnapshotZonePosesPerRobot, strictly increasing zoneIndex; see above

    bool operator==(const RobotWireState&) const = default;
};

// One numerous thing (a fuel piece), identified by a stable id that survives the whole match. Position is
// quantizeCentiI16 centimetres in the field frame.
struct EntityState {
    std::uint32_t id = 0;
    std::int16_t xCm = 0;
    std::int16_t yCm = 0;
    std::int16_t zCm = 0;

    bool operator==(const EntityState&) const = default;
};

// The changes of one entity group (matches an entity group of the telemetry snapshot) against `baselineId`.
struct EntityDelta {
    std::uint16_t groupId = 0;
    std::uint32_t baselineId = kNoBaseline;  // snapshotSeq acknowledged by the client, or kNoBaseline for full state
    std::vector<std::uint32_t> removed;      // <= kMaxEntityDeltaEntries
    std::vector<EntityState> upserted;       // <= kMaxEntityDeltaEntries

    bool operator==(const EntityDelta&) const = default;
};

struct MatchSnapshot {
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t runEpoch = proto::kUnspecified;
    std::uint32_t snapshotSeq = 0;   // per client, from 1, wraps
    std::uint64_t tick = 0;          // host tick the state belongs to
    std::int64_t simTimeUs = 0;
    // For the RECEIVING client's own robot: the newest input the host applied at or before `tick`, and the host tick it
    // was applied on (the client rewinds to the state at `tick` and replays inputs newer than ackInputSeq). kNoInputSeq and
    // proto::kNoAckTick when nothing was applied yet.
    std::uint32_t ackInputSeq = kNoInputSeq;
    std::uint64_t ackTick = proto::kNoAckTick;
    // Input lead feedback for the client's clock sync: targetTick of the newest received input minus the host tick it
    // arrived on (negative = it arrived late), in ticks, saturated. The client keeps this positive and small.
    std::int16_t inputLeadTicks = 0;
    MatchInfo info;
    std::vector<RobotWireState> robots;      // <= kMaxRobotsPerMatch (all participants including the host's robot)
    std::vector<EntityDelta> entityDeltas;   // <= kMaxEntityGroups
    std::vector<HudValue> gameHud;           // <= kMaxGameHudValues

    bool operator==(const MatchSnapshot&) const = default;
};

// Client -> host (unreliable, latest wins): the client applied snapshot `snapshotSeq`, so the host may use that snapshot's
// entity state as the baseline of later deltas.
struct MatchSnapshotAck {
    std::uint64_t sessionId = proto::kUnspecified;
    std::uint64_t runEpoch = proto::kUnspecified;
    std::uint32_t snapshotSeq = 0;

    bool operator==(const MatchSnapshotAck&) const = default;
};

inline bool fitsLimits(const MatchSnapshot& s) {
    if (s.robots.size() > kMaxRobotsPerMatch || s.entityDeltas.size() > kMaxEntityGroups ||
        s.gameHud.size() > kMaxGameHudValues || s.info.phaseKey.size() > kMaxNameBytes)
        return false;
    for (const RobotWireState& r : s.robots)
        if (r.wheels.size() > kMaxWheelsPerRobot || r.hud.size() > kMaxHudValuesPerRobot ||
            !zonePosesWellFormed(r.zonePoses, kMaxSnapshotZonePosesPerRobot))
            return false;
    for (const EntityDelta& d : s.entityDeltas)
        if (d.removed.size() > kMaxEntityDeltaEntries || d.upserted.size() > kMaxEntityDeltaEntries) return false;
    return true;
}

}  // namespace wpiemu::sdk::match
