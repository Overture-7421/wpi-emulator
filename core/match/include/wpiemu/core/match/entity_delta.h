#pragma once

// Per-client entity delta encoding of a practice match (fuel and other numerous things), see sdk/match/snapshot_messages.h
// for the protocol (and the client entity memory it defines) and sdk/match/entity_prediction.h for the model both sides
// extrapolate with. One EntityDeltaEncoder per client holds what the host knows that client's entity state to be:
//
//   * every snapshot sent is committed to a bounded history (kHistory) as the MIRROR the client will hold for that snapshotSeq
//     (its baseline plus exactly the upserts and removals that were sent), each entity stamped with the sim time of the
//     snapshot that upserted it;
//   * an acknowledgement (MatchSnapshotAck) pins the acknowledged snapshot as the baseline of later deltas;
//   * with no baseline (nothing acknowledged yet, or the acknowledged snapshot fell out of the history) the delta is
//     relative to kNoBaseline, the full state;
//   * the updates sent less than kEntityInFlightUs ago in snapshots that are not acknowledged yet are IN FLIGHT: the client
//     probably has them although the baseline does not.
//
// plan() lists what the client needs, best first: removals, then the upserts that are DUE. An entity is due when
//
//   * the client holds nothing for it (new; an entity whose velocity is not yet reliable waits one frame, see
//     EntityGroupFrame::unsettled), or
//   * the client's extrapolation of the update it holds is off the true position by more than
//     EntityPolicy::errorThresholdCm (restErrorThresholdCm when the entity rests: it must come to rest where it really is;
//     unsettledErrorCm while its velocity is unsettled). The update the client holds is the one in flight when there is one
//     (then nothing is repeated while it is right and it is replaced when it is wrong) and the baseline's otherwise, or
//   * it was last confirmed more than a keyframe ago (EntityPolicy::keyframeMovingUs for an entity that moves or flies,
//     keyframeRestUs for one at rest), so a client that went wrong for any reason is corrected within the keyframe interval
//     even if nothing in the model says so.
//
// REMOVALS. Every entity the host ever upserted to the client is remembered (`known_`) until its removal is known to have
// arrived: while the world no longer has it, every snapshot says so, until a snapshot that carried it is acknowledged. A
// removal the budget cut starts over. This does not depend on what any baseline holds, so no entity the host does not have
// can stay at the client, whatever was lost or reordered.
//
// Due upserts are ordered by tier, then by priority. Tier 0 is a wrong picture (and a keyframe that waited
// EntityPolicy::overdueFactor intervals, at the weight of a modest error): priority = (error + kErrorFloorCm) * (age +
// ageScale) / (1 + band), the error of the client's picture, weighted up the longer the entity was not confirmed (age = sim
// time since its update) and down by the distance band of the nearest robot (kBandCm per band). A far or nearly right entity
// therefore waits but cannot starve: its age keeps raising its priority until it is sent. Tier 1 is the keyframes, which
// fill what is left. The caller sends a prefix of the list (it trims to the snapshot size budget) with deltas(), and then
// commit()s the prefix it sent. Integer arithmetic on quantized values throughout, so the result is deterministic.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <utility>
#include <vector>

#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

// The entities of one group as of one snapshot, sorted by ascending id (ids are unique within a group). `unsettled` is empty
// (every entity settled) or parallel to `entities`: 1 = the velocity of that entity is not to be trusted yet (see
// entity_motion.h), so an update that only corrects a modest error, or announces a new entity, waits one frame.
struct EntityGroupFrame {
    std::uint16_t groupId = 0;
    std::vector<sdk::match::EntityState> entities;
    std::vector<std::uint8_t> unsettled;
};

// A robot position in the unit of EntityState (cm), the reference of the "nearest to a robot first" order.
struct RobotAnchor {
    std::int32_t xCm = 0;
    std::int32_t yCm = 0;
};

// When an entity update is due and how due updates are ranked (see the top of this file).
struct EntityPolicy {
    std::int32_t errorThresholdCm = 5;       // tolerated error of an entity that moves or flies
    std::int32_t restErrorThresholdCm = 1;   // tolerated error of an entity that rests: it must end up where it really is
    std::int32_t unsettledErrorCm = 15;      // tolerated error while its velocity is unsettled (it is corrected a frame later)
    std::int64_t keyframeMovingUs = 1'000'000;
    std::int64_t keyframeRestUs = 4'000'000;
    std::int64_t ageScaleUs = 250'000;
    std::int64_t overdueFactor = 3;          // a keyframe waiting this many intervals is promoted to the rank of a modest error
    bool suppressInFlight = true;            // do not repeat an update for kEntityInFlightUs while its acknowledgement is awaited
};

enum class EntityChangeReason : std::uint8_t { Removal, New, Error, Keyframe };

struct EntityChange {
    std::uint16_t groupId = 0;
    bool removal = false;
    EntityChangeReason reason = EntityChangeReason::New;   // why it is due (Removal for a removal)
    sdk::match::EntityState state;  // removal: only `id` is meaningful
};

struct EntityPlan {
    std::uint32_t baselineId = sdk::match::kNoBaseline;
    std::vector<std::uint16_t> groupIds;   // ascending: the groups the snapshot carries a delta for (even an empty one)
    std::vector<EntityChange> changes;     // send order, best first; at most kMaxEntityDeltaEntries per group and kind
    std::size_t omittedByCap = 0;          // changes left out because a group hit kMaxEntityDeltaEntries
};

class EntityDeltaEncoder {
public:
    static constexpr std::size_t kHistory = 32;      // snapshots remembered per client
    static constexpr std::int32_t kBandCm = 200;     // width of one "distance to the nearest robot" band
    static constexpr std::int64_t kErrorFloorCm = 3; // priority of an entity that is due only because of age

    EntityDeltaEncoder() = default;
    explicit EntityDeltaEncoder(const EntityPolicy& policy) : policy_(policy) {}

    // `simTimeUs` is the sim time of the snapshot being planned (the stamp its upserts get).
    EntityPlan plan(std::span<const EntityGroupFrame> current, std::span<const RobotAnchor> anchors, std::int64_t simTimeUs) const;
    // The wire form of the first `count` changes of `plan`: one EntityDelta per group of plan.groupIds, ids ascending.
    static std::vector<sdk::match::EntityDelta> deltas(const EntityPlan& plan, std::size_t count);
    // Records that the first `count` changes of `plan` were sent as `snapshotSeq` at sim time `simTimeUs`.
    void commit(const EntityPlan& plan, std::size_t count, std::uint32_t snapshotSeq, std::int64_t simTimeUs);
    // The client acknowledged `snapshotSeq`: it becomes the baseline if the history still holds it and it is newer than
    // the current one. Returns whether it did.
    bool onAck(std::uint32_t snapshotSeq);
    // Forget everything (new epoch, resumed connection): the next delta is the full state.
    void reset();

    std::uint32_t baselineId() const { return acked_; }
    std::size_t historySize() const { return history_.size(); }
    const EntityPolicy& policy() const { return policy_; }

private:
    struct HeldEntity {
        sdk::match::EntityState state;
        std::int64_t stampUs = 0;   // sim time of the snapshot that upserted it
    };
    struct HeldGroup {
        std::uint16_t groupId = 0;
        std::vector<HeldEntity> entities;   // ascending id
    };
    struct Entry {
        std::uint32_t seq = 0;
        std::vector<HeldGroup> state;  // ascending groupId
    };
    struct Pending {
        sdk::match::EntityState state;   // the newest update sent for the entity
        std::int64_t stampUs = 0;
        std::uint32_t seq = 0;           // the snapshot that carried it
    };
    const Entry* find(std::uint32_t seq) const;

    EntityPolicy policy_;
    std::deque<Entry> history_;
    // Updates sent less than inFlightUs ago: the client may already hold them although the baseline does not.
    std::map<std::pair<std::uint16_t, std::uint32_t>, Pending> pending_;
    // Every entity (group, id) the client may hold: upserted at any time (acknowledged or not) and not yet known to be removed
    // there. The value is 0 while the entity is live and, once the world no longer has it, the first snapshot that told the
    // client; it is removed in every snapshot until one that carried the removal is acknowledged. Independent of baselines, so
    // an update the baseline never held (it was in flight) or that a chain of baselines forgot cannot outlive its entity.
    std::map<std::pair<std::uint16_t, std::uint32_t>, std::uint32_t> known_;
    std::uint32_t acked_ = sdk::match::kNoBaseline;
};

}  // namespace wpiemu::core::match
