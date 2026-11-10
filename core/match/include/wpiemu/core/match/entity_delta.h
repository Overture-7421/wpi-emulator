#pragma once

// Per-client entity delta encoding of a practice match (fuel and other numerous things), see sdk/match/snapshot_messages.h
// for the protocol and sdk/match/entity_prediction.h for the model both sides extrapolate with. One EntityDeltaEncoder per
// client holds what the host knows that client's entity state to be:
//
//   * every snapshot sent is committed to a bounded history (kHistory) as the state the client will hold for that
//     snapshotSeq (its baseline plus exactly the upserts and removals that were sent), each entity stamped with the sim
//     time of the snapshot that upserted it;
//   * an acknowledgement (MatchSnapshotAck) pins the acknowledged snapshot as the baseline of later deltas;
//   * with no baseline (nothing acknowledged yet, or the acknowledged snapshot fell out of the history) the delta is
//     relative to kNoBaseline, the full state.
//
// plan() lists what the client needs, best first: removals (entities the baseline holds that are gone), then the upserts
// that are DUE. An entity is due when
//
//   * the client holds nothing for it (new), or
//   * the client's extrapolation of its baseline update (predictEntity at the snapshot's sim time) is off the true position
//     by more than EntityPolicy::errorThresholdCm (restErrorThresholdCm when the entity rests: it must come to rest where it
//     really is), or
//   * it was last confirmed more than a keyframe ago (EntityPolicy::keyframeMovingUs for an entity that moves or flies,
//     keyframeRestUs for one at rest), so a client that went wrong for any reason is corrected within the keyframe interval
//     even if nothing in the model says so.
//
// Due upserts are ordered by priority = (error + kErrorFloorCm) * (age + ageScale) / (1 + band): the error of the client's
// picture, weighted up the longer the entity was not confirmed (age = sim time since its baseline update), and down by the
// distance band of the nearest robot (kBandCm per band). A far or nearly right entity therefore waits but cannot starve: its
// age keeps raising its priority until it is sent. The caller sends a prefix of the list (it trims to the snapshot size
// budget) with deltas(), and then commit()s the prefix it sent. Integer arithmetic on quantized values throughout, so the
// result is deterministic.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

// The entities of one group as of one snapshot, sorted by ascending id (ids are unique within a group).
struct EntityGroupFrame {
    std::uint16_t groupId = 0;
    std::vector<sdk::match::EntityState> entities;
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
    std::int64_t keyframeMovingUs = 1'000'000;
    std::int64_t keyframeRestUs = 4'000'000;
    std::int64_t ageScaleUs = 250'000;
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
    const Entry* find(std::uint32_t seq) const;

    EntityPolicy policy_;
    std::deque<Entry> history_;
    std::uint32_t acked_ = sdk::match::kNoBaseline;
};

}  // namespace wpiemu::core::match
