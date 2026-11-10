#pragma once

// Per-client entity delta encoding of a practice match (fuel and other numerous things), see sdk/match/snapshot_messages.h
// for the protocol. One EntityDeltaEncoder per client holds what the host knows that client's entity state to be:
//
//   * every snapshot sent is committed to a bounded history (kHistory) as the state the client will hold for that
//     snapshotSeq (its baseline plus exactly the changes that were sent);
//   * an acknowledgement (MatchSnapshotAck) pins the acknowledged snapshot as the baseline of later deltas;
//   * with no baseline (nothing acknowledged yet, or the acknowledged snapshot fell out of the history) the delta is
//     relative to kNoBaseline, the full state.
//
// plan() lists every change against the baseline in send order, best first: removals, then upserts ordered by the
// distance band of the nearest robot (nearest first), then by how far the entity moved (most first), then by id. The
// caller sends a prefix of that list (it trims to the snapshot size budget) with deltas(), and then commit()s the prefix
// it sent. Everything is integer arithmetic on quantized values, so the result is deterministic.

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

struct EntityChange {
    std::uint16_t groupId = 0;
    bool removal = false;
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

    EntityPlan plan(std::span<const EntityGroupFrame> current, std::span<const RobotAnchor> anchors) const;
    // The wire form of the first `count` changes of `plan`: one EntityDelta per group of plan.groupIds.
    static std::vector<sdk::match::EntityDelta> deltas(const EntityPlan& plan, std::size_t count);
    // Records that the first `count` changes of `plan` were sent as `snapshotSeq`.
    void commit(const EntityPlan& plan, std::size_t count, std::uint32_t snapshotSeq);
    // The client acknowledged `snapshotSeq`: it becomes the baseline if the history still holds it and it is newer than
    // the current one. Returns whether it did.
    bool onAck(std::uint32_t snapshotSeq);
    // Forget everything (new epoch, resumed connection): the next delta is the full state.
    void reset();

    std::uint32_t baselineId() const { return acked_; }
    std::size_t historySize() const { return history_.size(); }

private:
    struct Entry {
        std::uint32_t seq = 0;
        std::vector<EntityGroupFrame> state;  // ascending groupId
    };
    const Entry* find(std::uint32_t seq) const;

    std::deque<Entry> history_;
    std::uint32_t acked_ = sdk::match::kNoBaseline;
};

}  // namespace wpiemu::core::match
