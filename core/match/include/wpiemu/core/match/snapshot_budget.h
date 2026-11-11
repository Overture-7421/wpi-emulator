#pragma once

// The ONE place that fits a MatchSnapshot into its encoded size budget (kSnapshotBudgetBytes by default, a config value of
// the host). A snapshot is assembled from independent building blocks:
//
//   core part    header + MatchInfo + ALL robots (RobotWireState) + game HUD   always sent, never trimmed to fit
//   entity part  EntityDeltas planned by EntityDeltaEncoder, best change first   fills what the core part leaves
//
// fitSnapshot() measures with the caller's `sizeOf`, which must return the ENCODED size of the whole message, so a field
// added to the core part (RobotWireState grows, zone poses...) is accounted for without touching this code. Only the HUD
// lists are dropped (per-robot first, then the game HUD) when the core part alone exceeds the budget; robots and info are
// never dropped.
//
// Splitting a snapshot into several datagrams later plugs in here: the same function would be called once per datagram with
// a part of the plan (and a core part reduced to a continuation header), each part with its own budget; the encoder's
// commit() then takes the sum of the changes sent across the parts. Nothing else in the host would change.

#include <cstddef>
#include <functional>

#include "wpiemu/core/match/entity_delta.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

// Encoded size in bytes of a snapshot (the whole message, as it is put on the wire).
using SnapshotSizer = std::function<std::size_t(const sdk::match::MatchSnapshot&)>;

struct SnapshotFit {
    std::size_t entityChanges = 0;   // the first `entityChanges` changes of the plan are in the snapshot's entityDeltas
    std::size_t encodedBytes = 0;    // encoded size of the snapshot as left by fitSnapshot
    bool coreOverBudget = false;     // robots + info + HUD alone exceeded the budget (per-robot HUD / game HUD were dropped)
    bool hudDropped = false;
};

// Running estimate of the encoded bytes one entity change adds; it only steers how many changes are tried first.
struct EntitySizeEstimate {
    double bytesPerChange = 9.0;
};

// `snapshot` has its core part filled in. On return its entityDeltas hold the largest prefix of `plan` that fits `budget`
// (possibly none), and `estimate` is refined from the measurement.
SnapshotFit fitSnapshot(sdk::match::MatchSnapshot& snapshot, const EntityPlan& plan, std::size_t budget, const SnapshotSizer& sizeOf,
                        EntitySizeEstimate& estimate);

}  // namespace wpiemu::core::match
