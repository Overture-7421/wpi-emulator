#pragma once

// The CLIENT ENTITY MEMORY of a practice match: applies the entity deltas of MatchSnapshots exactly as sdk/match/
// snapshot_messages.h specifies (baseline, removals, upserts stamped with the snapshot's simTimeUs, carrying updates newer
// than the baseline's from the previously applied snapshot) and exposes the picture the contract defines, drawn with the
// shared prediction model (sdk/match/entity_prediction.h). The game client uses it as it is; the host tests use it as the
// reference of what a client holds.
//
// Besides the picture of the newest snapshot it keeps the picture of each recent snapshot, so a client that draws a little in
// the past (its interpolation delay) can pick the newest snapshot that is not newer than its render time
// (newestAtOrBefore / groupOf) and blend from the snapshot before it (previousOf).

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "wpiemu/sdk/match/entity_prediction.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

class EntityMemory {
public:
    static constexpr std::size_t kHistory = 32;

    struct Held {
        sdk::match::EntityState state;
        std::int64_t stampUs = 0;
        bool operator==(const Held&) const = default;
    };
    using Group = std::map<std::uint32_t, Held>;

    enum class Result { Applied, Stale, UnknownBaseline };

    // Applies the entity part of `s` (nothing is changed unless it returns Applied). Acknowledge only on Applied.
    Result apply(const sdk::match::MatchSnapshot& s) {
        namespace wire = sdk::match;
        if (seen_ && !wire::seqNewer(s.snapshotSeq, latest_)) return Result::Stale;
        const auto prevDisplay = seen_ ? displays_.find(latest_) : displays_.end();
        std::map<std::uint16_t, Group> mirror, display;
        for (const wire::EntityDelta& d : s.entityDeltas) {
            Group base;
            if (d.baselineId != wire::kNoBaseline) {
                const auto it = mirrors_.find(d.baselineId);
                if (it == mirrors_.end()) return Result::UnknownBaseline;
                if (const auto g = it->second.find(d.groupId); g != it->second.end()) base = g->second;
            }
            for (const std::uint32_t id : d.removed) base.erase(id);
            for (const wire::EntityState& e : d.upserted) base[e.id] = {e, s.simTimeUs};
            Group shown = base;
            // Updates still in flight stay ahead of the (older) version the baseline holds, for kEntityInFlightUs.
            if (d.baselineId != wire::kNoBaseline && prevDisplay != displays_.end()) {
                if (const auto pg = prevDisplay->second.find(d.groupId); pg != prevDisplay->second.end()) {
                    for (const auto& [id, held] : pg->second) {
                        if (std::binary_search(d.removed.begin(), d.removed.end(), id)) continue;
                        if (s.simTimeUs - held.stampUs >= wire::kEntityInFlightUs) continue;
                        const auto cur = shown.find(id);
                        if (cur == shown.end() || cur->second.stampUs < held.stampUs) shown[id] = held;
                    }
                }
            }
            mirror[d.groupId] = std::move(base);
            display[d.groupId] = std::move(shown);
        }
        mirrors_[s.snapshotSeq] = std::move(mirror);
        displays_[s.snapshotSeq] = std::move(display);
        times_[s.snapshotSeq] = s.simTimeUs;
        order_.push_back(s.snapshotSeq);
        while (order_.size() > kHistory) {
            mirrors_.erase(order_.front());
            displays_.erase(order_.front());
            times_.erase(order_.front());
            order_.pop_front();
        }
        latest_ = s.snapshotSeq;
        latestTimeUs_ = s.simTimeUs;
        seen_ = true;
        return Result::Applied;
    }

    // Forgets everything (a new session or run epoch: the host starts a fresh entity stream).
    void clear() { *this = EntityMemory{}; }

    bool seen() const { return seen_; }
    std::uint32_t latest() const { return latest_; }
    std::int64_t latestTimeUs() const { return latestTimeUs_; }

    // What is drawn: the entities of the newest applied snapshot, with the in-flight updates ahead of the baseline's.
    const Group& group(std::uint16_t id) const {
        static const Group empty;
        if (!seen_) return empty;
        return groupOf(latest_, id);
    }

    // The same for an applied snapshot of the history (empty when it is not held).
    const Group& groupOf(std::uint32_t snapshotSeq, std::uint16_t id) const {
        static const Group empty;
        const auto it = displays_.find(snapshotSeq);
        if (it == displays_.end()) return empty;
        const auto g = it->second.find(id);
        return g == it->second.end() ? empty : g->second;
    }

    // The entity groups an applied snapshot of the history carries (empty when it is not held).
    std::vector<std::uint16_t> groupIds(std::uint32_t snapshotSeq) const {
        std::vector<std::uint16_t> out;
        const auto it = displays_.find(snapshotSeq);
        if (it != displays_.end())
            for (const auto& [id, group] : it->second) out.push_back(id);
        return out;
    }

    // The newest held snapshot whose sim time is <= `simTimeUs`, or the oldest held one when all are newer.
    std::optional<std::uint32_t> newestAtOrBefore(std::int64_t simTimeUs) const {
        if (order_.empty()) return std::nullopt;
        std::uint32_t pick = order_.front();
        for (const std::uint32_t seq : order_) {
            if (times_.at(seq) > simTimeUs) break;
            pick = seq;
        }
        return pick;
    }

    // The applied snapshot just before `snapshotSeq` in the history, if held.
    std::optional<std::uint32_t> previousOf(std::uint32_t snapshotSeq) const {
        for (std::size_t i = 1; i < order_.size(); ++i)
            if (order_[i] == snapshotSeq) return order_[i - 1];
        return std::nullopt;
    }

    std::int64_t timeOf(std::uint32_t snapshotSeq) const {
        const auto it = times_.find(snapshotSeq);
        return it == times_.end() ? 0 : it->second;
    }

    // The entity as drawn at sim time `atUs`.
    static sdk::match::EntityState shown(const Held& h, std::int64_t atUs) {
        return sdk::match::predictEntity(h.state, atUs - h.stampUs);
    }

    // The picture of the newest applied snapshot: id -> position (velocity and flight are cleared).
    std::map<std::uint32_t, sdk::match::EntityState> picture(std::uint16_t id) const {
        std::map<std::uint32_t, sdk::match::EntityState> out;
        for (const auto& [eid, held] : group(id)) {
            sdk::match::EntityState e = shown(held, latestTimeUs_);
            e.vxCms = e.vyCms = e.vzCms = 0;
            e.ballistic = false;
            out[eid] = e;
        }
        return out;
    }

private:
    std::map<std::uint32_t, std::map<std::uint16_t, Group>> mirrors_;    // per snapshotSeq: baseline + delta, what the host assumes
    std::map<std::uint32_t, std::map<std::uint16_t, Group>> displays_;   // per snapshotSeq: the mirror plus the updates in flight
    std::map<std::uint32_t, std::int64_t> times_;                        // per snapshotSeq: its sim time
    std::deque<std::uint32_t> order_;
    std::uint32_t latest_ = 0;
    std::int64_t latestTimeUs_ = 0;
    bool seen_ = false;
};

}  // namespace wpiemu::core::match
