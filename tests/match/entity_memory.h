#pragma once

// The reference CLIENT ENTITY MEMORY of tests: applies the entity deltas of MatchSnapshots exactly as sdk/match/snapshot_messages.h
// specifies (baseline, removals, upserts stamped with the snapshot's simTimeUs, carrying updates newer than the baseline's from
// the previously applied snapshot) and draws the picture with the shared prediction model. Tests of the host, of the encoder
// and the stress scenario all use this one implementation.

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>

#include "wpiemu/sdk/match/entity_prediction.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

namespace match_test {

class EntityMemory {
public:
    static constexpr std::size_t kHistory = 32;

    struct Held {
        wpiemu::sdk::match::EntityState state;
        std::int64_t stampUs = 0;
        bool operator==(const Held&) const = default;
    };
    using Group = std::map<std::uint32_t, Held>;

    enum class Result { Applied, Stale, UnknownBaseline };

    // Applies the entity part of `s` (nothing is changed unless it returns Applied). Acknowledge only on Applied.
    Result apply(const wpiemu::sdk::match::MatchSnapshot& s) {
        namespace wire = wpiemu::sdk::match;
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
        order_.push_back(s.snapshotSeq);
        while (order_.size() > kHistory) {
            mirrors_.erase(order_.front());
            displays_.erase(order_.front());
            order_.pop_front();
        }
        latest_ = s.snapshotSeq;
        latestTimeUs_ = s.simTimeUs;
        seen_ = true;
        return Result::Applied;
    }

    bool seen() const { return seen_; }
    std::uint32_t latest() const { return latest_; }
    std::int64_t latestTimeUs() const { return latestTimeUs_; }

    // What is drawn: the entities of the newest applied snapshot, with the in-flight updates ahead of the baseline's.
    const Group& group(std::uint16_t id) const {
        static const Group empty;
        const auto it = displays_.find(latest_);
        if (!seen_ || it == displays_.end()) return empty;
        const auto g = it->second.find(id);
        return g == it->second.end() ? empty : g->second;
    }

    // The entity as drawn at sim time `atUs`.
    static wpiemu::sdk::match::EntityState shown(const Held& h, std::int64_t atUs) {
        return wpiemu::sdk::match::predictEntity(h.state, atUs - h.stampUs);
    }

    // The picture of the newest applied snapshot: id -> position (velocity and flight are cleared).
    std::map<std::uint32_t, wpiemu::sdk::match::EntityState> picture(std::uint16_t id) const {
        std::map<std::uint32_t, wpiemu::sdk::match::EntityState> out;
        for (const auto& [eid, held] : group(id)) {
            wpiemu::sdk::match::EntityState e = shown(held, latestTimeUs_);
            e.vxCms = e.vyCms = e.vzCms = 0;
            e.ballistic = false;
            out[eid] = e;
        }
        return out;
    }

private:
    std::map<std::uint32_t, std::map<std::uint16_t, Group>> mirrors_;    // per snapshotSeq: baseline + delta, what the host assumes
    std::map<std::uint32_t, std::map<std::uint16_t, Group>> displays_;   // per snapshotSeq: the mirror plus the updates in flight
    std::deque<std::uint32_t> order_;
    std::uint32_t latest_ = 0;
    std::int64_t latestTimeUs_ = 0;
    bool seen_ = false;
};

}  // namespace match_test

namespace match_test {

// Every entity of `truth` is shown within `cm` of it and nothing else is shown (a rest-state upsert is exact, so `cm` is the
// rest tolerance).
inline bool sameEntities(const std::map<std::uint32_t, wpiemu::sdk::match::EntityState>& shown,
                         const std::map<std::uint32_t, wpiemu::sdk::match::EntityState>& truth, int cm) {
    if (shown.size() != truth.size()) return false;
    for (const auto& [id, e] : truth) {
        const auto it = shown.find(id);
        if (it == shown.end()) return false;
        const int dx = it->second.xCm - e.xCm, dy = it->second.yCm - e.yCm, dz = it->second.zCm - e.zCm;
        if (dx * dx + dy * dy + dz * dz > cm * cm) return false;
    }
    return true;
}

}  // namespace match_test
