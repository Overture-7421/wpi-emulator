#include "wpiemu/core/match/entity_delta.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace wpiemu::core::match {

using sdk::match::EntityDelta;
using sdk::match::EntityState;

namespace {

const EntityGroupFrame* findGroup(std::span<const EntityGroupFrame> groups, std::uint16_t id) {
    const auto it = std::lower_bound(groups.begin(), groups.end(), id,
                                     [](const EntityGroupFrame& g, std::uint16_t v) { return g.groupId < v; });
    return it != groups.end() && it->groupId == id ? &*it : nullptr;
}

std::int64_t isqrt(std::int64_t v) {
    if (v <= 0) return 0;
    auto r = static_cast<std::int64_t>(std::sqrt(static_cast<double>(v)));
    while (r * r > v) --r;
    while ((r + 1) * (r + 1) <= v) ++r;
    return r;
}

struct Upsert {
    std::int64_t band = 0;
    std::int64_t displacement = 0;
    std::uint16_t groupId = 0;
    EntityState state;
};

std::int64_t bandOf(const EntityState& e, std::span<const RobotAnchor> anchors) {
    if (anchors.empty()) return 0;
    std::int64_t best = std::numeric_limits<std::int64_t>::max();
    for (const RobotAnchor& a : anchors) {
        const std::int64_t dx = static_cast<std::int64_t>(e.xCm) - a.xCm;
        const std::int64_t dy = static_cast<std::int64_t>(e.yCm) - a.yCm;
        best = std::min(best, dx * dx + dy * dy);
    }
    return isqrt(best) / EntityDeltaEncoder::kBandCm;
}

std::int64_t displacementSq(const EntityState& a, const EntityState& b) {
    const std::int64_t dx = static_cast<std::int64_t>(a.xCm) - b.xCm;
    const std::int64_t dy = static_cast<std::int64_t>(a.yCm) - b.yCm;
    const std::int64_t dz = static_cast<std::int64_t>(a.zCm) - b.zCm;
    return dx * dx + dy * dy + dz * dz;
}

constexpr std::int64_t kNewEntityDisplacement = std::numeric_limits<std::int64_t>::max();

}  // namespace

const EntityDeltaEncoder::Entry* EntityDeltaEncoder::find(std::uint32_t seq) const {
    for (const Entry& e : history_)
        if (e.seq == seq) return &e;
    return nullptr;
}

EntityPlan EntityDeltaEncoder::plan(std::span<const EntityGroupFrame> current, std::span<const RobotAnchor> anchors) const {
    EntityPlan plan;
    const Entry* base = acked_ != sdk::match::kNoBaseline ? find(acked_) : nullptr;
    if (base != nullptr) plan.baselineId = acked_;
    const std::span<const EntityGroupFrame> baseline = base != nullptr ? std::span<const EntityGroupFrame>(base->state)
                                                                       : std::span<const EntityGroupFrame>();

    for (const EntityGroupFrame& g : current) plan.groupIds.push_back(g.groupId);
    for (const EntityGroupFrame& g : baseline) plan.groupIds.push_back(g.groupId);
    std::sort(plan.groupIds.begin(), plan.groupIds.end());
    plan.groupIds.erase(std::unique(plan.groupIds.begin(), plan.groupIds.end()), plan.groupIds.end());

    std::vector<EntityChange> removals;
    std::vector<Upsert> upserts;
    constexpr std::size_t cap = sdk::match::kMaxEntityDeltaEntries;
    for (const std::uint16_t gid : plan.groupIds) {
        const EntityGroupFrame* cur = findGroup(current, gid);
        const EntityGroupFrame* old = findGroup(baseline, gid);
        const std::span<const EntityState> c = cur != nullptr ? std::span<const EntityState>(cur->entities) : std::span<const EntityState>();
        const std::span<const EntityState> b = old != nullptr ? std::span<const EntityState>(old->entities) : std::span<const EntityState>();
        std::size_t removed = 0, upserted = 0, i = 0, j = 0;
        while (i < c.size() || j < b.size()) {
            const bool onlyBase = i >= c.size() || (j < b.size() && b[j].id < c[i].id);
            const bool onlyCur = j >= b.size() || (i < c.size() && c[i].id < b[j].id);
            if (onlyBase) {
                if (removed++ < cap) {
                    EntityChange r;
                    r.groupId = gid;
                    r.removal = true;
                    r.state.id = b[j].id;
                    removals.push_back(r);
                } else {
                    ++plan.omittedByCap;
                }
                ++j;
            } else if (onlyCur) {
                if (upserted++ < cap) upserts.push_back({bandOf(c[i], anchors), kNewEntityDisplacement, gid, c[i]});
                else ++plan.omittedByCap;
                ++i;
            } else {
                if (!(c[i] == b[j])) {
                    if (upserted++ < cap) upserts.push_back({bandOf(c[i], anchors), displacementSq(c[i], b[j]), gid, c[i]});
                    else ++plan.omittedByCap;
                }
                ++i;
                ++j;
            }
        }
    }
    std::sort(upserts.begin(), upserts.end(), [](const Upsert& a, const Upsert& b) {
        if (a.band != b.band) return a.band < b.band;
        if (a.displacement != b.displacement) return a.displacement > b.displacement;
        if (a.groupId != b.groupId) return a.groupId < b.groupId;
        return a.state.id < b.state.id;
    });
    plan.changes = std::move(removals);
    plan.changes.reserve(plan.changes.size() + upserts.size());
    for (const Upsert& u : upserts) plan.changes.push_back({u.groupId, false, u.state});
    return plan;
}

std::vector<EntityDelta> EntityDeltaEncoder::deltas(const EntityPlan& plan, std::size_t count) {
    std::vector<EntityDelta> out(plan.groupIds.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].groupId = plan.groupIds[i];
        out[i].baselineId = plan.baselineId;
    }
    const std::size_t n = std::min(count, plan.changes.size());
    for (std::size_t k = 0; k < n; ++k) {
        const EntityChange& c = plan.changes[k];
        const auto it = std::lower_bound(plan.groupIds.begin(), plan.groupIds.end(), c.groupId);
        EntityDelta& d = out[static_cast<std::size_t>(it - plan.groupIds.begin())];
        if (c.removal) d.removed.push_back(c.state.id);
        else d.upserted.push_back(c.state);
    }
    return out;
}

void EntityDeltaEncoder::commit(const EntityPlan& plan, std::size_t count, std::uint32_t snapshotSeq) {
    const Entry* base = plan.baselineId != sdk::match::kNoBaseline ? find(plan.baselineId) : nullptr;
    const std::span<const EntityGroupFrame> baseline = base != nullptr ? std::span<const EntityGroupFrame>(base->state)
                                                                       : std::span<const EntityGroupFrame>();
    std::vector<std::vector<std::uint32_t>> removed(plan.groupIds.size());
    std::vector<std::vector<EntityState>> upserted(plan.groupIds.size());
    const std::size_t n = std::min(count, plan.changes.size());
    for (std::size_t k = 0; k < n; ++k) {
        const EntityChange& c = plan.changes[k];
        const auto slot = static_cast<std::size_t>(std::lower_bound(plan.groupIds.begin(), plan.groupIds.end(), c.groupId) -
                                                   plan.groupIds.begin());
        if (c.removal) removed[slot].push_back(c.state.id);
        else upserted[slot].push_back(c.state);
    }

    Entry entry;
    entry.seq = snapshotSeq;
    for (std::size_t s = 0; s < plan.groupIds.size(); ++s) {
        std::sort(removed[s].begin(), removed[s].end());
        std::sort(upserted[s].begin(), upserted[s].end(), [](const EntityState& a, const EntityState& b) { return a.id < b.id; });
        const EntityGroupFrame* old = findGroup(baseline, plan.groupIds[s]);
        const std::span<const EntityState> b = old != nullptr ? std::span<const EntityState>(old->entities) : std::span<const EntityState>();
        EntityGroupFrame merged;
        merged.groupId = plan.groupIds[s];
        merged.entities.reserve(b.size() + upserted[s].size());
        std::size_t i = 0, j = 0;
        while (i < b.size() || j < upserted[s].size()) {
            if (j >= upserted[s].size() || (i < b.size() && b[i].id < upserted[s][j].id)) {
                if (!std::binary_search(removed[s].begin(), removed[s].end(), b[i].id)) merged.entities.push_back(b[i]);
                ++i;
            } else {
                if (i < b.size() && b[i].id == upserted[s][j].id) ++i;
                merged.entities.push_back(upserted[s][j]);
                ++j;
            }
        }
        if (!merged.entities.empty()) entry.state.push_back(std::move(merged));
    }

    std::erase_if(history_, [&](const Entry& e) { return e.seq == snapshotSeq; });
    history_.push_back(std::move(entry));
    while (history_.size() > kHistory) history_.pop_front();
    if (acked_ != sdk::match::kNoBaseline && find(acked_) == nullptr) acked_ = sdk::match::kNoBaseline;
}

bool EntityDeltaEncoder::onAck(std::uint32_t snapshotSeq) {
    if (snapshotSeq == sdk::match::kNoBaseline || find(snapshotSeq) == nullptr) return false;
    if (acked_ != sdk::match::kNoBaseline && !sdk::match::seqNewer(snapshotSeq, acked_)) return false;
    acked_ = snapshotSeq;
    return true;
}

void EntityDeltaEncoder::reset() {
    history_.clear();
    acked_ = sdk::match::kNoBaseline;
}

}  // namespace wpiemu::core::match
