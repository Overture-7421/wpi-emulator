#include "wpiemu/core/match/entity_delta.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wpiemu/sdk/match/entity_prediction.h"

namespace wpiemu::core::match {

using sdk::match::EntityDelta;
using sdk::match::EntityState;

namespace {

template <class Group>
const Group* findGroup(std::span<const Group> groups, std::uint16_t id) {
    const auto it = std::lower_bound(groups.begin(), groups.end(), id, [](const Group& g, std::uint16_t v) { return g.groupId < v; });
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
    std::int64_t score = 0;
    std::uint16_t groupId = 0;
    EntityChangeReason reason = EntityChangeReason::New;
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

std::int64_t distanceSq(const EntityState& a, const EntityState& b) {
    const std::int64_t dx = static_cast<std::int64_t>(a.xCm) - b.xCm;
    const std::int64_t dy = static_cast<std::int64_t>(a.yCm) - b.yCm;
    const std::int64_t dz = static_cast<std::int64_t>(a.zCm) - b.zCm;
    return dx * dx + dy * dy + dz * dz;
}

bool moves(const EntityState& e) { return e.ballistic || e.vxCms != 0 || e.vyCms != 0 || e.vzCms != 0; }

// A client that holds nothing for an entity is as wrong as it can be: it ranks like a miss of this many centimetres.
constexpr std::int64_t kNewEntityErrorCm = 100;

}  // namespace

const EntityDeltaEncoder::Entry* EntityDeltaEncoder::find(std::uint32_t seq) const {
    for (const Entry& e : history_)
        if (e.seq == seq) return &e;
    return nullptr;
}

EntityPlan EntityDeltaEncoder::plan(std::span<const EntityGroupFrame> current, std::span<const RobotAnchor> anchors,
                                    std::int64_t simTimeUs) const {
    EntityPlan plan;
    const Entry* base = acked_ != sdk::match::kNoBaseline ? find(acked_) : nullptr;
    if (base != nullptr) plan.baselineId = acked_;
    const std::span<const HeldGroup> baseline = base != nullptr ? std::span<const HeldGroup>(base->state) : std::span<const HeldGroup>();

    for (const EntityGroupFrame& g : current) plan.groupIds.push_back(g.groupId);
    for (const HeldGroup& g : baseline) plan.groupIds.push_back(g.groupId);
    std::sort(plan.groupIds.begin(), plan.groupIds.end());
    plan.groupIds.erase(std::unique(plan.groupIds.begin(), plan.groupIds.end()), plan.groupIds.end());

    std::vector<EntityChange> removals;
    std::vector<Upsert> upserts;
    constexpr std::size_t cap = sdk::match::kMaxEntityDeltaEntries;
    for (const std::uint16_t gid : plan.groupIds) {
        const EntityGroupFrame* cur = findGroup(current, gid);
        const HeldGroup* old = findGroup(baseline, gid);
        const std::span<const EntityState> c = cur != nullptr ? std::span<const EntityState>(cur->entities) : std::span<const EntityState>();
        const std::span<const HeldEntity> b = old != nullptr ? std::span<const HeldEntity>(old->entities) : std::span<const HeldEntity>();
        std::size_t removed = 0, i = 0, j = 0;
        while (i < c.size() || j < b.size()) {
            const bool onlyBase = i >= c.size() || (j < b.size() && b[j].state.id < c[i].id);
            const bool onlyCur = j >= b.size() || (i < c.size() && c[i].id < b[j].state.id);
            if (onlyBase) {
                if (removed++ < cap) {
                    EntityChange r;
                    r.groupId = gid;
                    r.removal = true;
                    r.reason = EntityChangeReason::Removal;
                    r.state.id = b[j].state.id;
                    removals.push_back(r);
                } else {
                    ++plan.omittedByCap;
                }
                ++j;
            } else if (onlyCur) {
                const std::int64_t score = (kNewEntityErrorCm + kErrorFloorCm) * policy_.ageScaleUs / (1 + bandOf(c[i], anchors));
                upserts.push_back({score, gid, EntityChangeReason::New, c[i]});
                ++i;
            } else {
                const HeldEntity& held = b[j];
                const std::int64_t ageUs = std::max<std::int64_t>(0, simTimeUs - held.stampUs);
                const EntityState shown = sdk::match::predictEntity(held.state, ageUs);
                const std::int64_t errSq = distanceSq(c[i], shown);
                const bool truthMoves = moves(c[i]);
                const std::int64_t threshold = truthMoves ? policy_.errorThresholdCm : policy_.restErrorThresholdCm;
                const bool off = errSq > threshold * threshold;
                const std::int64_t keyframeUs = truthMoves || moves(held.state) ? policy_.keyframeMovingUs : policy_.keyframeRestUs;
                const bool keyframe = ageUs >= keyframeUs;
                if (off || keyframe) {
                    const std::int64_t score = (isqrt(errSq) + kErrorFloorCm) * (ageUs + policy_.ageScaleUs) / (1 + bandOf(c[i], anchors));
                    upserts.push_back({score, gid, off ? EntityChangeReason::Error : EntityChangeReason::Keyframe, c[i]});
                }
                ++i;
                ++j;
            }
        }
    }
    std::sort(upserts.begin(), upserts.end(), [](const Upsert& a, const Upsert& b) {
        if (a.score != b.score) return a.score > b.score;
        if (a.groupId != b.groupId) return a.groupId < b.groupId;
        return a.state.id < b.state.id;
    });
    plan.changes = std::move(removals);
    plan.changes.reserve(plan.changes.size() + upserts.size());
    std::vector<std::size_t> perGroup(plan.groupIds.size(), 0);
    for (const Upsert& u : upserts) {
        const auto slot = static_cast<std::size_t>(std::lower_bound(plan.groupIds.begin(), plan.groupIds.end(), u.groupId) - plan.groupIds.begin());
        if (perGroup[slot]++ >= cap) {
            ++plan.omittedByCap;
            continue;
        }
        plan.changes.push_back({u.groupId, false, u.reason, u.state});
    }
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
    for (EntityDelta& d : out) {
        std::sort(d.removed.begin(), d.removed.end());
        std::sort(d.upserted.begin(), d.upserted.end(), [](const EntityState& a, const EntityState& b) { return a.id < b.id; });
    }
    return out;
}

void EntityDeltaEncoder::commit(const EntityPlan& plan, std::size_t count, std::uint32_t snapshotSeq, std::int64_t simTimeUs) {
    const Entry* base = plan.baselineId != sdk::match::kNoBaseline ? find(plan.baselineId) : nullptr;
    const std::span<const HeldGroup> baseline = base != nullptr ? std::span<const HeldGroup>(base->state) : std::span<const HeldGroup>();
    const std::vector<EntityDelta> sent = deltas(plan, count);

    Entry entry;
    entry.seq = snapshotSeq;
    for (const EntityDelta& d : sent) {
        const HeldGroup* old = findGroup(baseline, d.groupId);
        const std::span<const HeldEntity> b = old != nullptr ? std::span<const HeldEntity>(old->entities) : std::span<const HeldEntity>();
        HeldGroup merged;
        merged.groupId = d.groupId;
        merged.entities.reserve(b.size() + d.upserted.size());
        std::size_t i = 0, j = 0;
        while (i < b.size() || j < d.upserted.size()) {
            if (j >= d.upserted.size() || (i < b.size() && b[i].state.id < d.upserted[j].id)) {
                if (!std::binary_search(d.removed.begin(), d.removed.end(), b[i].state.id)) merged.entities.push_back(b[i]);
                ++i;
            } else {
                if (i < b.size() && b[i].state.id == d.upserted[j].id) ++i;
                merged.entities.push_back({d.upserted[j], simTimeUs});
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
