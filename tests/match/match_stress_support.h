#pragma once

// Fixtures of the entity stress scenario (test_match_entity_stress.cpp): the REAL 2026 plugin, SimulationSession and match
// host over impaired in-memory links, remote shooters driven by model clients that apply the entity deltas the way a game
// client does and measure how fresh the fuel they hold is.
//
//   TruthTap     IMatchHostSession over the real session. It records the TRUTH (id and quantized position of every fuel at
//                every snapshot the host builds, with the ids the plugin publishes) and changes nothing.
//   ModelClient  a joined client: sends inputs, applies snapshots and entity deltas on the baseline they name, acknowledges
//                them, and measures the staleness of what it holds against the truth.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>

#include "../game_2026/multirobot_session_rig.h"
#include "match_test_support.h"
#include "wpiemu/plugins/game_2026/types.h"
#include "wpiemu/sdk/core/environment.h"

namespace match_stress {

using namespace wpiemu;
namespace wire = wpiemu::sdk::match;
namespace proto = wpiemu::sdk::proto;
namespace mt = match_test;
using SteadyClock = std::chrono::steady_clock;

// ---- statistics -----------------------------------------------------------------------------------------------------

struct Stat {
    std::size_t n = 0;
    double p50 = 0;
    double p95 = 0;
    double max = 0;
};

inline Stat statOf(std::vector<double> values) {
    Stat s;
    s.n = values.size();
    if (values.empty()) return s;
    std::sort(values.begin(), values.end());
    const auto at = [&](double q) { return values[static_cast<std::size_t>(std::ceil(q * static_cast<double>(values.size()))) - 1]; };
    s.p50 = at(0.5);
    s.p95 = at(0.95);
    s.max = values.back();
    return s;
}

inline bool envFlag(const char* name) {
    const std::optional<std::string> v = sdk::readEnvironment(name);
    return v.has_value() && !v->empty() && (*v)[0] == '1';
}
inline bool reportRequested() { return envFlag("WPIEMU_STRESS_REPORT"); }
inline bool fullRunRequested() { return envFlag("WPIEMU_STRESS"); }

// ---- the fuel truth ---------------------------------------------------------------------------------------------------

struct TruthFrame {
    std::uint64_t tick = 0;
    std::int64_t simTimeUs = 0;
    std::vector<wire::EntityState> entities;   // ascending id
    std::size_t moving = 0;                    // entities whose quantized position differs from the previous frame (or are new)
};

class TruthTap final : public core::match::IMatchHostSession {
public:
    explicit TruthTap(core::runtime::SimulationSession& session) : inner_(session) {}

    std::string gamePluginId() const override { return inner_.gamePluginId(); }
    int wpilibYear() const override { return inner_.wpilibYear(); }
    std::string hostName() const override { return inner_.hostName(); }
    std::vector<sdk::HudWidgetDescriptor> hudDescriptors() const override { return inner_.hudDescriptors(); }
    std::int64_t stepUs() const override { return inner_.stepUs(); }
    std::uint64_t tickIndex() const override { return inner_.tickIndex(); }
    std::int64_t simTimeUs() const override { return inner_.simTimeUs(); }
    sdk::proto::DsState driverStation() const override { return inner_.driverStation(); }
    sdk::Status addRemoteRobot(const core::runtime::RemoteRobotRequest& r) override { return inner_.addRemoteRobot(r); }
    sdk::Status removeRemoteRobot(wire::ParticipantId id) override { return inner_.removeRemoteRobot(id); }
    sdk::Result<core::runtime::RemoteInputOutcome> submitRemoteInput(wire::ParticipantId id, const wire::MatchInput& in) override {
        return inner_.submitRemoteInput(id, in);
    }
    std::optional<core::runtime::RemoteRobotInfo> remoteRobotInfo(wire::ParticipantId id) const override {
        return inner_.remoteRobotInfo(id);
    }

    std::shared_ptr<const sdk::ISimSnapshot> buildSnapshot(bool paused, bool robotConnected) override {
        const SteadyClock::time_point t0 = SteadyClock::now();
        std::shared_ptr<const sdk::ISimSnapshot> snapshot = inner_.buildSnapshot(paused, robotConnected);
        record(*snapshot);
        tapTime_ += SteadyClock::now() - t0;
        lastBuild_ = snapshot;
        return snapshot;
    }

    const TruthFrame* truthAt(std::uint64_t tick) const {
        for (auto it = frames_.rbegin(); it != frames_.rend(); ++it)
            if (it->tick == tick) return &*it;
        return nullptr;
    }
    std::shared_ptr<const sdk::ISimSnapshot> lastSnapshot() const { return lastBuild_; }
    std::chrono::nanoseconds takeTapTime() {
        const auto t = tapTime_;
        tapTime_ = {};
        return t;
    }
    bool pluginPublishesIds() const { return pluginIds_; }
    std::size_t idMismatches() const { return idMismatches_; }

private:
    // Records the truth frame from the entity ids the plugin published.
    void record(const sdk::ISimSnapshot& snapshot) {
        TruthFrame frame;
        frame.tick = snapshot.tick();
        frame.simTimeUs = snapshot.simTimeUs();
        for (const sdk::EntityGroupSample& g : snapshot.entityGroups()) {
            if (g.group != "fuel") continue;
            if (!g.ids.empty()) pluginIds_ = true;
            if (g.ids.empty() || !sdk::entityIdsConsistent(g)) ++idMismatches_;
            for (std::size_t i = 0; i < g.ids.size() && i < g.positions.size(); ++i) {
                wire::EntityState e;
                e.id = g.ids[i];
                e.xCm = wire::quantizeCentiI16(g.positions[i].x);
                e.yCm = wire::quantizeCentiI16(g.positions[i].y);
                e.zCm = wire::quantizeCentiI16(g.positions[i].z);
                frame.entities.push_back(e);
            }
            std::sort(frame.entities.begin(), frame.entities.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
        }
        frame.moving = countMoving(frame.entities, frames_.empty() ? nullptr : &frames_.back().entities);
        frames_.push_back(std::move(frame));
        if (frames_.size() > 48) frames_.pop_front();
    }

    static std::size_t countMoving(const std::vector<wire::EntityState>& now, const std::vector<wire::EntityState>* before) {
        if (before == nullptr) return now.size();
        std::size_t moving = 0;
        std::size_t j = 0;
        for (const wire::EntityState& e : now) {
            while (j < before->size() && (*before)[j].id < e.id) ++j;
            if (j >= before->size() || (*before)[j].id != e.id || !((*before)[j] == e)) ++moving;
        }
        return moving;
    }

    core::match::SimulationMatchSession inner_;
    std::deque<TruthFrame> frames_;
    std::shared_ptr<const sdk::ISimSnapshot> lastBuild_;
    std::chrono::nanoseconds tapTime_{};
    bool pluginIds_ = false;
    std::size_t idMismatches_ = 0;
};

// ---- the model client -------------------------------------------------------------------------------------------------

struct ClientMetrics {
    std::vector<double> ageMs;          // sim time between two updates of the same entity that moved in between
    std::vector<double> ageSnapshots;   // the same in snapshots sent to this client (lost ones count)
    std::uint64_t redundantUpserts = 0; // upserts of a value the client already held (resent until acknowledged)
    std::vector<double> firstUpdateMs;  // spawn to first update
    std::vector<double> upserts;        // per received snapshot
    std::vector<double> removals;
    std::vector<double> entityBytes;    // encoded size of the whole snapshot
    std::vector<double> moving;         // truth: entities that moved since the previous snapshot build
    std::vector<double> behind;         // entities whose held state differs from the truth, per received snapshot
    std::vector<double> fuelOnField;
    std::set<std::uint32_t> starvedIds; // moving entities not updated for more than kStarvedMs at some snapshot
    std::uint64_t starvedEvents = 0;    // (snapshot, entity) pairs above kStarvedMs
    double maxStaleMs = 0;
    std::size_t maxStuck = 0;
    std::uint64_t received = 0;
    std::uint64_t staleDropped = 0;
    std::uint64_t unappliable = 0;      // the named baseline was not held
    std::uint64_t fullStates = 0;       // deltas without a baseline
    std::uint64_t incomplete = 0;       // snapshots without all robots or without a Running info
    std::uint64_t noTruth = 0;
};

class ModelClient {
public:
    static constexpr std::int64_t kStarvedMs = 500;
    static constexpr std::int64_t kStuckUs = 1'500'000;
    static constexpr std::size_t kHeldHistory = 64;

    ModelClient(mt::Harness& h, const sdk::testing::ImpairmentProfile& profile, std::uint64_t seed, TruthTap& tap)
        : h_(h), tap_(tap), sc_(h, profile, seed) {}

    mt::ScriptedTransport& hostEnd() { return *sc_.transport; }
    mt::TestClient& client() { return *sc_.client; }
    wire::ParticipantId id() const { return sc_.client->accept->participantId; }

    // One input at 50 Hz: the shooter zone (bit 0) held. `leadTicks` ahead of the newest host tick seen.
    void sendInput(std::uint64_t leadTicks, std::uint32_t zoneFlags, std::size_t wheels) {
        const mt::TestClient& c = *sc_.client;
        wire::MatchInput in;
        in.inputSeq = ++seq_;
        in.targetTick = hostTick_ + leadTicks;
        in.zoneFlags = zoneFlags;
        in.wheels.assign(wheels, wire::WheelInput{0.f, 0});
        wire::appendInput(packet_, std::move(in));
        packet_.sessionId = c.accept->sessionId;
        packet_.runEpoch = c.accept->runEpoch;
        (void)sc_.client->send(packet_);
    }

    void seedClock(std::uint64_t tick) { hostTick_ = tick; }

    // Receives what is ready and applies it; `measuring` selects whether the statistics are kept.
    void pump(bool measuring, std::size_t expectedRobots) {
        sc_.client->drain();
        for (const proto::Message& m : sc_.client->log)
            if (const auto* s = std::get_if<wire::MatchSnapshot>(&m)) process(*s, measuring, expectedRobots);
        sc_.client->log.clear();
    }

    std::size_t laggingNow() const { return lagging_; }
    // Entities the client holds wrongly although they have not moved for kStuckUs (starved at rest).
    std::size_t stuckNow() const { return stuck_; }
    const ClientMetrics& metrics() const { return metrics_; }
    ClientMetrics takeMetrics() { return std::exchange(metrics_, ClientMetrics{}); }
    std::size_t heldCount() const { return held_.empty() ? 0 : held_.rbegin()->second.size(); }

private:
    using State = std::unordered_map<std::uint32_t, wire::EntityState>;

    struct Track {
        std::int64_t firstTruthUs = 0;
        std::int64_t lastUpdateUs = 0;
        std::uint32_t lastUpdateSeq = 0;
        bool updated = false;
        bool restedSince = false;           // at rest at some snapshot since its last update (not awake continuously)
        bool truthKnown = false;
        wire::EntityState truth;            // the newest truth seen for this entity and when it last changed
        std::int64_t truthChangedUs = 0;
    };

    void process(const wire::MatchSnapshot& s, bool measuring, std::size_t expectedRobots) {
        const mt::TestClient& c = *sc_.client;
        if (s.sessionId != c.accept->sessionId || s.runEpoch != c.accept->runEpoch) return;
        if (haveSeq_ && !wire::seqNewer(s.snapshotSeq, lastSeq_)) {
            if (measuring) ++metrics_.staleDropped;
            return;
        }
        if (s.tick > hostTick_) hostTick_ = s.tick;

        State state;
        const wire::EntityDelta* delta = s.entityDeltas.empty() ? nullptr : &s.entityDeltas.front();
        if (delta != nullptr && delta->baselineId != wire::kNoBaseline) {
            const auto base = held_.find(delta->baselineId);
            if (base == held_.end()) {
                if (measuring) ++metrics_.unappliable;
                return;   // no ack: the host falls back to full state
            }
            state = base->second;
        } else if (delta != nullptr && measuring) {
            ++metrics_.fullStates;
        } else if (delta == nullptr && !held_.empty()) {
            state = latest_;
        }
        haveSeq_ = true;
        lastSeq_ = s.snapshotSeq;
        if (delta != nullptr) {
            for (const std::uint32_t r : delta->removed) {
                state.erase(r);
                track_.erase(r);
            }
        }

        const TruthFrame* truth = tap_.truthAt(s.tick);
        const std::int64_t now = s.simTimeUs;
        if (truth != nullptr) {
            for (const wire::EntityState& e : truth->entities) {
                Track& t = track_[e.id];
                if (!t.truthKnown) {
                    t.truthKnown = true;
                    t.firstTruthUs = now;
                    t.truth = e;
                    t.truthChangedUs = now;
                } else if (!(t.truth == e)) {
                    t.truth = e;
                    t.truthChangedUs = now;
                    t.restedSince = false;
                } else {
                    t.restedSince = true;   // did not move since the previous snapshot: a gap in updates is not starvation
                }
            }
        }
        if (delta != nullptr) {
            for (const wire::EntityState& u : delta->upserted) {
                const auto before = latest_.find(u.id);
                const bool known = before != latest_.end();
                const bool teleported = known && (std::abs(before->second.xCm - u.xCm) > 100 || std::abs(before->second.yCm - u.yCm) > 100);
                state[u.id] = u;
                Track& t = track_[u.id];
                if (teleported) t = Track{};   // a recycled id: a new entity
                if (known && before->second == u && !teleported) {
                    if (measuring) ++metrics_.redundantUpserts;   // the client already held this value
                    continue;
                }
                if (t.updated && !t.restedSince && measuring) {
                    metrics_.ageMs.push_back(static_cast<double>(now - t.lastUpdateUs) / 1000.0);
                    metrics_.ageSnapshots.push_back(static_cast<double>(s.snapshotSeq - t.lastUpdateSeq));
                } else if (!t.updated && t.truthKnown && measuring) {
                    metrics_.firstUpdateMs.push_back(static_cast<double>(now - t.firstTruthUs) / 1000.0);
                }
                t.updated = true;
                t.restedSince = false;
                t.lastUpdateUs = now;
                t.lastUpdateSeq = s.snapshotSeq;
            }
        }

        std::size_t behind = 0;
        std::size_t stuck = 0;
        if (truth != nullptr) {
            for (const wire::EntityState& e : truth->entities) {
                const auto it = state.find(e.id);
                const bool same = it != state.end() && it->second == e;
                if (same) continue;
                ++behind;
                Track& t = track_[e.id];
                if (now - t.truthChangedUs >= kStuckUs) ++stuck;
                const std::int64_t since = t.updated ? t.lastUpdateUs : t.firstTruthUs;
                const double staleMs = static_cast<double>(now - since) / 1000.0;
                if (measuring && !t.restedSince) {
                    metrics_.maxStaleMs = std::max(metrics_.maxStaleMs, staleMs);
                    if (staleMs > static_cast<double>(kStarvedMs)) {
                        ++metrics_.starvedEvents;
                        metrics_.starvedIds.insert(e.id);
                    }
                }
            }
        } else if (measuring) {
            ++metrics_.noTruth;
        }
        lagging_ = behind;
        stuck_ = stuck;
        if (measuring) metrics_.maxStuck = std::max<std::size_t>(metrics_.maxStuck, stuck);

        held_[s.snapshotSeq] = state;
        latest_ = std::move(state);
        while (held_.size() > kHeldHistory) held_.erase(held_.begin());
        (void)sc_.client->send(wire::MatchSnapshotAck{s.sessionId, s.runEpoch, s.snapshotSeq});

        if (!measuring) return;
        ++metrics_.received;
        if (s.robots.size() != expectedRobots || s.info.state != wire::MatchState::Running) ++metrics_.incomplete;
        metrics_.upserts.push_back(delta != nullptr ? static_cast<double>(delta->upserted.size()) : 0.0);
        metrics_.removals.push_back(delta != nullptr ? static_cast<double>(delta->removed.size()) : 0.0);
        proto::Message message = s;
        std::vector<std::byte> bytes;
        if (h_.codec->encode(message, bytes).ok()) metrics_.entityBytes.push_back(static_cast<double>(bytes.size()));
        if (truth != nullptr) {
            metrics_.moving.push_back(static_cast<double>(truth->moving));
            metrics_.behind.push_back(static_cast<double>(behind));
            metrics_.fuelOnField.push_back(static_cast<double>(truth->entities.size()));
        }
    }

    mt::Harness& h_;
    TruthTap& tap_;
    mt::ScriptedClient sc_;
    std::uint32_t seq_ = 0;
    wire::MatchInputPacket packet_;
    std::uint64_t hostTick_ = 0;
    bool haveSeq_ = false;
    std::uint32_t lastSeq_ = 0;
    std::map<std::uint32_t, State> held_;   // entity state per received snapshotSeq (the client's baselines)
    State latest_;                          // the state of the newest applied snapshot
    std::unordered_map<std::uint32_t, Track> track_;
    std::size_t lagging_ = 0;
    std::size_t stuck_ = 0;
    ClientMetrics metrics_;
};

}  // namespace match_stress
