#pragma once

// Per-robot plugin state keyed by participant id, for game states that treat each robot of a practice match on its own
// (fuel meter, shot counter, held items, zone runtimes...). The pattern every multi-robot game state needs:
//
//   sdk::PerRobotState<RobotState> robots_;
//   onRobotAdded:   if (!robot.isHost) robots_.add(robot.participantId, makeState(robot));   // host: nothing to create
//   onRobotRemoved: robots_.remove(participant);                                              // release what it held
//   step/publish:   robots_.forEach([&](ParticipantId id, RobotState& s) { ... });
//
// The HOST robot is always present (participant 0, created with the container, outside the remote map) so a game that
// plays without any roster keeps working and the host's behaviour never depends on a join. Iteration is DETERMINISTIC:
// the host first, then the remote robots in ascending participant id, whatever order they joined in; use it for every
// loop that spawns, consumes or draws randomness, so a replay and a client prediction agree.
//
// SIM THREAD only. Not copyable or movable: references to a robot's state stay valid until that robot is removed.

#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk {

template <class State>
class PerRobotState {
public:
    PerRobotState() = default;
    explicit PerRobotState(State host) : host_(std::move(host)) {}
    PerRobotState(const PerRobotState&) = delete;
    PerRobotState& operator=(const PerRobotState&) = delete;

    State& host() { return host_; }
    const State& host() const { return host_; }

    // Creates (or replaces) the state of remote robot `id` and returns it. The host id returns the host state unchanged.
    State& add(match::ParticipantId id, State state) {
        if (id == match::kHostParticipantId) return host_;
        return remotes_.insert_or_assign(id, std::move(state)).first->second;
    }
    // Drops the state of remote robot `id`; false when there is none. The host state cannot be removed.
    bool remove(match::ParticipantId id) { return id != match::kHostParticipantId && remotes_.erase(id) != 0; }
    // Forgets every remote robot (the host state stays).
    void clearRemotes() { remotes_.clear(); }

    // nullptr when no such robot is known (the host id always resolves).
    State* find(match::ParticipantId id) {
        if (id == match::kHostParticipantId) return &host_;
        const auto it = remotes_.find(id);
        return it != remotes_.end() ? &it->second : nullptr;
    }
    const State* find(match::ParticipantId id) const {
        if (id == match::kHostParticipantId) return &host_;
        const auto it = remotes_.find(id);
        return it != remotes_.end() ? &it->second : nullptr;
    }

    std::size_t size() const { return 1 + remotes_.size(); }  // the host included
    std::size_t remoteCount() const { return remotes_.size(); }

    // Calls fn(ParticipantId, State&): the host first, then the remote robots by ascending id.
    template <class Fn>
    void forEach(Fn&& fn) {
        fn(match::kHostParticipantId, host_);
        for (auto& entry : remotes_) fn(entry.first, entry.second);
    }
    template <class Fn>
    void forEach(Fn&& fn) const {
        fn(match::kHostParticipantId, host_);
        for (const auto& entry : remotes_) fn(entry.first, entry.second);
    }
    // Calls fn(ParticipantId, State&) for the remote robots only, by ascending id.
    template <class Fn>
    void forEachRemote(Fn&& fn) {
        for (auto& entry : remotes_) fn(entry.first, entry.second);
    }

    // Every id in iteration order: the host, then the remote robots ascending.
    std::vector<match::ParticipantId> ids() const {
        std::vector<match::ParticipantId> out;
        out.reserve(size());
        out.push_back(match::kHostParticipantId);
        for (const auto& entry : remotes_) out.push_back(entry.first);
        return out;
    }

private:
    State host_{};
    std::map<match::ParticipantId, State> remotes_;
};

}  // namespace wpiemu::sdk
