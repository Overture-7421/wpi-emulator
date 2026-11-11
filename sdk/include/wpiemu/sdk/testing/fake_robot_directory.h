#pragma once

#include <algorithm>
#include <span>
#include <utility>
#include <vector>

#include "wpiemu/sdk/game/robot_view.h"

namespace wpiemu::sdk::testing {

// A mutable IRobotDirectory for tests: the test edits `list` directly (host first by convention).
class FakeRobotDirectory final : public IRobotDirectory {
public:
    std::span<const RobotView> robots() const override { return list; }

    RobotView& add(RobotView view) {
        list.push_back(std::move(view));
        return list.back();
    }
    bool remove(ParticipantId id) {
        const auto it = std::find_if(list.begin(), list.end(), [id](const RobotView& r) { return r.participantId == id; });
        if (it == list.end()) return false;
        list.erase(it);
        return true;
    }

    std::vector<RobotView> list;
};

}  // namespace wpiemu::sdk::testing
