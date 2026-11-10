#include <doctest/doctest.h>

#include <memory>
#include <vector>

#include "wpiemu/sdk/game/per_robot_state.h"

using namespace wpiemu::sdk;

namespace {

struct Held {
    int count = 0;
    std::unique_ptr<int> payload;  // move-only states are supported
};

}  // namespace

TEST_CASE("PerRobotState always has the host and iterates host first then ascending id") {
    PerRobotState<Held> robots;
    CHECK(robots.size() == 1);
    CHECK(robots.remoteCount() == 0);
    CHECK(robots.find(match::kHostParticipantId) == &robots.host());

    robots.add(7, Held{7, nullptr});
    robots.add(2, Held{2, nullptr});
    robots.add(5, Held{5, std::make_unique<int>(5)});
    CHECK(robots.size() == 4);
    CHECK(robots.ids() == std::vector<match::ParticipantId>{0, 2, 5, 7});

    std::vector<match::ParticipantId> seen;
    robots.forEach([&](match::ParticipantId id, Held&) { seen.push_back(id); });
    CHECK(seen == robots.ids());

    seen.clear();
    const PerRobotState<Held>& view = robots;
    view.forEach([&](match::ParticipantId id, const Held& h) {
        seen.push_back(id);
        if (id != match::kHostParticipantId) CHECK(h.count == id);
    });
    CHECK(seen.size() == 4);

    seen.clear();
    robots.forEachRemote([&](match::ParticipantId id, Held&) { seen.push_back(id); });
    CHECK(seen == std::vector<match::ParticipantId>{2, 5, 7});
}

TEST_CASE("PerRobotState add replaces a remote robot and leaves the host alone") {
    PerRobotState<Held> robots;
    robots.host().count = 41;
    robots.add(3, Held{1, nullptr});
    robots.add(3, Held{2, nullptr});
    CHECK(robots.remoteCount() == 1);
    CHECK(robots.find(3)->count == 2);
    CHECK(&robots.add(match::kHostParticipantId, Held{99, nullptr}) == &robots.host());
    CHECK(robots.host().count == 41);
    CHECK(robots.remoteCount() == 1);
}

TEST_CASE("PerRobotState remove drops a remote robot only and clearRemotes keeps the host") {
    PerRobotState<Held> robots;
    robots.add(1, Held{});
    robots.add(2, Held{});
    CHECK(robots.remove(1));
    CHECK_FALSE(robots.remove(1));
    CHECK_FALSE(robots.remove(match::kHostParticipantId));
    CHECK(robots.find(1) == nullptr);
    CHECK(robots.find(2) != nullptr);
    robots.clearRemotes();
    CHECK(robots.ids() == std::vector<match::ParticipantId>{match::kHostParticipantId});
    CHECK(robots.find(match::kHostParticipantId) != nullptr);
}

TEST_CASE("PerRobotState references stay valid while other robots join and leave") {
    PerRobotState<Held> robots;
    Held& two = robots.add(2, Held{2, nullptr});
    for (match::ParticipantId id = 3; id < 9; ++id) robots.add(id, Held{static_cast<int>(id), nullptr});
    robots.remove(5);
    CHECK(&two == robots.find(2));
    CHECK(two.count == 2);
}
