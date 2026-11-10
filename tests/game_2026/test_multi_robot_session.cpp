#include <doctest/doctest.h>

#include <cmath>
#include <vector>

#include "multirobot_session_rig.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26session::Options;
using g26session::Rig;
using sdk::proto::AllianceStation;

namespace {

constexpr float kMidX = dim::kFieldLengthM * 0.5f;
constexpr float kMidY = dim::kFieldWidthM * 0.5f;

struct Outcome {
    std::uint64_t hash = 0;
    int fuelOnField = 0;
    int hostFuel = 0;
    std::vector<int> counts;   // per remote robot: fuel count, shots
    bool operator==(const Outcome& o) const {
        return hash == o.hash && fuelOnField == o.fuelOnField && hostFuel == o.hostFuel && counts == o.counts;
    }
};

// A pickup robot in the neutral zone and a shooter robot next to it, both driven by their zone flags only.
Outcome scenario(int workerCount) {
    Options options;
    options.workerCount = workerCount;
    Rig rig(options);
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.add(2, AllianceStation::Blue2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX - 1.2f, kMidY, 0.04f});
    rig.teleport(2, {kMidX + 1.2f, kMidY - 1.5f, 0.04f}, 3.14159f);
    rig.run(150, {{1, 0b01}, {2, 0b10}});
    const auto snap = rig.snapshot();
    Outcome out;
    out.hash = rig.session->world().stateHash();
    out.fuelOnField = Rig::fuelOnField(*snap);
    const sdk::Value* hostFuel = snap->hudValue(hud::kFuelCount);
    out.hostFuel = hostFuel != nullptr ? static_cast<int>(hostFuel->asInt()) : -1;
    for (const sdk::ParticipantId id : std::vector<sdk::ParticipantId>{1, 2}) {
        const sdk::RobotSample* sample = Rig::sampleOf(*snap, id);
        REQUIRE(sample != nullptr);
        out.counts.push_back(Rig::hudInt(*sample, hud::kFuelCount));
        out.counts.push_back(Rig::hudInt(*sample, hud::kShots));
    }
    return out;
}

}  // namespace

TEST_CASE("multi robot session: remote robots pick up and shoot fuel of the real field through their zone flags") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.add(2, AllianceStation::Blue2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX - 1.2f, kMidY, 0.04f});
    rig.teleport(2, {kMidX + 1.2f, kMidY - 1.5f, 0.04f}, 3.14159f);

    // without flags nothing happens, and the HUD already shows the preload of every robot
    rig.run(20, {{1, 0}, {2, 0}});
    auto snap = rig.snapshot();
    const int staged = Rig::fuelOnField(*snap);
    CHECK(staged == 360 + 2 * 24);
    REQUIRE(Rig::sampleOf(*snap, 1) != nullptr);
    CHECK(Rig::hudInt(*Rig::sampleOf(*snap, 1), hud::kFuelCount) == 8);
    CHECK(Rig::hudInt(*Rig::sampleOf(*snap, 1), hud::kFuelCapacity) == 30);
    CHECK(Rig::hudInt(*Rig::sampleOf(*snap, 2), hud::kShots) == 0);
    CHECK(snap->hudValue(hud::kFuelCount)->asInt() == 8);

    rig.run(120, {{1, 0b01}, {2, 0b10}});
    snap = rig.snapshot();
    const sdk::RobotSample& picker = *Rig::sampleOf(*snap, 1);
    const sdk::RobotSample& shooter = *Rig::sampleOf(*snap, 2);
    CHECK(Rig::hudInt(picker, hud::kFuelCount) > 8);
    CHECK(Rig::hudInt(picker, hud::kShots) == 0);
    CHECK(Rig::hudInt(shooter, hud::kShots) > 0);
    CHECK(Rig::hudInt(shooter, hud::kFuelCount) < 8);
    CHECK(snap->hudValue(hud::kFuelCount)->asInt() == 8);   // the host robot is untouched
    CHECK(snap->hudValue(hud::kShots)->asInt() == 0);

    // the runtime reports each robot's zones from its own flags
    REQUIRE(picker.zones.size() == 2);
    CHECK(picker.zones[0].active);
    CHECK_FALSE(picker.zones[1].active);
    CHECK_FALSE(shooter.zones[0].active);
    CHECK(shooter.zones[1].active);
}

TEST_CASE("multi robot session: the host's station and its robot view cannot diverge") {
    Rig rig;
    rig.add(1, AllianceStation::Red1, Rig::fuelRobot());
    for (const AllianceStation station : {AllianceStation::Blue3, AllianceStation::Red2}) {
        rig.setStation(station);
        rig.run(2, {});
        const sdk::RobotView* host = rig.session->robots().find(sdk::match::kHostParticipantId);
        REQUIRE(host != nullptr);
        CHECK(host->station == station);
        CHECK(host->station == rig.session->driverStation().station);
        CHECK(host->isHost);
    }
}

TEST_CASE("multi robot session: removing and re-adding a robot resets its meter, reset restores everyone") {
    Rig rig;
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.teleport(1, {kMidX - 1.2f, kMidY, 0.04f});
    rig.run(120, {{1, 0b01}});
    auto snap = rig.snapshot();
    REQUIRE(Rig::hudInt(*Rig::sampleOf(*snap, 1), hud::kFuelCount) > 8);

    REQUIRE(rig.session->removeRemoteRobot(1).ok());
    snap = rig.snapshot();
    CHECK(Rig::sampleOf(*snap, 1) == nullptr);
    rig.add(1, AllianceStation::Red2, Rig::fuelRobot(6.0));
    rig.run(1, {});
    snap = rig.snapshot();
    CHECK(Rig::hudInt(*Rig::sampleOf(*snap, 1), hud::kFuelCount) == 8);

    rig.teleport(1, {kMidX - 1.2f, kMidY + 1.5f, 0.04f});  // fuel the first stint left
    rig.run(120, {{1, 0b01}});
    REQUIRE(Rig::hudInt(*Rig::sampleOf(*rig.snapshot(), 1), hud::kFuelCount) > 8);
    rig.session->reset();
    rig.run(1, {});
    snap = rig.snapshot();
    REQUIRE(Rig::sampleOf(*snap, 1) != nullptr);   // the roster survives a reset
    CHECK(Rig::hudInt(*Rig::sampleOf(*snap, 1), hud::kFuelCount) == 8);
    CHECK(Rig::fuelOnField(*snap) == 360 + 2 * 24);
}

TEST_CASE("multi robot session: the same scripted inputs give the same game state for 1, 2 and 4 physics workers") {
    const Outcome reference = scenario(1);
    CHECK(reference.counts[0] > 8);   // the scenario really picks up
    CHECK(reference.counts[3] > 0);   // and shoots
    CHECK(scenario(1) == reference);
    CHECK(scenario(2) == reference);
    CHECK(scenario(4) == reference);
}
