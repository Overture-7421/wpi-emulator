#include <doctest/doctest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/game_sounds.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/game/sound_cue.h"

using namespace wpiemu;
using namespace g26test;
using plugins::game_2026::sound::kShiftChange;

namespace {

struct Cue {
    std::string id;
    double atS;
};

struct Rig {
    Rig() {
        emptyField(s);
        s.start();
    }

    // Ticks `seconds` of sim time. fromRemaining >= 0 is a Practice clock counting down from it, as the DS publishes it.
    void run(double seconds, double fromRemaining, bool enabled = true,
             sdk::proto::RobotMode mode = sdk::proto::RobotMode::Teleop) {
        const double dt = s.env.clock.fixedStepSeconds();
        const int ticks = static_cast<int>(seconds / dt + 0.5);
        for (int i = 0; i < ticks; ++i) {
            sdk::proto::DsState& ds = s.env.driverStation;
            ds.enabled = enabled;
            ds.mode = mode;
            ds.matchTimeS = fromRemaining >= 0.0 ? std::max(0.0, fromRemaining - i * dt) : -1.0;
            s.tick();
            for (const sdk::SoundCue& c : s.env.sounds.take()) cues.push_back({c.id, sdk::usToSeconds(c.simTimeUs)});
        }
    }
    // The DS ended the match: no match time, disabled.
    void stop(double seconds, bool eStop = false) {
        s.env.driverStation.eStop = eStop;
        run(seconds, -1.0, false);
    }
    int count(const std::string& id) const {
        return static_cast<int>(std::count_if(cues.begin(), cues.end(), [&](const Cue& c) { return c.id == id; }));
    }
    double firstAt(const std::string& id) const {
        for (const Cue& c : cues)
            if (c.id == id) return c.atS;
        return -1.0;
    }

    Session s;
    std::vector<Cue> cues;
};

}  // namespace

TEST_CASE("sound cues: a full TELEOP sounds the four shifts, the end game and then the result (manual Table 5-4)") {
    Rig r;
    r.run(140.0, 140.0);
    REQUIRE(r.count(kShiftChange) == 4);
    std::vector<double> shifts;
    for (const Cue& c : r.cues)
        if (c.id == kShiftChange) shifts.push_back(c.atS);
    CHECK(shifts[0] == doctest::Approx(10.0).epsilon(0.002));   // 2:10
    CHECK(shifts[1] == doctest::Approx(35.0).epsilon(0.001));   // 1:45
    CHECK(shifts[2] == doctest::Approx(60.0).epsilon(0.001));   // 1:20
    CHECK(shifts[3] == doctest::Approx(85.0).epsilon(0.001));   // 0:55
    REQUIRE(r.count(std::string(sdk::sound_cue::kEndGame)) == 1);
    CHECK(r.firstAt(std::string(sdk::sound_cue::kEndGame)) == doctest::Approx(110.0).epsilon(0.001));   // 0:30
    CHECK(r.count(std::string(sdk::sound_cue::kMatchResult)) == 0);   // still within the scoring window

    r.stop(5.0);   // the DS finished the match
    REQUIRE(r.count(std::string(sdk::sound_cue::kMatchResult)) == 1);
    CHECK(r.firstAt(std::string(sdk::sound_cue::kMatchResult)) - 140.0 == doctest::Approx(3.0).epsilon(0.05));   // 6.5 grace
    CHECK(r.cues.size() == 6);
}

TEST_CASE("sound cues: nothing sounds on the first TELEOP tick, in free play, in Test or when re-enabling inside a phase") {
    {
        Rig r;   // a shortened teleop starts inside SHIFT 3: only the boundaries it crosses sound
        r.run(31.0, 60.0);
        CHECK(r.count(kShiftChange) == 1);
        CHECK(r.firstAt(kShiftChange) == doctest::Approx(5.0).epsilon(0.01));
        CHECK(r.count(std::string(sdk::sound_cue::kEndGame)) == 1);
    }
    {
        Rig r;   // enabled without a match clock
        r.run(20.0, -1.0);
        r.run(20.0, -1.0, true, sdk::proto::RobotMode::Test);
        CHECK(r.cues.empty());
    }
    {
        Rig r;   // pause and re-enable inside SHIFT 1
        r.run(12.0, 140.0);
        REQUIRE(r.count(kShiftChange) == 1);
        r.run(1.0, 128.0, false);
        r.run(2.0, 128.0, true);
        CHECK(r.count(kShiftChange) == 1);
    }
}

TEST_CASE("sound cues: the result chime is skipped after an E-Stop and cancelled by a reset") {
    const std::string result(sdk::sound_cue::kMatchResult);
    {
        Rig r;
        r.run(40.0, 140.0);
        r.stop(6.0, true);
        CHECK(r.count(result) == 0);
    }
    {
        Rig r;
        r.run(40.0, 140.0);
        r.stop(1.0);   // aborted by Disable: the score still posts once the window closes
        r.stop(5.0);
        CHECK(r.count(result) == 1);
    }
    {
        Rig r;
        r.run(40.0, 140.0);
        r.stop(0.5);
        r.s.stateOwner->reset(r.s.env.ctx);
        r.stop(6.0);
        CHECK(r.count(result) == 0);
    }
}

TEST_CASE("sound cues: every cue the plugin declares is namespaced and names an asset") {
    const auto cues = plugins::game_2026::soundCueInfos();
    REQUIRE_FALSE(cues.empty());
    for (const sdk::SoundCueInfo& cue : cues) {
        CHECK(cue.id.starts_with("game_2026."));
        CHECK_FALSE(cue.asset.empty());
    }
    Session s;
    CHECK(s.module().contributeSoundCues().size() == cues.size());
}
