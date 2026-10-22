#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/match_sounds.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using sdk::proto::RobotMode;

namespace {

constexpr double kStep = 0.005;

struct Heard {
    std::string id;
    double timeS;
};

// Drives the game state with scripted Driver Station states and records every cue the game raises.
class Match {
public:
    Match() {
        g26test::emptyField(s);
        s.start();
        idle(0.1);  // the first untimed state arms the sounds
    }

    // Seconds of an untimed state (free play / idle / a stopped match).
    void idle(double seconds, bool enabled = false, RobotMode mode = RobotMode::Teleop, bool eStop = false) {
        run(seconds, [&](int) {
            sdk::proto::DsState ds;
            ds.enabled = enabled;
            ds.mode = mode;
            ds.eStop = eStop;
            return ds;
        });
    }
    // A timed segment counting down from `fromS`, like the Driver Station's match timer.
    void timed(RobotMode mode, bool enabled, double fromS, double seconds) {
        run(seconds, [&](int i) {
            sdk::proto::DsState ds;
            ds.enabled = enabled;
            ds.mode = mode;
            ds.matchTimeS = fromS - i * kStep;
            return ds;
        });
    }
    // AUTO, the gap, TELEOP: the Driver Station's Practice sequence.
    void practice(double autoS, double gapS, double teleopS) {
        timed(RobotMode::Autonomous, true, autoS, autoS);
        if (gapS > 0.0) timed(RobotMode::Teleop, false, gapS, gapS);
        timed(RobotMode::Teleop, true, teleopS, teleopS);
    }
    void tickWith(const sdk::proto::DsState& ds) {
        s.env.driverStation = ds;
        s.tick();
        collect();
    }

    std::vector<std::string> ids() const {
        std::vector<std::string> out;
        for (const Heard& h : heard) out.push_back(h.id);
        return out;
    }

    g26test::Session s;
    std::vector<Heard> heard;

private:
    template <class Make>
    void run(double seconds, Make make) {
        const int steps = static_cast<int>(seconds / kStep + 0.5);
        for (int i = 0; i < steps; ++i) tickWith(make(i));
    }
    void collect() {
        for (const sdk::SoundCueEvent& e : s.game->sounds().log().events()) {
            if (e.sequence <= lastSequence_) continue;
            lastSequence_ = e.sequence;
            heard.push_back({e.cueId, sdk::usToSeconds(e.simTimeUs)});
        }
    }
    std::uint64_t lastSequence_ = 0;
};

const std::vector<std::string> kFullMatch = {sound::kMatchStart, sound::kAutoEnd,   sound::kTeleopStart,
                                             sound::kShiftStart, sound::kShiftStart, sound::kShiftStart,
                                             sound::kShiftStart, sound::kEndGame,    sound::kMatchEnd};

}  // namespace

TEST_CASE("match sounds: a full Practice match raises every Table 5-4 cue at its timer value") {
    Match m;
    const double t0 = sdk::usToSeconds(m.s.env.clock.simTimeUs());
    m.practice(20.0, 3.0, 140.0);
    m.idle(0.1);  // the Driver Station ends the finished match

    REQUIRE(m.ids() == kFullMatch);
    // AUTO 0:20, AUTO end 0:00, TELEOP 2:20, shifts 2:10 / 1:45 / 1:20 / 0:55, END GAME 0:30, match end 0:00.
    const double expected[] = {0.0, 20.0, 23.0, 33.0, 58.0, 83.0, 108.0, 133.0, 163.0};
    for (std::size_t i = 0; i < m.heard.size(); ++i) CHECK(std::abs(m.heard[i].timeS - t0 - expected[i]) < 0.03);
}

TEST_CASE("match sounds: with no gap the AUTO end and TELEOP start coincide") {
    Match m;
    m.practice(20.0, 0.0, 140.0);
    m.idle(0.1);
    REQUIRE(m.ids() == kFullMatch);
    CHECK(m.heard[1].timeS == doctest::Approx(m.heard[2].timeS).epsilon(0.001));
}

TEST_CASE("match sounds: stopping a timed match early raises only the Foghorn") {
    SUBCASE("during AUTO") {
        Match m;
        m.timed(RobotMode::Autonomous, true, 20.0, 5.0);
        m.idle(0.1);
        CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart, sound::kMatchStopped});
    }
    SUBCASE("during the gap") {
        Match m;
        m.timed(RobotMode::Autonomous, true, 20.0, 20.0);
        m.timed(RobotMode::Teleop, false, 3.0, 1.0);
        m.idle(0.1);
        CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart, sound::kAutoEnd, sound::kMatchStopped});
    }
    SUBCASE("in the middle of TELEOP") {
        Match m;
        m.timed(RobotMode::Autonomous, true, 20.0, 20.0);
        m.timed(RobotMode::Teleop, false, 3.0, 3.0);
        m.timed(RobotMode::Teleop, true, 140.0, 30.0);
        m.idle(0.1);
        CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart, sound::kAutoEnd, sound::kTeleopStart,
                                                  sound::kShiftStart, sound::kMatchStopped});
    }
    SUBCASE("by an emergency stop") {
        Match m;
        m.timed(RobotMode::Autonomous, true, 20.0, 5.0);
        m.idle(0.1, false, RobotMode::Autonomous, true);
        CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart, sound::kMatchStopped});
    }
}

TEST_CASE("match sounds: a match that runs out is the Buzzer, not the Foghorn, even when the last state lags the clock") {
    Match m;
    m.timed(RobotMode::Autonomous, true, 20.0, 20.0);
    m.timed(RobotMode::Teleop, false, 3.0, 3.0);
    m.timed(RobotMode::Teleop, true, 140.0, 139.4);  // 0.6 s of the clock were never delivered
    m.idle(0.1);
    CHECK(m.ids().back() == sound::kMatchEnd);

    Match early;
    early.timed(RobotMode::Autonomous, true, 20.0, 20.0);
    early.timed(RobotMode::Teleop, false, 3.0, 3.0);
    early.timed(RobotMode::Teleop, true, 140.0, 138.0);  // 2 s left
    early.idle(0.1);
    CHECK(early.ids().back() == sound::kMatchStopped);
}

TEST_CASE("match sounds: a Reset stays silent even while the old timed Driver Station state is still being delivered") {
    Match m;
    m.timed(RobotMode::Autonomous, true, 20.0, 5.0);
    REQUIRE(m.ids() == std::vector<std::string>{sound::kMatchStart});

    m.s.stateOwner->reset(m.s.env.ctx);
    CHECK(m.s.game->sounds().log().events().empty());
    m.timed(RobotMode::Autonomous, true, 15.0, 0.1);  // stale state of the match that was running
    m.timed(RobotMode::Teleop, true, 60.0, 2.0);
    m.idle(0.1);                                        // the Driver Station caught up
    CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart});

    m.practice(20.0, 3.0, 5.0);                         // the next match works again
    CHECK(m.ids().size() > 1);
    CHECK(m.ids()[1] == sound::kMatchStart);
}

TEST_CASE("match sounds: free play and test mode are silent") {
    Match m;
    m.idle(10.0, true, RobotMode::Autonomous);
    m.idle(10.0, true, RobotMode::Teleop);
    m.idle(10.0, true, RobotMode::Test);
    CHECK(m.ids().empty());
}

TEST_CASE("match sounds: a second match raises its cues again and sequences keep increasing") {
    Match m;
    m.timed(RobotMode::Autonomous, true, 20.0, 2.0);
    m.idle(0.1);
    m.timed(RobotMode::Autonomous, true, 20.0, 2.0);
    m.idle(0.1);
    CHECK(m.ids() == std::vector<std::string>{sound::kMatchStart, sound::kMatchStopped, sound::kMatchStart,
                                              sound::kMatchStopped});
    CHECK(m.s.game->sounds().log().lastSequence() == 4);
}

TEST_CASE("match sounds: cues reach the snapshot until a Reset clears them") {
    Match m;
    m.timed(RobotMode::Autonomous, true, 20.0, 1.0);

    sdk::SnapshotBuilder before;
    m.s.stateOwner->publish(before);
    const auto snapshot = before.build();
    REQUIRE(snapshot->soundCues().size() == 1);
    CHECK(snapshot->soundCues()[0].cueId == sound::kMatchStart);
    CHECK(snapshot->soundCues()[0].sequence == 1);

    m.s.stateOwner->reset(m.s.env.ctx);
    sdk::SnapshotBuilder after;
    m.s.stateOwner->publish(after);
    CHECK(after.build()->soundCues().empty());
}

TEST_CASE("match sounds: the declared cues are plugin-owned, unique and every raised cue is declared") {
    g26test::Session s;
    const std::vector<sdk::SoundCueInfo> cues = s.module().contributeSoundCues();
    std::set<std::string> ids;
    for (const sdk::SoundCueInfo& c : cues) {
        CHECK(sdk::ownsPluginId("game_2026", c.id));
        CHECK(ids.insert(c.id).second);
        CHECK_FALSE(c.label.empty());
        CHECK_FALSE(c.asset.empty());
    }
    for (const std::string& raised : kFullMatch) CHECK(ids.count(raised) == 1);
    CHECK(ids.count(sound::kMatchStopped) == 1);
    const auto stopped = std::find_if(cues.begin(), cues.end(), [](const auto& c) { return c.id == sound::kMatchStopped; });
    REQUIRE(stopped != cues.end());
    CHECK(stopped->interruptsOthers);
}

#ifdef WPIEMU_PLUGINS_DIR
TEST_CASE("match sounds: every cue's audio file exists and is a short PCM WAV") {
    g26test::Session s;
    for (const sdk::SoundCueInfo& c : s.module().contributeSoundCues()) {
        INFO(c.asset);
        const std::filesystem::path path = std::filesystem::path(WPIEMU_PLUGINS_DIR) / c.asset;
        std::ifstream file(path, std::ios::binary);
        REQUIRE(file.good());
        const std::vector<unsigned char> b((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        REQUIRE(b.size() > 44);
        CHECK(std::string(b.begin(), b.begin() + 4) == "RIFF");
        CHECK(std::string(b.begin() + 8, b.begin() + 12) == "WAVE");

        auto u32 = [&](std::size_t i) { return b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (static_cast<std::uint32_t>(b[i + 3]) << 24); };
        auto u16 = [&](std::size_t i) { return b[i] | (b[i + 1] << 8); };
        double rate = 0.0, byteRate = 0.0;
        std::uint32_t dataBytes = 0;
        for (std::size_t i = 12; i + 8 <= b.size();) {
            const std::string id(b.begin() + static_cast<std::ptrdiff_t>(i), b.begin() + static_cast<std::ptrdiff_t>(i) + 4);
            const std::uint32_t size = u32(i + 4);
            if (id == "fmt ") {
                CHECK(u16(i + 8) == 1);  // PCM
                rate = u32(i + 12);
                byteRate = u32(i + 16);
            } else if (id == "data") {
                dataBytes = size;
            }
            i += 8 + size + (size & 1);
        }
        REQUIRE(byteRate > 0.0);
        CHECK(rate >= 8000.0);
        const double seconds = dataBytes / byteRate;
        CHECK(seconds > 0.3);
        CHECK(seconds < 10.0);
    }
}
#endif
