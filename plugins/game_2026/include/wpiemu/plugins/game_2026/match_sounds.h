#pragma once

// FMS audio cues of REBUILT (manual 5.12, Table 5-4), raised from the match timeline the plugin already follows.
// The cues are observed transitions of the Driver Station / match clock state, never first observations:
// a Reset can leave the previous (timed) Driver Station state around for a few ticks, and that must stay silent.

#include <vector>

#include "wpiemu/plugins/game_2026/match_clock.h"
#include "wpiemu/sdk/game/sound_cue.h"
#include "wpiemu/sdk/ipc/driver_station.h"

namespace wpiemu::plugins::game_2026 {

namespace sound {
inline constexpr const char* kMatchStart = "game_2026.sound.match_start";      // Cavalry Charge, AUTO begins (0:20)
inline constexpr const char* kAutoEnd = "game_2026.sound.auto_end";            // Buzzer, AUTO ends (0:00)
inline constexpr const char* kTeleopStart = "game_2026.sound.teleop_start";    // 3 Bells, TELEOP & TRANSITION (2:20)
inline constexpr const char* kShiftStart = "game_2026.sound.shift_start";      // POWER UP - Linear Popping (2:10 ... 0:55)
inline constexpr const char* kEndGame = "game_2026.sound.end_game";           // Steam Whistle (0:30)
inline constexpr const char* kMatchEnd = "game_2026.sound.match_end";          // Buzzer (0:00)
inline constexpr const char* kMatchStopped = "game_2026.sound.match_stopped";  // Foghorn, the match was cut short
}  // namespace sound

// The cues and their audio files (relative to plugins/, embedded by the app).
std::vector<sdk::SoundCueInfo> soundCueInfos();

class MatchSounds {
public:
    // A timed match that ends with at most this much time left (the Driver Station state lags the clock by a few
    // ticks, more at high time scales) ended naturally; any earlier end was stopped by the operator.
    static constexpr double kNaturalEndToleranceS = 1.0;

    // New session or Reset: forget the match and stay silent until an untimed Driver Station state is seen.
    void reset();
    // Once per tick after the match clock. `timed`: the Driver Station runs a timed match (Practice).
    void update(const MatchStatus& status, const sdk::proto::DsState& ds, bool timed, sdk::SimTimeUs nowUs);
    void publish(sdk::SnapshotBuilder& out) const { log_.publish(out); }
    const sdk::SoundCueLog& log() const { return log_; }

private:
    void endMatch();
    void remember(const MatchStatus& status);

    sdk::SoundCueLog log_;
    bool armed_ = false;
    bool inMatch_ = false;
    bool started_ = false;     // the AUTO start was observed
    bool autoEnded_ = false;
    bool ended_ = false;       // match_end was raised
    bool teleopStarted_ = false;
    MatchPhase phase_ = MatchPhase::PreMatch;
    double remainingS_ = 0.0;
};

}  // namespace wpiemu::plugins::game_2026
