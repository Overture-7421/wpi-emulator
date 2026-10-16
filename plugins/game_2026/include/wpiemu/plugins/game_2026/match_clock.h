#pragma once

// Match timeline of REBUILT (manual 6.4, Table 6-2) driven by the SDK driver-station state.
//
//   AUTO 20 s | TELEOP 140 s = TRANSITION 10 | SHIFT 1..4 25 each | END GAME 30
//
// Time source: DsState::matchTimeS when >= 0 is taken as WPILib's convention (seconds REMAINING in
// the current period, counting down); otherwise the clock integrates the sim step while the robot is
// enabled in that mode. Test mode is a practice mode without a match structure.

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::plugins::game_2026 {

inline constexpr double kAutoDurationS = 20.0;
inline constexpr double kTeleopDurationS = 140.0;
inline constexpr double kTransitionDurationS = 10.0;
inline constexpr double kShiftDurationS = 25.0;
inline constexpr double kEndGameDurationS = 30.0;
inline constexpr double kScoringGraceS = 3.0;  // manual 6.5: fuel keeps being assessed this long after a period/hub ends

enum class MatchPhase {
    PreMatch,   // nothing has been enabled yet
    Auto,
    AutoEnded,  // AUTO elapsed (or the robot was disabled after it) and TELEOP has not started
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    EndGame,
    PostMatch,  // TELEOP elapsed
    Test,       // practice: enabled in test mode
};

const char* phaseName(MatchPhase phase);
bool isShift(MatchPhase phase);

struct MatchStatus {
    MatchPhase phase = MatchPhase::PreMatch;
    bool running = false;           // enabled and inside a period (hub scoring can be open)
    bool autoPeriod = false;        // the most recently enabled period was AUTO (attribution of scored fuel)
    bool teleopStarted = false;     // TELEOP has been enabled at least once in this match
    double periodElapsedS = 0.0;    // in the current period (AUTO or TELEOP)
    double periodRemainingS = 0.0;
    double phaseRemainingS = 0.0;   // until the next phase boundary
};

// Phase of TELEOP after `elapsedS` seconds; `phaseRemainingS` is filled as well.
MatchStatus teleopStatusAt(double elapsedS);

class MatchClock {
public:
    void reset();
    // Call once per sim tick with the latest DS state and the fixed step.
    MatchStatus update(const sdk::proto::DsState& ds, double dtSeconds);
    const MatchStatus& status() const { return status_; }

private:
    double autoElapsedS_ = 0.0;
    double teleopElapsedS_ = 0.0;
    bool autoSeen_ = false;
    bool teleopSeen_ = false;
    bool lastWasAuto_ = false;
    MatchStatus status_;
};

}  // namespace wpiemu::plugins::game_2026
