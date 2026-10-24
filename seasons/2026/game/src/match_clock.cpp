#include "wpiemu/plugins/game_2026/match_clock.h"

#include <algorithm>

namespace wpiemu::plugins::game_2026 {

const char* phaseName(MatchPhase phase) {
    switch (phase) {
        case MatchPhase::PreMatch: return "PRE-MATCH";
        case MatchPhase::Auto: return "AUTO";
        case MatchPhase::AutoEnded: return "AUTO ENDED";
        case MatchPhase::Transition: return "TRANSITION";
        case MatchPhase::Shift1: return "SHIFT 1";
        case MatchPhase::Shift2: return "SHIFT 2";
        case MatchPhase::Shift3: return "SHIFT 3";
        case MatchPhase::Shift4: return "SHIFT 4";
        case MatchPhase::EndGame: return "END GAME";
        case MatchPhase::PostMatch: return "POST-MATCH";
        case MatchPhase::Test: return "TEST";
    }
    return "?";
}

bool isShift(MatchPhase phase) {
    return phase == MatchPhase::Shift1 || phase == MatchPhase::Shift2 || phase == MatchPhase::Shift3 ||
           phase == MatchPhase::Shift4;
}

MatchStatus teleopStatusAt(double elapsedS) {
    MatchStatus s;
    s.teleopStarted = true;
    s.autoPeriod = false;
    const double e = std::clamp(elapsedS, 0.0, kTeleopDurationS);
    s.periodElapsedS = e;
    s.periodRemainingS = kTeleopDurationS - e;

    struct Segment {
        MatchPhase phase;
        double length;
    };
    const Segment segments[] = {{MatchPhase::Transition, kTransitionDurationS}, {MatchPhase::Shift1, kShiftDurationS},
                                {MatchPhase::Shift2, kShiftDurationS},         {MatchPhase::Shift3, kShiftDurationS},
                                {MatchPhase::Shift4, kShiftDurationS},         {MatchPhase::EndGame, kEndGameDurationS}};
    double start = 0.0;
    for (const Segment& seg : segments) {
        if (e < start + seg.length) {
            s.phase = seg.phase;
            s.phaseRemainingS = start + seg.length - e;
            return s;
        }
        start += seg.length;
    }
    s.phase = MatchPhase::PostMatch;
    s.phaseRemainingS = 0.0;
    return s;
}

void MatchClock::reset() { *this = MatchClock{}; }

MatchStatus MatchClock::update(const sdk::proto::DsState& ds, double dtSeconds) {
    const bool enabled = ds.enabled && !ds.eStop;
    const bool useDsTime = ds.matchTimeS >= 0.0;

    if (enabled) {
        switch (ds.mode) {
            case sdk::proto::RobotMode::Autonomous:
                autoSeen_ = true;
                lastWasAuto_ = true;
                autoElapsedS_ = useDsTime ? std::clamp(kAutoDurationS - ds.matchTimeS, 0.0, kAutoDurationS)
                                          : std::min(kAutoDurationS, autoElapsedS_ + dtSeconds);
                break;
            case sdk::proto::RobotMode::Teleop:
                teleopSeen_ = true;
                lastWasAuto_ = false;
                teleopElapsedS_ = useDsTime ? std::clamp(kTeleopDurationS - ds.matchTimeS, 0.0, kTeleopDurationS)
                                            : std::min(kTeleopDurationS, teleopElapsedS_ + dtSeconds);
                break;
            case sdk::proto::RobotMode::Test:
                break;
        }
    }

    MatchStatus s;
    if (enabled && ds.mode == sdk::proto::RobotMode::Test) {
        s.phase = MatchPhase::Test;
        s.running = true;
    } else if (teleopSeen_) {
        s = teleopStatusAt(teleopElapsedS_);
        s.running = enabled && ds.mode == sdk::proto::RobotMode::Teleop && teleopElapsedS_ < kTeleopDurationS;
    } else if (autoSeen_) {
        s.autoPeriod = true;
        s.periodElapsedS = autoElapsedS_;
        s.periodRemainingS = kAutoDurationS - autoElapsedS_;
        s.phaseRemainingS = s.periodRemainingS;
        s.phase = autoElapsedS_ >= kAutoDurationS ? MatchPhase::AutoEnded : MatchPhase::Auto;
        s.running = enabled && ds.mode == sdk::proto::RobotMode::Autonomous && autoElapsedS_ < kAutoDurationS;
    }
    s.autoPeriod = lastWasAuto_ && !teleopSeen_;
    s.teleopStarted = teleopSeen_;
    status_ = s;
    return s;
}

}  // namespace wpiemu::plugins::game_2026
