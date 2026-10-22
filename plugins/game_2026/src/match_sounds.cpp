#include "wpiemu/plugins/game_2026/match_sounds.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr const char* kAudioDir = "game_2026/assets/audio/";

// Position in the match timeline; Test has none, so it never counts as a forward step.
int ordinal(MatchPhase phase) {
    switch (phase) {
        case MatchPhase::PreMatch: return 0;
        case MatchPhase::Auto: return 1;
        case MatchPhase::AutoEnded: return 2;
        case MatchPhase::Transition: return 3;
        case MatchPhase::Shift1: return 4;
        case MatchPhase::Shift2: return 5;
        case MatchPhase::Shift3: return 6;
        case MatchPhase::Shift4: return 7;
        case MatchPhase::EndGame: return 8;
        case MatchPhase::PostMatch: return 9;
        case MatchPhase::Test: break;
    }
    return -1;
}

sdk::SoundCueInfo cue(const char* id, const char* label, const char* description, const char* file,
                      bool interruptsOthers = false) {
    sdk::SoundCueInfo info;
    info.id = id;
    info.label = label;
    info.description = description;
    info.asset = std::string(kAudioDir) + file;
    info.interruptsOthers = interruptsOthers;
    return info;
}

}  // namespace

std::vector<sdk::SoundCueInfo> soundCueInfos() {
    return {
        cue(sound::kMatchStart, "Match start", "Cavalry Charge: AUTO begins (0:20)", "start.wav"),
        cue(sound::kAutoEnd, "AUTO ends", "Buzzer: AUTO ends (0:00)", "end.wav"),
        cue(sound::kTeleopStart, "TELEOP begins", "3 Bells: TELEOP and the TRANSITION SHIFT begin (2:20)", "resume.wav"),
        cue(sound::kShiftStart, "Alliance shift starts", "POWER UP - Linear Popping: a shift starts (2:10, 1:45, 1:20, 0:55)",
            "shift_change.wav"),
        cue(sound::kEndGame, "END GAME begins", "Steam Whistle: END GAME begins (0:30)", "warning.wav"),
        cue(sound::kMatchEnd, "Match end", "Buzzer: the match ends (0:00)", "end.wav"),
        cue(sound::kMatchStopped, "Match stopped", "Foghorn: the match was stopped before its end", "abort.wav", true),
    };
}

void MatchSounds::reset() {
    log_.clear();
    armed_ = false;
    endMatch();
}

void MatchSounds::endMatch() {
    inMatch_ = false;
    started_ = false;
    autoEnded_ = false;
    ended_ = false;
    teleopStarted_ = false;
    phase_ = MatchPhase::PreMatch;
    remainingS_ = 0.0;
}

void MatchSounds::remember(const MatchStatus& status) {
    phase_ = status.phase;
    teleopStarted_ = status.teleopStarted;
    remainingS_ = status.periodRemainingS;
}

void MatchSounds::update(const MatchStatus& status, const sdk::proto::DsState& ds, bool timed, sdk::SimTimeUs nowUs) {
    const bool autoRunning = ds.enabled && !ds.eStop && ds.mode == sdk::proto::RobotMode::Autonomous;

    if (!timed) {
        if (!armed_) {
            armed_ = true;
        } else if (inMatch_ && started_ && !ended_) {
            const bool endOfMatch = (phase_ == MatchPhase::EndGame || phase_ == MatchPhase::PostMatch) &&
                                    remainingS_ <= MatchSounds::kNaturalEndToleranceS;
            log_.raise(endOfMatch ? sound::kMatchEnd : sound::kMatchStopped, nowUs);
        }
        endMatch();
        return;
    }
    if (!armed_) return;  // the Driver Station state of a match that was running before the Reset

    if (!inMatch_) {
        inMatch_ = true;
        if (autoRunning) {
            log_.raise(sound::kMatchStart, nowUs);
            started_ = true;
        }
        remember(status);
        return;
    }
    if (!started_) {
        remember(status);
        return;
    }

    if (!autoEnded_ && !autoRunning) {
        log_.raise(sound::kAutoEnd, nowUs);
        autoEnded_ = true;
    }
    if (status.teleopStarted && !teleopStarted_) {
        log_.raise(sound::kTeleopStart, nowUs);
    } else if (ordinal(status.phase) > ordinal(phase_)) {
        if (isShift(status.phase)) {
            log_.raise(sound::kShiftStart, nowUs);
        } else if (status.phase == MatchPhase::EndGame) {
            log_.raise(sound::kEndGame, nowUs);
        } else if (status.phase == MatchPhase::PostMatch && !ended_) {
            log_.raise(sound::kMatchEnd, nowUs);
            ended_ = true;
        }
    }
    remember(status);
}

}  // namespace wpiemu::plugins::game_2026
