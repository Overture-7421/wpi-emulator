#include "wpiemu/plugins/game_2026/hub_rules.h"

#include <algorithm>

namespace wpiemu::plugins::game_2026 {

bool hubActiveInPhase(Alliance a, MatchPhase phase, Alliance firstInactive) {
    switch (phase) {
        case MatchPhase::Auto:
        case MatchPhase::Transition:
        case MatchPhase::EndGame:
        case MatchPhase::Test:
            return true;
        case MatchPhase::Shift1:
        case MatchPhase::Shift3:
            return a != firstInactive;
        case MatchPhase::Shift2:
        case MatchPhase::Shift4:
            return a == firstInactive;
        case MatchPhase::PreMatch:
        case MatchPhase::AutoEnded:
        case MatchPhase::PostMatch:
            return false;
    }
    return false;
}

void HubRules::reset() {
    firstInactiveKnown_ = false;
    firstInactive_ = Alliance::Red;
    active_ = {false, false};
    graceLeftS_ = {0.0, 0.0};
    scores_ = {};
}

void HubRules::update(const MatchStatus& status, const sdk::proto::DsState& ds, double dtSeconds, sdk::Rng& rng) {
    if (status.teleopStarted && !firstInactiveKnown_) {
        // Priority: explicit config, then the FMS game-data character, then the AUTO result (tie: random).
        if (mode_ == FirstInactiveMode::Red) firstInactive_ = Alliance::Red;
        else if (mode_ == FirstInactiveMode::Blue) firstInactive_ = Alliance::Blue;
        else if (!ds.gameSpecificMessage.empty() && (ds.gameSpecificMessage[0] == 'R' || ds.gameSpecificMessage[0] == 'B'))
            firstInactive_ = ds.gameSpecificMessage[0] == 'R' ? Alliance::Red : Alliance::Blue;
        else {
            const int red = scores_[index(Alliance::Red)].autoFuel, blue = scores_[index(Alliance::Blue)].autoFuel;
            if (red != blue) firstInactive_ = red > blue ? Alliance::Red : Alliance::Blue;
            else firstInactive_ = (rng.nextU64() & 1u) != 0 ? Alliance::Red : Alliance::Blue;
        }
        firstInactiveKnown_ = true;
    }

    for (const Alliance a : kAlliances) {
        bool on = false;
        if (status.running) {
            on = enforceShifts_ ? hubActiveInPhase(a, status.phase, firstInactive_) : true;
        }
        active_[index(a)] = on;
        double& grace = graceLeftS_[index(a)];
        grace = on ? kScoringGraceS : std::max(0.0, grace - dtSeconds);
    }
}

bool HubRules::recordFuel(Alliance a, const MatchStatus& status) {
    AllianceScore& s = scores_[index(a)];
    if (!scoringOpen(a)) {
        ++s.inactiveFuel;
        return false;
    }
    if (status.autoPeriod) ++s.autoFuel;
    else ++s.teleopFuel;
    return true;
}

}  // namespace wpiemu::plugins::game_2026
