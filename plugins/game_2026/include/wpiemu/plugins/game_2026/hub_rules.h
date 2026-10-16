#pragma once

// Hub active/inactive rules (manual 6.4.1, Tables 5-3 and 6-3) and the per-alliance fuel score.

#include <array>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/match_clock.h"
#include "wpiemu/plugins/game_2026/types.h"
#include "wpiemu/sdk/core/rng.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::plugins::game_2026 {

// Pure schedule: is `a`'s hub active in `phase` given which alliance's hub is inactive in shift 1?
bool hubActiveInPhase(Alliance a, MatchPhase phase, Alliance firstInactive);

struct AllianceScore {
    int autoFuel = 0;      // scored in an active (or grace-period) hub during AUTO
    int teleopFuel = 0;
    int inactiveFuel = 0;  // entered a hub that was not scoring: no points
    int points() const { return autoFuel + teleopFuel; }  // 1 point per fuel in AUTO and TELEOP (Table 6-4)
    int fuelProcessed() const { return autoFuel + teleopFuel + inactiveFuel; }
};

class HubRules {
public:
    HubRules() = default;
    HubRules(bool enforceShifts, FirstInactiveMode mode) : enforceShifts_(enforceShifts), mode_(mode) {}

    void reset();
    // One call per sim tick. Latches the first-inactive alliance when TELEOP starts.
    void update(const MatchStatus& status, const sdk::proto::DsState& ds, double dtSeconds, sdk::Rng& rng);

    // Hub is in its active state (drives the lights / HUD badge).
    bool active(Alliance a) const { return active_[index(a)]; }
    // Fuel entering now still counts: active, or deactivated less than kScoringGraceS ago.
    bool scoringOpen(Alliance a) const { return active_[index(a)] || graceLeftS_[index(a)] > 0.0; }
    bool firstInactiveKnown() const { return firstInactiveKnown_; }
    Alliance firstInactive() const { return firstInactive_; }

    // Records one fuel through `a`'s hub; returns true when it earned a point.
    bool recordFuel(Alliance a, const MatchStatus& status);
    const AllianceScore& score(Alliance a) const { return scores_[index(a)]; }

private:
    bool enforceShifts_ = true;
    FirstInactiveMode mode_ = FirstInactiveMode::FromAutoResult;
    bool firstInactiveKnown_ = false;
    Alliance firstInactive_ = Alliance::Red;
    std::array<bool, 2> active_{false, false};
    std::array<double, 2> graceLeftS_{0.0, 0.0};
    std::array<AllianceScore, 2> scores_{};
};

}  // namespace wpiemu::plugins::game_2026
