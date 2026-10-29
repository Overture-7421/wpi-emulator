# 17. The vault and power-ups

## 17.1 Objectives

After this chapter the reader can model a capacity-limited vault, distinguish
a queued power from an active power, and integrate timed effects without
scoring past their expiration. Chapter 16 is the prerequisite. The vault and
power modules address F-13 through F-16.

## 17.2 Problem statement

An exchange cube may enter one of three vault columns. Depositing it scores
points; pressing a button creates a separate effect. Conflating those actions
would either consume a cube twice or permit a repeated button press to score
again.

## 17.3 Background and definitions

Each column holds at most three cubes, worth five points each. Cubes remain
in the vault after activation, and later deposits can still score. Force and
Boost run for ten seconds. Level 1 affects the home switch, level 2 the scale,
and level 3 both [1, section 4.3 and Table 4-2].

Force supplies effective ownership. Boost doubles points on a target already
owned. Each type may be used once per alliance. Only one Force or Boost is
active across both alliances: an opponent request queues; an alliance's
other request while its own power is active is ignored.

Levitate requires three cubes and creates one endgame climb credit. It is not
a ten-second ownership effect. Chapter 19 combines that credit with physical
and virtual robot finishes.

## 17.4 Design

One large state machine could combine inventory, buttons, scoring and timers.
Separate vault and power objects instead make their inputs explicit. The
vault knows deposited counts. The power object reads a column's level when
accepting a press and retains that level even if more cubes arrive.

**Invariant 17.1 (one-shot activation).** An accepted or queued press consumes
that alliance's use of the power, not its cubes. Repeated presses cannot restart
the timer or change the latched level.

A scoring interval that crosses expiration must be split. Applying the old
effect to the whole interval would over-award points; advancing the power
first would under-award them.

## 17.5 Implementation

### 17.5.1 Pure vault

The vault accepts typed deposits only in running teleoperation. Capacity and
point changes occur together. Physical custody remains the adapter's concern.

Listing 17.1. Vault counts and deposit interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\vault.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\vault.h -->
```cpp
#pragma once

#include "rules_types.h"

namespace wpiemu::plugins::game_2027 {

struct VaultSnapshot {
    std::array<std::array<int, 3>, 2> cubes{};
    std::array<int, 2> points{};
};

class Vault {
public:
    // [GM section4.2/Table4-1], [GM section4.3/Table4-2].
    // Capacity three per column: GM section3.6.3, pp29-31.
    RulesStatus deposit(std::size_t alliance, Power column, const RulesContext& context,
                        int count = 1) noexcept;
    [[nodiscard]] const VaultSnapshot& snapshot() const noexcept { return state_; }
    void reset() noexcept { state_ = {}; }

private:
    VaultSnapshot state_{};
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.2. Capacity and deposit scoring. <code>seasons\2027\game\src\vault.cpp</code>.

<!-- book-file: seasons\2027\game\src\vault.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/vault.h"

namespace wpiemu::plugins::game_2027 {

RulesStatus Vault::deposit(std::size_t alliance, Power column, const RulesContext& context,
                           int count) noexcept {
    if (alliance >= 2) return RulesStatus::InvalidAlliance;
    if (!validPower(column)) return RulesStatus::InvalidPower;
    if (count < 1 || count > 3) return RulesStatus::InvalidCount;
    const auto status = teleopStatus(context);
    if (status != RulesStatus::Accepted) return status;
    auto& cubes = state_.cubes[alliance][powerIndex(column)];
    if (count > 3 - cubes) return RulesStatus::ColumnFull;
    cubes += count;
    state_.points[alliance] += 5 * count;
    return RulesStatus::Accepted;
}

}  // namespace wpiemu::plugins::game_2027
```

### 17.5.2 Power state machine

The snapshot contains used flags, Levitate credits, and optional active and
queued powers. The timer promotes the queue at the exact expiration boundary.
Disabled intervals freeze it.

Listing 17.3. Power states and latched levels. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\power_ups.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\power_ups.h -->
```cpp
#pragma once

#include <optional>

#include "vault.h"

namespace wpiemu::plugins::game_2027 {

struct ActivePower {
    std::size_t alliance{};
    Power power{Power::Force};
    int level{};
    RulesTimeUs remainingUs{10 * kRulesSecondUs};
};
struct PowerSnapshot {
    std::array<std::array<bool, 3>, 2> used{};
    std::array<bool, 2> levitate{};
    std::optional<ActivePower> active;
    std::optional<ActivePower> queued;
};

class PowerUps {
public:
    // [GM section4.3/Table4-2]. Accepted/Queued consumes the button once, not
    // cubes.
    RulesStatus press(std::size_t alliance, Power power, const Vault& vault,
                      const RulesContext& context) noexcept;
    // Timers freeze when disabled/eStopped. Integrator splits scoring at
    // remainingUs BEFORE advancing this timer; exact expiration immediately
    // promotes the queue.
    RulesStatus advance(RulesTimeUs elapsedUs, const RulesContext& context) noexcept;
    [[nodiscard]] OwnershipEffects effects() const noexcept;
    [[nodiscard]] const PowerSnapshot& snapshot() const noexcept { return state_; }
    void reset() noexcept { state_ = {}; }

private:
    PowerSnapshot state_{};
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.4. Button acceptance, queueing and expiration. <code>seasons\2027\game\src\power_ups.cpp</code>.

<!-- book-file: seasons\2027\game\src\power_ups.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/power_ups.h"

#include <algorithm>

namespace wpiemu::plugins::game_2027 {

RulesStatus PowerUps::press(std::size_t alliance, Power power, const Vault& vault,
                            const RulesContext& context) noexcept {
    if (alliance >= 2) return RulesStatus::InvalidAlliance;
    if (!validPower(power)) return RulesStatus::InvalidPower;
    const auto status = teleopStatus(context);
    if (status != RulesStatus::Accepted) return status;
    const auto index = powerIndex(power);
    if (state_.used[alliance][index]) return RulesStatus::AlreadyUsed;
    if (power != Power::Levitate) {
        if (state_.active && state_.active->alliance == alliance)
            return RulesStatus::OwnPowerActive;
        if (state_.queued && state_.queued->alliance == alliance)
            return RulesStatus::OwnPowerQueued;
    }
    const auto level = vault.snapshot().cubes[alliance][index];
    if (level < (power == Power::Levitate ? 3 : 1)) return RulesStatus::InsufficientCubes;
    state_.used[alliance][index] = true;
    if (power == Power::Levitate) {
        state_.levitate[alliance] = true;
        return RulesStatus::Accepted;
    }
    const ActivePower accepted{alliance, power, level};
    if (state_.active) {
        state_.queued = accepted;
        return RulesStatus::Queued;
    }
    state_.active = accepted;
    return RulesStatus::Accepted;
}

RulesStatus PowerUps::advance(RulesTimeUs elapsedUs, const RulesContext& context) noexcept {
    if (elapsedUs < 0) return RulesStatus::InvalidDuration;
    const auto status = teleopStatus(context);
    if (status != RulesStatus::Accepted) return status;
    while (state_.active && elapsedUs > 0) {
        const auto step = std::min(elapsedUs, state_.active->remainingUs);
        state_.active->remainingUs -= step;
        elapsedUs -= step;
        if (state_.active->remainingUs == 0) {
            state_.active = state_.queued;
            state_.queued.reset();
        }
    }
    return RulesStatus::Accepted;
}

OwnershipEffects PowerUps::effects() const noexcept {
    OwnershipEffects result;
    if (!state_.active) return result;
    const auto& active = *state_.active;
    auto& targets = active.power == Power::Force ? result.force : result.boost;
    targets[active.alliance][0] = active.level == 1 || active.level == 3;
    targets[active.alliance][1] = active.level == 2 || active.level == 3;
    return result;
}

}  // namespace wpiemu::plugins::game_2027
```

### 17.5.3 Physical adapter and configuration

At a human-action opportunity, the adapter requires an exchange identity before
depositing. The sequence policy fills Force, Boost and Levitate in that order.
The six button times are teleoperation elapsed seconds; -1 disables a press.
A rejected press is logged with its typed status.

Listing 17.5. Custody-backed deposits and scheduled presses. <code>seasons\2027\game\src\game_vault.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_vault.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

void State::updateVault(s::SimContext& ctx, bool actionDue) {
    const RulesContext rules{RulesPhase::Teleop, true, false};
    if (actionDue && config_.exchangeMode == "vault") {
        for (std::size_t a = 0; a < 2; ++a) {
            if (pool_.count(CubeLocation::Exchange, a) == 0) continue;
            for (const auto power : {Power::Force, Power::Boost, Power::Levitate}) {
                const auto p = powerIndex(power);
                const auto name = power == Power::Force   ? "force"
                                  : power == Power::Boost ? "boost"
                                                          : "levitate";
                if (config_.vaultColumn != "sequence" && config_.vaultColumn != name) continue;
                if (vault_.snapshot().cubes[a][p] == 3) continue;
                const auto status = vault_.deposit(a, power, rules);
                if (status != RulesStatus::Accepted || !pool_.deposit(a))
                    ctx.logger().error(kId, "Vault custody transition rejected");
                break;
            }
        }
    }
    const std::array<std::array<double, 3>, 2> times{
        {{config_.blueForceS, config_.blueBoostS, config_.blueLevitateS},
         {config_.redForceS, config_.redBoostS, config_.redLevitateS}}};
    for (std::size_t a = 0; a < 2; ++a) {
        for (const auto power : {Power::Force, Power::Boost, Power::Levitate}) {
            const auto p = powerIndex(power);
            if (times[a][p] < 0.0 || buttonPressed_[a][p] ||
                teleopUs_ < static_cast<RulesTimeUs>(std::llround(times[a][p] * kRulesSecondUs)))
                continue;
            buttonPressed_[a][p] = true;
            const auto status = powers_.press(a, power, vault_, rules);
            if (status != RulesStatus::Accepted && status != RulesStatus::Queued)
                ctx.logger().warn(kId, "Power-up press rejected: alliance=" + std::to_string(a) +
                                           " power=" + std::to_string(p) +
                                           " status=" + std::to_string(static_cast<int>(status)));
        }
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.6. Vault and scheduled-button settings. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\config.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\config.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
struct GameConfig {
    std::string assignment = "LRL";
    bool preload = true;
    bool timedFeed = false;
    double humanPeriodS = 2.0;
    std::string exchangeMode = "hold";
    std::string vaultColumn = "sequence";
    double blueForceS = -1.0, blueBoostS = -1.0, blueLevitateS = -1.0;
    double redForceS = -1.0, redBoostS = -1.0, redLevitateS = -1.0;
};

bool validAssignment(const std::string& value);
s::PluginConfigSchema configSchema();
GameConfig parseConfig(const s::Value& data);

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.7. Vault choices and button-time bindings. <code>seasons\2027\game\src\config.cpp</code>.

<!-- book-file: seasons\2027\game\src\config.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/config.h"

#include "wpiemu/sdk/config/config_bindings.h"

namespace wpiemu::plugins::game_2027 {
namespace {

s::ConfigBindings<GameConfig> bindings() {
    s::ConfigBindings<GameConfig> b;
    b.add(s::props::choice("assignment", "Field assignment", "Scenario", "LRL",
                           {{"LLL", "LLL"}, {"LRL", "LRL"}, {"RLR", "RLR"}, {"RRR", "RRR"}}),
          &GameConfig::assignment);
    b.add(s::props::flag("preload", "One preloaded cube", "Scenario", true), &GameConfig::preload);
    b.add(s::props::flag("timedFeed", "Timed portal feeding", "Human player", false),
          &GameConfig::timedFeed);
    b.add(s::props::real("humanPeriodS", "Action period", "Human player", 2.0, 0.1, 60.0,
                         s::PropertyUnit::Seconds),
          &GameConfig::humanPeriodS);
    b.add(s::props::choice("exchangeMode", "Exchange destination", "Human player", "hold",
                           {{"hold", "Retain"}, {"return", "Upper return"}, {"vault", "Vault"}}),
          &GameConfig::exchangeMode);
    b.add(s::props::choice("vaultColumn", "Vault column", "Human player", "sequence",
                           {{"sequence", "Force, Boost, Levitate"},
                            {"force", "Force"},
                            {"boost", "Boost"},
                            {"levitate", "Levitate"}}),
          &GameConfig::vaultColumn);
    using TimedButton = std::pair<const char*, double GameConfig::*>;
    const std::array<TimedButton, 6> buttons{{{"blueForceS", &GameConfig::blueForceS},
                                              {"blueBoostS", &GameConfig::blueBoostS},
                                              {"blueLevitateS", &GameConfig::blueLevitateS},
                                              {"redForceS", &GameConfig::redForceS},
                                              {"redBoostS", &GameConfig::redBoostS},
                                              {"redLevitateS", &GameConfig::redLevitateS}}};
    for (const auto& [key, member] : buttons)
        b.add(s::props::real(key, key, "Power-up buttons", -1.0, -1.0, 135.0,
                             s::PropertyUnit::Seconds,
                             "Teleop elapsed seconds; -1 disables the press"),
              member);
    return b;
}

}  // namespace

bool validAssignment(const std::string& value) {
    return value == "LLL" || value == "LRL" || value == "RLR" || value == "RRR";
}

s::PluginConfigSchema configSchema() {
    s::PluginConfigSchema schema;
    schema.schemaVersion = 1;
    schema.properties = bindings().schema();
    return schema;
}

GameConfig parseConfig(const s::Value& data) {
    return bindings().decode(data);
}

}  // namespace wpiemu::plugins::game_2027
```

### 17.5.4 Integration and publication

The state owns the vault and powers, and reset clears both. The scoring adapter
scores a chunk with the current effect before advancing its timer. Publication
exposes column counts, points, active state and remaining time for inspection.

Listing 17.8. State declaration with vault and power ownership. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_layout.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/cube_zones.h"
#include "wpiemu/plugins/game_2027/human_player.h"
#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/plate.h"
#include "wpiemu/plugins/game_2027/plates.h"
#include "wpiemu/plugins/game_2027/power_ups.h"

namespace wpiemu::plugins::game_2027 {
class State final : public s::IGameState {
public:
    State(s::SimContext& ctx, GameConfig config);

    s::Status build();

    std::string_view pluginId() const override;

    void reset(s::SimContext& ctx) override;

    void step(s::SimContext& ctx, double dt) override;

    void publish(s::SnapshotBuilder& out) const override;

    std::optional<std::string> gameSpecificMessage() const override;

    std::optional<double> gameValue(std::string_view id) const override;

private:
    PlateOwnership observedOwnership() const;
    void score(s::SimContext& ctx, double dt, bool autonomous);
    void updateVault(s::SimContext& ctx, bool actionDue);
    void publishRules(s::SnapshotBuilder& out) const;
    s::IPhysicsWorld& world_;
    CubePool pool_;
    Plates plates_;
    CubeZones zones_;
    OwnershipScoring scoring_;
    HumanPlayer human_;
    Vault vault_;
    PowerUps powers_;
    std::array<std::array<bool, 3>, 2> buttonPressed_{};
    RulesTimeUs teleopUs_ = 0;
    GameConfig config_;
    bool autoSeen_ = false;
    std::string assignment_;
    bool preload_;
    std::array<double, 2> scores_{};
    std::array<bool, 2> autoRuns_{};
    double autoSeconds_ = 0.0;
    double teleopSeconds_ = 0.0;
    double endgameSeconds_ = 0.0;
    double remainingS_ = -1.0;
    std::string phase_ = "Disabled";
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.9. Reset of deposits, timers and button history. <code>seasons\2027\game\src\game_reset.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_reset.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::reset(s::SimContext& ctx) {
    scores_ = {};
    scoring_.reset();
    human_.reset();
    vault_.reset();
    powers_.reset();
    buttonPressed_ = {};
    teleopUs_ = 0;
    autoSeen_ = false;
    autoRuns_ = {};
    const auto alliance = s::proto::allianceOf(ctx.driverStation().station);
    pool_.reset(preload_, alliance == s::Alliance::Red ? 1 : 0);
    plates_.reset();
    autoSeconds_ = teleopSeconds_ = endgameSeconds_ = 0.0;
    zones_.reset();
    phase_ = "Disabled";
    remainingS_ = -1.0;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.10. Scoring split at power expiration. <code>seasons\2027\game\src\game_scoring.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_scoring.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

PlateOwnership State::observedOwnership() const {
    PlateOwnership result;
    for (std::size_t i = 0; i < 6; ++i) {
        const auto pair = i / 2;
        const auto owner =
            static_cast<std::size_t>(plateOwner(assignment_, pair, static_cast<int>(i % 2)));
        const auto surface = plates_.center(i).z + kPlateHalfM.z;
        if (pair < 2 && owner == pair)
            result.switchDown[owner] = surface <= 0.1524f;              // [GM 4.2] 6 in.
        if (pair == 2 && surface <= 1.4224f) result.scaleDown = owner;  // [GM 4.2] 56 in.
    }
    return result;
}

void State::score(s::SimContext& ctx, double dt, bool autonomous) {
    const RulesContext rules{autonomous ? RulesPhase::Auto : RulesPhase::Teleop, true, false};
    const auto us = static_cast<RulesTimeUs>(std::llround(dt * kRulesSecondUs));
    auto remaining = us;
    while (remaining > 0) {
        const auto& active = powers_.snapshot().active;
        const auto chunk =
            !autonomous && active ? std::min(remaining, active->remainingUs) : remaining;
        const auto status = scoring_.advance(chunk, rules, observedOwnership(), powers_.effects());
        if (status != RulesStatus::Accepted) ctx.logger().error(kId, "Ownership interval rejected");
        if (!autonomous) {
            const auto timer = powers_.advance(chunk, rules);
            if (timer != RulesStatus::Accepted)
                ctx.logger().error(kId, "Power-up interval rejected");
        }
        remaining -= chunk;
    }
    if (!autonomous) teleopUs_ += us;
    if (autonomous && ctx.chassisBody().valid()) {
        const auto side =
            s::proto::allianceOf(ctx.driverStation().station) == s::Alliance::Red ? 1U : 0U;
        const auto pose = world_.pose(ctx.chassisBody());
        const auto half = ctx.config().chassis.sizeM * 0.5f;
        float leading = side == 0 ? -1e6f : 1e6f;
        for (const float x : {-half.x, half.x})
            for (const float y : {-half.y, half.y})
                for (const float z : {-half.z, half.z}) {
                    const auto p = pose.transformPoint({x, y, z});
                    leading = side == 0 ? std::max(leading, p.x) : std::min(leading, p.x);
                }
        const bool crossed = side == 0 ? leading >= kAutoLineM : leading <= kLengthM - kAutoLineM;
        if (crossed && !autoRuns_[side]) {
            const auto award = scoring_.awardAutoRun(side, 0, rules);
            if (award == RulesStatus::Accepted)
                autoRuns_[side] = true;
            else
                ctx.logger().error(kId, "Auto-run award rejected");
        }
    }
    for (std::size_t a = 0; a < 2; ++a)
        scores_[a] = static_cast<double>(scoring_.snapshot().microPoints[a] / kRulesSecondUs +
                                         vault_.snapshot().points[a]);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.11. Human actions before ownership integration. <code>seasons\2027\game\src\game_tick.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_tick.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"
#include "wpiemu/plugins/game_2027/match_clock.h"

namespace wpiemu::plugins::game_2027 {
void State::step(s::SimContext& ctx, double dt) {
    const auto& ds = ctx.driverStation();
    remainingS_ = ds.matchTimeS;
    const auto phase = phaseOf(ds);
    const bool running = phase != MatchPhase::Disabled;
    const bool autonomous = phase == MatchPhase::Autonomous;
    const bool endgame = phase == MatchPhase::Endgame;
    const bool teleop = phase == MatchPhase::Teleop || endgame;
    phase_ = phaseName(phase);
    if (autoSeen_ && !autonomous && !scoring_.snapshot().autoEndCaptured) scoring_.captureAutoEnd();
    autoSeen_ |= autonomous;
    if (running) {
        if (autonomous) autoSeconds_ += dt;
        if (teleop) teleopSeconds_ += dt;
        if (endgame) endgameSeconds_ += dt;
        zones_.update(ctx);
    }

    plates_.update(dt);
    if (running && teleop) human_.observe(pool_);
    if (running && teleop) {
        const auto us = static_cast<RulesTimeUs>(std::llround(dt * kRulesSecondUs));
        updateVault(ctx, human_.advance(pool_, us, config_));
    }
    if (running && (autonomous || teleop)) {
        score(ctx, dt, autonomous);
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.12. Published vault and power audit values. <code>seasons\2027\game\src\game_rule_snapshot.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_rule_snapshot.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

void State::publishRules(s::SnapshotBuilder& out) const {
    const auto& power = powers_.snapshot();
    for (std::size_t a = 0; a < 2; ++a) {
        const auto prefix = std::string("game_2027.") + (a == 0 ? "blue_" : "red_");
        out.setHud(prefix + "vault_points", vault_.snapshot().points[a]);
        out.setHud(prefix + "exchange", static_cast<int>(pool_.count(CubeLocation::Exchange, a)));
        out.setHud(prefix + "vault_cubes", static_cast<int>(pool_.count(CubeLocation::Vault, a)));
        out.setHud(prefix + "levitate", power.levitate[a]);
        out.setHud(prefix + "auto_quest", scoring_.snapshot().autoQuest[a]);
        for (std::size_t p = 0; p < 3; ++p)
            out.setHud(prefix + "column_" + std::to_string(p), vault_.snapshot().cubes[a][p]);
    }
    out.setHud("game_2027.power_active", power.active.has_value());
    out.setHud("game_2027.power_queued", power.queued.has_value());
    out.setHud(
        "game_2027.power_seconds",
        power.active ? static_cast<double>(power.active->remainingUs) / kRulesSecondUs : 0.0);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.13. Core publication plus rule details. <code>seasons\2027\game\src\game_snapshot.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_snapshot.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::publish(s::SnapshotBuilder& out) const {
    publishRules(out);
    out.setHud("game_2027.blue_score", scores_[0]);
    out.setHud("game_2027.red_score", scores_[1]);
    out.setHud("game_2027.phase", phase_);
    out.setHud("game_2027.remaining_s", remainingS_);
    out.setHud("game_2027.game_data", assignment_);
    out.setHud("game_2027.inventory", pool_.inventory());
    out.setHud("game_2027.released", pool_.released());
    out.setHud("game_2027.portal_blue", static_cast<int>(pool_.count(CubeLocation::Reserve, 0)));
    out.setHud("game_2027.portal_red", static_cast<int>(pool_.count(CubeLocation::Reserve, 1)));
    out.setHud("game_2027.auto_seconds", autoSeconds_);
    out.setHud("game_2027.teleop_seconds", teleopSeconds_);
    out.setHud("game_2027.endgame_seconds", endgameSeconds_);
    for (std::size_t i = 0; i < plates_.values().size(); ++i)
        out.setHud("game_2027.plate_" + std::to_string(i), plates_.values()[i].cubes);

    s::EntityGroupSample cubes;
    cubes.group = "game_2027.cubes";
    cubes.shape = "box";
    cubes.halfExtentsM = kCubeHalfM;
    cubes.color = "#ffda19";
    cubes.positions = pool_.positions(CubeLocation::Field);
    out.addEntityGroup(std::move(cubes));
    s::EntityGroupSample supply;
    supply.group = "game_2027.portal_supply";
    supply.shape = "box";
    supply.halfExtentsM = kCubeHalfM;
    supply.color = "#ffda19";
    supply.positions = pool_.positions(CubeLocation::Reserve);
    out.addEntityGroup(std::move(supply));

    for (std::size_t i = 0; i < plates_.values().size(); ++i) {
        s::EntityGroupSample plate;
        plate.group = "game_2027.plate_" + std::to_string(i);
        plate.shape = "box";
        plate.halfExtentsM = kPlateHalfM;
        plate.color =
            plateOwner(assignment_, i / 2, static_cast<int>(i % 2)) == 0 ? "#2457c5" : "#c52c30";
        plate.positions = {world_.pose(plates_.values()[i].body).position};
        out.addEntityGroup(std::move(plate));
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 17.14. Library sources through Chapter 17. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp
    src/match_clock.cpp
    src/cube_zones.cpp
    src/config.cpp
    src/plugin_info.cpp
    src/game_ui.cpp
    src/game_values.cpp
    src/game_state.cpp
    src/game_reset.cpp
    src/game_tick.cpp
    src/game_snapshot.cpp
    src/ownership_scoring.cpp
    src/game_scoring.cpp
    src/human_player.cpp
    src/cube_return.cpp
    src/vault.cpp
    src/power_ups.cpp
    src/game_vault.cpp
    src/game_rule_snapshot.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 17.6 Verification

The pure tests cover column limits, additional deposits after use, all levels,
same-alliance rejection, opponent queueing, exact expiration and freezing.
Scoring tests compare Force and Boost effects against explicit totals.

Listing 17.15. Vault capacity and scoring cases. <code>tests\game_2027\test_rules_vault.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_vault.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/vault.h"

using namespace wpiemu::plugins::game_2027;

TEST_CASE("rules vault columns each accept three cubes and award five per cube") {
    Vault vault;
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        for (auto column : {Power::Force, Power::Boost, Power::Levitate}) {
            CHECK(vault.deposit(alliance, column, {RulesPhase::Teleop}, 3) ==
                  RulesStatus::Accepted);
            CHECK(vault.deposit(alliance, column, {RulesPhase::Teleop}) == RulesStatus::ColumnFull);
        }
        CHECK(vault.snapshot().points[alliance] == 45);
    }
}

TEST_CASE("rules vault deposit validates phase gate count and identities without mutation") {
    Vault vault;
    const RulesContext teleop{RulesPhase::Teleop};
    CHECK(vault.deposit(0, Power::Force, {RulesPhase::Auto}) == RulesStatus::WrongPhase);
    CHECK(vault.deposit(0, Power::Force, {RulesPhase::Finished}) == RulesStatus::WrongPhase);
    CHECK(vault.deposit(0, Power::Force, {RulesPhase::Teleop, false}) == RulesStatus::Frozen);
    CHECK(vault.deposit(0, Power::Force, {RulesPhase::Teleop, true, true}) == RulesStatus::Frozen);
    CHECK(vault.deposit(2, Power::Force, teleop) == RulesStatus::InvalidAlliance);
    CHECK(vault.deposit(0, static_cast<Power>(99), teleop) == RulesStatus::InvalidPower);
    CHECK(vault.deposit(0, Power::Force, teleop, 0) == RulesStatus::InvalidCount);
    CHECK(vault.deposit(0, Power::Force, teleop, -1) == RulesStatus::InvalidCount);
    CHECK(vault.deposit(0, Power::Force, teleop, 4) == RulesStatus::InvalidCount);
    CHECK(vault.snapshot().points[0] == 0);
}

TEST_CASE("rules vault multi cube overfill is atomic and reset clears every column") {
    Vault vault;
    CHECK(vault.deposit(1, Power::Boost, {RulesPhase::Teleop}, 2) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Boost, {RulesPhase::Teleop}, 2) == RulesStatus::ColumnFull);
    CHECK(vault.snapshot().cubes[1][powerIndex(Power::Boost)] == 2);
    CHECK(vault.snapshot().points[1] == 10);
    CHECK(vault.deposit(1, Power::Boost, {RulesPhase::Teleop}) == RulesStatus::Accepted);
    CHECK(vault.snapshot().points[1] == 15);
    vault.reset();
    CHECK(vault.snapshot().points[1] == 0);
    CHECK(vault.snapshot().cubes[1][powerIndex(Power::Boost)] == 0);
}
```

Listing 17.16. Power acceptance, queue and duration cases. <code>tests\game_2027\test_rules_power.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_power.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/power_ups.h"

using namespace wpiemu::plugins::game_2027;
namespace {
constexpr RulesContext teleop{RulesPhase::Teleop};
constexpr RulesTimeUs second = kRulesSecondUs;
}  // namespace

TEST_CASE("rules global power queue expires at exact ten second boundary") {
    Vault vault;
    PowerUps powers;
    CHECK(vault.deposit(0, Power::Force, teleop, 1) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Boost, teleop, 2) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.advance(5 * second, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Boost, vault, teleop) == RulesStatus::Queued);
    REQUIRE(powers.snapshot().active);
    CHECK(powers.snapshot().active->alliance == 0);
    CHECK(powers.advance(5 * second - 1, teleop) == RulesStatus::Accepted);
    CHECK(powers.snapshot().active->remainingUs == 1);
    CHECK(powers.advance(1, teleop) == RulesStatus::Accepted);
    REQUIRE(powers.snapshot().active);
    CHECK(powers.snapshot().active->alliance == 1);
    CHECK(powers.snapshot().active->remainingUs == 10 * second);
    CHECK_FALSE(powers.snapshot().queued);
    CHECK(powers.advance(10 * second, teleop) == RulesStatus::Accepted);
    CHECK_FALSE(powers.snapshot().active);
}

TEST_CASE("rules simultaneous buttons order global ownership and reject own other power") {
    Vault vault;
    PowerUps powers;
    for (std::size_t alliance = 0; alliance < 2; ++alliance)
        for (auto power : {Power::Force, Power::Boost, Power::Levitate})
            CHECK(vault.deposit(alliance, power, teleop, 3) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Boost, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Force, vault, teleop) == RulesStatus::OwnPowerActive);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Queued);
    CHECK(powers.press(0, Power::Boost, vault, teleop) == RulesStatus::OwnPowerQueued);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::AlreadyUsed);
    CHECK(powers.press(1, Power::Levitate, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Levitate, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.advance(10 * second, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Force, vault, teleop) == RulesStatus::Queued);
    CHECK(powers.press(0, Power::Boost, vault, teleop) == RulesStatus::OwnPowerActive);
    CHECK(powers.advance(20 * second, teleop) == RulesStatus::Accepted);
    CHECK_FALSE(powers.snapshot().active);
    CHECK(powers.press(0, Power::Boost, vault, teleop) == RulesStatus::Accepted);
}

TEST_CASE("rules accepted queued levels stay latched while later cubes still earn points") {
    Vault vault;
    PowerUps powers;
    CHECK(vault.deposit(0, Power::Force, teleop) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Boost, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Boost, vault, teleop) == RulesStatus::Queued);
    CHECK(vault.deposit(0, Power::Force, teleop, 2) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Boost, teleop, 2) == RulesStatus::Accepted);
    REQUIRE(powers.snapshot().active);
    REQUIRE(powers.snapshot().queued);
    CHECK(powers.snapshot().active->level == 1);
    CHECK(powers.snapshot().queued->level == 1);
    CHECK(vault.snapshot().points[0] == 15);
    CHECK(vault.snapshot().points[1] == 15);
    CHECK(powers.advance(10 * second, teleop) == RulesStatus::Accepted);
    CHECK(powers.effects().boost[1][0]);
    CHECK_FALSE(powers.effects().boost[1][1]);
}

TEST_CASE("rules each force and boost level targets switch scale or both") {
    for (auto power : {Power::Force, Power::Boost}) {
        for (int level = 1; level <= 3; ++level) {
            Vault vault;
            PowerUps powers;
            CHECK(vault.deposit(1, power, teleop, level) == RulesStatus::Accepted);
            CHECK(powers.press(1, power, vault, teleop) == RulesStatus::Accepted);
            const auto effects = powers.effects();
            const auto& target = power == Power::Force ? effects.force : effects.boost;
            CHECK(target[1][0] == (level == 1 || level == 3));
            CHECK(target[1][1] == (level == 2 || level == 3));
            CHECK_FALSE(target[0][0]);
            CHECK_FALSE(target[0][1]);
        }
    }
}

TEST_CASE("rules power press phase validation insufficiency and once only levitate") {
    Vault vault;
    PowerUps powers;
    CHECK(powers.press(2, Power::Force, vault, teleop) == RulesStatus::InvalidAlliance);
    CHECK(powers.press(0, static_cast<Power>(-1), vault, teleop) == RulesStatus::InvalidPower);
    CHECK(powers.press(0, Power::Force, vault, {RulesPhase::Auto}) == RulesStatus::WrongPhase);
    CHECK(powers.press(0, Power::Force, vault, {RulesPhase::Finished}) == RulesStatus::WrongPhase);
    CHECK(powers.press(0, Power::Force, vault, {RulesPhase::Teleop, false}) == RulesStatus::Frozen);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::InsufficientCubes);
    CHECK(vault.deposit(0, Power::Levitate, teleop, 2) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Levitate, vault, teleop) == RulesStatus::InsufficientCubes);
    CHECK(vault.deposit(0, Power::Levitate, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Levitate, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.snapshot().levitate[0]);
    CHECK(powers.press(0, Power::Levitate, vault, teleop) == RulesStatus::AlreadyUsed);
}

TEST_CASE("rules power timing freezes while disabled estopped or zero and reset clears queue") {
    Vault vault;
    PowerUps powers;
    for (std::size_t alliance = 0; alliance < 2; ++alliance)
        CHECK(vault.deposit(alliance, Power::Force, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Force, vault, teleop) == RulesStatus::Queued);
    CHECK(powers.advance(second, {RulesPhase::Teleop, false}) == RulesStatus::Frozen);
    CHECK(powers.advance(second, {RulesPhase::Teleop, true, true}) == RulesStatus::Frozen);
    CHECK(powers.advance(0, teleop) == RulesStatus::Accepted);
    CHECK(powers.advance(-1, teleop) == RulesStatus::InvalidDuration);
    CHECK(powers.advance(second, {RulesPhase::Finished}) == RulesStatus::WrongPhase);
    CHECK(powers.snapshot().active->remainingUs == 10 * second);
    powers.reset();
    CHECK_FALSE(powers.snapshot().active);
    CHECK_FALSE(powers.snapshot().queued);
    CHECK_FALSE(powers.snapshot().used[0][0]);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
}
```

Listing 17.17. Ownership effects and split integration. <code>tests\game_2027\test_rules_power_scoring.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_power_scoring.cpp -->
```cpp
#include <doctest/doctest.h>

#include <algorithm>

#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/power_ups.h"

using namespace wpiemu::plugins::game_2027;
namespace {
constexpr RulesContext teleop{RulesPhase::Teleop};
constexpr RulesTimeUs second = kRulesSecondUs;
}  // namespace

TEST_CASE("rules force immediately establishes ownership and blocks opposing scale scoring") {
    OwnershipScoring scoring;
    CHECK(scoring.advance(second, teleop, {{}, 1}) == RulesStatus::Accepted);
    OwnershipEffects effects;
    effects.force[0][1] = true;
    CHECK(scoring.advance(second, teleop, {{}, 1}, effects) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().owned[0][1]);
    CHECK_FALSE(scoring.snapshot().owned[1][1]);
    CHECK(scoring.snapshot().microPoints[0] == 2 * second);
    CHECK(scoring.snapshot().microPoints[1] == second);
    CHECK(scoring.advance(second, teleop, {{}, 1}) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 2 * second);
    CHECK(scoring.snapshot().microPoints[1] == 3 * second);
}

TEST_CASE("rules force takeover of already owned target grants no duplicate bonus") {
    OwnershipScoring scoring;
    const PlateOwnership plates{{true, false}, {}};
    CHECK(scoring.advance(second, teleop, plates) == RulesStatus::Accepted);
    OwnershipEffects effects;
    effects.force[0][0] = true;
    CHECK(scoring.advance(second, teleop, plates, effects) == RulesStatus::Accepted);
    CHECK(scoring.advance(second, teleop, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 3 * second);
}

TEST_CASE("rules boost doubles only owned targets and never manufactures ownership") {
    OwnershipScoring scoring;
    OwnershipEffects effects;
    effects.boost[0] = {true, true};
    CHECK(scoring.advance(second, teleop, {}, effects) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 0);
    CHECK(scoring.advance(second, teleop, {{true, false}, {}}, effects) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 2 * second);
    CHECK(scoring.advance(250'001, teleop, {{true, false}, {}}, effects) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 2'500'002);
    CHECK(scoring.advance(250'001, teleop, {{true, false}, {}}) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 2'750'003);
}

TEST_CASE("rules long tick splits at force expiration and queued boost activation") {
    Vault vault;
    PowerUps powers;
    OwnershipScoring scoring;
    CHECK(vault.deposit(0, Power::Force, teleop) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Boost, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Boost, vault, teleop) == RulesStatus::Queued);
    // Integration example: phase and plate transitions must also be split externally.
    RulesTimeUs remaining = 20 * second;
    while (remaining > 0) {
        const auto step = powers.snapshot().active
                              ? std::min(remaining, powers.snapshot().active->remainingUs)
                              : remaining;
        CHECK(scoring.advance(step, teleop, {{false, true}, {}}, powers.effects()) ==
              RulesStatus::Accepted);
        CHECK(powers.advance(step, teleop) == RulesStatus::Accepted);
        remaining -= step;
    }
    CHECK(scoring.snapshot().microPoints[0] == 11 * second);
    CHECK(scoring.snapshot().microPoints[1] == 30 * second);
    CHECK_FALSE(powers.snapshot().active);
}

TEST_CASE("rules force timer crosses to queue including leftover subsecond time") {
    Vault vault;
    PowerUps powers;
    CHECK(vault.deposit(0, Power::Force, teleop) == RulesStatus::Accepted);
    CHECK(vault.deposit(1, Power::Force, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Force, vault, teleop) == RulesStatus::Accepted);
    CHECK(powers.press(1, Power::Force, vault, teleop) == RulesStatus::Queued);
    CHECK(powers.advance(10 * second + 123'456, teleop) == RulesStatus::Accepted);
    REQUIRE(powers.snapshot().active);
    CHECK(powers.snapshot().active->alliance == 1);
    CHECK(powers.snapshot().active->remainingUs == 10 * second - 123'456);
}
```

The integration test releases a physical preload through the lower exchange,
deposits that identity and starts Force. It cannot supply an artificial deposit
directly to the pure vault.

Listing 17.18. Physical exchange-to-vault-to-Force history. <code>tests\game_2027\test_vault_physics.cpp</code>.

<!-- book-file: tests\game_2027\test_vault_physics.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: physical exchange custody supplies a one-shot Force") {
    Match match("LRL");
    match.game.reset();
    auto& data = match.services.config.plugins["game_2027"].data;
    data["humanPeriodS"] = 0.1;
    data["exchangeMode"] = "vault";
    data["vaultColumn"] = "force";
    data["blueForceS"] = 1.0;
    auto built = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE(built);
    match.game = std::move(built.value());
    match.game->reset(match.ctx);
    auto& zone = match.services.config.zones[0];
    zone.local.position = {-0.75f, 0.f, 0.f};
    match.world->setPose(match.ctx.chassisBody(), {{1.05f, 2.4f, 0.15f}, {}});
    match.world->setLinearVelocity(match.ctx.chassisBody(), {-3.2f, 0.f, 0.f});
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Teleop;
    match.services.driverStation.matchTimeS = 135.0;
    match.advance(1);
    match.world->setLinearVelocity(match.ctx.chassisBody(), {});
    match.advance(399);
    CHECK(match.hud("game_2027.blue_vault_cubes") == 1);
    CHECK(match.hud("game_2027.blue_vault_points") == 5);
    CHECK(match.hud("game_2027.blue_column_0") == 1);
    CHECK(match.snapshot()->hudValue("game_2027.power_active")->asBool());
    CHECK(match.hud("game_2027.blue_score") >= 6);
    const auto timer = match.hud("game_2027.power_seconds");
    match.services.driverStation.enabled = false;
    match.advance(200);
    CHECK(match.hud("game_2027.power_seconds") == timer);
    match.game->reset(match.ctx);
    CHECK(match.hud("game_2027.blue_vault_cubes") == 0);
    CHECK(match.hud("game_2027.blue_score") == 0);
    CHECK_FALSE(match.snapshot()->hudValue("game_2027.power_active")->asBool());
}
```

## 17.7 Reproduction

Extract through Chapter 17 and run the pure-rule and component groups from
Chapter 15. For the physical vault experiment, select exchange destination
vault, column Force, a 0.1 s human period and a Blue Force press at 1 s.
The Chapter 18 process fixture supplies a repeatable real-robot variant.

## 17.8 Discussion

Scripted button times replace human decisions, not robot actions or cube
supply. A press scheduled before sufficient cubes arrive is rejected once;
it is not silently retried. F-14 through F-16 distinguish this interface policy
from the implemented rule effects.

## 17.9 Summary

Deposits, activation and ownership effects have separate state. The global
active slot and opponent queue prevent simultaneous timed powers. Exact
interval splitting keeps scoring consistent at timer boundaries.

## 17.10 Exercises

1. Determine the affected targets for each Force level.
2. Explain why adding a third cube during a level-1 active power cannot upgrade it.
3. Calculate the expected total for a Boost interval that expires halfway
   through a one-second scoring update.

[Chapter 18](18-climbing.md)
