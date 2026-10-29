# 19. Partners, opponents and final results

## 19.1 Objectives

After this chapter the reader can distinguish scripted referee outcomes from
physical observations, calculate qualification ranking points, and finalize a
match exactly once. Chapter 18 is the prerequisite. The scenario and ranking
modules address F-16 and F-19 through F-20.

## 19.2 Problem statement

One physical robot cannot demonstrate a three-robot Auto Quest or Face the Boss
achievement alone. Spawning score increments would conceal that limitation.
The remaining robots therefore need explicitly labeled virtual outcomes,
while any opponent cube placement must still pass through physical ownership.

## 19.3 Background and definitions

Auto Quest requires all three auto-runs and ownership of the home switch at
autonomous end. Face the Boss requires three credited climbs. Qualification
awards are two ranking points for a win, one for a tie, and one for each
achievement. Playoff results do not award ranking points [1, section 4.2].

**Definition 19.1 (virtual outcome).** A virtual outcome is a declared referee
input for a robot that has no simulated body. It is not a measurement of robot
motion, support, or collision.

The built-in scenario represents the physical Blue robot as robot slot 0,
with two virtual Blue partners and three virtual Red opponents. It is off
by default. An enabled Red station stops the script and reports an error.

## 19.4 Design

Multiple physical robot processes would require session and contact support
beyond this case study. A finite ordered event sequence instead permits a
small, auditable experiment. Its clock counts enabled autonomous and
teleoperation microseconds, excluding disabled time.

The script supplies five virtual auto-runs at two enabled seconds and virtual
finishes at 150 enabled seconds. Blue partners climb; Red robots park.
At 20 teleoperation seconds the adapter consumes one Red reserve cube and
drops that physical body above Red's scale plate.

**Invariant 19.1 (provenance).** Scripted auto-runs and finishes are identified
as scripted. A scripted cube placement changes custody and pose, never ownership
or score directly.

## 19.5 Implementation

### 19.5.1 Pure result calculation

The ranking function receives three finish states per alliance and points that
exclude endgame. Levitate selects an uncredited robot first, otherwise upgrades
a parked robot. A credited robot cannot also retain parking points; three is
the maximum climb count.

Listing 19.1. Finish records and result structure. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\ranking.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\ranking.h -->
```cpp
#pragma once

#include <optional>

#include "rules_types.h"

namespace wpiemu::plugins::game_2027 {

enum class RobotFinish { None, Parked, Climbed };
struct AllianceFinish {
    std::array<RobotFinish, 3> robots{};
    bool levitate{};
    bool autoQuest{};
    // Caller may select this with context RNG; deterministic cyclic tie-break.
    std::size_t levitateChoice{};
};
struct RankingAwards {
    int physicalClimbs{};
    int creditedClimbs{};
    int parked{};
    int endgamePoints{};
    bool faceBoss{};
    std::optional<std::size_t> levitateRobot;
    std::int64_t totalPoints{};
    int rankingPoints{};
};
struct RankingResult {
    RulesStatus status{RulesStatus::Accepted};
    std::array<RankingAwards, 2> awards{};
};

// [GM section4.2/Table4-1], [GM section4.3/Table4-2].
// Stateless final evaluation: pre-endgame points exclude
// parking/physical/virtual climbs. Scripted outcomes are admissible inputs, NOT
// evidence of a physics-measured climb.
[[nodiscard]] RankingResult rankMatch(const std::array<AllianceFinish, 2>& finishes,
                                      const std::array<std::int64_t, 2>& pointsBeforeEndgame,
                                      bool qualification = true) noexcept;

}  // namespace wpiemu::plugins::game_2027
```

Listing 19.2. Endgame points and qualification ranking. <code>seasons\2027\game\src\ranking.cpp</code>.

<!-- book-file: seasons\2027\game\src\ranking.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/ranking.h"

#include <limits>

namespace wpiemu::plugins::game_2027 {

RankingResult rankMatch(const std::array<AllianceFinish, 2>& finishes,
                        const std::array<std::int64_t, 2>& pointsBeforeEndgame,
                        bool qualification) noexcept {
    RankingResult result;
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        const auto& finish = finishes[alliance];
        if (finish.levitateChoice >= 3) {
            result.status = RulesStatus::InvalidRobot;
            return result;
        }
        if (pointsBeforeEndgame[alliance] < 0 ||
            pointsBeforeEndgame[alliance] > std::numeric_limits<std::int64_t>::max() - 90) {
            result.status = RulesStatus::InvalidCount;
            return result;
        }
        for (auto robot : finish.robots) {
            if (robot != RobotFinish::None && robot != RobotFinish::Parked &&
                robot != RobotFinish::Climbed) {
                result.status = RulesStatus::InvalidRobot;
                return result;
            }
        }
    }
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        const auto& finish = finishes[alliance];
        auto robots = finish.robots;
        auto& award = result.awards[alliance];
        for (auto robot : robots)
            if (robot == RobotFinish::Climbed) ++award.physicalClimbs;
        if (finish.levitate) {
            for (auto eligible : {RobotFinish::None, RobotFinish::Parked}) {
                for (std::size_t offset = 0; offset < 3; ++offset) {
                    const auto robot = (finish.levitateChoice + offset) % 3;
                    if (robots[robot] == eligible) {
                        robots[robot] = RobotFinish::Climbed;
                        award.levitateRobot = robot;
                        break;
                    }
                }
                if (award.levitateRobot) break;
            }
        }
        for (auto robot : robots) {
            if (robot == RobotFinish::Climbed) ++award.creditedClimbs;
            if (robot == RobotFinish::Parked) ++award.parked;
        }
        award.faceBoss =
            award.physicalClimbs == 3 || (award.physicalClimbs == 2 && finish.levitate);
        award.endgamePoints = award.creditedClimbs * 30 + award.parked * 5;
        award.totalPoints = pointsBeforeEndgame[alliance] + award.endgamePoints;
    }
    if (qualification) {
        for (std::size_t alliance = 0; alliance < 2; ++alliance) {
            auto& award = result.awards[alliance];
            const auto opponent = result.awards[1 - alliance].totalPoints;
            award.rankingPoints = (award.totalPoints > opponent    ? 2
                                   : award.totalPoints == opponent ? 1
                                                                   : 0) +
                                  (finishes[alliance].autoQuest ? 1 : 0) + (award.faceBoss ? 1 : 0);
        }
    }
    return result;
}

}  // namespace wpiemu::plugins::game_2027
```

### 19.5.2 Event cursor

The scenario object validates event order and indices. Calls drain due events
in input order, including equal timestamps. It neither owns a physics world
nor dispatches effects.

Listing 19.3. Typed scenario events and cursor. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\scenario.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\scenario.h -->
```cpp
#pragma once

#include <optional>
#include <span>

#include "ranking.h"

namespace wpiemu::plugins::game_2027 {

enum class ScriptedAction { AutoRun, VaultDeposit, PowerPress, RobotFinish };
struct ScenarioEvent {
    RulesTimeUs atUs{};
    ScriptedAction action{ScriptedAction::AutoRun};
    std::size_t alliance{};
    std::size_t robot{};
    Power power{Power::Force};
    int count{1};
    RobotFinish finish{RobotFinish::None};
};
struct ScenarioDelivery {
    RulesStatus status{RulesStatus::Complete};
    std::optional<ScenarioEvent> event;
    bool scripted{true};
};
struct ScenarioSnapshot {
    bool scripted{true};
    std::size_t scheduled{};
    std::size_t delivered{};
    std::optional<RulesTimeUs> nextAtUs;
};

// Pure finite event cursor: never moves a rigid body or claims a measured
// outcome. Caller supplies enabled match elapsed time and dispatches each event
// to rules. Equal timestamps preserve input ordering; repeated next(now) drains
// due events.
class Scenario {
public:
    RulesStatus configure(std::span<const ScenarioEvent> events) noexcept;
    [[nodiscard]] ScenarioDelivery next(RulesTimeUs nowUs) noexcept;
    [[nodiscard]] ScenarioSnapshot snapshot() const noexcept;
    void reset() noexcept {
        events_ = {};
        count_ = 0;
        cursor_ = 0;
        lastUs_ = 0;
    }

private:
    std::array<ScenarioEvent, 64> events_{};
    std::size_t count_{};
    std::size_t cursor_{};
    RulesTimeUs lastUs_{};
};

// Blue robots 1/2 are virtual partners; all three Red robots are virtual
// opponents. These are pedagogical scripted referee outcomes, not simulated
// robot motion.
[[nodiscard]] std::array<ScenarioEvent, 10> partnerOpponentScript() noexcept;

}  // namespace wpiemu::plugins::game_2027
```

Listing 19.4. Validated schedules and built-in virtual outcomes. <code>seasons\2027\game\src\scenario.cpp</code>.

<!-- book-file: seasons\2027\game\src\scenario.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/scenario.h"

#include <algorithm>

namespace wpiemu::plugins::game_2027 {

RulesStatus Scenario::configure(std::span<const ScenarioEvent> events) noexcept {
    if (events.size() > events_.size()) return RulesStatus::InvalidSchedule;
    RulesTimeUs previous = 0;
    for (const auto& event : events) {
        if (event.atUs < previous || event.alliance >= 2 || event.robot >= 3 ||
            !validPower(event.power) || event.count < 1 || event.count > 3)
            return RulesStatus::InvalidSchedule;
        switch (event.action) {
            case ScriptedAction::AutoRun:
            case ScriptedAction::VaultDeposit:
            case ScriptedAction::PowerPress:
                break;
            case ScriptedAction::RobotFinish:
                if (event.finish != RobotFinish::None && event.finish != RobotFinish::Parked &&
                    event.finish != RobotFinish::Climbed)
                    return RulesStatus::InvalidSchedule;
                break;
            default:
                return RulesStatus::InvalidSchedule;
        }
        previous = event.atUs;
    }
    reset();
    std::copy(events.begin(), events.end(), events_.begin());
    count_ = events.size();
    return RulesStatus::Accepted;
}

ScenarioDelivery Scenario::next(RulesTimeUs nowUs) noexcept {
    if (nowUs < lastUs_) return {RulesStatus::TimeReversed, std::nullopt};
    lastUs_ = nowUs;
    if (cursor_ == count_) return {};
    if (events_[cursor_].atUs > nowUs) return {RulesStatus::Queued, std::nullopt};
    return {RulesStatus::Accepted, events_[cursor_++]};
}

ScenarioSnapshot Scenario::snapshot() const noexcept {
    ScenarioSnapshot result{true, count_, cursor_, std::nullopt};
    if (cursor_ < count_) result.nextAtUs = events_[cursor_].atUs;
    return result;
}

std::array<ScenarioEvent, 10> partnerOpponentScript() noexcept {
    std::array<ScenarioEvent, 10> events{};
    constexpr std::array<std::size_t, 5> alliances{0, 0, 1, 1, 1};
    constexpr std::array<std::size_t, 5> robots{1, 2, 0, 1, 2};
    for (std::size_t i = 0; i < 5; ++i) {
        events[i] = {2 * kRulesSecondUs, ScriptedAction::AutoRun, alliances[i], robots[i]};
        events[i + 5] = {150 * kRulesSecondUs,
                         ScriptedAction::RobotFinish,
                         alliances[i],
                         robots[i],
                         Power::Force,
                         1,
                         i < 2 ? RobotFinish::Climbed : RobotFinish::Parked};
    }
    return events;
}

}  // namespace wpiemu::plugins::game_2027
```

### 19.5.3 Integration and finalization

The adapter dispatches supported events. A vault or power event lacking a
custody-backed handler is logged as an error rather than granted points.
Finalization uses the cached physical finish, seeded context randomness for
eligible Levitate choices, and the pure result calculation.

Listing 19.5. Event dispatch, physical opponent placement and finalization. <code>seasons\2027\game\src\game_scenario.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_scenario.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

void State::advanceScenario(s::SimContext& ctx, RulesTimeUs elapsedUs, const RulesContext& rules) {
    matchUs_ += elapsedUs;
    if (!config_.virtualScenario || scenarioFailed_) return;
    if (s::proto::allianceOf(ctx.driverStation().station) != s::Alliance::Blue) {
        ctx.logger().error(kId,
                           "Virtual scenario requires the physical robot on Blue; reset required");
        scenarioFailed_ = true;
        return;
    }
    for (;;) {
        const auto delivery = scenario_.next(matchUs_);
        if (delivery.status == RulesStatus::Complete || delivery.status == RulesStatus::Queued)
            break;
        if (delivery.status != RulesStatus::Accepted || !delivery.event) {
            ctx.logger().error(kId, "Scenario delivery rejected");
            break;
        }
        const auto& event = *delivery.event;
        if (event.action == ScriptedAction::AutoRun) {
            if (scoring_.awardAutoRun(event.alliance, event.robot, rules) != RulesStatus::Accepted)
                ctx.logger().error(kId, "Scripted auto-run rejected");
        } else if (event.action == ScriptedAction::RobotFinish) {
            finishes_[event.alliance].robots[event.robot] = event.finish;
        } else {
            ctx.logger().error(kId, "Scenario action has no custody-backed handler");
        }
    }
    if (!opponentPlaced_ && teleopUs_ >= 20 * kRulesSecondUs) {
        const auto side = plateOwner(assignment_, 2, 0) == 1 ? 0U : 1U;
        const auto p = plates_.center(4 + side) + s::Vec3{0.f, 0.f, 0.5f};
        if (!pool_.feed(1, {p, {}}))
            ctx.logger().warn(kId, "Scripted opponent has no reserve cube");
        opponentPlaced_ = true;
    }
    for (std::size_t a = 0; a < 2; ++a)
        scores_[a] = static_cast<double>(scoring_.snapshot().microPoints[a] / kRulesSecondUs +
                                         vault_.snapshot().points[a]);
}

void State::finish(s::SimContext& ctx) {
    const auto physical =
        s::proto::allianceOf(ctx.driverStation().station) == s::Alliance::Red ? 1U : 0U;
    finishes_[physical].robots[0] = endgame_.climbed  ? RobotFinish::Climbed
                                    : endgame_.parked ? RobotFinish::Parked
                                                      : RobotFinish::None;
    std::array<std::int64_t, 2> points{};
    for (std::size_t a = 0; a < 2; ++a) {
        auto& finish = finishes_[a];
        finish.levitate = powers_.snapshot().levitate[a];
        finish.autoQuest = scoring_.snapshot().autoQuest[a];
        std::vector<std::size_t> eligible;
        for (std::size_t r = 0; r < 3; ++r)
            if (finish.robots[r] == RobotFinish::None) eligible.push_back(r);
        if (eligible.empty())
            for (std::size_t r = 0; r < 3; ++r)
                if (finish.robots[r] == RobotFinish::Parked) eligible.push_back(r);
        if (finish.levitate && !eligible.empty())
            finish.levitateChoice = eligible[static_cast<std::size_t>(
                ctx.rng().nextDouble() * static_cast<double>(eligible.size()))];
        points[a] =
            scoring_.snapshot().microPoints[a] / kRulesSecondUs + vault_.snapshot().points[a];
    }
    ranking_ = rankMatch(finishes_, points, config_.qualification);
    if (ranking_.status != RulesStatus::Accepted) ctx.logger().error(kId, "Final ranking rejected");
    for (std::size_t a = 0; a < 2; ++a)
        scores_[a] = static_cast<double>(ranking_.awards[a].totalPoints);
    finalized_ = true;
}

}  // namespace wpiemu::plugins::game_2027
```

The new settings select virtual participation and qualification scoring.
The alliance check belongs in the enabled update, not in the factory:
`RobotSession` applies its Driver Station settings after constructing the
simulation. The factory therefore sees a temporary default station. A Red
station during enabled play logs one error and sets `scenario_failed` until
reset; no virtual outcomes are then supplied.

Listing 19.6. Scenario and qualification settings. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\config.h</code>.

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
    double climbMaxForceN = 4000.0;
    bool virtualScenario = false;
    bool qualification = true;
};

bool validAssignment(const std::string& value);
s::PluginConfigSchema configSchema();
GameConfig parseConfig(const s::Value& data);

}  // namespace wpiemu::plugins::game_2027
```

Listing 19.7. Bindings for scenario selection. <code>seasons\2027\game\src\config.cpp</code>.

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
    b.add(s::props::real("climbMaxForceN", "Cable load limit", "Climber", 4000.0, 1.0, 10000.0,
                         s::PropertyUnit::None),
          &GameConfig::climbMaxForceN);
    b.add(s::props::flag("virtualScenario", "Scripted partners and opponent", "Scenario", false),
          &GameConfig::virtualScenario);
    b.add(s::props::flag("qualification", "Qualification ranking points", "Scenario", true),
          &GameConfig::qualification);
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

The state adds the cursor, finish records and a finalization latch.
The first zero-time input after teleoperation finalizes the result.
Later ticks may settle plates but cannot award the result again.

Listing 19.8. Scenario state and finalization latch. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/climb.h"
#include "wpiemu/plugins/game_2027/cube_layout.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/cube_zones.h"
#include "wpiemu/plugins/game_2027/human_player.h"
#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/plate.h"
#include "wpiemu/plugins/game_2027/plates.h"
#include "wpiemu/plugins/game_2027/power_ups.h"
#include "wpiemu/plugins/game_2027/scenario.h"

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
    void updateClimb(s::SimContext& ctx, double dt);
    void advanceScenario(s::SimContext& ctx, RulesTimeUs elapsedUs, const RulesContext& rules);
    void finish(s::SimContext& ctx);
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
    Climb climb_;
    EndgamePose endgame_;
    Scenario scenario_;
    std::array<AllianceFinish, 2> finishes_{};
    RankingResult ranking_;
    RulesTimeUs matchUs_ = 0;
    bool opponentPlaced_ = false, teleopSeen_ = false, finalized_ = false;
    bool scenarioFailed_ = false;
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

Listing 19.9. Reset of scripts and final awards. <code>seasons\2027\game\src\game_reset.cpp</code>.

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
    climb_.reset(ctx.coupling());
    endgame_ = {};
    scenario_.reset();
    if (config_.virtualScenario &&
        scenario_.configure(partnerOpponentScript()) != RulesStatus::Accepted)
        ctx.logger().error(kId, "Built-in scenario is invalid");
    finishes_ = {};
    ranking_ = {};
    matchUs_ = 0;
    opponentPlaced_ = teleopSeen_ = finalized_ = false;
    scenarioFailed_ = false;
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

Listing 19.10. Enabled match time and one finalization. <code>seasons\2027\game\src\game_tick.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_tick.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"
#include "wpiemu/plugins/game_2027/match_clock.h"

namespace wpiemu::plugins::game_2027 {
void State::step(s::SimContext& ctx, double dt) {
    const auto& ds = ctx.driverStation();
    if (finalized_) {
        plates_.update(dt);
        return;
    }
    remainingS_ = ds.matchTimeS;
    const auto phase = phaseOf(ds);
    const bool running = phase != MatchPhase::Disabled;
    const bool autonomous = phase == MatchPhase::Autonomous;
    const bool endgame = phase == MatchPhase::Endgame;
    const bool teleop = phase == MatchPhase::Teleop || endgame;
    teleopSeen_ |= teleop;
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
        updateClimb(ctx, dt);
    }
    if (running && (autonomous || teleop)) {
        score(ctx, dt, autonomous);
        advanceScenario(ctx, static_cast<RulesTimeUs>(std::llround(dt * kRulesSecondUs)),
                        {autonomous ? RulesPhase::Auto : RulesPhase::Teleop, true, false});
    }
    if (teleopSeen_ && ds.matchTimeS == 0.0) finish(ctx);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 19.11. Physical, virtual and final-result audit keys. <code>seasons\2027\game\src\game_rule_snapshot.cpp</code>.

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
        out.setHud(prefix + "ranking_points", ranking_.awards[a].rankingPoints);
        out.setHud(prefix + "climbs", ranking_.awards[a].creditedClimbs);
        out.setHud(prefix + "endgame_points", ranking_.awards[a].endgamePoints);
        for (std::size_t p = 0; p < 3; ++p)
            out.setHud(prefix + "column_" + std::to_string(p), vault_.snapshot().cubes[a][p]);
    }
    out.setHud("game_2027.power_active", power.active.has_value() && !finalized_);
    out.setHud("game_2027.power_queued", power.queued.has_value() && !finalized_);
    out.setHud("game_2027.scripted", config_.virtualScenario);
    out.setHud("game_2027.scenario_failed", scenarioFailed_);
    out.setHud("game_2027.finalized", finalized_);
    out.setHud("game_2027.parked", endgame_.parked);
    out.setHud("game_2027.climbed", endgame_.climbed);
    out.setHud("game_2027.climb_failed", climb_.failed());
    out.setHud(
        "game_2027.power_seconds",
        power.active ? static_cast<double>(power.active->remainingUs) / kRulesSecondUs : 0.0);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 19.12. Library sources through Chapter 19. <code>seasons\2027\game\CMakeLists.txt</code>.

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
    src/game_rule_snapshot.cpp
    src/climb.cpp
    src/game_climb.cpp
    src/ranking.cpp
    src/scenario.cpp
    src/game_scenario.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 19.6 Verification

Ranking tests cover wins, ties, losses, achievement combinations, Levitate
selection and playoff exclusion. Cursor tests cover validation, equal times,
repeated polling and reset.

Listing 19.13. Endgame and ranking truth tables. <code>tests\game_2027\test_rules_ranking.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_ranking.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/ranking.h"

using namespace wpiemu::plugins::game_2027;

TEST_CASE("rules qualification win tie and loss awards include bonuses but playoffs do not") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[0].autoQuest = true;
    finishes[0].robots = {RobotFinish::Climbed, RobotFinish::Climbed, RobotFinish::Climbed};
    auto result = rankMatch(finishes, {100, 100});
    CHECK(result.status == RulesStatus::Accepted);
    CHECK(result.awards[0].rankingPoints == 4);
    CHECK(result.awards[1].rankingPoints == 0);
    result = rankMatch(finishes, {10, 100});
    CHECK(result.awards[0].totalPoints == 100);
    CHECK(result.awards[0].rankingPoints == 3);
    CHECK(result.awards[1].rankingPoints == 1);
    result = rankMatch(finishes, {0, 100});
    CHECK(result.awards[0].rankingPoints == 2);
    CHECK(result.awards[1].rankingPoints == 2);
    result = rankMatch(finishes, {100, 0}, false);
    CHECK(result.awards[0].totalPoints == 190);
    CHECK(result.awards[0].rankingPoints == 0);
    CHECK(result.awards[1].rankingPoints == 0);
}

TEST_CASE("rules levitate chooses unused robot before upgrading parked robot") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[0] = {{RobotFinish::Parked, RobotFinish::None, RobotFinish::Climbed}, true, false, 0};
    const auto result = rankMatch(finishes, {0, 0});
    CHECK(result.status == RulesStatus::Accepted);
    CHECK(result.awards[0].levitateRobot == 1);
    CHECK(result.awards[0].physicalClimbs == 1);
    CHECK(result.awards[0].creditedClimbs == 2);
    CHECK(result.awards[0].parked == 1);
    CHECK(result.awards[0].endgamePoints == 65);
    CHECK_FALSE(result.awards[0].faceBoss);
}

TEST_CASE("rules levitate upgrades parking without double counting its five points") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[0] = {
        {RobotFinish::Parked, RobotFinish::Climbed, RobotFinish::Climbed}, true, false, 2};
    const auto result = rankMatch(finishes, {0, 0});
    CHECK(result.awards[0].levitateRobot == 0);
    CHECK(result.awards[0].physicalClimbs == 2);
    CHECK(result.awards[0].creditedClimbs == 3);
    CHECK(result.awards[0].parked == 0);
    CHECK(result.awards[0].endgamePoints == 90);
    CHECK(result.awards[0].faceBoss);
    CHECK(result.awards[0].rankingPoints == 3);
}

TEST_CASE("rules three physical climbs cap levitate at three and award Face Boss") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[1] = {
        {RobotFinish::Climbed, RobotFinish::Climbed, RobotFinish::Climbed}, true, false, 0};
    const auto result = rankMatch(finishes, {0, 0});
    CHECK(result.awards[1].creditedClimbs == 3);
    CHECK(result.awards[1].endgamePoints == 90);
    CHECK_FALSE(result.awards[1].levitateRobot);
    CHECK(result.awards[1].faceBoss);
}

TEST_CASE("rules parking and levitate alone cannot satisfy Face Boss") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[0] = {{RobotFinish::Parked, RobotFinish::Parked, RobotFinish::Parked}, true, false, 2};
    auto result = rankMatch(finishes, {0, 0});
    CHECK(result.awards[0].levitateRobot == 2);
    CHECK(result.awards[0].endgamePoints == 40);
    CHECK_FALSE(result.awards[0].faceBoss);
    finishes[0].levitate = false;
    result = rankMatch(finishes, {0, 0});
    CHECK(result.awards[0].endgamePoints == 15);
}

TEST_CASE("rules deterministic levitate choice cycles among otherwise equal unused robots") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[0].levitate = true;
    finishes[0].levitateChoice = 2;
    const auto first = rankMatch(finishes, {0, 0});
    const auto repeated = rankMatch(finishes, {0, 0});
    CHECK(first.awards[0].levitateRobot == 2);
    CHECK(repeated.awards[0].levitateRobot == first.awards[0].levitateRobot);
    CHECK(first.awards[0].endgamePoints == 30);
}

TEST_CASE("rules invalid final outcome returns typed status without awarding partial scores") {
    std::array<AllianceFinish, 2> finishes{};
    finishes[1].levitateChoice = 3;
    CHECK(rankMatch(finishes, {0, 0}).status == RulesStatus::InvalidRobot);
    finishes[1].levitateChoice = 0;
    finishes[1].robots[1] = static_cast<RobotFinish>(99);
    const auto result = rankMatch(finishes, {0, 0});
    CHECK(result.status == RulesStatus::InvalidRobot);
    CHECK(result.awards[0].totalPoints == 0);
    CHECK(rankMatch({}, {-1, 0}).status == RulesStatus::InvalidCount);
}
```

Listing 19.14. Scenario cursor and validation. <code>tests\game_2027\test_rules_scenario.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_scenario.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/scenario.h"

using namespace wpiemu::plugins::game_2027;

TEST_CASE("rules scenario cursor orders same-time events exactly once and rejects time reversal") {
    Scenario scenario;
    const std::array<ScenarioEvent, 3> events{
        {{10, ScriptedAction::VaultDeposit, 0, 0, Power::Force},
         {10, ScriptedAction::PowerPress, 0, 0, Power::Force},
         {11, ScriptedAction::RobotFinish, 1, 2, Power::Force, 1, RobotFinish::Climbed}}};
    CHECK(scenario.configure(events) == RulesStatus::Accepted);
    CHECK(scenario.next(9).status == RulesStatus::Queued);
    const auto first = scenario.next(10);
    CHECK(first.scripted);
    REQUIRE(first.event);
    CHECK(first.event->action == ScriptedAction::VaultDeposit);
    const auto second = scenario.next(10);
    REQUIRE(second.event);
    CHECK(second.event->action == ScriptedAction::PowerPress);
    CHECK(scenario.next(9).status == RulesStatus::TimeReversed);
    CHECK(scenario.snapshot().delivered == 2);
    CHECK(scenario.snapshot().nextAtUs == 11);
    const auto final = scenario.next(50);
    REQUIRE(final.event);
    CHECK(final.event->robot == 2);
    CHECK(scenario.next(50).status == RulesStatus::Complete);
    CHECK_FALSE(scenario.snapshot().nextAtUs);
}

TEST_CASE("rules scenario rejects invalid schedules transactionally") {
    Scenario scenario;
    const auto script = partnerOpponentScript();
    CHECK(scenario.configure(script) == RulesStatus::Accepted);
    std::array<ScenarioEvent, 2> invalid{{{2}, {1}}};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    CHECK(scenario.snapshot().scheduled == script.size());
    invalid = {{{-1}, {1}}};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid = {{{0}, {1, ScriptedAction::AutoRun, 2}}};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid = {{{0}, {1, ScriptedAction::AutoRun, 0, 3}}};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid[1] = {1, static_cast<ScriptedAction>(99)};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid[1] = {1, ScriptedAction::PowerPress, 0, 0, static_cast<Power>(99)};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid[1] = {1, ScriptedAction::VaultDeposit, 0, 0, Power::Force, 4};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    invalid[1] = {1, ScriptedAction::RobotFinish, 0, 0, Power::Force,
                  1, static_cast<RobotFinish>(99)};
    CHECK(scenario.configure(invalid) == RulesStatus::InvalidSchedule);
    CHECK(scenario.configure(std::array<ScenarioEvent, 65>{}) == RulesStatus::InvalidSchedule);
    CHECK(scenario.snapshot().delivered == 0);
}

TEST_CASE("rules default script explicitly supplies two partners and three opponents") {
    Scenario scenario;
    CHECK(scenario.configure(partnerOpponentScript()) == RulesStatus::Accepted);
    CHECK(scenario.snapshot().scripted);
    std::array<std::array<int, 3>, 2> runCounts{};
    std::array<std::array<int, 3>, 2> endCounts{};
    for (int i = 0; i < 10; ++i) {
        const auto delivery = scenario.next(150 * kRulesSecondUs);
        REQUIRE(delivery.status == RulesStatus::Accepted);
        REQUIRE(delivery.event);
        CHECK(delivery.scripted);
        const auto& event = *delivery.event;
        if (event.action == ScriptedAction::AutoRun)
            ++runCounts[event.alliance][event.robot];
        else
            ++endCounts[event.alliance][event.robot];
    }
    CHECK(runCounts[0] == std::array<int, 3>{0, 1, 1});
    CHECK(runCounts[1] == std::array<int, 3>{1, 1, 1});
    CHECK(endCounts == runCounts);
    CHECK(scenario.next(150 * kRulesSecondUs).status == RulesStatus::Complete);
}

TEST_CASE("rules scenario reset clears script cursor and monotonic clock") {
    Scenario scenario;
    CHECK(scenario.configure(partnerOpponentScript()) == RulesStatus::Accepted);
    CHECK(scenario.next(150 * kRulesSecondUs).status == RulesStatus::Accepted);
    scenario.reset();
    CHECK(scenario.snapshot().scheduled == 0);
    CHECK(scenario.snapshot().delivered == 0);
    CHECK(scenario.next(0).status == RulesStatus::Complete);
    CHECK(scenario.configure(partnerOpponentScript()) == RulesStatus::Accepted);
    CHECK(scenario.next(2 * kRulesSecondUs).status == RulesStatus::Accepted);
}
```

Listing 19.15. Rule composition with explicit virtual provenance. <code>tests\game_2027\test_rules_scenario_integration.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_scenario_integration.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/power_ups.h"
#include "wpiemu/plugins/game_2027/scenario.h"

using namespace wpiemu::plugins::game_2027;

TEST_CASE("rules scripted partner opponent outcomes compose with measured-rule inputs") {
    Scenario scenario;
    OwnershipScoring scoring;
    Vault vault;
    PowerUps powers;
    constexpr RulesContext autonomous{RulesPhase::Auto};
    constexpr RulesContext teleop{RulesPhase::Teleop};
    const PlateOwnership plates{{true, false}, {}};
    CHECK(scenario.configure(partnerOpponentScript()) == RulesStatus::Accepted);
    CHECK(scoring.advance(2 * kRulesSecondUs, autonomous, plates) == RulesStatus::Accepted);
    CHECK(scoring.awardAutoRun(0, 0, autonomous) == RulesStatus::Accepted);
    for (int i = 0; i < 5; ++i) {
        const auto delivery = scenario.next(2 * kRulesSecondUs);
        REQUIRE(delivery.scripted);
        REQUIRE(delivery.event);
        CHECK(scoring.awardAutoRun(delivery.event->alliance, delivery.event->robot, autonomous) ==
              RulesStatus::Accepted);
    }
    CHECK(scoring.advance(13 * kRulesSecondUs, autonomous, plates) == RulesStatus::Accepted);
    CHECK(scoring.captureAutoEnd() == RulesStatus::Accepted);
    CHECK(vault.deposit(0, Power::Levitate, teleop, 3) == RulesStatus::Accepted);
    CHECK(powers.press(0, Power::Levitate, vault, teleop) == RulesStatus::Accepted);
    CHECK(scoring.advance(135 * kRulesSecondUs, teleop, plates) == RulesStatus::Accepted);
    std::array<AllianceFinish, 2> finishes{};
    finishes[0].robots[0] = RobotFinish::Parked;
    for (int i = 0; i < 5; ++i) {
        const auto delivery = scenario.next(150 * kRulesSecondUs);
        REQUIRE(delivery.scripted);
        REQUIRE(delivery.event);
        const auto& event = *delivery.event;
        finishes[event.alliance].robots[event.robot] = event.finish;
    }
    std::array<std::int64_t, 2> beforeEndgame{};
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        finishes[alliance].autoQuest = scoring.snapshot().autoQuest[alliance];
        finishes[alliance].levitate = powers.snapshot().levitate[alliance];
        beforeEndgame[alliance] = scoring.snapshot().microPoints[alliance] / kRulesSecondUs +
                                  vault.snapshot().points[alliance];
    }
    const auto result = rankMatch(finishes, beforeEndgame);
    CHECK(result.status == RulesStatus::Accepted);
    CHECK(result.awards[0].totalPoints == 285);
    CHECK(result.awards[1].totalPoints == 30);
    CHECK(result.awards[0].rankingPoints == 4);
    CHECK(result.awards[1].rankingPoints == 0);
    CHECK(scenario.next(150 * kRulesSecondUs).status == RulesStatus::Complete);
}
```

The state test runs the full enabled history and checks the physical reserve
change, one finalization and reset. The process test adds final climb awards
and a complete C++ match with the optional scenario.

Listing 19.16. Complete scripted history through the physical state. <code>tests\game_2027\test_scenario_session.cpp</code>.

<!-- book-file: tests\game_2027\test_scenario_session.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: scenario construction precedes the configured Driver Station") {
    Match match("LRL", true);
    match.game.reset();
    match.services.config.plugins["game_2027"].data["virtualScenario"] = true;
    auto built = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE(built);
    match.game = std::move(built.value());
    match.services.driverStation.station = s::proto::AllianceStation::Blue1;
    match.game->reset(match.ctx);
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(401);
    CHECK_FALSE(match.snapshot()->hudValue("game_2027.scenario_failed")->asBool());
    CHECK(match.hud("game_2027.blue_score") >= 10);
    CHECK(match.services.logger.count(s::LogLevel::Error) == 0);
}

TEST_CASE("Power UP: an enabled Red scenario rejects once until reset") {
    Match match("LRL", true);
    match.game.reset();
    match.services.config.plugins["game_2027"].data["virtualScenario"] = true;
    auto built = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE(built);
    match.game = std::move(built.value());
    match.game->reset(match.ctx);
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(600);
    CHECK(match.snapshot()->hudValue("game_2027.scenario_failed")->asBool());
    CHECK(match.hud("game_2027.blue_score") == 0);
    CHECK(match.services.logger.count(s::LogLevel::Error) == 1);
    match.game->reset(match.ctx);
    CHECK_FALSE(match.snapshot()->hudValue("game_2027.scenario_failed")->asBool());
}

TEST_CASE("Power UP: scripted partners and opponent finalize once and reset") {
    Match match("LRL");
    match.game.reset();
    match.services.config.plugins["game_2027"].data["virtualScenario"] = true;
    auto built = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE(built);
    match.game = std::move(built.value());
    match.game->reset(match.ctx);
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(3000);
    match.services.driverStation.mode = s::proto::RobotMode::Teleop;
    match.services.driverStation.matchTimeS = 135.0;
    match.advance(27000);
    CHECK(match.hud("game_2027.portal_red") == 13);
    match.services.driverStation.matchTimeS = 0.0;
    match.advance(1);
    CHECK(match.snapshot()->hudValue("game_2027.finalized")->asBool());
    CHECK(match.hud("game_2027.blue_climbs") == 2);
    CHECK(match.hud("game_2027.blue_endgame_points") == 60);
    CHECK(match.hud("game_2027.red_endgame_points") == 15);
    const auto score = match.hud("game_2027.blue_score");
    match.advance(400);
    CHECK(match.hud("game_2027.blue_score") == score);
    match.game->reset(match.ctx);
    CHECK_FALSE(match.snapshot()->hudValue("game_2027.finalized")->asBool());
    CHECK(match.hud("game_2027.blue_score") == 0);
    CHECK(match.hud("game_2027.portal_red") == 14);
}
```

Listing 19.17. Real-HAL feature awards and virtual-partner match. <code>app\host\tests\power_up_features.cpp</code>.

<!-- book-file: app\host\tests\power_up_features.cpp -->
```cpp
#include "power_up_match.h"

namespace power_up_match {

void configureFeature(s::RobotConfig& config, const std::string& routine) {
    auto& data = config.plugins["game_2027"].data;
    if (routine == "scenario") {
        data["virtualScenario"] = true;
        return;
    }
    if (routine == "vault") {
        config.startPose = s::RobotStartPose{0.4f, 2.4f, static_cast<float>(s::kPi)};
        config.zones[0].local.position.z = 0.05f;
        data["humanPeriodS"] = 0.1;
        data["exchangeMode"] = "vault";
        data["vaultColumn"] = "force";
        data["blueForceS"] = 2.0;
        return;
    }
    REQUIRE_MESSAGE(routine == "climb", "Unknown Power UP test routine");
    data["preload"] = false;
    config.startPose = s::RobotStartPose{6.9f, 8.2296f / 2.f, 0.f};
    auto winch = config.mechanisms[0];
    winch.id = "winch";
    winch.label = "Climber activation motor";
    for (auto& mapping : winch.mappings) {
        mapping.slot.owner = "winch";
        mapping.device.id = 3;
    }
    config.mechanisms.push_back(std::move(winch));
    auto hook = config.zones[0];
    hook.id = "hook";
    hook.kind = "game_2027.climb";
    hook.local.position = {0.f, 0.f, 1.5f};
    hook.activation.conditions[0].mechanismId = "winch";
    config.zones = {hook};
}

TEST_CASE("e2e: Power UP robot PWM releases an exchange cube and activates Force") {
    for (const std::string language : {"java", "cpp"}) {
        CAPTURE(language);
        Match match(language, "vault");
        match.advance(1);
        match.period(s::proto::RobotMode::Teleop, 5);
        CHECK(match.number("game_2027.released") == 1);
        CHECK(match.number("game_2027.inventory") == 0);
        CHECK(match.number("game_2027.blue_vault_cubes") == 1);
        CHECK(match.number("game_2027.blue_vault_points") == 5);
        CHECK(match.snapshot()->hudValue("game_2027.power_active")->asBool());
        match.ds.enabled = false;
        match.session->controller().setDriverStation(match.ds);
        const auto timer = match.number("game_2027.power_seconds");
        match.advance(1);
        CHECK(match.number("game_2027.power_seconds") == timer);
        std::printf("POWER_UP FEATURE %s vault=%.0f Force seconds=%.3f\n", language.c_str(),
                    match.number("game_2027.blue_vault_cubes"), timer);
    }
}

TEST_CASE("e2e: Power UP robot PWM engages the anchored climber") {
    for (const std::string language : {"java", "cpp"}) {
        CAPTURE(language);
        Match match(language, "climb");
        match.advance(1);
        match.period(s::proto::RobotMode::Teleop, 10);
        CHECK_FALSE(match.snapshot()->hudValue("game_2027.climb_failed")->asBool());
        CHECK(match.snapshot()->hudValue("game_2027.climbed")->asBool());
        CHECK(match.snapshot()->chassis().pose.position.z > 0.45f);
        match.ds.enabled = false;
        match.ds.matchTimeS = 0.0;
        match.session->controller().setDriverStation(match.ds);
        match.advance(1);
        CHECK(match.number("game_2027.blue_climbs") == 1);
        CHECK(match.number("game_2027.blue_endgame_points") == 30);
        std::printf("POWER_UP FEATURE %s climbs=%.0f endgame=%.0f z=%.3f\n", language.c_str(),
                    match.number("game_2027.blue_climbs"),
                    match.number("game_2027.blue_endgame_points"),
                    match.snapshot()->chassis().pose.position.z);
    }
}

TEST_CASE("e2e: Power UP full match with virtual partners and a physical opponent cube") {
    Match match("cpp", "scenario");
    match.advance(1);
    match.period(s::proto::RobotMode::Autonomous, 15);
    match.ds.enabled = false;
    match.ds.matchTimeS = 0.0;
    match.session->controller().setDriverStation(match.ds);
    match.advance(2);
    match.period(s::proto::RobotMode::Teleop, 135);
    match.ds.enabled = false;
    match.ds.matchTimeS = 0.0;
    match.session->controller().setDriverStation(match.ds);
    match.advance(1);
    CHECK(match.snapshot()->hudValue("game_2027.blue_auto_quest")->asBool());
    CHECK(match.number("game_2027.blue_climbs") == 2);
    CHECK(match.number("game_2027.red_endgame_points") == 15);
    CHECK(match.number("game_2027.blue_ranking_points") == 3);
    CHECK(match.number("game_2027.portal_red") == 13);
    CHECK(match.number("game_2027.red_score") > 30);
    std::printf("POWER_UP SCENARIO blue=%.0f red=%.0f RP=%.0f\n",
                match.number("game_2027.blue_score"), match.number("game_2027.red_score"),
                match.number("game_2027.blue_ranking_points"));
}

}  // namespace power_up_match
```

## 19.7 Reproduction

Extract through Chapter 19. Run the pure-rule and component groups, then the
opt-in real-HAL test. For a desktop experiment, enable the scripted scenario,
select Blue station 1 and run a complete 15 s plus 135 s timed match.
The `scripted` publication key distinguishes this run from a single-robot run.

## 19.8 Discussion

The fixed scenario targets the nominal full match. Shortened practice periods
do not rescale its event timestamps: events beyond the enabled history remain
undelivered. This is a scenario choice, not a second Driver Station clock.

## 19.9 Summary

Final scoring composes measured and declared outcomes without disguising their
origins. Physical opponent cubes still require plate qualification. A latch
prevents repeated final awards, and reset restores a fresh scenario.

## 19.10 Exercises

1. Calculate ranking points for a tie with Auto Quest but without Face the Boss.
2. Explain why a virtual climb is not evidence that a physical climbing design works.
3. Predict the effect of replacing 135 s teleoperation with a 60 s practice period.

[Chapter 20](20-presentation.md)
