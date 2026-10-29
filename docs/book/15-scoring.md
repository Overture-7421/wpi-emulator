# 15. Ownership and score accounting

## 15.1 Objectives

After this chapter the reader can separate physical evidence from a rule
state machine, implement one-second ownership acquisition, and explain the
rounding policy used for published scores. Chapter 14 is the prerequisite.
The scoring module addresses R6, R7 and F-08 through F-10.

## 15.2 Problem statement

The core awards points whenever one plate carries more cubes. The manual
instead defines ownership through a plate height maintained for one second.
Its scoring table also awards points for establishing ownership [1, section
4.2 and Table 4-1]. Counting cubes and accumulating points in the same loop
cannot represent those distinctions adequately.

## 15.3 Background and definitions

**Definition 15.1 (qualifying observation).** A home switch qualifies while
its assigned plate surface is at most 6 in above the floor. A scale qualifies
for an alliance while its plate surface is at most 56 in above the floor.
One uninterrupted enabled second establishes ownership.

The manual gives 2 establishment points and 2 points/s in autonomous, and
1 establishment point and 1 point/s in teleoperation. Auto-run gives 5 points
once per robot. An alliance cannot score ownership of its opponent's switch.

The manual does not specify a fractional rounding algorithm or every boundary
of recapture and phase transition. The case study uses the policy in Appendix A:
ownership is lost immediately when its height condition fails, reacquisition
requires another second, and ownership continuing into teleoperation earns no
second establishment award. These are explicit model choices, not a claim
about FIRST's scoring-engine implementation.

## 15.4 Design

A whole-second timer would need a convention for where those seconds begin.
A floating-point accumulator introduces another numerical policy. The selected
model integrates in integer micro-points: one point equals 1,000,000 units.
The published whole score is the floor of the accumulated total.

For rate r and elapsed microseconds d, the increment is r times d.
The acquisition interval is consumed before elapsed-ownership points accrue;
an interval ending exactly at acquisition earns only establishment points.
This also permits tests with irregular subdivisions of the same interval.

**Invariant 15.1 (accounting).** Subdividing an interval with unchanged phase
and observations cannot change its final score. Rounding occurs at publication,
not independently at each tick or plate.

The floor differs from its unrounded total by less than one point per alliance.
This bound does not compare the final model with Chapter 12, whose acquisition
rule and plate travel are different.

## 15.5 Implementation

### 15.5.1 Rule inputs and state

The common types carry time, phase, typed rejection results and later power-up
effects. An invalid alliance or interval has an explicit result. The rule
module depends on neither a physics body nor a snapshot builder.

Listing 15.1. Shared rule inputs and outcomes. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\rules_types.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\rules_types.h -->
```cpp
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace wpiemu::plugins::game_2027 {

using RulesTimeUs = std::int64_t;
inline constexpr RulesTimeUs kRulesSecondUs = 1'000'000;
enum class RulesPhase { Disabled, Auto, Teleop, Finished };
enum class Power { Force, Boost, Levitate };
enum class RulesStatus {
    Accepted,
    Queued,
    InvalidAlliance,
    InvalidRobot,
    InvalidPower,
    InvalidDuration,
    InvalidCount,
    WrongPhase,
    Frozen,
    AlreadyAwarded,
    ColumnFull,
    InsufficientCubes,
    AlreadyUsed,
    OwnPowerActive,
    OwnPowerQueued,
    InvalidSchedule,
    TimeReversed,
    Complete
};

struct RulesContext {
    RulesPhase phase{RulesPhase::Disabled};
    bool enabled{true};
    bool eStopped{};
    [[nodiscard]] bool running() const noexcept { return enabled && !eStopped; }
};

[[nodiscard]] constexpr bool validPower(Power power) noexcept {
    return power == Power::Force || power == Power::Boost || power == Power::Levitate;
}
[[nodiscard]] constexpr std::size_t powerIndex(Power power) noexcept {
    return static_cast<std::size_t>(power);
}
[[nodiscard]] inline RulesStatus teleopStatus(const RulesContext& context) noexcept {
    if (context.phase != RulesPhase::Teleop) return RulesStatus::WrongPhase;
    return context.running() ? RulesStatus::Accepted : RulesStatus::Frozen;
}

// Each target is independent; switch[0] is Blue's own switch, switch[1] Red's.
struct OwnershipEffects {
    std::array<std::array<bool, 2>, 2> force{};  // [alliance][switch, scale]
    std::array<std::array<bool, 2>, 2> boost{};
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 15.2. Ownership state and public operations. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\ownership_scoring.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\ownership_scoring.h -->
```cpp
#pragma once

#include <optional>

#include "rules_types.h"

namespace wpiemu::plugins::game_2027 {

struct PlateOwnership {
    std::array<bool, 2> switchDown{};
    std::optional<std::size_t> scaleDown;
};
struct OwnershipSnapshot {
    // Exact fixed-point points: divide by 1,000,000 only for presentation.
    std::array<std::int64_t, 2> microPoints{};
    std::array<std::array<bool, 2>, 2> owned{};
    std::array<std::array<bool, 3>, 2> autoRuns{};
    std::array<bool, 2> autoQuest{};
    bool autoEndCaptured{};
};

class OwnershipScoring {
public:
    // [GM section4.2/Table4-1], [GM section4.3/Table4-2].
    // Input is constant on [start,end]. Split at phase, plate and power changes.
    // Acquisition takes one continuous enabled second. Ownership at the exact end
    // earns establishment points, but no elapsed-ownership points yet.
    // AUTO->TELEOP preserves acquisition/ownership and fractional points; only
    // the future rate changes, with NO new teleop establishment bonus.
    // Disabled/eStop/zero ticks freeze both score and acquisition.
    RulesStatus advance(RulesTimeUs elapsedUs, const RulesContext& context,
                        const PlateOwnership& plates,
                        const OwnershipEffects& effects = {}) noexcept;
    RulesStatus awardAutoRun(std::size_t alliance, std::size_t robot,
                             const RulesContext& context) noexcept;
    // Call once after the final AUTO interval, BEFORE TELEOP inputs/forces.
    RulesStatus captureAutoEnd() noexcept;
    [[nodiscard]] const OwnershipSnapshot& snapshot() const noexcept { return state_; }
    void reset() noexcept {
        state_ = {};
        heldUs_ = {};
    }

private:
    OwnershipSnapshot state_{};
    std::array<std::array<RulesTimeUs, 2>, 2> heldUs_{};
};

}  // namespace wpiemu::plugins::game_2027
```

The implementation treats each home switch and each alliance's scale ownership
separately. Acquisition consumes part of an interval, then any remainder accrues
at the current rate. Auto-run records have three robot slots per alliance so
Chapter 19 can supply explicitly virtual partner outcomes.

Listing 15.3. Acquisition, accrual and Auto Quest capture. <code>seasons\2027\game\src\ownership_scoring.cpp</code>.

<!-- book-file: seasons\2027\game\src\ownership_scoring.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/ownership_scoring.h"

#include <algorithm>
#include <limits>

namespace wpiemu::plugins::game_2027 {

RulesStatus OwnershipScoring::advance(RulesTimeUs elapsedUs, const RulesContext& context,
                                      const PlateOwnership& plates,
                                      const OwnershipEffects& effects) noexcept {
    // Four points/us per target is the maximum rate (defensive overflow check).
    constexpr auto max = std::numeric_limits<std::int64_t>::max();
    if (elapsedUs < 0 || elapsedUs > (max - 8 * kRulesSecondUs) / 8)
        return RulesStatus::InvalidDuration;
    if (plates.scaleDown && *plates.scaleDown >= 2) return RulesStatus::InvalidAlliance;
    if (context.phase != RulesPhase::Auto && context.phase != RulesPhase::Teleop)
        return RulesStatus::WrongPhase;
    if (!context.running()) return RulesStatus::Frozen;
    if (elapsedUs == 0) return RulesStatus::Accepted;
    for (const auto score : state_.microPoints)
        if (score > max - 8 * elapsedUs - 8 * kRulesSecondUs) return RulesStatus::InvalidDuration;
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        for (std::size_t target = 0; target < 2; ++target) {
            const bool down =
                target == 0 ? plates.switchDown[alliance] : plates.scaleDown == alliance;
            auto& held = heldUs_[alliance][target];
            if (!down) held = 0;
            const auto delay = down ? kRulesSecondUs - held : elapsedUs + 1;
            const bool forced =
                context.phase == RulesPhase::Teleop && effects.force[alliance][target];
            const bool blocked = target == 1 && context.phase == RulesPhase::Teleop &&
                                 effects.force[1 - alliance][1];
            const bool available = !blocked && (forced || (down && delay <= elapsedUs));
            const auto wait = forced ? RulesTimeUs{0} : delay;
            const auto rate =
                (context.phase == RulesPhase::Auto ? 2 : 1) *
                (context.phase == RulesPhase::Teleop && effects.boost[alliance][target] ? 2 : 1);
            if (available) {
                if (!state_.owned[alliance][target] || wait > 0)
                    state_.microPoints[alliance] += rate * kRulesSecondUs;
                state_.microPoints[alliance] += rate * (elapsedUs - wait);
            }
            state_.owned[alliance][target] = available;
            if (down) held += std::min(elapsedUs, kRulesSecondUs - held);
        }
    }
    return RulesStatus::Accepted;
}

RulesStatus OwnershipScoring::awardAutoRun(std::size_t alliance, std::size_t robot,
                                           const RulesContext& context) noexcept {
    if (alliance >= 2) return RulesStatus::InvalidAlliance;
    if (robot >= 3) return RulesStatus::InvalidRobot;
    if (context.phase != RulesPhase::Auto || state_.autoEndCaptured) return RulesStatus::WrongPhase;
    if (!context.running()) return RulesStatus::Frozen;
    if (state_.autoRuns[alliance][robot]) return RulesStatus::AlreadyAwarded;
    if (state_.microPoints[alliance] >
        std::numeric_limits<std::int64_t>::max() - 5 * kRulesSecondUs)
        return RulesStatus::InvalidDuration;
    state_.autoRuns[alliance][robot] = true;
    state_.microPoints[alliance] += 5 * kRulesSecondUs;
    return RulesStatus::Accepted;
}

RulesStatus OwnershipScoring::captureAutoEnd() noexcept {
    if (state_.autoEndCaptured) return RulesStatus::AlreadyAwarded;
    state_.autoEndCaptured = true;
    for (std::size_t alliance = 0; alliance < 2; ++alliance) {
        const auto& runs = state_.autoRuns[alliance];
        state_.autoQuest[alliance] = runs[0] && runs[1] && runs[2] && state_.owned[alliance][0];
    }
    return RulesStatus::Accepted;
}

}  // namespace wpiemu::plugins::game_2027
```

### 15.5.2 Physical observations

The original 0.06 m switch travel never reaches the manual's 6 in threshold
from its 0.2286 m level surface. Switch travel becomes 0.09 m and scale travel
0.12 m; the selected rate is 0.04 m/s. These are actuator approximations.
The surface crosses the switch threshold after 1.905 s of uninterrupted
downward travel and the scale threshold after 2.54 s, before acquisition.

Listing 15.4. Revised travel that reaches the ownership heights. <code>seasons\2027\game\src\plates.cpp</code>.

<!-- book-file: seasons\2027\game\src\plates.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {

void Plates::update(double dt) {
    for (Plate& plate : plates_) {
        overlaps_.clear();
        world_.shapesOverlapping(plate.sensor, overlaps_);
        plate.cubes = 0;
        const auto center = world_.pose(plate.body).position;
        for (const s::ShapeId shape : overlaps_) {
            const auto tag = world_.shapeTag(shape);
            if (tag < kCubeTag || tag >= kCubeTag + kCubeCount) continue;
            const auto p = world_.pose(world_.shapeBody(shape)).position;
            if (std::abs(p.x - center.x) <= kPlateHalfM.x &&
                std::abs(p.y - center.y) <= kPlateHalfM.y &&
                p.z >= center.z + kPlateHalfM.z + kCubeHalfM.z - 0.05f &&
                s::length(world_.linearVelocity(world_.shapeBody(shape))) < 0.4f)
                ++plate.cubes;
        }
    }

    for (std::size_t i = 0; i < plates_.size(); ++i) {
        Plate& plate = plates_[i];
        const int delta = plate.cubes - plates_[i ^ 1].cubes;
        const float travel =
            i / 2 == 2 ? 0.12f : 0.09f;  // [approx] reaches the manual's ownership heights.
        const float offset = delta > 0 ? -travel : delta < 0 ? travel : 0.f;
        const auto position = world_.pose(plate.body).position;
        const float target = plate.levelCenter.z + offset;
        const float dz = std::clamp(target - position.z, -0.04f * static_cast<float>(dt),
                                    0.04f * static_cast<float>(dt));
        world_.setLinearVelocity(plate.body, {0.f, 0.f, dz / static_cast<float>(dt)});
    }
}

}  // namespace wpiemu::plugins::game_2027
```

The adapter measures actual plate poses and submits their qualifying states.
Auto-run uses the leading extent of the rotated chassis box at the auto line.
The configured chassis box stands in for bumper geometry, as classified in F-10.

Listing 15.5. Physical observations and rule integration. <code>seasons\2027\game\src\game_scoring.cpp</code>.

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
    const auto status = scoring_.advance(us, rules, observedOwnership());
    if (status != RulesStatus::Accepted) ctx.logger().error(kId, "Ownership interval rejected");
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
        scores_[a] = static_cast<double>(scoring_.snapshot().microPoints[a] / kRulesSecondUs);
}

}  // namespace wpiemu::plugins::game_2027
```

### 15.5.3 Lifecycle

The state now owns a scoring module. Reset clears it; the first transition out
of autonomous captures Auto Quest once. Tick orchestration delegates the
accounting instead of retaining Chapter 12's count-based scoring loop.

Listing 15.6. State declaration with the scoring ledger. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_layout.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/cube_zones.h"
#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/plate.h"
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {
class State final : public s::IGameState {
public:
    State(s::SimContext& ctx, std::string assignment, bool preload);

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
    s::IPhysicsWorld& world_;
    CubePool pool_;
    Plates plates_;
    CubeZones zones_;
    OwnershipScoring scoring_;
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

Listing 15.7. Reset of ownership and acquisition history. <code>seasons\2027\game\src\game_reset.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_reset.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::reset(s::SimContext& ctx) {
    scores_ = {};
    scoring_.reset();
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

Listing 15.8. Phase transitions and the scoring adapter. <code>seasons\2027\game\src\game_tick.cpp</code>.

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
    if (running && (autonomous || teleop)) {
        score(ctx, dt, autonomous);
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 15.9. Library sources through Chapter 15. <code>seasons\2027\game\CMakeLists.txt</code>.

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
    src/game_scoring.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 15.6 Verification

The pure tests exercise exact microsecond boundaries, interrupted acquisition,
phase changes, frozen intervals and subdivision invariance. Auto-run tests
independently check one award per robot and Auto Quest capture.

Listing 15.10. Ownership and accounting boundary cases. <code>tests\game_2027\test_rules_ownership.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_ownership.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/ownership_scoring.h"

using namespace wpiemu::plugins::game_2027;
namespace {
constexpr RulesContext autoContext{RulesPhase::Auto};
constexpr RulesContext teleopContext{RulesPhase::Teleop};
constexpr RulesTimeUs second = kRulesSecondUs;
}  // namespace

TEST_CASE("rules ownership requires one uninterrupted second, including exact boundary") {
    OwnershipScoring scoring;
    PlateOwnership plates{{true, false}, std::nullopt};
    CHECK(scoring.advance(second - 1, autoContext, plates) == RulesStatus::Accepted);
    CHECK_FALSE(scoring.snapshot().owned[0][0]);
    CHECK(scoring.snapshot().microPoints[0] == 0);
    CHECK(scoring.advance(1, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().owned[0][0]);
    CHECK(scoring.snapshot().microPoints[0] == 2 * second);
    CHECK(scoring.advance(5 * second, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 12 * second);
}

TEST_CASE("rules interrupted acquisition and lost ownership restart qualification") {
    OwnershipScoring scoring;
    PlateOwnership plates{{true, false}, std::nullopt};
    CHECK(scoring.advance(900'000, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.advance(1, autoContext, {}) == RulesStatus::Accepted);
    CHECK(scoring.advance(999'999, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 0);
    CHECK(scoring.advance(1, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.advance(second, autoContext, {}) == RulesStatus::Accepted);
    CHECK_FALSE(scoring.snapshot().owned[0][0]);
    CHECK(scoring.advance(second, teleopContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 3 * second);
}

TEST_CASE("rules phase transition preserves precision and never grants another bonus") {
    OwnershipScoring scoring;
    const PlateOwnership plates{{true, false}, std::nullopt};
    CHECK(scoring.advance(1'250'001, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 2'500'002);
    CHECK(scoring.captureAutoEnd() == RulesStatus::Accepted);
    CHECK(scoring.advance(749'999, teleopContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 3'250'001);
}

TEST_CASE("rules phase transition during acquisition awards only teleop establishment") {
    OwnershipScoring scoring;
    const PlateOwnership plates{{true, false}, std::nullopt};
    CHECK(scoring.advance(600'000, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.advance(400'000, teleopContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == second);
}

TEST_CASE("rules disabled estop and zero duration freeze acquisition and ownership") {
    OwnershipScoring scoring;
    const PlateOwnership plates{{true, false}, std::nullopt};
    CHECK(scoring.advance(500'000, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.advance(second, {RulesPhase::Auto, false}, {}) == RulesStatus::Frozen);
    CHECK(scoring.advance(second, {RulesPhase::Auto, true, true}, {}) == RulesStatus::Frozen);
    CHECK(scoring.advance(500'000, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.advance(0, autoContext, {}) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().owned[0][0]);
    CHECK(scoring.advance(second, {RulesPhase::Auto, false}, {}) == RulesStatus::Frozen);
    CHECK(scoring.snapshot().microPoints[0] == 2 * second);
}

TEST_CASE("rules two switches accrue independently but scale changes exclusive owner") {
    OwnershipScoring scoring;
    PlateOwnership plates{{true, true}, 0};
    CHECK(scoring.advance(2 * second, autoContext, plates) == RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 8 * second);
    CHECK(scoring.snapshot().microPoints[1] == 4 * second);
    plates.scaleDown = 1;
    CHECK(scoring.advance(second, teleopContext, plates) == RulesStatus::Accepted);
    CHECK_FALSE(scoring.snapshot().owned[0][1]);
    CHECK(scoring.snapshot().owned[1][1]);
    CHECK(scoring.snapshot().microPoints[0] == 9 * second);
    CHECK(scoring.snapshot().microPoints[1] == 6 * second);
}

TEST_CASE("rules acquisition is invariant under microsecond tick subdivision") {
    OwnershipScoring whole, split;
    const PlateOwnership plates{{true, true}, 1};
    CHECK(whole.advance(2'345'678, autoContext, plates) == RulesStatus::Accepted);
    for (const RulesTimeUs step : {123'456, 876'543, 1, 999'999, 345'679})
        CHECK(split.advance(step, autoContext, plates) == RulesStatus::Accepted);
    CHECK(whole.snapshot().microPoints == split.snapshot().microPoints);
    CHECK(whole.snapshot().owned == split.snapshot().owned);
}

TEST_CASE("rules invalid ownership intervals cannot mutate points") {
    OwnershipScoring scoring;
    CHECK(scoring.advance(-1, autoContext, {}) == RulesStatus::InvalidDuration);
    CHECK(scoring.advance(second, {}, {}) == RulesStatus::WrongPhase);
    CHECK(scoring.advance(second, {RulesPhase::Finished}, {}) == RulesStatus::WrongPhase);
    CHECK(scoring.advance(second, autoContext, {{}, 2}) == RulesStatus::InvalidAlliance);
    CHECK(scoring.snapshot().microPoints == std::array<std::int64_t, 2>{});
}
```

Listing 15.11. Auto-run and Auto Quest rules. <code>tests\game_2027\test_rules_auto.cpp</code>.

<!-- book-file: tests\game_2027\test_rules_auto.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/ownership_scoring.h"

using namespace wpiemu::plugins::game_2027;

TEST_CASE("rules auto runs award five once per robot with typed rejection") {
    OwnershipScoring scoring;
    const RulesContext context{RulesPhase::Auto};
    CHECK(scoring.awardAutoRun(2, 0, context) == RulesStatus::InvalidAlliance);
    CHECK(scoring.awardAutoRun(0, 3, context) == RulesStatus::InvalidRobot);
    CHECK(scoring.awardAutoRun(0, 0, {RulesPhase::Teleop}) == RulesStatus::WrongPhase);
    CHECK(scoring.awardAutoRun(0, 0, {RulesPhase::Auto, false}) == RulesStatus::Frozen);
    CHECK(scoring.awardAutoRun(0, 0, {RulesPhase::Auto, true, true}) == RulesStatus::Frozen);
    CHECK(scoring.awardAutoRun(0, 0, context) == RulesStatus::Accepted);
    CHECK(scoring.awardAutoRun(0, 0, context) == RulesStatus::AlreadyAwarded);
    CHECK(scoring.snapshot().microPoints[0] == 5 * kRulesSecondUs);
    CHECK(scoring.snapshot().microPoints[1] == 0);
}

TEST_CASE("rules Auto Quest latches three auto runs and own switch at auto end only") {
    OwnershipScoring scoring;
    const RulesContext context{RulesPhase::Auto};
    for (std::size_t robot = 0; robot < 3; ++robot) {
        CHECK(scoring.awardAutoRun(0, robot, context) == RulesStatus::Accepted);
        CHECK(scoring.awardAutoRun(1, robot, context) == RulesStatus::Accepted);
    }
    CHECK(scoring.advance(kRulesSecondUs, context, {{true, false}, 1}) == RulesStatus::Accepted);
    CHECK(scoring.captureAutoEnd() == RulesStatus::Accepted);
    CHECK(scoring.snapshot().autoQuest[0]);
    CHECK_FALSE(scoring.snapshot().autoQuest[1]);
    CHECK(scoring.captureAutoEnd() == RulesStatus::AlreadyAwarded);
    CHECK(scoring.advance(kRulesSecondUs, {RulesPhase::Teleop}, {{false, true}, {}}) ==
          RulesStatus::Accepted);
    CHECK(scoring.snapshot().autoQuest[0]);
    CHECK_FALSE(scoring.snapshot().autoQuest[1]);
    CHECK(scoring.awardAutoRun(1, 2, context) == RulesStatus::WrongPhase);
}

TEST_CASE("rules Auto Quest rejects missing auto run and last moment lost switch") {
    OwnershipScoring scoring;
    const RulesContext context{RulesPhase::Auto};
    for (std::size_t robot = 0; robot < 3; ++robot)
        CHECK(scoring.awardAutoRun(1, robot, context) == RulesStatus::Accepted);
    CHECK(scoring.awardAutoRun(0, 0, context) == RulesStatus::Accepted);
    CHECK(scoring.advance(kRulesSecondUs, context, {{true, true}, {}}) == RulesStatus::Accepted);
    CHECK(scoring.advance(1, context, {{true, false}, {}}) == RulesStatus::Accepted);
    CHECK(scoring.captureAutoEnd() == RulesStatus::Accepted);
    CHECK_FALSE(scoring.snapshot().autoQuest[0]);
    CHECK_FALSE(scoring.snapshot().autoQuest[1]);
}

TEST_CASE("rules scoring reset clears accumulated ownership runs and quest") {
    OwnershipScoring scoring;
    CHECK(scoring.awardAutoRun(0, 0, {RulesPhase::Auto}) == RulesStatus::Accepted);
    CHECK(scoring.advance(kRulesSecondUs, {RulesPhase::Auto}, {{true, true}, 0}) ==
          RulesStatus::Accepted);
    CHECK(scoring.captureAutoEnd() == RulesStatus::Accepted);
    scoring.reset();
    CHECK(scoring.snapshot().microPoints[0] == 0);
    CHECK_FALSE(scoring.snapshot().autoRuns[0][0]);
    CHECK_FALSE(scoring.snapshot().autoEndCaptured);
    CHECK_FALSE(scoring.snapshot().owned[0][0]);
    CHECK(scoring.advance(kRulesSecondUs - 1, {RulesPhase::Auto}, {{true, true}, 0}) ==
          RulesStatus::Accepted);
    CHECK(scoring.snapshot().microPoints[0] == 0);
}
```

Physical tests retain delivery as the input. Their settling interval increases
because plate travel and acquisition now take longer. The expected phase rates
remain independent assertions.

Listing 15.12. Revised physical travel oracle. <code>tests\game_2027\test_plate_motion.cpp</code>.

<!-- book-file: tests\game_2027\test_plate_motion.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plates.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: a physical resting cube lowers a plate then reset levels it") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::Field field(*world);
    REQUIRE(field.build());
    pu::Plates plates(*world);
    REQUIRE(plates.build());
    plates.reset();
    pu::Bodies bodies(*world);
    const auto initial = plates.center(0);
    const auto cube = bodies.box({initial + s::Vec3{0.f, 0.f, 0.5f}, {}}, pu::kCubeHalfM,
                                 s::BodyType::Dynamic, s::layer::Fuel, pu::kCubeTag);
    REQUIRE(cube);
    for (int i = 0; i < 1200; ++i) {
        world->step(0.005f);
        plates.update(0.005);
    }
    CHECK(plates.values()[0].cubes == 1);
    CHECK(plates.center(0).z == doctest::Approx(initial.z - 0.09f).epsilon(0.01));
    CHECK(plates.center(1).z > initial.z);
    plates.reset();
    CHECK(plates.center(0) == initial);
    CHECK(plates.values()[0].cubes == 0);
}
```

Listing 15.13. Delivery with the refined ownership rule. <code>tests\game_2027\test_plate_scoring.cpp</code>.

<!-- book-file: tests\game_2027\test_plate_scoring.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: real cube settles on each alliance switch and scores at period rates") {
    for (const std::string assignment : {"LLL", "LRL", "RLR", "RRR"}) {
        for (const bool red : {false, true}) {
            CAPTURE(assignment);
            CAPTURE(red);
            Match match(assignment, red);
            CHECK(match.game->gameSpecificMessage() == assignment);
            const auto bodyBudget = match.world->stats().bodies;
            match.services.driverStation.enabled = true;
            match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
            match.services.driverStation.matchTimeS = 15.0;
            match.advance(1000);
            const auto* key = red ? "game_2027.red_score" : "game_2027.blue_score";
            CHECK(match.hud("game_2027.released") == 1);
            CHECK(match.hud("game_2027.inventory") == 0);
            REQUIRE(match.hud(key) > 1.0);
            const double autoScore = match.hud(key);
            match.advance(400);
            CHECK(match.hud(key) - autoScore == doctest::Approx(4.0));

            match.services.driverStation.mode = s::proto::RobotMode::Teleop;
            match.services.driverStation.matchTimeS = 30.0;
            const double teleopStart = match.hud(key);
            match.advance(2000);
            CHECK(match.hud(key) - teleopStart == doctest::Approx(10.0));
            CHECK(match.hud("game_2027.endgame_seconds") == doctest::Approx(10.0));
            CHECK(match.hud(red ? "game_2027.blue_score" : "game_2027.red_score") == 0);
            const double finalScore = match.hud(key);
            match.services.driverStation.enabled = false;
            match.advance(400);
            CHECK(match.hud(key) == finalScore);
            match.services.driverStation.enabled = true;
            match.services.driverStation.matchTimeS = 0.0;
            match.advance(400);
            CHECK(match.hud(key) == finalScore);
            match.services.driverStation.matchTimeS = 10.0;
            match.services.driverStation.eStop = true;
            match.advance(400);
            CHECK(match.hud(key) == finalScore);

            CHECK(match.world->stats().bodies == bodyBudget);
            match.game->reset(match.ctx);
            CHECK(match.hud(key) == 0);
            CHECK(match.hud("game_2027.inventory") == 1);
            CHECK(match.hud("game_2027.released") == 0);
            CHECK(match.hud("game_2027.auto_seconds") == 0);
            CHECK(match.hud("game_2027.teleop_seconds") == 0);
            CHECK(match.snapshot()->entityGroups()[0].positions.size() == 32);
            CHECK(match.hud(red ? "game_2027.portal_red" : "game_2027.portal_blue") == 13);
            match.game.reset();
            match.field.reset();
            match.world->destroyBody(match.ctx.chassisBody());
            CHECK(match.world->stats().bodies == 0);
        }
    }
}
```

## 15.7 Reproduction

Extract through Chapter 15, rebuild and run both groups:

```powershell
ctest --test-dir build\book -R "Power UP:" --no-tests=error --output-on-failure
.\build\book\bin\wpiemu_tests.exe '--source-file=*test_rules*'
```

The native runner must report a nonzero test count. The pure tests deliberately
have names beginning with `rules`, rather than the component prefix.

## 15.8 Discussion

The physical threshold is sampled once per tick. The integer integrator can
split known time boundaries exactly, but cannot infer a plate crossing between
physics samples. F-08 and F-09 record these different approximation sources.

## 15.9 Summary

Physics supplies observations; a separate state machine owns qualification and
points. Integer accounting makes rounding and subdivision behavior explicit.
The implementation follows the manual's point values while documenting choices
the manual leaves unspecified.

## 15.10 Exercises

1. Calculate the micro-point total after 1.25 s of qualifying autonomous
   ownership, starting unqualified.
2. Explain why flooring each tick independently would violate Invariant 15.1.
3. Add an irregular subdivision to the existing invariance test.

[Chapter 16](16-human-players.md)
