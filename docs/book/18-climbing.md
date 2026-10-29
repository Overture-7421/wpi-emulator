# 18. Parking and climbing

## 18.1 Objectives

After this chapter the reader can use a field-anchored chassis constraint,
inspect its load, and define observable parking and climbing conditions.
Chapter 17 is the prerequisite. The climber and process scenarios address
F-17 and F-18.

## 18.2 Problem statement

A chassis raised above the platform is not necessarily supported by the scale.
Awarding a climb from height alone would count a falling or jumping robot.
A physical model needs both a geometric condition and evidence of support.

## 18.3 Background and definitions

The manual awards 5 points for parking and 30 for climbing. The robot must
satisfy support and location conditions at the finish. The 12 in brick height
is measured above the platform surface, not above the carpet [1, sections 3.3
and 4.2]. Levitate cannot add a fourth climb or preserve a second parking award
for the same credited robot.

**Definition 18.1 (modeled climb).** All eight chassis-box corners project
inside the own-platform footprint; the box bottom is above the platform plus
0.3048 m; and an engaged anchored cable reports upward support above 1 N.

The chassis box stands in for bumper geometry. Direct and transitive
multi-robot support in the manual is broader than Definition 18.1; F-18
records that distinction.

## 18.4 Design

A pose-only rule is easy to test but cannot distinguish suspension from flight.
The SDK instead supplies field-anchored distance constraints through chassis
coupling. A physical spike confirmed that a shortened cable can lift the
50 kg fixture and that an insufficient load limit breaks support.

The selected model captures a hook within 0.5 m of an anchor and shortens the
cable at 0.20 m/s to a minimum 0.10 m. These are chosen actuator parameters,
not dimensions specified by the game. It is a winch approximation rather than
a motor, gearbox and elastic rope model.

**Invariant 18.1 (support).** An unsupported or overloaded cable cannot produce
a positive modeled-climb result. Reset removes the owned constraint.

## 18.5 Implementation

### 18.5.1 Constraint and finish observation

The climber checks backend capabilities before creating a constraint. It reads
the previous completed physics step's load before replacing the shortened
constraint. Unsupported coupling warns once; overload latches failure until
deactivation or reset.

The adapter also samples support before replacing the handle. A newly submitted
constraint has no completed-step load; querying it immediately would report no
support even while the previous cable has physically hoisted the chassis.

Listing 18.1. Cable state and endgame observation interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\climb.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\climb.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {

class Climb {
public:
    void reset(s::IMechanismPhysicsCoupling& coupling);
    void update(s::SimContext& ctx, s::Vec3 localHook, bool active, double dt, std::size_t alliance,
                float maxForceN);
    bool supported(s::IMechanismPhysicsCoupling& coupling) const;
    bool failed() const { return failed_; }

private:
    s::ConstraintHandle handle_;
    s::AnchorConstraintDesc cable_;
    bool failed_ = false;
    bool warned_ = false;
};

struct EndgamePose {
    bool parked = false;
    bool climbed = false;
};

EndgamePose endgamePose(const s::Pose3& chassis, const s::Vec3& size, const s::Vec3& velocity,
                        std::size_t alliance, bool supported);

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.2. Load-limited hoisting and geometric finish conditions. <code>seasons\2027\game\src\climb.cpp</code>.

<!-- book-file: seasons\2027\game\src\climb.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/climb.h"

namespace wpiemu::plugins::game_2027 {

void Climb::reset(s::IMechanismPhysicsCoupling& coupling) {
    if (handle_.valid()) coupling.removeConstraint(handle_);
    handle_ = {};
    failed_ = warned_ = false;
}

bool Climb::supported(s::IMechanismPhysicsCoupling& coupling) const {
    const auto load = coupling.constraintLoad(handle_);
    return load.valid && load.engaged && load.force.z > 1.f;
}

void Climb::update(s::SimContext& ctx, s::Vec3 localHook, bool active, double dt,
                   std::size_t alliance, float maxForceN) {
    auto& coupling = ctx.coupling();
    if (!active) {
        reset(coupling);
        return;
    }
    if (!coupling.capabilities().constraints) {
        if (!warned_) ctx.logger().warn(kId, "Climbing requires chassis constraints");
        warned_ = true;
        return;
    }
    if (failed_ || !ctx.chassisBody().valid()) return;
    const auto load = coupling.constraintLoad(handle_);
    if (load.valid && !load.engaged) {
        failed_ = true;
        ctx.logger().warn(kId, "Climb cable exceeded its load limit");
        return;
    }
    if (!handle_.valid()) {
        cable_.kind = s::AnchorKind::Distance;
        cable_.owner = "game_2027.climb";
        cable_.fieldPoint = {alliance == 0 ? 6.90f : kLengthM - 6.90f, kWidthM / 2.f, 2.10f};
        cable_.chassisLocalPoint = localHook;
        const auto hook = ctx.physics().pose(ctx.chassisBody()).transformPoint(localHook);
        cable_.lengthM = s::length(hook - cable_.fieldPoint);
        if (cable_.lengthM > 0.5f) return;  // [approx] hook capture distance.
        cable_.maxForceN = maxForceN;
    } else {
        coupling.removeConstraint(handle_);
    }
    cable_.lengthM = std::max(0.10f, cable_.lengthM - 0.20f * static_cast<float>(dt));
    handle_ = coupling.addConstraint(cable_);
    if (!handle_.valid()) {
        failed_ = true;
        ctx.logger().warn(kId, "Climb constraint creation failed");
    }
}

EndgamePose endgamePose(const s::Pose3& chassis, const s::Vec3& size, const s::Vec3& velocity,
                        std::size_t alliance, bool supported) {
    float bottom = 1e6f;
    bool ownPlatform = true;
    const float x = alliance == 0 ? 6.90f : kLengthM - 6.90f;
    for (const float sx : {-0.5f, 0.5f})
        for (const float sy : {-0.5f, 0.5f})
            for (const float sz : {-0.5f, 0.5f}) {
                const auto p = chassis.transformPoint({sx * size.x, sy * size.y, sz * size.z});
                bottom = std::min(bottom, p.z);
                ownPlatform &=
                    std::abs(p.x - x) <= 0.525f && std::abs(p.y - kWidthM / 2.f) <= 1.32f;
            }
    const bool climbed =
        ownPlatform && supported && bottom > 0.09f + 0.3048f;  // [GM 3.3] 12 in above platform.
    const bool parked =
        ownPlatform && std::abs(bottom - 0.09f) < 0.03f && s::length(velocity) < 0.1f;
    return {parked && !climbed, climbed};
}

}  // namespace wpiemu::plugins::game_2027
```

Parking uses the same footprint and a bottom within 0.03 m of platform height,
with linear speed below 0.1 m/s. These operational tolerances are tested model
parameters. They do not prove every official contact condition.

### 18.5.2 Zone activation

The hook is a zone whose activation can observe a mapped mechanism. The common
attachment and activation calculation is exposed for both cube and climb
adapters rather than copied.

Listing 18.3. Shared active local-pose query. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_zones.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_zones.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_pool.h"

namespace wpiemu::plugins::game_2027 {

class CubeZones {
public:
    CubeZones(CubePool& pool, s::IPhysicsWorld& world) : pool_(pool), world_(world) {}
    void reset() { missingWarnings_.clear(); }
    void update(s::SimContext& ctx);
    std::optional<s::Pose3> activePose(s::SimContext& ctx, const s::ZoneConfig& zone);

private:
    void warnMissing(s::SimContext& ctx, const std::string& id);
    CubePool& pool_;
    s::IPhysicsWorld& world_;
    std::vector<std::string> missingWarnings_;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.4. Shared attachment resolution and cube actions. <code>seasons\2027\game\src\cube_zones.cpp</code>.

<!-- book-file: seasons\2027\game\src\cube_zones.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/cube_zones.h"

namespace wpiemu::plugins::game_2027 {
void CubeZones::warnMissing(s::SimContext& ctx, const std::string& id) {
    if (std::find(missingWarnings_.begin(), missingWarnings_.end(), id) != missingWarnings_.end())
        return;
    missingWarnings_.push_back(id);
    ctx.logger().warn("game_2027",
                      "Power UP zone has an unresolved attachment or mechanism: " + id);
}

std::optional<s::Pose3> CubeZones::activePose(s::SimContext& ctx, const s::ZoneConfig& zone) {
    s::Pose3 attachment;
    if (!zone.attachment.onChassis()) {
        const auto pose = ctx.mechanisms() == nullptr
                              ? std::optional<s::Pose3>{}
                              : ctx.mechanisms()->linkPoseInChassis(
                                    zone.attachment.parentMechanismId, zone.attachment.parentLink);
        if (!pose) {
            warnMissing(ctx, zone.attachment.parentMechanismId);
            return std::nullopt;
        }
        attachment = *pose;
    }
    std::vector<std::string> missing;
    const bool active = s::evaluateActivation(zone.activation, ctx.mechanisms(), &missing);
    for (const auto& id : missing) warnMissing(ctx, id);
    if (!active) return std::nullopt;
    return attachment * zone.local;
}

void CubeZones::update(s::SimContext& ctx) {
    if (!ctx.chassisBody().valid()) return;
    const auto chassis = world_.pose(ctx.chassisBody());
    for (const auto& zone : ctx.config().zones) {
        if (zone.kind != kPickup && zone.kind != kRelease) continue;
        const auto local = activePose(ctx, zone);
        if (!local) continue;
        const auto pose = chassis * *local;
        if (zone.kind == kRelease) {
            const auto velocity = world_.pointVelocity(ctx.chassisBody(), pose.position) +
                                  pose.transformVector({0.2f, 0.f, 0.f});
            pool_.release(pose, velocity);
        } else {
            pool_.pickup(pose, zone.sizeM * 0.5f);
        }
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.5. Activated hook and physical finish observation. <code>seasons\2027\game\src\game_climb.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_climb.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

void State::updateClimb(s::SimContext& ctx, double dt) {
    const auto alliance =
        s::proto::allianceOf(ctx.driverStation().station) == s::Alliance::Red ? 1U : 0U;
    bool active = false;
    s::Vec3 hook;
    for (const auto& zone : ctx.config().zones) {
        if (zone.kind != "game_2027.climb") continue;
        const auto local = zones_.activePose(ctx, zone);
        if (!local) continue;
        hook = local->position;
        active = true;
        break;
    }
    const bool supported = climb_.supported(ctx.coupling());
    climb_.update(ctx, hook, active, dt, alliance, static_cast<float>(config_.climbMaxForceN));
    if (ctx.chassisBody().valid())
        endgame_ = endgamePose(world_.pose(ctx.chassisBody()), ctx.config().chassis.sizeM,
                               world_.linearVelocity(ctx.chassisBody()), alliance, supported);
}

}  // namespace wpiemu::plugins::game_2027
```

The module declares the hook zone, and configuration supplies its load limit.
No custom mechanism is needed: the generic motor provides an activation value,
while the plugin owns the cable.

Listing 18.6. Zone descriptors including the climber hook. <code>seasons\2027\game\src\game_ui.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_ui.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {
std::vector<s::HudWidgetDescriptor> Module::contributeHud() const {
    s::HudWidgetDescriptor scores;
    scores.id = "game_2027.scoreboard";
    scores.title = "POWER UP";
    scores.kind = s::HudWidgetKind::Scoreboard;
    scores.anchor = s::HudAnchor::TopCenter;
    scores.valueKey = "game_2027.phase";
    scores.sides = {{"BLUE", "game_2027.blue_score", "", "#2457c5"},
                    {"RED", "game_2027.red_score", "", "#c52c30"}};

    s::HudWidgetDescriptor inventory;
    inventory.id = "game_2027.inventory";
    inventory.title = "Held cube";
    inventory.valueKey = inventory.id;
    s::HudWidgetDescriptor assignment;
    assignment.id = "game_2027.game_data";
    assignment.title = "FMS";
    assignment.kind = s::HudWidgetKind::Text;
    assignment.valueKey = assignment.id;

    s::HudWidgetDescriptor blueSupply;
    blueSupply.id = "game_2027.portal_blue";
    blueSupply.title = "Blue portal supply";
    blueSupply.valueKey = blueSupply.id;
    s::HudWidgetDescriptor redSupply;
    redSupply.id = "game_2027.portal_red";
    redSupply.title = "Red portal supply";
    redSupply.valueKey = redSupply.id;
    return {scores, inventory, assignment, blueSupply, redSupply};
}

std::vector<s::ZoneKindInfo> Module::contributeZoneKinds() const {
    s::ZoneKindInfo pickup;
    pickup.kind = kPickup;
    pickup.displayName = "Cube pickup";
    pickup.shape = s::ZoneShape::Volume;
    pickup.defaultLocal.position = {0.5f, 0.f, 0.f};
    pickup.defaultSizeM = {0.5f, 0.6f, 0.5f};
    s::ZoneKindInfo release;
    release.kind = kRelease;
    release.displayName = "Cube release";
    release.shape = s::ZoneShape::Muzzle;
    release.defaultLocal.position = {0.75f, 0.f, 0.75f};
    s::ZoneKindInfo climb;
    climb.kind = "game_2027.climb";
    climb.displayName = "Climber hook";
    climb.shape = s::ZoneShape::Muzzle;
    climb.defaultLocal.position = {0.f, 0.f, 1.5f};
    return {pickup, release, climb};
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.7. Cable load-limit setting. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\config.h</code>.

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
};

bool validAssignment(const std::string& value);
s::PluginConfigSchema configSchema();
GameConfig parseConfig(const s::Value& data);

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.8. Load-limit binding and validation range. <code>seasons\2027\game\src\config.cpp</code>.

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

### 18.5.3 State integration

The state retains the latest enabled finish observation. The next chapter
uses it for final awards rather than observing a chassis that has already
fallen after disable.

Listing 18.9. Climber ownership and cached finish observation. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

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

Listing 18.10. Constraint removal and finish reset. <code>seasons\2027\game\src\game_reset.cpp</code>.

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

Listing 18.11. Climb updates in running teleoperation. <code>seasons\2027\game\src\game_tick.cpp</code>.

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
        updateClimb(ctx, dt);
    }
    if (running && (autonomous || teleop)) {
        score(ctx, dt, autonomous);
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.12. Published support result and failure flag. <code>seasons\2027\game\src\game_rule_snapshot.cpp</code>.

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
    out.setHud("game_2027.parked", endgame_.parked);
    out.setHud("game_2027.climbed", endgame_.climbed);
    out.setHud("game_2027.climb_failed", climb_.failed());
    out.setHud(
        "game_2027.power_seconds",
        power.active ? static_cast<double>(power.active->remainingUs) / kRulesSecondUs : 0.0);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 18.13. Library sources through Chapter 18. <code>seasons\2027\game\CMakeLists.txt</code>.

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
    src/game_climb.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

### 18.5.4 Ordinary robot outputs

The revised demonstration programs retain autonomous delivery. In teleoperation
they stop drive motors and energize the roller and a fourth PWM motor.
PWM 3 is mapped to a generic winch only in the climb fixture; the ordinary
delivery model leaves it unmapped. This controlled routine is intended for
testing, not driver operation.

Listing 18.14. C++ program with teleoperation feature outputs. <code>samples\season_2027\power_up_cpp\src\main\cpp\Robot.cpp</code>.

<!-- book-file: samples\season_2027\power_up_cpp\src\main\cpp\Robot.cpp -->
```cpp
#include <iostream>
#include <wpi/drivers/motor/PWMSparkMax.hpp>
#include <wpi/driverstation/MatchState.hpp>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/framework/TimedRobot.hpp>

class Robot final : public wpi::TimedRobot {
public:
    void AutonomousInit() override {
        stop();
        const auto gameData = wpi::MatchState::GetGameData();
        validAssignment_ = gameData && (*gameData == "LLL" || *gameData == "LRL" ||
                                        *gameData == "RLR" || *gameData == "RRR");
        autoTicks_ = 0;
        std::cout << "POWER_UP auto " << gameData.value_or("") << std::endl;
    }

    void AutonomousPeriodic() override {
        ++autoTicks_;
        const double drive = validAssignment_ && autoTicks_ < 300 ? 0.25 : 0.0;
        left_.SetThrottle(drive);
        right_.SetThrottle(drive);
        roller_.SetThrottle(validAssignment_ && autoTicks_ >= 350 && autoTicks_ < 400 ? 0.4 : 0.0);
    }

    void TeleopInit() override {
        stop();
        std::cout << "POWER_UP teleop" << std::endl;
    }
    void TeleopPeriodic() override {
        left_.StopMotor();
        right_.StopMotor();
        roller_.SetThrottle(0.4);
        winch_.SetThrottle(0.4);
    }
    void DisabledInit() override { stop(); }

private:
    void stop() {
        left_.StopMotor();
        right_.StopMotor();
        roller_.StopMotor();
        winch_.StopMotor();
    }

    wpi::PWMSparkMax left_{0};
    wpi::PWMSparkMax right_{1};
    wpi::PWMSparkMax roller_{2};
    wpi::PWMSparkMax winch_{3};
    int autoTicks_ = 0;
    bool validAssignment_ = false;
};

int main() {
    return wpi::StartRobot<Robot>();
}
```

Listing 18.15. Equivalent Java feature outputs. <code>samples\season_2027\power_up_java\src\main\java\frc\robot\Main.java</code>.

<!-- book-file: samples\season_2027\power_up_java\src\main\java\frc\robot\Main.java -->
```java
package frc.robot;

import org.wpilib.driverstation.MatchState;
import org.wpilib.drivers.motor.PWMSparkMax;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

    public static final class Robot extends TimedRobot {
        private final PWMSparkMax left = new PWMSparkMax(0);
        private final PWMSparkMax right = new PWMSparkMax(1);
        private final PWMSparkMax roller = new PWMSparkMax(2);
        private final PWMSparkMax winch = new PWMSparkMax(3);
        private int autoTicks;
        private boolean validAssignment;

        @Override
        public void autonomousInit() {
            stop();
            String gameData = MatchState.getGameData().orElse("");
            validAssignment = switch (gameData) {
                case "LLL", "LRL", "RLR", "RRR" -> true;
                default -> false;
            };
            autoTicks = 0;
            System.out.printf("POWER_UP auto %s%n", gameData);
        }

        @Override
        public void autonomousPeriodic() {
            ++autoTicks;
            double drive = validAssignment && autoTicks < 300 ? 0.25 : 0.0;
            left.setThrottle(drive);
            right.setThrottle(drive);
            roller.setThrottle(validAssignment && autoTicks >= 350 && autoTicks < 400 ? 0.4 : 0.0);
        }

        @Override
        public void teleopInit() {
            stop();
            System.out.println("POWER_UP teleop");
        }

        @Override
        public void teleopPeriodic() {
            left.stopMotor();
            right.stopMotor();
            roller.setThrottle(0.4);
            winch.setThrottle(0.4);
        }

        @Override
        public void disabledInit() {
            stop();
        }

        private void stop() {
            left.stopMotor();
            right.stopMotor();
            roller.stopMotor();
            winch.stopMotor();
        }
    }
}
```

## 18.6 Verification

The physical test hoists a dynamic 50 kg chassis, checks its support load and
height, resets the constraint, and repeats with a breaking load limit.
It also rejects an airborne chassis lacking support.

Listing 18.16. Suspension, overload, parking and unsupported-backend cases. <code>tests\game_2027\test_climb.cpp</code>.

<!-- book-file: tests\game_2027\test_climb.cpp -->
```cpp
#include "match_fixture.h"
#include "wpiemu/core/physics/chassis_coupling.h"
#include "wpiemu/plugins/game_2027/climb.h"

using namespace power_up_test;
namespace pu = wpiemu::plugins::game_2027;

TEST_CASE("Power UP: real anchored cable hoists and supports a chassis") {
    Match match("LRL");
    match.world->destroyBody(match.ctx.chassisBody());
    s::BodyDesc bd;
    bd.type = s::BodyType::Dynamic;
    bd.pose.position = {6.90f, pu::kWidthM / 2.f, 0.20f};
    const auto body = match.world->createBody(bd);
    s::ShapeDesc shape;
    shape.geometry = s::BoxShape{{0.35f, 0.35f, 0.10f}};
    shape.category = s::layer::Robot;
    shape.mask = s::layer::Field;
    REQUIRE(match.world->addShape(body, shape).valid());
    const s::MassProperties mass{50.f, {}, {2.f, 2.f, 2.f}};
    match.world->setMassProperties(body, mass);
    match.ctx.bindChassisBody(body);
    wpiemu::core::physics::ChassisPhysicsCoupling coupling(*match.world, body, mass);
    match.ctx.bindCoupling(&coupling);
    pu::Climb climb;
    for (int i = 0; i < 600; ++i) {
        climb.update(match.ctx, {0.f, 0.f, 1.5f}, true, 0.005, 0, 4000.f);
        match.world->step(0.005f);
        coupling.onStepCompleted();
    }
    CHECK_FALSE(climb.failed());
    CHECK(climb.supported(coupling));
    const auto pose = match.world->pose(body);
    CAPTURE(pose.position.z);
    CHECK(pose.position.z > 0.45f);
    const auto result = pu::endgamePose(pose, {0.7f, 0.7f, 0.2f}, match.world->linearVelocity(body),
                                        0, climb.supported(coupling));
    CHECK(result.climbed);
    CHECK_FALSE(result.parked);
    climb.reset(coupling);
    CHECK(coupling.constraintCount() == 0);
    for (int i = 0; i < 10; ++i) {
        climb.update(match.ctx, {0.f, 0.f, 1.5f}, true, 0.005, 0, 10.f);
        match.world->step(0.005f);
        coupling.onStepCompleted();
    }
    CHECK(climb.failed());
    CHECK_FALSE(climb.supported(coupling));
    climb.reset(coupling);
    match.ctx.bindCoupling(nullptr);
}

TEST_CASE("Power UP: parked geometry alone cannot claim a climb") {
    const s::Vec3 size{0.7f, 0.7f, 0.2f};
    const s::Pose3 parked{{6.9f, pu::kWidthM / 2.f, 0.19f}, {}};
    CHECK(pu::endgamePose(parked, size, {}, 0, false).parked);
    CHECK_FALSE(pu::endgamePose(parked, size, {}, 0, false).climbed);
    auto airborne = parked;
    airborne.position.z = 0.5f;
    CHECK_FALSE(pu::endgamePose(airborne, size, {}, 0, false).climbed);
    CHECK(pu::endgamePose(airborne, size, {}, 0, true).climbed);
    CHECK_FALSE(pu::endgamePose(airborne, size, {}, 1, true).climbed);
}

TEST_CASE("Power UP: unsupported climb coupling warns once and never claims support") {
    Match match("LRL");
    pu::Climb climb;
    for (int i = 0; i < 10; ++i) climb.update(match.ctx, {0.f, 0.f, 1.5f}, true, 0.005, 0, 4000.f);
    CHECK_FALSE(climb.supported(match.ctx.coupling()));
    CHECK(match.services.logger.count(s::LogLevel::Warn) == 1);
}
```

The real-HAL fixture gains a routine selector. The feature setup derives a
configuration from the saved model and validates it through the same pipeline.
The vault case places the release endpoint at the lower exchange; it tests
PWM-controlled custody transfer, not a collision trajectory through the wall.
The latter is covered separately by Chapter 16's physical exchange test.

Listing 18.17. Revised fixture interface for feature configurations. <code>app\host\tests\power_up_match.h</code>.

<!-- book-file: app\host\tests\power_up_match.h -->
```cpp
#pragma once

#include <functional>

#include "season_probe.h"
#include "test_support.h"
#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/extension_locator.h"
#include "wpiemu/sdk/testing/wait.h"

namespace power_up_match {

namespace s = wpiemu::sdk;
namespace host = wpiemu::app::host;
using namespace host::testing;
using namespace std::chrono_literals;

struct Match {
    Scratch scratch{"wpiemu-power-up-match-"};
    TestRegistries registries;
    std::unique_ptr<host::RobotSession> session;
    std::vector<std::string> console;
    s::proto::DsState ds;
    std::string assignment;

    explicit Match(const std::string& language, const std::string& routine = {});
    ~Match();
    bool pump(std::chrono::milliseconds timeout, const std::function<bool()>& predicate);
    void advance(int seconds);
    void period(s::proto::RobotMode mode, int seconds);
    std::shared_ptr<const s::ISimSnapshot> snapshot() const { return session->latestSnapshot(); }
    double number(const char* key) const;
    std::string text(const char* key) const;
    void reset();
};

void configureFeature(s::RobotConfig& config, const std::string& routine);

}  // namespace power_up_match
```

Listing 18.18. Validate the derived model before launching a robot. <code>app\host\tests\power_up_launch.cpp</code>.

<!-- book-file: app\host\tests\power_up_launch.cpp -->
```cpp
#include <algorithm>

#include "power_up_match.h"

namespace power_up_match {

Match::Match(const std::string& language, const std::string& routine) {
    assignment = language == "java" ? "LRL" : "RLR";
    const auto source = season::samples() / ("power_up_" + language);
    copyProject(source, scratch.root / "robot");
    wpiemu::core::config::FileConfigStore store(wpiemu::core::config::projectConfigPath(source),
                                                "Power UP textbook fixture", registries.json,
                                                registries.codec);
    const auto saved = store.load();
    REQUIRE_MESSAGE(saved.ok(), (saved.ok() ? "" : saved.error().message));
    auto config = saved.value();
    if (!routine.empty()) configureFeature(config, routine);
    const auto checked = registries.pipeline.load(registries.codec.toValue(config));
    REQUIRE_MESSAGE(checked.ok(), (checked.ok() ? "" : checked.error().message));
    REQUIRE(checked.value().valid());
    REQUIRE(checked.value().config.project.language == language);
    host::Options options;
    options.projectDir = scratch.root / "robot";
    options.scratchDir = scratch.root / "scratch";
    options.extensionLibrary = wpiemu::core::launcher::locateHalsimExtension(
        season::binaryDir(), &s::readEnvironment, season::year(), season::buildExtension());
    options.driverStation.dsAttached = options.driverStation.fmsAttached = true;
    options.driverStation.station = s::proto::AllianceStation::Blue1;
    options.driverStation.gameSpecificMessage = "NOT_PLUGIN_DATA";
    options.timeScale = 0.0;
    options.startPaused = true;
    options.handshakeTimeout = 8min;
    options.extraGradleArgs = {"--no-daemon"};
    if (!env("WPIEMU_E2E_JAVA_HOME").empty()) options.javaHome = env("WPIEMU_E2E_JAVA_HOME");
    auto started = host::RobotSession::start(registries.view(), checked.value().config, options);
    REQUIRE_MESSAGE(started.ok(), (started.ok() ? "" : started.error().message));
    session = std::move(started.value());
    ds = options.driverStation;
    REQUIRE(pump(8min, [&] {
        return session->status().programStarted && snapshot() && snapshot()->paused() &&
               session->runner().robotHello().has_value();
    }));
    REQUIRE_FALSE(session->coreOnly());
    CHECK(session->gameModule()->metadata().id == "game_2027");
    CHECK(snapshot()->driverStation().gameSpecificMessage == assignment);
    CHECK(session->runner().robotHello()->wpilibYear == season::year());
}

Match::~Match() {
    if (!session) return;
    const auto hello = session->runner().robotHello();
    session->requestStop();
    session->waitUntilFinished();
    if (hello) CHECK_FALSE(processAlive(hello->pid));
}

bool Match::pump(std::chrono::milliseconds timeout, const std::function<bool()>& predicate) {
    const bool ready = s::testing::waitUntil(
        [&] {
            drainConsole(*session, console, !env("WPIEMU_E2E_VERBOSE").empty());
            return predicate() || !session->status().active();
        },
        timeout, 10ms);
    if (!ready || !session->status().active()) dumpConsole(console);
    return ready && session->status().active() && predicate();
}

}  // namespace power_up_match
```

The climb setup maps PWM 3 and places the chassis under its anchor. Both
languages must activate the constraint and suspend the chassis. No pose change
or point award occurs after launch.

Listing 18.19. Real-HAL vault and anchored-climb scenarios. <code>app\host\tests\power_up_features.cpp</code>.

<!-- book-file: app\host\tests\power_up_features.cpp -->
```cpp
#include "power_up_match.h"

namespace power_up_match {

void configureFeature(s::RobotConfig& config, const std::string& routine) {
    auto& data = config.plugins["game_2027"].data;
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
        std::printf("POWER_UP FEATURE %s physical climb z=%.3f\n", language.c_str(),
                    match.snapshot()->chassis().pose.position.z);
    }
}

}  // namespace power_up_match
```

Listing 18.20. Compile the additional feature scenarios. <code>app\host\CMakeLists.txt</code>.

<!-- book-edit: app\host\CMakeLists.txt -->
```json
{
  "old": "        add_executable(wpiemu_power_up_e2e_tests\n            tests/e2e_power_up_test.cpp tests/power_up_launch.cpp tests/power_up_control.cpp)\n        add_dependencies(wpiemu_power_up_e2e_tests halsim_wpiemu)\n",
  "new": "        add_executable(wpiemu_power_up_e2e_tests\n            tests/e2e_power_up_test.cpp tests/power_up_launch.cpp tests/power_up_control.cpp\n            tests/power_up_features.cpp)\n        add_dependencies(wpiemu_power_up_e2e_tests halsim_wpiemu)\n"
}
```

## 18.7 Reproduction

Extract through Chapter 18, rebuild the unit and process-test executables,
and run the component group and opt-in match test. The additional process
cases build both revised robot programs automatically.

```powershell
ctest --test-dir build\book -R "Power UP:.*(cable|climb|parked)" --no-tests=error --output-on-failure
$env:WPIEMU_E2E = '1'
ctest --test-dir build\book -R "e2e.power_up_" --no-tests=error --output-on-failure
```

## 18.8 Discussion

The constraint submission follows physics, so its changed length affects the
next step. The support observation refers to the previous completed step.
This one-tick relationship is part of the coupling contract, not a reason to
query an unfinished physics calculation.

## 18.9 Summary

The model distinguishes supported suspension from height alone and reports
constraint failure. Physical and real-HAL tests exercise the selected cable
model. Final point and ranking calculations remain a separate rule operation.

## 18.10 Exercises

1. Calculate the weight of the 50 kg fixture and compare it with a 10 N cable.
2. Explain why the brick threshold is 0.3948 m above the carpet in this model.
3. Identify the observation that prevents an airborne robot receiving a climb.

[Chapter 19](19-scenarios.md)
