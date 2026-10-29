# 20. Sound, editor panels and the module contract

## 20.1 Objectives

After this chapter the reader can declare original sound assets, request
one-shot cues on rule transitions, and supply a generic editor panel without
Qt. Chapter 19 is the prerequisite. Presentation descriptors address R11 and F-23.

## 20.2 Problem statement

A snapshot can display a score continuously, but playing a sound on every tick
of ownership would produce hundreds of overlapping requests. An editor panel
also needs to reuse configuration definitions rather than invent a second
schema.

## 20.3 Background and definitions

**Definition 20.1 (edge-triggered cue).** A cue is edge-triggered when a state
transition requests it once, rather than requesting it throughout a persistent
condition.

The app owns audio playback, volume and Qt objects. The plugin declares cue
identities and requests them through the context. Common match-lifecycle cues
already belong to the host and are not duplicated here.

## 20.4 Design

Official game recordings would introduce asset provenance and redistribution
questions unrelated to implementation. A generated original tone provides a
reproducible demonstration instead. The same tone serves three declared cues;
different identities still permit later independent assets.

For editing, a custom widget would break the plugin boundary. An
`EditorPanelDescriptor` instead filters the existing Scenario property group.
The generic editor retains validation, persistence and undo behavior.

**Invariant 20.1 (descriptor consistency).** Panel properties come from the
module's schema, and every HUD value key is published by the game state.

## 20.5 Implementation

### 20.5.1 Descriptors and asset

The declaration gains editor-panel and sound-cue methods. Metadata describes
the completed case study rather than the earlier physical core.

Listing 20.1. Complete module declaration. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_module.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_module.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/config.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/plugin.h"

namespace wpiemu::plugins::game_2027 {
class Module final : public s::IGameModule {
public:
    s::GameMetadata metadata() const override;

    s::Status registerMechanisms(s::MechanismFactory&) override;

    s::Result<std::unique_ptr<s::IField>> createField(s::IPhysicsWorld& world) override;

    s::Result<std::unique_ptr<s::IGameState>> createGameState(s::SimContext& ctx) override;

    s::PluginConfigSchema contributeConfigSchema() const override;

    s::FieldFrame fieldFrame() const override;

    std::vector<s::ZoneKindInfo> contributeZoneKinds() const override;

    std::vector<s::GameValueInfo> contributeGameValues() const override;

    std::vector<s::HudWidgetDescriptor> contributeHud() const override;
    std::vector<s::EditorPanelDescriptor> contributeEditorPanels() const override;
    std::vector<s::SoundCueInfo> contributeSoundCues() const override;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 20.2. Completed case-study metadata. <code>seasons\2027\game\src\plugin_info.cpp</code>.

<!-- book-file: seasons\2027\game\src\plugin_info.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {
s::GameMetadata Module::metadata() const {
    s::GameMetadata result;
    result.id = kId;
    result.displayName = "FIRST POWER UP (2018 rules, 2027 ABI)";
    result.year = 2027;
    result.description =
        "POWER UP case study: physical cubes, kinematic plates, vault, power-ups and "
        "anchored climb model; optional scripted human players and virtual partners.";
    return result;
}

s::FieldFrame Module::fieldFrame() const {
    return {kLengthM, kWidthM, s::FieldSymmetry::Rotational};
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 20.3. Original cue declarations and generic editor panel. <code>seasons\2027\game\src\game_sounds.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_sounds.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

namespace wpiemu::plugins::game_2027 {

std::vector<s::SoundCueInfo> Module::contributeSoundCues() const {
    return {
        {"game_2027.ownership", "Ownership established", "A target gains an effective owner",
         "tone"},
        {"game_2027.vault", "Vault deposit", "A conserved cube enters the vault", "tone"},
        {"game_2027.power", "Power-up starts", "Force or Boost enters the active slot", "tone"}};
}

std::vector<s::EditorPanelDescriptor> Module::contributeEditorPanels() const {
    s::EditorPanelDescriptor panel;
    panel.id = "game_2027.scenario";
    panel.title = "POWER UP scenario";
    panel.scope = s::PanelScope::PluginConfig;
    for (const auto& property : configSchema().properties.properties)
        if (property.group == "Scenario") panel.properties.properties.push_back(property);
    return {panel};
}

}  // namespace wpiemu::plugins::game_2027
```

The generator writes a mono 16 kHz, 16-bit WAV containing a 120 ms, 660 Hz
tone with a smooth envelope. The season build discovers the generated asset
under its standard sound directory.

Listing 20.4. Generate the original demonstration tone. <code>seasons\2027\game\tools\generate_sounds.py</code>.

<!-- book-file: seasons\2027\game\tools\generate_sounds.py -->
```python
"""Generate an original short cue; no official game recording is redistributed."""
import math
from pathlib import Path
import struct
import wave

destination = Path(__file__).resolve().parents[1] / "assets" / "sfx" / "tone.wav"
destination.parent.mkdir(parents=True, exist_ok=True)
rate = 16_000
duration = 0.12
samples = bytearray()
for i in range(int(rate * duration)):
    envelope = math.sin(math.pi * i / (rate * duration)) ** 2
    value = int(6000 * envelope * math.sin(2 * math.pi * 660 * i / rate))
    samples.extend(struct.pack("<h", value))
with wave.open(str(destination), "wb") as stream:
    stream.setnchannels(1)
    stream.setsampwidth(2)
    stream.setframerate(rate)
    stream.writeframes(samples)
print(destination)
```

### 20.5.2 Transition requests

Ownership cues compare effective owned flags with the previous publication
state. Power cues compare active alliance/type identities, not remaining time.
The vault requests its cue only after a successful custody-backed deposit.

Listing 20.5. Edge-triggered ownership and power cues. <code>seasons\2027\game\src\game_cues.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_cues.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {

void State::requestCues(s::SimContext& ctx) {
    const auto& owned = scoring_.snapshot().owned;
    for (std::size_t a = 0; a < 2; ++a)
        for (std::size_t target = 0; target < 2; ++target)
            if (owned[a][target] && !previousOwned_[a][target])
                ctx.requestSound("game_2027.ownership");
    previousOwned_ = owned;
    const auto& active = powers_.snapshot().active;
    const auto current =
        active ? std::optional(std::pair(active->alliance, active->power)) : std::nullopt;
    if (current && current != previousPower_) ctx.requestSound("game_2027.power");
    previousPower_ = current;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 20.6. Deposit cue after successful transfer. <code>seasons\2027\game\src\game_vault.cpp</code>.

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
                else
                    ctx.requestSound("game_2027.vault");
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

The state owns the previous observations and clears them on reset.
Requests follow rule updates, so the cue corresponds to the newly observed
transition.

Listing 20.7. Complete state declaration including cue history. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

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
    void requestCues(s::SimContext& ctx);
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
    std::array<std::array<bool, 2>, 2> previousOwned_{};
    std::optional<std::pair<std::size_t, Power>> previousPower_;
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

Listing 20.8. Reset of transition observations. <code>seasons\2027\game\src\game_reset.cpp</code>.

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
    previousOwned_ = {};
    previousPower_.reset();
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

Listing 20.9. Final tick orchestration. <code>seasons\2027\game\src\game_tick.cpp</code>.

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
        requestCues(ctx);
    }
    if (teleopSeen_ && ds.matchTimeS == 0.0) finish(ctx);
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 20.10. Complete plugin source list. <code>seasons\2027\game\CMakeLists.txt</code>.

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
    src/game_scenario.cpp
    src/game_sounds.cpp
    src/game_cues.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

### 20.5.3 Contract coverage

| Module method | Case-study use |
|---|---|
| metadata | Identity, host season and model description |
| registerMechanisms | No custom templates; generic mechanisms suffice |
| createField | Checked construction of static colliders and visuals |
| createGameState | Checked per-session state construction |
| contributeConfigSchema | Shared typed bindings |
| contributeEditorPanels | Scenario group from the same schema |
| contributeHud | Scoreboard, assignment and custody counters |
| contributeZoneKinds | Pickup, release and hook |
| contributeGameValues | Held inventory for generic sensor sources |
| contributeSoundCues | Ownership, deposit and active-power transitions |
| aprilTagLayout | Default empty layout; historical game has no AprilTags |
| fieldFrame | NWU dimensions and rotational symmetry |
| onTick | Default no-op; state already owns orchestration |

This table distinguishes unused defaults from missing implementations.

## 20.6 Verification

The fixture gains a sound sink that records requests without playing audio.
Tests compare descriptor keys with an actual snapshot and panel properties
with the declared schema. Sustained ownership must request one ownership cue.

Listing 20.11. Integration fixture with a recording cue sink. <code>tests\game_2027\match_fixture.h</code>.

<!-- book-file: tests\game_2027\match_fixture.h -->
```cpp
#pragma once

#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

namespace power_up_test {

namespace s = wpiemu::sdk;

struct CueSink final : s::ISoundCueSink {
    std::vector<s::SoundCue> cues;
    void post(s::SoundCue cue) override { cues.push_back(std::move(cue)); }
};

struct Match {
    std::unique_ptr<s::IPhysicsWorld> world;
    s::testing::FakeSimEnvironment services;
    s::GamePluginRegistry registry;
    CueSink sounds;
    s::SimContext ctx{{*world,
                       services.clock,
                       services.hub,
                       services.logger,
                       services.config,
                       services.rng,
                       services.driverStation,
                       {},
                       nullptr,
                       nullptr,
                       &sounds}};
    std::unique_ptr<s::IField> field;
    std::unique_ptr<s::IGameState> game;

    Match(const std::string& assignment, bool red = false, bool preload = true, int workers = 1)
        : world([workers] {
              s::PhysicsConfig config;
              config.workerCount = workers;
              return wpiemu::core::physics::createBox3dWorld(config);
          }()) {
        REQUIRE(world);
        REQUIRE(wpiemu::plugins::game_2027::registerPlugin(registry));
        auto* module = registry.find("game_2027");
        REQUIRE(module);
        auto builtField = module->createField(*world);
        REQUIRE(builtField);
        field = std::move(builtField.value());
        services.config.plugins["game_2027"] = {1, s::Value::makeObject()};
        services.config.plugins["game_2027"].data["assignment"] = assignment;
        services.config.plugins["game_2027"].data["preload"] = preload;
        services.driverStation.station =
            red ? s::proto::AllianceStation::Red1 : s::proto::AllianceStation::Blue1;
        services.config.chassis.sizeM = {0.7f, 0.7f, 0.2f};
        s::BodyDesc chassis;
        chassis.type = s::BodyType::Kinematic;
        const char side = assignment[red ? 2 : 0];
        const float localY = 8.2296f / 2.f + (side == 'L' ? 1.3716f : -1.3716f);
        chassis.pose.position = {red ? 16.4592f - 3.17f : 3.17f, red ? 8.2296f - localY : localY,
                                 0.3f};
        chassis.pose.rotation = s::Quat::fromYaw(red ? static_cast<float>(s::kPi) : 0.f);
        ctx.bindChassisBody(world->createBody(chassis));
        REQUIRE(ctx.chassisBody().valid());

        s::ZoneConfig release;
        release.id = "release";
        release.kind = "game_2027.release";
        release.local.position = {0.75f, 0.f, 0.65f};
        services.config.zones.push_back(release);
        auto builtGame = module->createGameState(ctx);
        REQUIRE_MESSAGE(builtGame.ok(), (builtGame ? "" : builtGame.error().message));
        game = std::move(builtGame.value());
        game->reset(ctx);
        services.driverStation.station =
            red ? s::proto::AllianceStation::Red1 : s::proto::AllianceStation::Blue1;
    }

    void advance(int steps) {
        for (int i = 0; i < steps; ++i) {
            world->step(0.005f);
            services.clock.advance();
            game->step(ctx, 0.005);
        }
    }

    std::shared_ptr<const s::ISimSnapshot> snapshot() {
        s::SnapshotBuilder out;
        game->publish(out);
        return out.build();
    }

    double hud(const char* key) {
        const auto snap = snapshot();
        const auto* value = snap->hudValue(key);
        REQUIRE(value);
        return value->asDouble();
    }
};

}  // namespace power_up_test
```

Listing 20.12. Descriptor consistency and one-shot cue tests. <code>tests\game_2027\test_presentation.cpp</code>.

<!-- book-file: tests\game_2027\test_presentation.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: descriptors name published keys and declared schema properties") {
    Match match("LRL");
    const auto* module = match.registry.find("game_2027");
    const auto snapshot = match.snapshot();
    for (const auto& widget : module->contributeHud()) {
        if (!widget.valueKey.empty()) CHECK(snapshot->hudValue(widget.valueKey) != nullptr);
        for (const auto& side : widget.sides) CHECK(snapshot->hudValue(side.scoreKey) != nullptr);
    }
    for (const auto& value : module->contributeGameValues())
        CHECK(match.game->gameValue(value.id).has_value());
    const auto schema = module->contributeConfigSchema();
    for (const auto& panel : module->contributeEditorPanels())
        for (const auto& property : panel.properties.properties)
            CHECK(std::any_of(schema.properties.properties.begin(),
                              schema.properties.properties.end(),
                              [&](const auto& spec) { return spec.key == property.key; }));
    CHECK(module->aprilTagLayout().tags.empty());
}

TEST_CASE("Power UP: sustained ownership requests only one cue") {
    Match match("LRL");
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(2000);
    auto count = [&] {
        return std::count_if(match.sounds.cues.begin(), match.sounds.cues.end(),
                             [](const auto& cue) { return cue.id == "game_2027.ownership"; });
    };
    CHECK(count() == 1);
    match.advance(200);
    CHECK(count() == 1);
}
```

These tests establish requests, not speaker output. The desktop inspection in
Appendix B checks rendering and audible playback separately.

## 20.7 Reproduction

Extract through Chapter 20 and generate the asset before configuring the app:

```powershell
python seasons\2027\game\tools\generate_sounds.py
cmake --build build\book --target wpiemu_tests
ctest --test-dir build\book -R "Power UP:.*(descriptors|cue)" --no-tests=error --output-on-failure
```

Reconfigure after generating an asset if the build was configured earlier.
Appendix B describes the full desktop build.

## 20.8 Discussion

The plugin supplies English descriptor data in the same form as the existing
season plugin; the app owns its presentation framework. No Qt translation,
audio or persistence object crosses the SDK boundary.

## 20.9 Summary

The completed module uses all required game seams and documents intentional
defaults. Sound requests follow rule transitions, and editor properties retain
one definition. The source remains an SDK-only static library.

## 20.10 Exercises

1. Explain why comparing remaining power time would request a cue every tick.
2. Add another original tone and assign it to the deposit cue.
3. Identify the two distinct checks needed to establish a valid cue request
   and audible playback.

[Chapter 21](21-another-season.md)
