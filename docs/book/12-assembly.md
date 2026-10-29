# 12. Assembly

## 12.1 Objectives

After this chapter the reader can compose the physical components, implement
reset in dependency order, and publish an immutable observation of a match.
Chapter 11 is the prerequisite. State files and integration tests address R8,
R10 and R11.

## 12.2 Problem statement

Individually correct components do not establish that their lifetimes, reset
order or publication agree. The game state is the composition point. It owns
the pool, plates and zone adapter, but borrows the physics world.

## 12.3 Background and definitions

**Definition 12.1 (composition root).** A composition root connects components
and determines their ownership. It delegates their internal rules rather than
reimplementing them.

The module's factory creates the state, calls its checked build operation and
returns it through `IGameState`. Runtime calls reset before the first tick.
The module does not retain the state.

## 12.4 Design

One large state object could contain every pool, plate and zone algorithm.
That would hide independently testable responsibilities behind one class.
Composition instead keeps those algorithms in their own complete files and
reserves the state for ordering, score accumulation and publication.

**Invariant 12.1 (reset).** Reset restores cube custody, zero velocities, level
plates, empty diagnostics and zero rule totals without allocating more bodies.
The configured assignment is unchanged.

## 12.5 Implementation

The declaration lists owned components together. Member order matters:
`zones_` borrows `pool_`, and both are constructed after the borrowed world
reference has been established.

Listing 12.1. State ownership and lifecycle. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_layout.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/cube_zones.h"
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
    s::IPhysicsWorld& world_;
    CubePool pool_;
    Plates plates_;
    CubeZones zones_;
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

`build` constructs the pool before plates and propagates the first failure.
`gameSpecificMessage` supplies assignment data through the existing runtime
path. It does not call HAL or write a robot-process global.

Listing 12.2. Construction and SDK queries. <code>seasons\2027\game\src\game_state.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_state.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
State::State(s::SimContext& ctx, std::string assignment, bool preload)
    : world_(ctx.physics()),
      pool_(world_),
      plates_(world_),
      zones_(pool_, world_),
      assignment_(std::move(assignment)),
      preload_(preload) {}

s::Status State::build() {
    auto status = pool_.build();
    if (!status) return status;
    return plates_.build();
}

std::string_view State::pluginId() const {
    return kId;
}

std::optional<std::string> State::gameSpecificMessage() const {
    return assignment_;
}

std::optional<double> State::gameValue(std::string_view id) const {
    if (id == "game_2027.inventory") return pool_.inventory();
    return std::nullopt;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 12.3. Reset of all owned match state. <code>seasons\2027\game\src\game_reset.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_reset.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::reset(s::SimContext& ctx) {
    scores_ = {};
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

The initial tick implements the continuous scoring baseline described in
Chapter 9. Only a switch's home alliance can score that switch. The scale may
score for either alliance. Auto-run is recorded once for the single physical
robot; Chapter 15 refines its bumper-plane observation.

Listing 12.4. Baseline tick orchestration. <code>seasons\2027\game\src\game_tick.cpp</code>.

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
    if (running) {
        if (autonomous) autoSeconds_ += dt;
        if (teleop) teleopSeconds_ += dt;
        if (endgame) endgameSeconds_ += dt;
        zones_.update(ctx);
    }

    plates_.update(dt);
    if (running && (autonomous || teleop)) {
        const double rate = autonomous ? 2.0 : 1.0;
        for (std::size_t pair = 0; pair < 3; ++pair) {
            const int a = plates_.values()[pair * 2].cubes;
            const int b = plates_.values()[pair * 2 + 1].cubes;
            if (a == b) continue;
            const int side = a > b ? 0 : 1;
            const int owner = plateOwner(assignment_, pair, side);
            if ((pair == 0 && owner != 0) || (pair == 1 && owner != 1)) continue;
            scores_[static_cast<std::size_t>(owner)] += dt * rate;
        }

        if (autonomous && ctx.chassisBody().valid()) {
            const auto alliance = s::proto::allianceOf(ds.station);
            const std::size_t side = alliance == s::Alliance::Blue ? 0 : 1;
            const float x = world_.pose(ctx.chassisBody()).position.x;
            const float halfLength = ctx.config().chassis.sizeM.x * 0.5f;
            const bool crossed =
                side == 0 ? x - halfLength > 3.048f : x + halfLength < kLengthM - 3.048f;
            if (crossed && !autoRuns_[side]) {
                scores_[side] += 5.0;
                autoRuns_[side] = true;
            }
        }
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Publication copies values and positions into a `SnapshotBuilder`.
The builder has no access to the private pool arrays. Both moving plates
and cube groups read the same physical poses that collision detection uses.

Listing 12.5. Snapshot contribution. <code>seasons\2027\game\src\game_snapshot.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_snapshot.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::publish(s::SnapshotBuilder& out) const {
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

The factory now returns the constructed state rather than `NullGameState`.
Failure destroys the partially built owner and its bodies.

Listing 12.6. Complete core factories, revision 4. <code>seasons\2027\game\src\game_module.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
s::Status Module::registerMechanisms(s::MechanismFactory&) {
    return s::okStatus();
}

s::Result<std::unique_ptr<s::IField>> Module::createField(s::IPhysicsWorld& world) {
    auto field = std::make_unique<Field>(world);
    const auto status = field->build();
    if (!status) return status.error();
    return std::unique_ptr<s::IField>(std::move(field));
}

s::Result<std::unique_ptr<s::IGameState>> Module::createGameState(s::SimContext& ctx) {
    s::Value config =
        ctx.config().findPlugin(kId) ? ctx.config().findPlugin(kId)->data : s::Value::makeObject();
    const auto parsed = parseConfig(config);
    if (!validAssignment(parsed.assignment))
        return s::makeError(s::ErrorCode::InvalidArgument,
                            "Power UP assignment must be LLL, LRL, RLR or RRR");
    auto state = std::make_unique<State>(ctx, parsed.assignment, parsed.preload);
    const auto status = state->build();
    if (!status) return status.error();
    return std::unique_ptr<s::IGameState>(std::move(state));
}

s::PluginConfigSchema Module::contributeConfigSchema() const {
    return configSchema();
}

}  // namespace wpiemu::plugins::game_2027

namespace wpiemu::plugins::game_2027 {
WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

Listing 12.7. Complete core library sources. <code>seasons\2027\game\CMakeLists.txt</code>.

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
    src/game_snapshot.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 12.6 Verification

The fixture uses the real Box3D backend, a kinematic test chassis and normal
zone configuration. The chassis pose is a fixture input, not a claim about a
drivetrain. Chapter 14 replaces it with a real robot process.

Listing 12.8. Shared real-physics integration fixture. <code>tests\game_2027\match_fixture.h</code>.

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

struct Match {
    std::unique_ptr<s::IPhysicsWorld> world;
    s::testing::FakeSimEnvironment services;
    s::GamePluginRegistry registry;
    s::SimContext ctx{{*world,
                       services.clock,
                       services.hub,
                       services.logger,
                       services.config,
                       services.rng,
                       services.driverStation,
                       {},
                       nullptr}};
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

The tests cover all four assignments and both alliances; physical release and
settling; disabled, emergency-stop and zero-time score freezing; custody; invalid
configuration; five seconds of staging stability; and complete teardown.
The assertions remain alongside the behavior they check.

Listing 12.9. Delivery and phase-rate integration. <code>tests\game_2027\test_plate_scoring.cpp</code>.

<!-- book-file: tests\game_2027\test_plate_scoring.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE(
    "Power UP: real cube settles on each alliance switch and scores at "
    "period rates") {
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
            match.advance(500);
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

Listing 12.10. Starting layout, stability and reset. <code>tests\game_2027\test_cube_layout.cpp</code>.

<!-- book-file: tests\game_2027\test_cube_layout.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE(
    "Power UP: reset stages switch rows and stable pyramids with portal "
    "supply off field") {
    Match match("LRL", false, false);
    const auto initial = match.snapshot();
    const auto positions = initial->entityGroups()[0].positions;
    REQUIRE(positions.size() == 32);
    REQUIRE(initial->entityGroups()[1].group == "game_2027.portal_supply");
    const auto supply = initial->entityGroups()[1].positions;
    REQUIRE(supply.size() == 28);
    for (std::size_t i = 0; i < 16; ++i) {
        CHECK(positions[i].x + positions[i + 16].x == doctest::Approx(16.4592));
        CHECK(positions[i].y + positions[i + 16].y == doctest::Approx(8.2296));
        CHECK(positions[i].z == positions[i + 16].z);
    }
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(positions[i].x > 4.2672f + 0.7362f);
        CHECK(positions[i].x < 5.3f);
        CHECK(positions[i].z < 0.16f);
        if (i != 0) CHECK(positions[i].y - positions[i - 1].y == doctest::Approx(0.65));
    }
    std::array<int, 3> tiers{};
    for (std::size_t i = 6; i < 16; ++i) {
        CHECK(positions[i].x > 2.38f);
        CHECK(positions[i].x + 0.1651f < 4.2672f - 0.7362f);
        CHECK(std::abs(positions[i].y - 8.2296f / 2.f) < 0.5f);
        ++tiers[positions[i].z < 0.3f ? 0 : positions[i].z < 0.6f ? 1 : 2];
    }
    CHECK((tiers == std::array<int, 3>{6, 3, 1}));
    for (const auto& p : supply) CHECK((p.x < 0.f || p.x > 16.4592f));
    CHECK(match.hud("game_2027.portal_blue") == 14);
    CHECK(match.hud("game_2027.portal_red") == 14);

    match.advance(1000);
    const auto settled = match.snapshot();
    const auto& settledCubes = settled->entityGroups()[0].positions;
    REQUIRE(settledCubes.size() == positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        CHECK(std::abs(settledCubes[i].x - positions[i].x) < 0.04f);
        CHECK(std::abs(settledCubes[i].y - positions[i].y) < 0.04f);
        CHECK(std::abs(settledCubes[i].z - positions[i].z) < 0.04f);
    }
    CHECK(match.hud("game_2027.blue_score") == 0);
    CHECK(match.hud("game_2027.red_score") == 0);
    auto& pickup = match.services.config.zones[0];
    pickup.kind = "game_2027.pickup";
    pickup.local.position = {0.f, 0.f, -0.15f};
    match.world->setPose(match.ctx.chassisBody(), {{supply[0].x, supply[0].y, 0.30f}, {}});
    match.services.driverStation.enabled = true;
    match.advance(4);
    CHECK(match.hud("game_2027.inventory") == 0);
    CHECK(match.hud("game_2027.portal_blue") == 14);
    match.game->reset(match.ctx);
    const auto restored = match.snapshot();
    CHECK(restored->entityGroups()[0].positions == positions);
    CHECK(restored->entityGroups()[1].positions == supply);
}
```

Listing 12.11. Pickup and release conservation. <code>tests\game_2027\test_cube_zones.cpp</code>.

<!-- book-file: tests\game_2027\test_cube_zones.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE(
    "Power UP: pickup and release reuse a cube without creating a "
    "phantom preload") {
    Match match("LRL", false, false);
    auto& zone = match.services.config.zones[0];
    zone.kind = "game_2027.pickup";
    zone.local.position = {0.f, 0.f, -0.15f};
    zone.sizeM = {0.30f, 0.30f, 0.30f};
    const auto initialCube = match.snapshot()->entityGroups()[0].positions[0];
    match.world->setPose(match.ctx.chassisBody(), {{initialCube.x, initialCube.y, 0.30f}, {}});
    const auto budget = match.world->stats().bodies;
    match.services.driverStation.enabled = true;
    match.advance(2);
    CHECK(match.hud("game_2027.inventory") == 1);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 31);
    zone.kind = "game_2027.release";
    zone.local.position = {0.75f, 0.f, 0.65f};
    match.advance(2);
    CHECK(match.hud("game_2027.inventory") == 0);
    CHECK(match.hud("game_2027.released") == 1);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 32);
    CHECK(match.world->stats().bodies == budget);
    match.game->reset(match.ctx);
    CHECK(match.hud("game_2027.inventory") == 0);
    CHECK(match.hud("game_2027.released") == 0);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 32);
}
```

Listing 12.12. Missing mechanism diagnostics. <code>tests\game_2027\test_zone_activation.cpp</code>.

<!-- book-file: tests\game_2027\test_zone_activation.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: unresolved zone activation disables release and warns once") {
    Match match("LRL");
    s::ZoneCondition condition;
    condition.mechanismId = "missing_roller";
    match.services.config.zones[0].activation.conditions.push_back(condition);
    match.services.driverStation.enabled = true;
    match.advance(20);
    CHECK(match.hud("game_2027.inventory") == 1);
    CHECK(match.hud("game_2027.released") == 0);
    CHECK(match.services.logger.count(s::LogLevel::Warn) == 1);
}
```

Listing 12.13. Checked construction without leaks. <code>tests\game_2027\test_config.cpp</code>.

<!-- book-file: tests\game_2027\test_config.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE(
    "Power UP: invalid FMS assignment rejects game construction without "
    "leaking bodies") {
    Match match("LRL");
    match.game.reset();
    const auto bodyBudget = match.world->stats().bodies;
    match.services.config.plugins["game_2027"].data["assignment"] = "BAD";
    const auto result = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE_FALSE(result);
    CHECK(result.error().code == s::ErrorCode::InvalidArgument);
    CHECK(match.world->stats().bodies == bodyBudget);
}
```

The configuration pipeline also constructs a synchronous session with the real
physics backend. This test crosses the boundary between the module and its host;
the component fixture alone cannot establish that selection and defaults work.

Listing 12.14. Selection, defaults and session publication. <code>tests\game_2027\test_session.cpp</code>.

<!-- book-file: tests\game_2027\test_session.cpp -->
```cpp
#include <doctest/doctest.h>

#include "../runtime/runtime_fixtures.h"
#include "wpiemu/core/config/robot_config_codec.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

TEST_CASE("Power UP: configuration pipeline builds a real synchronous session") {
    using namespace wpiemu;
    test_runtime::Registries registries(false);
    REQUIRE(sdk::registerBuiltinGamePlugins(registries.plugins));
    core::config::RobotConfigCodec codec;
    core::config::ConfigValidator validator;
    core::config::ConfigPipeline pipeline(codec, validator, registries.registrar,
                                          registries.drivetrains, registries.plugins);
    auto config = test_runtime::sampleConfig();
    config.project.wpilibYear = 2027;
    config.plugins["game_2027"] = {1, sdk::Value::makeObject()};
    config.plugins["game_2027"].data["assignment"] = "LRL";
    const auto loaded = pipeline.load(codec.toValue(config));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    REQUIRE(loaded.value().valid());
    CHECK(loaded.value().pluginId == "game_2027");
    CHECK(loaded.value().config.findPlugin("game_2027")->data.find("preload")->asBool());

    core::runtime::SessionParts parts;
    parts.config = loaded.value().config;
    parts.world = core::physics::createBox3dWorld(parts.config.sim.physics);
    parts.mechanisms = &registries.mechanisms;
    parts.drivetrains = &registries.drivetrains;
    parts.tireModels = &registries.tires;
    parts.plugins = &registries.plugins;
    const auto built = core::runtime::SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    auto& session = *built.value();
    REQUIRE(session.gameModule());
    CHECK(session.gameModule()->metadata().id == "game_2027");
    sdk::proto::SensorFrame frame;
    for (int i = 0; i < 200; ++i) session.tick(frame, false);
    const auto snapshot = session.buildSnapshot(false, true);
    REQUIRE(snapshot->hudValue("game_2027.inventory"));
    CHECK(snapshot->hudValue("game_2027.inventory")->asInt() == 1);
    CHECK(snapshot->hudValue("game_2027.blue_score")->asDouble() == 0);
    CHECK(session.gameSpecificMessage() == std::optional<std::string>{"LRL"});
    CHECK(snapshot->entityGroups()[0].positions.size() == 32);
}
```

## 12.7 Reproduction

Extract through Chapter 12, build and run all core cases:

```powershell
ctest --test-dir build\book -R "Power UP:" --no-tests=error --output-on-failure
```

## 12.8 Discussion

F-09 identifies the baseline accounting used here. Tests that establish physical
delivery remain useful after score refinement; only their accounting oracle
changes. Snapshot publication establishes observability without coupling the
simulation to a UI refresh rate.

## 12.9 Summary

The plugin now owns a complete physical core and publishes its state. Reset
has a defined invariant and tests across both alliances. The remaining
integration question is whether ordinary robot code can drive the same path.

## 12.10 Exercises

1. Remove one reset operation locally and identify the test that detects it.
2. Compare two snapshots retained across a reset and explain their independence.
3. Add a publication-key test using the module's actual HUD descriptors.

[Chapter 13](13-robot-programs.md)
