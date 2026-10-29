# Inspection workshop: make the physics prove it

**In 30 seconds:** a fake world can prove a branch ran; it cannot prove the cube stayed on the
plate. These tests use the real Box3D backend through the SDK interface. They create no Qt window
and launch no robot; real-HAL verification comes next.

Create <code>tests\plugins\test_power_up.cpp</code> by appending these blocks in order.
The existing test CMake automatically discovers this file. Tests may compose core and plugins;
the plugin itself still depends only on SDK.

## 1. Build a deliberately small test pit

The fixture borrows clock, logger, device hub and config fakes, but **not** their physics world.
`ctx` points at the real world. Member order ensures state and field are destroyed before it.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

namespace {

namespace s = wpiemu::sdk;

struct Match {
    std::unique_ptr<s::IPhysicsWorld> world = wpiemu::core::physics::createBox3dWorld();
    s::testing::FakeSimEnvironment services;
    s::GamePluginRegistry registry;
    s::SimContext ctx{{*world, services.clock, services.hub, services.logger, services.config, services.rng,
                       services.driverStation, {}, nullptr}};
    std::unique_ptr<s::IField> field;
    std::unique_ptr<s::IGameState> game;
```

Register the real plugin and create its field, then set the chosen scenario and station **before
reset**. This fixture uses a kinematic chassis already at the fence. That isolates the release
and plate from drivetrain tuning; it is not evidence that a robot can drive there.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
    Match(const std::string& assignment, bool red = false, bool preload = true) {
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
        services.driverStation.station = red ? s::proto::AllianceStation::Red1 : s::proto::AllianceStation::Blue1;
        services.config.chassis.sizeM = {0.7f, 0.7f, 0.2f};
        s::BodyDesc chassis;
        chassis.type = s::BodyType::Kinematic;
        const char side = assignment[red ? 2 : 0];
        const float localY = 8.2296f / 2.f + (side == 'L' ? 1.3716f : -1.3716f);
        chassis.pose.position = {red ? 16.4592f - 3.17f : 3.17f, red ? 8.2296f - localY : localY, 0.3f};
        chassis.pose.rotation = s::Quat::fromYaw(red ? static_cast<float>(s::kPi) : 0.f);
        ctx.bindChassisBody(world->createBody(chassis));
        REQUIRE(ctx.chassisBody().valid());
```

An always-active release zone makes the first enabled tick release the preload. The main match
test later requires actual PWM roller motion instead. Build game state after the config is ready,
and call reset as the runtime would.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
        s::ZoneConfig release;
        release.id = "release";
        release.kind = "game_2027.release";
        release.local.position = {0.75f, 0.f, 0.65f};
        services.config.zones.push_back(release);
        auto builtGame = module->createGameState(ctx);
        REQUIRE_MESSAGE(builtGame.ok(), (builtGame ? "" : builtGame.error().message));
        game = std::move(builtGame.value());
        game->reset(ctx);
        services.driverStation.station = red ? s::proto::AllianceStation::Red1 : s::proto::AllianceStation::Blue1;
    }
```

Advance physics before game rules, at 5 ms per tick, then snapshot through the public builder.
The HUD accessor requires the key to exist: an absent value must not silently become zero and
make a scoring test appear to pass.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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

}  // namespace
```

## 2. Exercise physical scoring from both station views

Loop over all four assignments and both alliances. We require an actual released cube to settle
and produce a positive score, then measure the subsequent ownership rate. An assertion that only
checked the release counter would miss a cube falling off the plate.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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
            match.advance(500);
            const auto* key = red ? "game_2027.red_score" : "game_2027.blue_score";
            CHECK(match.hud("game_2027.released") == 1);
            CHECK(match.hud("game_2027.inventory") == 0);
            REQUIRE(match.hud(key) > 1.0);
            const double autoScore = match.hud(key);
            match.advance(400);
            CHECK(match.hud(key) - autoScore == doctest::Approx(4.0));
```

Switch to teleop and verify the lower point rate. Disable, expire the timer and apply emergency
stop independently; none should add points. These transitions intentionally keep physics running
so a frozen score cannot be explained by a frozen simulation.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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
```

Reset must restore inventory, counters, display pieces and reserve accounting without changing
the body budget. Then destroy both owners and the test chassis; the world must contain no bodies.
That final assertion catches a missing destructor even if every scoring assertion passes.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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

## 3. Test the failure and inventory paths

Malformed FMS input must reject construction without leaking a partial game. The next case starts
without a preload, picks up an actual staged field cube and releases **that same piece**.
The field count returns to 32, not 33, and reset still starts without a held cube.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
TEST_CASE("Power UP: invalid FMS assignment rejects game construction without leaking bodies") {
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

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
TEST_CASE("Power UP: pickup and release reuse a cube without creating a phantom preload") {
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

An unresolved roller condition must keep the cube held. Repeated ticks should issue one warning,
not flood the log. This is a meaningful failure mode for a mentor renaming a mechanism in the editor.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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

## 4. Inspect the arrangement before calling it Power Up

Check the mirrored pairs, switch rows, pyramid tier counts and off-field reserve. These tests
capture both a bad layout and the earlier mistake of treating portal reserves as loose field pieces.
They are not a claim that the nominal coordinates match every official construction dimension.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
TEST_CASE("Power UP: reset stages switch rows and stable pyramids with portal supply off field") {
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
    for (const auto& p : supply)
        CHECK((p.x < 0.f || p.x > 16.4592f));
    CHECK(match.hud("game_2027.portal_blue") == 14);
    CHECK(match.hud("game_2027.portal_red") == 14);
```

Let five simulated seconds pass. A pyramid that explodes or slides off its supports is not a
successful starting layout. We bound drift, reject pickup from an off-field reserve location,
and require reset to reproduce the initial positions exactly.

<!-- book-file: tests\plugins\test_power_up.cpp -->
```cpp
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

## 5. Run the focused proof

From the scratch export root:

```powershell
cmake --build build\power-up-book --config Release --target wpiemu_tests --parallel
ctest --test-dir build\power-up-book -C Release -R "^Power UP:" --output-on-failure
```

Expect five test cases. The count of assertions may grow when a regression is added; check that
actual cases ran, not merely that the executable returned zero after matching no tests.
The tests prove their scenarios, not vaults, climbs or full manual fidelity.

Next: [launch both ordinary robot projects through the real HALSim extension](05c-full-match.md).
