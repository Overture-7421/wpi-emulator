#include <doctest/doctest.h>

#include "wpiemu/plugins/_template/plugin.h"
#include "wpiemu/plugins/_template/plugin_info.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu;

TEST_CASE("template plugin registers once and creates its field and HUD") {
    sdk::GamePluginRegistry registry;
    REQUIRE(plugins::_template::registerPlugin(registry).ok());
    CHECK(registry.size() == 1);
    const auto duplicate = plugins::_template::registerPlugin(registry);
    REQUIRE_FALSE(duplicate.ok());
    CHECK(duplicate.error().code == sdk::ErrorCode::AlreadyExists);
    CHECK(registry.size() == 1);
    auto* module = registry.find(plugins::_template::pluginId());
    REQUIRE(module != nullptr);
    CHECK(module->metadata().id == "_template");
    CHECK(module->metadata().year == plugins::_template::gameYear());
    CHECK(registry.findByYear(2026) == nullptr);

    sdk::MechanismFactory mechanisms;
    CHECK(module->registerMechanisms(mechanisms).ok());
    sdk::testing::FakeSimEnvironment env;
    const auto field = module->createField(env.physics);
    REQUIRE(field.ok());
    REQUIRE(field.value() != nullptr);
    CHECK(field.value()->description().lengthM > 0.f);
    CHECK(field.value()->description().widthM > 0.f);
    CHECK(module->contributeConfigSchema().schemaVersion == 1);
    const auto widgets = module->contributeHud();
    REQUIRE(widgets.size() == 1);
    CHECK(widgets.front().kind == sdk::HudWidgetKind::Counter);
    CHECK(widgets.front().valueKey == "_template.ticks");
}

TEST_CASE("template plugin steps resets and isolates per-session HUD state") {
    sdk::GamePluginRegistry registry;
    REQUIRE(plugins::_template::registerPlugin(registry).ok());
    auto* module = registry.find("_template");
    REQUIRE(module != nullptr);
    sdk::testing::FakeSimEnvironment env;
    auto first = module->createGameState(env.ctx);
    auto second = module->createGameState(env.ctx);
    REQUIRE(first.ok());
    REQUIRE(second.ok());
    REQUIRE(first.value() != nullptr);
    REQUIRE(second.value() != nullptr);
    CHECK(first.value()->pluginId() == "_template");
    const auto key = module->contributeHud().front().valueKey;
    const auto snapshot = [&](const sdk::IGameState& state) {
        sdk::SnapshotBuilder builder;
        state.publish(builder);
        return builder.build();
    };
    const auto ticks = [&](const sdk::IGameState& state) {
        const auto view = snapshot(state);
        const auto* value = view->hudValue(key);
        REQUIRE(value != nullptr);
        REQUIRE(value->isInt());
        return value->asInt();
    };

    first.value()->reset(env.ctx);
    CHECK(ticks(*first.value()) == 0);
    for (int i = 0; i < 3; ++i) {
        env.advance();
        first.value()->step(env.ctx, env.clock.fixedStepSeconds());
        module->onTick(env.ctx, env.clock.fixedStepSeconds());
    }
    const auto beforeReset = snapshot(*first.value());
    CHECK(ticks(*first.value()) == 3);
    CHECK(ticks(*second.value()) == 0);
    first.value()->reset(env.ctx);
    CHECK(ticks(*first.value()) == 0);
    CHECK(beforeReset->hudValue(key)->asInt() == 3);
    first.value()->step(env.ctx, env.clock.fixedStepSeconds());
    CHECK(ticks(*first.value()) == 1);
    CHECK(ticks(*second.value()) == 0);
}
