// 2026 (REBUILT) game module: the thin IGameModule facade over the plugin's parts (field layout,
// fuel game state, zone kinds, config schema, HUD/editor descriptors). The module
// itself holds no session state.
#include <memory>
#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/april_tags.h"
#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
#include "wpiemu/plugins/game_2026/game_state.h"
#include "wpiemu/plugins/game_2026/game_ui.h"
#include "wpiemu/plugins/game_2026/game_values.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::plugins::game_2026 {

namespace {

class Game2026Module final : public sdk::IGameModule {
public:
    sdk::GameMetadata metadata() const override {
        sdk::GameMetadata m;
        m.id = std::string(pluginId());
        m.displayName = "REBUILT (2026)";
        m.year = gameYear();
        m.description = "FRC 2026 REBUILT: fuel, hubs with active/inactive shifts, bumps, trenches, towers.";
        m.pluginVersion = 1;
        m.minSdkMajor = 1;
        return m;
    }

    // The plugin owns zone kinds, not mechanism templates: the pickup and the shooter are RobotConfig::zones.
    sdk::Status registerMechanisms(sdk::MechanismFactory&) override { return sdk::okStatus(); }

    sdk::Result<std::unique_ptr<sdk::IField>> createField(sdk::IPhysicsWorld& world) override {
        auto field = Game2026Field::create(world, buildFieldLayout());
        if (!field.ok()) return field.error();
        return std::unique_ptr<sdk::IField>(std::move(field).value());
    }

    sdk::Result<std::unique_ptr<sdk::IGameState>> createGameState(sdk::SimContext& ctx) override {
        const sdk::PluginSection* section = ctx.config().findPlugin(pluginId());
        GameConfig config = parseConfig(section != nullptr ? section->data : sdk::Value::makeObject());
        auto state = std::make_unique<Game2026State>(ctx, std::move(config), buildFieldLayout());
        state->reset(ctx);  // valid immediately; the runtime resets again before the first tick
        return std::unique_ptr<sdk::IGameState>(std::move(state));
    }

    sdk::PluginConfigSchema contributeConfigSchema() const override { return configSchema(); }

    std::vector<sdk::HudWidgetDescriptor> contributeHud() const override { return hudWidgets(); }
    std::vector<sdk::ZoneKindInfo> contributeZoneKinds() const override { return zoneKinds(); }
    std::vector<sdk::GameValueInfo> contributeGameValues() const override { return gameValueInfos(); }

    sdk::AprilTagLayout aprilTagLayout() const override { return game2026AprilTagLayout(); }

    // Authored for blue and rotated 180 degrees about the field centre for red (field_layout.cpp).
    sdk::FieldFrame fieldFrame() const override {
        return {dim::kFieldLengthM, dim::kFieldWidthM, sdk::FieldSymmetry::Rotational};
    }

    // All per-tick game logic lives in Game2026State::step (it needs per-session state).
    void onTick(sdk::SimContext&, double) override {}
};

}  // namespace

WPIEMU_REGISTER_GAME_PLUGIN(Game2026Module)

}  // namespace wpiemu::plugins::game_2026
