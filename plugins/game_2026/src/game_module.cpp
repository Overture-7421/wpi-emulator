// 2026 (REBUILT) game module: the thin IGameModule facade over the plugin's parts (field layout,
// fuel game state, 2026 mechanisms, config schema, HUD/editor descriptors, migrations). The module
// itself holds no session state.
#include <memory>
#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/fuel_intake.h"
#include "wpiemu/plugins/game_2026/fuel_shooter.h"
#include "wpiemu/plugins/game_2026/game_state.h"
#include "wpiemu/plugins/game_2026/game_ui.h"
#include "wpiemu/plugins/game_2026/migration.h"
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
        m.minSdkMinor = 1;  // migrateRobotConfig and resolved mechanism link poses (SDK 1.1)
        return m;
    }

    sdk::Status registerMechanisms(sdk::MechanismFactory& factory) override {
        if (sdk::Status s = factory.registerTemplate(fuelIntakeTemplate(), createFuelIntake); !s) return s;
        return factory.registerTemplate(fuelShooterTemplate(), createFuelShooter);
    }

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
    std::vector<std::unique_ptr<sdk::IConfigMigrator>> configMigrators() const override { return makeConfigMigrators(); }
    sdk::Status migrateRobotConfig(sdk::RobotConfig& config, int persistedPluginVersion) const override {
        return migrateGame2026Config(config, persistedPluginVersion);
    }

    std::vector<sdk::EditorPanelDescriptor> contributeEditorPanels() const override { return editorPanels(); }
    std::vector<sdk::HudWidgetDescriptor> contributeHud() const override { return hudWidgets(); }

    // All per-tick game logic lives in Game2026State::step (it needs per-session state).
    void onTick(sdk::SimContext&, double) override {}
};

}  // namespace

WPIEMU_REGISTER_GAME_PLUGIN(Game2026Module)

}  // namespace wpiemu::plugins::game_2026
