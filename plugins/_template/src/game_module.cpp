// SKELETON year plugin. To start a new season, copy plugins/_template to plugins/game_<year>/, rename ids /
// namespaces / targets (see README.md and docs/new-year-plugin.md), then replace the placeholder bodies below.
// Everything marked TODO(new season) is a decision you must make; the rest is wiring that already works:
// the skeleton registers, builds a field, steps a game state, publishes a HUD value and passes the contract tests.
//
// RULES (checked by tools/check_boundaries.py and the tests):
//   * include only wpiemu/sdk/** and this plugin's own headers - no Qt, Box3D, core, app, other plugins;
//   * no globals / singletons: per-session state lives in the IField / IGameState the module creates;
//   * no exceptions across SDK interfaces: return Status / Result;
//   * physics only through sdk::IPhysicsWorld (the Box3D adapter is core's business).
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/plugins/_template/plugin.h"
#include "wpiemu/plugins/_template/plugin_info.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::plugins::_template {

namespace {

// ---- 2. the static field ---------------------------------------------------------------------------------------
// Owns whatever createField() added to the physics world and removes it in the destructor (RAII). It is destroyed
// BEFORE the world. TODO(new season): create a static body with the season's colliders (boxes, cylinders, meshes) and
// sensor volumes, fill FieldDescription::visuals for the renderer, and keep the ids you need.
class TemplateField final : public sdk::IField {
public:
    explicit TemplateField(sdk::IPhysicsWorld& /*world*/) {
        description_.name = "Template field (no game)";
        description_.lengthM = 16.5f;  // along +X
        description_.widthM = 8.f;     // along +Y
    }
    const sdk::FieldDescription& description() const override { return description_; }

private:
    sdk::FieldDescription description_;
};

// ---- 3. the game state ------------------------------------------------------------------------------------------
// Rules + entities for ONE simulation session (game pieces, scoring, timers...). Created after the field, destroyed
// before the physics world. step() runs once per sim tick AFTER the physics step (fresh sensor/contact events).
// TODO(new season): spawn game pieces through ctx.physics(), score from IPhysicsWorld::sensorEvents(), read the
// driver-station state from ctx.driverStation() and the robot from ctx.mechanisms() / ctx.chassisBody().
class TemplateGameState final : public sdk::IGameState {
public:
    std::string_view pluginId() const override { return _template::pluginId(); }
    void reset(sdk::SimContext& /*ctx*/) override { ticks_ = 0; }  // restore the initial layout, counters, timers
    void step(sdk::SimContext& /*ctx*/, double /*dtSeconds*/) override { ++ticks_; }
    // Feed the snapshot: HUD values (keys are "<pluginId>.<name>") and entity groups for instanced rendering.
    void publish(sdk::SnapshotBuilder& out) const override { out.setHud(hudKey(), ticks_); }

    static std::string hudKey() { return std::string(_template::pluginId()) + ".ticks"; }

private:
    int ticks_ = 0;
};

// ---- the module: a stateless facade ---------------------------------------------------------------------------
class TemplateModule final : public sdk::IGameModule {
public:
    // 0. identity. TODO(new season): displayName, description (and minSdkMinor if you need a newer SDK minor).
    sdk::GameMetadata metadata() const override {
        sdk::GameMetadata m;
        m.id = std::string(pluginId());
        m.displayName = "Template (no game)";
        m.year = gameYear();  // 0 is allowed only for ids starting with '_'; real plugins need the season
        return m;
    }

    // 1. season-specific mechanism templates. Keys MUST be prefixed with the plugin id ("<id>.<name>") so the
    //    ownership rule (sdk::ownsMechanismType) holds. Prefer wrapping the generic core mechanisms through the
    //    values they publish (see zones in plugins/game_2026/include/.../fuel_zones.h) over re-implementing them.
    sdk::Status registerMechanisms(sdk::MechanismFactory& /*factory*/) override { return sdk::okStatus(); }

    // 2. the static field.
    sdk::Result<std::unique_ptr<sdk::IField>> createField(sdk::IPhysicsWorld& world) override {
        return std::unique_ptr<sdk::IField>(std::make_unique<TemplateField>(world));
    }

    // 3. the game state. Read your config section with ctx.config().findPlugin(pluginId()).
    sdk::Result<std::unique_ptr<sdk::IGameState>> createGameState(sdk::SimContext& /*ctx*/) override {
        return std::unique_ptr<sdk::IGameState>(std::make_unique<TemplateGameState>());
    }

    // 4. config fields shown by the generic inspector and persisted under RobotConfig::plugins[<id>].
    //    The SDK stays at version 1 until release: a section with another schemaVersion is rejected (no migration).
    //    TODO(new season): describe your fields with PropertySpec (key, label, type, default, range, group).
    sdk::PluginConfigSchema contributeConfigSchema() const override {
        sdk::PluginConfigSchema schema;
        schema.schemaVersion = 1;
        return schema;
    }

    // 5. optional UI contributions (Qt-free descriptors the app renders generically).
    // std::vector<sdk::EditorPanelDescriptor> contributeEditorPanels() const override;
    std::vector<sdk::HudWidgetDescriptor> contributeHud() const override {
        sdk::HudWidgetDescriptor w;
        w.id = std::string(pluginId()) + ".ticks";
        w.title = "Game ticks";
        w.kind = sdk::HudWidgetKind::Counter;
        w.valueKey = TemplateGameState::hudKey();
        return {std::move(w)};
    }

    // 6. cross-cutting per-tick hook after IGameState::step. Usually unnecessary: keep logic in the game state.
    void onTick(sdk::SimContext& /*ctx*/, double /*dtSeconds*/) override {}
};

}  // namespace

// Defines `sdk::Status registerPlugin(sdk::GamePluginRegistry&)` in THIS namespace (declared in plugin.h).
// plugins/builtin_plugins.cpp calls it; that is the only registration a new season needs.
WPIEMU_REGISTER_GAME_PLUGIN(TemplateModule)

}  // namespace wpiemu::plugins::_template
