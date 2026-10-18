#pragma once

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_state.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::sdk {

struct GameMetadata {
    std::string id;             // unique, lower-case, == folder/namespace: "game_2026"
    std::string displayName;    // "REBUILT (2026)"
    int year = 0;               // season; matches RobotConfig::project.wpilibYear / GradleRIO year
    std::string description;
    int pluginVersion = 1;      // plugin's own version (informational)
    int minSdkMajor = 1;        // plugin was written against this SDK major (refused otherwise)
    int minSdkMinor = 0;        // minimum SDK minor the plugin needs (refused otherwise)
};

// The YEAR PLUGIN contract. One implementation per season, living entirely in plugins/game_<year>/.
// Core and UI know only this interface; there is no `if (year == ...)` anywhere outside plugins.
//
// Dependencies: a plugin includes ONLY wpiemu/sdk/** (enforced by tools/check_boundaries.py).
// Lifetime: the module object is created at registration, owned by GamePluginRegistry, and
// lives until shutdown; it must be cheap and stateless between sessions (per-session state
// belongs in IGameState / IField, which the module creates).
// Threading: all calls happen on the sim thread, except metadata()/contribute*() which the UI
// thread may also call (they must be const, pure and thread-safe).
// Errors: no exceptions; return Status/Result.
//
// Per-tick order (see docs/ARCHITECTURE.md):
//   hub ingest -> mechanisms.tick -> drivetrain.step -> physics.step -> gameState.step -> module.onTick -> snapshot
class IGameModule {
    WPIEMU_INTERFACE(IGameModule)
public:
    virtual GameMetadata metadata() const = 0;

    // Register season-specific mechanism templates (keys should be prefixed with the plugin id).
    virtual Status registerMechanisms(MechanismFactory& factory) = 0;

    // Build the static field (colliders, sensor volumes) in `world`. The returned IField owns
    // them (RAII). The default-constructed pose of the world is the NWU field frame.
    virtual Result<std::unique_ptr<IField>> createField(IPhysicsWorld& world) = 0;

    // Build the game entities/rules. `ctx` is fully wired (physics, hub, config) except gameState().
    virtual Result<std::unique_ptr<IGameState>> createGameState(SimContext& ctx) = 0;

    // Plugin-declared config fields (fuel capacity, intake zone, shooter params, ...), rendered
    // by the generic inspector and persisted under RobotConfig::plugins[metadata().id].
    virtual PluginConfigSchema contributeConfigSchema() const = 0;

    virtual std::vector<EditorPanelDescriptor> contributeEditorPanels() const { return {}; }
    virtual std::vector<HudWidgetDescriptor> contributeHud() const { return {}; }

    // Zone kinds this plugin owns (ZoneConfig::kind). Every `kind` must start with metadata().id + '.';
    // core rejects zones whose kind is not owned by the selected plugin and validates ZoneConfig::properties
    // against the kind's schema. The plugin runtime builds the actual intake/shooter behaviour from the zones.
    virtual std::vector<ZoneKindInfo> contributeZoneKinds() const { return {}; }

    // Cross-cutting game logic hook after IGameState::step each tick (e.g. match timer phases).
    virtual void onTick(SimContext& /*ctx*/, double /*dtSeconds*/) {}
};

}  // namespace wpiemu::sdk
