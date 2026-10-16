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
    int minSdkMinor = 0;        // SDK 1.1: minimum SDK minor the plugin needs (e.g. 1 for migrateRobotConfig/coupling)
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
    // Steps upgrading older sections to contributeConfigSchema().schemaVersion (v1->v2, ...).
    virtual std::vector<std::unique_ptr<IConfigMigrator>> configMigrators() const { return {}; }

    // SDK 1.1. Whole-config migration hook for changes a section-only IConfigMigrator cannot express
    // (renaming/adding/removing this plugin's mechanisms, re-pointing their mappings or attachments).
    //
    // Config load pipeline (the coordinator in core/app, in this order):
    //   1. core migration      IRobotConfigCodec::fromValue upgrades RobotConfig::schemaVersion;
    //   2. plugin migration    for the SELECTED plugin only, when its persisted section version is older
    //                          than contributeConfigSchema().schemaVersion:
    //                            a. configMigrators() chain upgrades the section's `data`;
    //                            b. then migrateRobotConfig(config, persistedPluginVersion);
    //   3. defaults            missing core/plugin properties get their schema defaults;
    //   4. validation          IConfigValidator;
    //   5. construction        mechanisms/field/game state are built from the validated config.
    // `persistedPluginVersion` is the section version as READ FROM STORAGE (before step 2a), so the hook
    // can branch on how old the file was; by the time it runs, config.plugins[id].schemaVersion is already
    // current. Not called when the section is absent or already current. Newer-than-supported sections are
    // refused by the coordinator (VersionMismatch) before any hook runs.
    //
    // OWNERSHIP RULE: the hook may only modify this plugin's section and mechanisms for which
    // ownsMechanismType(metadata().id, mechanism.type) holds (config/plugin_config.h). It must not touch
    // anything else; the coordinator may reject a result that does. Idempotent and deterministic; pure
    // data transformation (no logging side effects needed, no SimContext exists yet).
    virtual Status migrateRobotConfig(RobotConfig& /*config*/, int /*persistedPluginVersion*/) const {
        return okStatus();
    }

    virtual std::vector<EditorPanelDescriptor> contributeEditorPanels() const { return {}; }
    virtual std::vector<HudWidgetDescriptor> contributeHud() const { return {}; }

    // Cross-cutting game logic hook after IGameState::step each tick (e.g. match timer phases).
    virtual void onTick(SimContext& /*ctx*/, double /*dtSeconds*/) {}
};

}  // namespace wpiemu::sdk
