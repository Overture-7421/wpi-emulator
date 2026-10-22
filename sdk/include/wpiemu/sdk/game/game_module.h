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
#include "wpiemu/sdk/game/game_value.h"
#include "wpiemu/sdk/game/sound_cue.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"
#include "wpiemu/sdk/physics/physics_world.h"
#include "wpiemu/sdk/vision/apriltag.h"

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
// Threading: descriptor queries (including layouts/frames) may run on UI/caller threads and
// must be pure and thread-safe. Registration/construction run on the caller's setup thread,
// including isolated UI field previews. Live game-state ticking/publication run on the sim thread.
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

    // Numbers the game state publishes every tick for generic sensors (SensorConfig::source). Every id must start
    // with metadata().id + '.'; IGameState::gameValue(id) returns the live value.
    virtual std::vector<GameValueInfo> contributeGameValues() const { return {}; }

    // Sound cues this plugin owns, beyond the common ones in sound_cue.h. Every id must start with metadata().id + '.';
    // `asset` names plugins/<id>/assets/sfx/<asset>.wav, which the app embeds and plays when game state calls
    // SimContext::requestSound(id).
    virtual std::vector<SoundCueInfo> contributeSoundCues() const { return {}; }

    // The season's AprilTag layout (field frame), used by the simulated vision cameras. Empty = the season has none
    // and cameras never see a tag.
    virtual AprilTagLayout aprilTagLayout() const { return {}; }

    // Length, width and symmetry of the season's field; lets the editor reuse one alliance's start position for the
    // other. Default: unknown (no symmetry).
    virtual FieldFrame fieldFrame() const { return {}; }

    // Cross-cutting game logic hook after IGameState::step each tick (e.g. match timer phases).
    virtual void onTick(SimContext& /*ctx*/, double /*dtSeconds*/) {}
};

}  // namespace wpiemu::sdk
