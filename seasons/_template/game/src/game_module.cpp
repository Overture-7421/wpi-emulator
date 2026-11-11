// SKELETON year plugin. To start a new season, copy seasons/_template/game to seasons/<year>/game/, rename ids /
// namespaces / targets (see README.md and docs/extending.md), then replace the placeholder bodies below.
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
#include "wpiemu/sdk/core/sim_context.h"
#include "wpiemu/sdk/game/alliance.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/per_robot_state.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/game/robot_view.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::plugins::_template {

namespace {

// ---- 2. the static field ---------------------------------------------------------------------------------------
// Owns whatever createField() added to the physics world and removes it in the destructor (RAII). It is destroyed
// BEFORE the world. TODO(new season): create a static body with the season's colliders (boxes, cylinders, meshes) and
// sensor volumes, fill FieldDescription::visuals for the renderer, optionally provide
// FieldDescription::driverStations for accurate Run-view Driver POV positions, and keep the ids you need.
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
//
// SEVERAL ROBOTS (practice matches), the pattern that works (docs/guides/game-plugins.md, "Several robots"):
//   * everything one robot owns (a meter, held pieces, shot counters, zone runtimes) lives in a RobotState kept in an
//     sdk::PerRobotState: created in onRobotAdded, dropped in onRobotRemoved, iterated host first, then by ascending id;
//   * the host robot is always in the container; remote robots come and go. A remote robot has no mechanisms: its zones
//     are the ones of its Join spec (RobotView::zones) and each is on while the client's flag is set
//     (RobotView::zoneFlagActive), the robot is enabled and the match is running (RobotView::enabled);
//   * the alliance of a robot is RobotView::alliance() (from its station), never a plugin setting;
//   * a remote robot's values reach the UI through publishRobot (SnapshotBuilder::setRobotHud).
// The skeleton counts the ticks a remote robot spent with a marker zone active. Replace RobotState and the body of
// stepRemote with the season's per-robot rules; sdk::testing::remoteRobotContractViolation checks the wiring.
class TemplateGameState final : public sdk::IGameState {
public:
    std::string_view pluginId() const override { return _template::pluginId(); }
    void reset(sdk::SimContext& /*ctx*/) override {  // restore the initial layout, counters, timers; keep the roster
        ticks_ = 0;
        robots_.forEach([](sdk::ParticipantId, RobotState& robot) { robot = RobotState{}; });
    }
    void step(sdk::SimContext& ctx, double /*dtSeconds*/) override {
        ++ticks_;
        robots_.forEachRemote([&](sdk::ParticipantId id, RobotState& robot) {
            if (const sdk::RobotView* view = ctx.robots().find(id)) stepRemote(*view, robot);
        });
    }
    // Feed the snapshot: HUD values (keys are "<pluginId>.<name>") and entity groups for instanced rendering.
    void publish(sdk::SnapshotBuilder& out) const override { out.setHud(hudKey(), ticks_); }

    void onRobotAdded(sdk::SimContext& /*ctx*/, const sdk::RobotView& robot) override {
        if (!robot.isHost) robots_.add(robot.participantId, RobotState{});  // the host is always present
    }
    void onRobotRemoved(sdk::SimContext& /*ctx*/, sdk::ParticipantId participant) override {
        robots_.remove(participant);  // release what it held (return pieces to the field...)
    }
    void publishRobot(sdk::SnapshotBuilder& out, const sdk::RobotView& robot) const override {
        const RobotState* state = robots_.find(robot.participantId);
        if (state == nullptr) return;
        out.setRobotHud(robot.participantId, markerKey(), state->markerTicks);
        out.setRobotHud(robot.participantId, allianceKey(), std::string(sdk::allianceKey(robot.alliance())));
    }

    static std::string hudKey() { return std::string(_template::pluginId()) + ".ticks"; }
    static std::string markerKey() { return std::string(_template::pluginId()) + ".marker_ticks"; }
    static std::string allianceKey() { return std::string(_template::pluginId()) + ".alliance"; }
    static std::string markerKind() { return std::string(_template::pluginId()) + ".marker"; }

private:
    struct RobotState {
        int markerTicks = 0;  // TODO(new season): the season's per-robot state
    };

    // `view.zones` is in the robot's zone order; zone i is on when its flag bit i is set.
    static void stepRemote(const sdk::RobotView& view, RobotState& robot) {
        if (!view.enabled || view.zones == nullptr) return;
        for (std::size_t i = 0; i < view.zones->size(); ++i)
            if ((*view.zones)[i].kind == markerKind() && view.zoneFlagActive(i)) {
                ++robot.markerTicks;
                return;
            }
    }

    int ticks_ = 0;
    sdk::PerRobotState<RobotState> robots_;
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
        m.supportsMultiRobot = true;  // the game state below reacts to remote robots; leave false if yours does not
        return m;
    }

    // 1. season-specific mechanism templates. Keys MUST be prefixed with the plugin id ("<id>.<name>") so the
    //    ownership rule (sdk::ownsPluginId) holds. Prefer wrapping the generic core mechanisms through the
    //    values they publish (see zones in seasons/2026/game/include/.../fuel_zones.h) over re-implementing them.
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
    //    TODO(new season): describe your fields with PropertySpec (key, label, type, default, range, group), bound to a
    //    typed config struct through sdk::ConfigBindings (see seasons/2026/game/src/config.cpp): one table, no repeats.
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

    // 6. optional: zone kinds the editor offers and robots (also remote ones) declare in RobotConfig::zones. Every kind is
    //    prefixed with the plugin id. A plugin that declares zone kinds must react to remote robots' zones: see the
    //    game state above and sdk/testing/multi_robot_contract.h.
    std::vector<sdk::ZoneKindInfo> contributeZoneKinds() const override {
        sdk::ZoneKindInfo marker;
        marker.kind = TemplateGameState::markerKind();
        marker.displayName = "Marker";
        marker.description = "Placeholder zone: the game counts the ticks a remote robot holds it active.";
        return {std::move(marker)};
    }

    // 7. optional: the season's AprilTag layout for the simulated vision cameras (default: empty = no tags).
    //    Set fieldLengthM/fieldWidthM too: Limelight red- and center-origin poses are not published without them.
    //    Poses are in the field frame; a tag's +X points out of its face (WPILib convention). See
    //    seasons/2026/game/src/april_tags.cpp for a table copied from the official WPILib JSON.
    // sdk::AprilTagLayout aprilTagLayout() const override;

    // 8. cross-cutting per-tick hook after IGameState::step. Usually unnecessary: keep logic in the game state.
    void onTick(sdk::SimContext& /*ctx*/, double /*dtSeconds*/) override {}
};

}  // namespace

// Defines `sdk::Status registerPlugin(sdk::GamePluginRegistry&)` in THIS namespace (declared in plugin.h).
// seasons/builtin_plugins.cpp calls it; that is the only registration a new season needs.
WPIEMU_REGISTER_GAME_PLUGIN(TemplateModule)

}  // namespace wpiemu::plugins::_template
