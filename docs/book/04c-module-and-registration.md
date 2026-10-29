# Week 5 workshop: introduce the game to the emulator

**In 30 seconds:** a module is a stateless factory and catalog. Live cubes, timers and scores
belong to `State`, not to this object. Finish the source begun in the previous workshops.

## 1. Describe it honestly, then construct it

The ID and year must agree with the season pack. The description names the prototype's limitations
so the game selector does not promise a full rules implementation. No special mechanism template
is needed: our release uses the already-registered generic rotary mechanism.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
class Module final : public s::IGameModule {
public:
    s::GameMetadata metadata() const override {
        s::GameMetadata result;
        result.id = kId;
        result.displayName = "FIRST POWER UP (2018, temporary 2027)";
        result.year = 2027;
        result.description = "Temporary single-robot prototype: physical cubes and plates, ownership scoring. "
                             "Not a complete manual implementation: no vault, power-ups or assisted climbs.";
        return result;
    }

    s::Status registerMechanisms(s::MechanismFactory&) override { return s::okStatus(); }
```

Factories create an owner, call its fallible build method and return an SDK interface pointer.
If any shape failed, the local unique pointer cleans up partial construction. No invalid object
escapes disguised as a successfully built game.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::Result<std::unique_ptr<s::IField>> createField(s::IPhysicsWorld& world) override {
        auto field = std::make_unique<Field>(world);
        const auto status = field->build();
        if (!status) return status.error();
        return std::unique_ptr<s::IField>(std::move(field));
    }
```

For game state, decode the plugin section, supply defaults, validate the assignment, then
construct. The normal app config pipeline also validates schemas; this factory check protects
direct SDK callers such as our physics fixture. Keep per-match controls out of the module object.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::Result<std::unique_ptr<s::IGameState>> createGameState(s::SimContext& ctx) override {
        s::Value config = ctx.config().findPlugin(kId) ? ctx.config().findPlugin(kId)->data : s::Value::makeObject();
        s::applyDefaults(configSchema().properties, config);
        const auto assignment = config.get("assignment").asString();
        if (!validAssignment(assignment))
            return s::makeError(s::ErrorCode::InvalidArgument, "Power UP assignment must be LLL, LRL, RLR or RRR");
        auto state = std::make_unique<State>(ctx, assignment, config.get("preload").asBool());
        const auto status = state->build();
        if (!status) return status.error();
        return std::unique_ptr<s::IGameState>(std::move(state));
    }
```

## 2. Give the editor metadata, not Qt

Declare the field frame and the two robot interaction kinds. A Volume has extents; a Muzzle
describes the release pose. These declarations let the generic editor offer the controls.
Their behavior was implemented in `interact`; the descriptor by itself does nothing.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    s::PluginConfigSchema contributeConfigSchema() const override { return configSchema(); }

    s::FieldFrame fieldFrame() const override { return {kLengthM, kWidthM, s::FieldSymmetry::Rotational}; }

    std::vector<s::ZoneKindInfo> contributeZoneKinds() const override {
        s::ZoneKindInfo pickup;
        pickup.kind = kPickup;
        pickup.displayName = "Cube pickup";
        pickup.shape = s::ZoneShape::Volume;
        pickup.defaultLocal.position = {0.5f, 0.f, 0.f};
        pickup.defaultSizeM = {0.5f, 0.6f, 0.5f};
        s::ZoneKindInfo release;
        release.kind = kRelease;
        release.displayName = "Cube release";
        release.shape = s::ZoneShape::Muzzle;
        release.defaultLocal.position = {0.75f, 0.f, 0.75f};
        return {pickup, release};
    }
```

Expose the numeric inventory value for generic sensors, then declare the scoreboard. A HUD
descriptor's value key must match a key published by `State`; misspelling it gives a blank widget,
not a new rule. Field colors and score keys are kept together here.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
    std::vector<s::GameValueInfo> contributeGameValues() const override {
        return {{"game_2027.inventory", "Held cube", "One when the robot holds a cube", s::PropertyUnit::Count}};
    }

    std::vector<s::HudWidgetDescriptor> contributeHud() const override {
        s::HudWidgetDescriptor scores;
        scores.id = "game_2027.scoreboard";
        scores.title = "POWER UP";
        scores.kind = s::HudWidgetKind::Scoreboard;
        scores.anchor = s::HudAnchor::TopCenter;
        scores.valueKey = "game_2027.phase";
        scores.sides = {{"BLUE", "game_2027.blue_score", "", "#2457c5"},
                        {"RED", "game_2027.red_score", "", "#c52c30"}};
```

Add the inventory and FMS widgets. These are small, useful checks on the live scene: after release,
inventory should go from one to zero, while the assignment stays unchanged.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        s::HudWidgetDescriptor inventory;
        inventory.id = "game_2027.inventory";
        inventory.title = "Held cube";
        inventory.valueKey = inventory.id;
        s::HudWidgetDescriptor assignment;
        assignment.id = "game_2027.game_data";
        assignment.title = "FMS";
        assignment.kind = s::HudWidgetKind::Text;
        assignment.valueKey = assignment.id;
```

Finally show each alliance's portal reserve and register the factory. The macro defines the
public `registerPlugin` declared in our header. This closes both namespaces and completes the
translation unit. You now have real plugin source, not instructions to write it later.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
        s::HudWidgetDescriptor blueSupply;
        blueSupply.id = "game_2027.portal_blue";
        blueSupply.title = "Blue portal supply";
        blueSupply.valueKey = blueSupply.id;
        s::HudWidgetDescriptor redSupply;
        redSupply.id = "game_2027.portal_red";
        redSupply.title = "Red portal supply";
        redSupply.valueKey = redSupply.id;
        return {scores, inventory, assignment, blueSupply, redSupply};
    }
};

}  // namespace

WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

## 3. Connect the two composition seams

In the scratch export, replace the existing season manifest with this listing. Only its optional
game entry changes; the ABI, WPILib pin and toolchain stay the season pack's responsibility.

<!-- book-file: seasons\2027\season.json -->
```json
{
  "year": 2027,
  "channel": "preview",
  "default": false,
  "wpilib": {
    "version": "2027.0.0-alpha-7",
    "verified": ["2027.0.0-alpha-7"],
    "installFolder": "2027_alpha7"
  },
  "platform": { "abi": "systemcore", "launch": "gradlerio-modern" },
  "toolchain": { "javaMajor": 25 },
  "vendors": {
    "photonvision": {
      "schema": "photon-ns",
      "defaultVersion": "dev-v2027.0.0-alpha-2-86-gda55d785",
      "verified": ["dev-v2027.0.0-alpha-2-86-gda55d785"]
    }
  },
  "game": { "plugin": "game_2027" },
  "probes": "samples/season_2027",
  "samples": [
    "samples/season_2027/java",
    "samples/season_2027/cpp",
    "samples/season_2027/vendor_java",
    "samples/season_2027/vision_java",
    "samples/season_2027/vision_cpp"
  ]
}
```

Replace the existing aggregator with this listing. It includes the public entry point and
registers both shipped games. Preserve its first-error handling. **Do not edit**
`seasons/CMakeLists.txt`: it already discovers each manifest's game folder and links its target.
The app and extension do not acquire a direct dependency on this plugin.

<!-- book-file: seasons\builtin_plugins.cpp -->
```cpp
// The plugins/ aggregation seam: the ONLY translation unit that knows every built-in year plugin.
// Declared in sdk/game/builtin_plugins.h so app/core can call it without seeing any plugin.
// The season manifest loop discovers the game target. Include its public entry point and
// add its registerPlugin call below; core and app do not depend on the individual plugin.
#include <utility>

#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::sdk {

Status registerBuiltinGamePlugins(GamePluginRegistry& registry) {
    Status first = okStatus();
    auto keepFirstError = [&first](Status s) {
        if (first.ok() && !s.ok()) first = std::move(s);
    };

    keepFirstError(plugins::game_2026::registerPlugin(registry));
    keepFirstError(plugins::game_2027::registerPlugin(registry));

    return first;
}

}  // namespace wpiemu::sdk
```

## 4. First complete build checkpoint

From the scratch export root, use the toolchain setup in [Kickoff](01-kickoff.md):

```powershell
cmake -S . -B build\power-up-book -G "Visual Studio 18 2026" -A x64 `
  -DWPIEMU_BUILD_APP=OFF -DWPIEMU_BUILD_TESTS=ON `
  -DWPIEMU_WPILIB_SEASON=2027 -DWPIEMU_WPILIB_VERSION=2027.0.0-alpha-7 `
  -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build build\power-up-book --config Release --target wpiemu_plugins_builtin --parallel
python tools\check_boundaries.py
python tools\check_seasons.py
```

**Quick win:** a successful aggregator build proves the field, game state, module and registration
compile together with the SDK-only boundary. It does not yet prove that a cube can settle on a
plate. That is the next inspection, not a conclusion to draw from the linker.

Next: [plan the inspection](05-test-and-run.md), then connect the three-PWM robot and saved model.
