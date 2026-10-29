# 5. Plugin skeleton and registration

## 5.1 Objectives

After this chapter the reader can build a static plugin library, register its
module, and select it through a season manifest. The prerequisites are Chapters
1-4 and a working repository build. The artifacts are the plugin entry point,
library target and registry test; they address R1 and R2.

## 5.2 Problem statement

The smallest useful checkpoint establishes that the host can find the module.
It need not contain a field or score a match. Separating this integration from
physics makes a missing registration distinguishable from a failed collider.

## 5.3 Background and definitions

**Definition 5.1 (module).** A module is a registry-owned factory for a field,
game state and presentation descriptors. The field owns static bodies; game
state owns mutable match data. The module itself has no per-match state.

Registration is static linking, not discovery of an external DLL. A static
library is an archive of object files that the build links into the application
through the plugin aggregator. CMake describes those relationships. The SDK
macro defines a namespaced `registerPlugin()` function; the aggregator calls it.

## 5.4 Design

The repository template supplies a larger starting example. An alternative is
a module using `EmptyField` and `NullGameState`. The latter is selected because
its first test has only one concern: identity. These SDK null objects are
intentional implementations of an empty game, not substitutes for failed
construction.

**Invariant 5.1 (identity).** Folder, namespace, metadata ID, registry lookup and
manifest name agree on `game_2027`. The metadata year identifies the host season
pack, while the display name identifies the historical rules.

## 5.5 Implementation

The entry-point header exposes only registration. No application header depends
on the concrete module. The CMake target exports that header and depends only
on the SDK.

Listing 5.1. The public registration entry point. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\plugin.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\plugin.h -->
```cpp
#pragma once

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {
class GamePluginRegistry;
}

namespace wpiemu::plugins::game_2027 {
sdk::Status registerPlugin(sdk::GamePluginRegistry& registry);
}
```

Listing 5.2. The initial static library. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

The initial module returns SDK null objects. Construction still uses `Result`,
so later field and state failures can follow the same interface.

Listing 5.3. The initial module, revision 1. <code>seasons\2027\game\src\game_module.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/sdk/game/game_module.h"

#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::plugins::game_2027 {
namespace s = sdk;

class Module final : public s::IGameModule {
public:
    s::GameMetadata metadata() const override {
        s::GameMetadata result;
        result.id = "game_2027";
        result.displayName = "FIRST POWER UP (2018 rules, 2027 ABI)";
        result.year = 2027;
        return result;
    }

    s::Status registerMechanisms(s::MechanismFactory&) override { return s::okStatus(); }

    s::Result<std::unique_ptr<s::IField>> createField(s::IPhysicsWorld&) override {
        return std::unique_ptr<s::IField>(std::make_unique<s::EmptyField>());
    }

    s::Result<std::unique_ptr<s::IGameState>> createGameState(s::SimContext&) override {
        return std::unique_ptr<s::IGameState>(std::make_unique<s::NullGameState>("game_2027"));
    }

    s::PluginConfigSchema contributeConfigSchema() const override { return {}; }
};

WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

Three existing files connect the library to the repository. The manifest makes
the target discoverable by the season CMake loop. The aggregator supplies the
registration call. The registry test gains an explicit identity assertion as
well as the changed count; changing a count alone could conceal the wrong plugin.

Listing 5.4. Assign the game to the season pack. <code>seasons\2027\season.json</code>.

<!-- book-edit: seasons\2027\season.json -->
```json
{
  "old": "  },\n  \"game\": null,\n  \"probes\": \"samples/season_2027\",\n",
  "new": "  },\n  \"game\": { \"plugin\": \"game_2027\" },\n  \"probes\": \"samples/season_2027\",\n"
}
```

Listing 5.5. Include and register the module. <code>seasons\builtin_plugins.cpp</code>.

<!-- book-edit: seasons\builtin_plugins.cpp -->
```json
{
  "old": "// Declared in sdk/game/builtin_plugins.h so app/core can call it without seeing any plugin.\n// Adding a season: add the plugin's CMake target to seasons/CMakeLists.txt (aggregator link line)\n// and one `registerPlugin` call below. Nothing else in the repo changes.\n#include <utility>\n\n#include \"wpiemu/plugins/game_2026/plugin.h\"\n#include \"wpiemu/sdk/game/builtin_plugins.h\"\n#include \"wpiemu/sdk/game/plugin_registry.h\"\n\nnamespace wpiemu::sdk {\n\nStatus registerBuiltinGamePlugins(GamePluginRegistry& registry) {\n    Status first = okStatus();\n    auto keepFirstError = [&first](Status s) {\n        if (first.ok() && !s.ok()) first = std::move(s);\n    };\n\n    keepFirstError(plugins::game_2026::registerPlugin(registry));\n    // keepFirstError(plugins::game_2027::registerPlugin(registry));\n\n",
  "new": "// Declared in sdk/game/builtin_plugins.h so app/core can call it without seeing any plugin.\n// The season manifest loop discovers the game target. Include its public entry point and\n// add its registerPlugin call below; core and app do not depend on the individual plugin.\n#include <utility>\n\n#include \"wpiemu/plugins/game_2026/plugin.h\"\n#include \"wpiemu/plugins/game_2027/plugin.h\"\n#include \"wpiemu/sdk/game/builtin_plugins.h\"\n#include \"wpiemu/sdk/game/plugin_registry.h\"\n\nnamespace wpiemu::sdk {\n\nStatus registerBuiltinGamePlugins(GamePluginRegistry& registry) {\n    Status first = okStatus();\n    auto keepFirstError = [&first](Status s) {\n        if (first.ok() && !s.ok()) first = std::move(s);\n    };\n\n    keepFirstError(plugins::game_2026::registerPlugin(registry));\n    keepFirstError(plugins::game_2027::registerPlugin(registry));\n\n"
}
```

Listing 5.6. Account for the new built-in identity. <code>tests\plugins\test_new_year_plugin.cpp</code>.

<!-- book-edit: tests\plugins\test_new_year_plugin.cpp -->
```json
{
  "old": "    World w;\n    REQUIRE(w.reg.plugins.size() == 2);\n    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);\n    REQUIRE(w.reg.plugins.findByYear(2998) != nullptr);\n    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == \"game_2026\");\n    CHECK(w.reg.plugins.findByYear(2998)->metadata().id == \"game_2998\");\n",
  "new": "    World w;\n    REQUIRE(w.reg.plugins.size() == 3);\n    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);\n    REQUIRE(w.reg.plugins.findByYear(2027) != nullptr);\n    REQUIRE(w.reg.plugins.findByYear(2998) != nullptr);\n    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == \"game_2026\");\n    CHECK(w.reg.plugins.findByYear(2027)->metadata().id == \"game_2027\");\n    CHECK(w.reg.plugins.findByYear(2998)->metadata().id == \"game_2998\");\n"
}
```

The new test also checks duplicate registration. A duplicate is rejected rather
than replacing a module that a caller may already be using.

Listing 5.7. Registry identity and duplicate rejection. <code>tests\game_2027\test_module.cpp</code>.

<!-- book-file: tests\game_2027\test_module.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2027/plugin.h"
#include "wpiemu/sdk/game/plugin_registry.h"

TEST_CASE("Power UP: module identity and season lookup agree") {
    wpiemu::sdk::GamePluginRegistry registry;
    REQUIRE(wpiemu::plugins::game_2027::registerPlugin(registry));
    const auto* module = registry.find("game_2027");
    REQUIRE(module);
    CHECK(registry.findByYear(2027) == module);
    CHECK(module->metadata().year == 2027);
    CHECK_FALSE(wpiemu::plugins::game_2027::registerPlugin(registry));
}
```

## 5.6 Verification

The identity test uses the registry as its oracle. The existing synthetic
new-season tests remain intact and test selection through the configuration
pipeline. At this checkpoint there is no claim about geometry or robot behavior.
Boundary and season checks independently reject forbidden dependencies and
year-specific host branches.

## 5.7 Reproduction

Create the disposable export and extract through Chapter 5 using
[Appendix B](appendix-b-reproduction.md). Configure with warnings as errors.
Build <code>wpiemu_tests</code> and run:

```powershell
ctest --test-dir build\book -R "Power UP: module identity" --no-tests=error --output-on-failure
python tools\check_boundaries.py
python tools\check_seasons.py
```

## 5.8 Discussion

Invariant 5.1 is an integration property, not an ABI compatibility proof. The
season pack already supplies systemcore support. The field and rule model
remain separate requirements, as specified by R3-R12.

## 5.9 Summary

The module is selectable without adding a year branch to core or the app.
The registry owns the factory, while later chapters supply its per-session
objects. The first checkpoint isolates integration failures.

## 5.10 Exercises

1. Trace the aggregator dependency from the application to the plugin library.
2. Explain why mutable score fields cannot belong in the registry-owned module.
3. Change the duplicate-registration test to inspect the error code.

[Chapter 6](06-field.md)
