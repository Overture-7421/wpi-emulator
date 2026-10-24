// The plugins/ aggregation seam: the ONLY translation unit that knows every built-in year plugin.
// Declared in sdk/game/builtin_plugins.h so app/core can call it without seeing any plugin.
// Adding a season: add the plugin's CMake target to seasons/CMakeLists.txt (aggregator link line)
// and one `registerPlugin` call below. Nothing else in the repo changes.
#include <utility>

#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"

namespace wpiemu::sdk {

Status registerBuiltinGamePlugins(GamePluginRegistry& registry) {
    Status first = okStatus();
    auto keepFirstError = [&first](Status s) {
        if (first.ok() && !s.ok()) first = std::move(s);
    };

    keepFirstError(plugins::game_2026::registerPlugin(registry));
    // keepFirstError(plugins::game_2027::registerPlugin(registry));

    return first;
}

}  // namespace wpiemu::sdk
