#pragma once

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {

class GamePluginRegistry;

// Aggregation seam (ADR 0004/0005). DECLARED here so app/core can call it without including or
// linking any plugin; DEFINED by the `wpiemu_plugins_builtin` target owned by plugins/, which is
// the only place that knows every built-in plugin. The app links that one target.
// Adding a year = new plugins/game_<year>/ folder + one line in plugins/builtin_plugins.cpp.
// Returns the first registration error (all plugins are still attempted).
Status registerBuiltinGamePlugins(GamePluginRegistry& registry);

}  // namespace wpiemu::sdk
