#pragma once

// PUBLIC ENTRY POINT of the plugin: the only symbol seasons/builtin_plugins.cpp needs.
// When you copy this folder: rename the namespace (wpiemu::plugins::game_<year>) and the include path.

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::sdk {
class GamePluginRegistry;
}

namespace wpiemu::plugins::_template {

// Defined by WPIEMU_REGISTER_GAME_PLUGIN at the end of src/game_module.cpp: adds ONE instance of this plugin's
// IGameModule to the registry. Returns AlreadyExists for a duplicate id, InvalidArgument for bad metadata.
sdk::Status registerPlugin(sdk::GamePluginRegistry& registry);

}  // namespace wpiemu::plugins::_template
