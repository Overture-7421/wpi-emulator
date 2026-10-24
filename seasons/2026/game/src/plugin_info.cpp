#include "wpiemu/plugins/game_2026/plugin_info.h"

#include "wpiemu/sdk/version.h"

namespace wpiemu::plugins::game_2026 {

std::string_view pluginId() { return "game_2026"; }

int gameYear() { return 2026; }

}  // namespace wpiemu::plugins::game_2026
