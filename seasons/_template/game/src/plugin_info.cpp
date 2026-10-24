#include "wpiemu/plugins/_template/plugin_info.h"

namespace wpiemu::plugins::_template {

// TODO(new season): "game_<year>"
std::string_view pluginId() { return "_template"; }

// TODO(new season): the season year, e.g. 2027
int gameYear() { return 0; }

}  // namespace wpiemu::plugins::_template
