#pragma once

// Identity of the plugin, kept in one place so the module, tests and docs never disagree.
// When you copy this folder: pluginId() -> "game_<year>", gameYear() -> the season (e.g. 2027).

#include <string_view>

namespace wpiemu::plugins::_template {

// Lower-case, unique, equal to the folder and namespace name ("game_2026"). Namespaces plugin config sections,
// mechanism template keys ("<id>.fuel_intake") and HUD keys ("<id>.fuel.count").
std::string_view pluginId();

// The WPILib / GradleRIO season this plugin implements. The template uses 0, which the registry only accepts for
// ids starting with '_' (so the template can never be selected by a real project).
int gameYear();

}  // namespace wpiemu::plugins::_template
