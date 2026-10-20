#pragma once

#include <vector>

#include "wpiemu/sdk/game/ui_descriptors.h"

namespace wpiemu::plugins::game_2026 {

// HUD widgets (fuel meter, hub badges, scores, timers) keyed by the values Game2026State publishes.
std::vector<sdk::HudWidgetDescriptor> hudWidgets();

}  // namespace wpiemu::plugins::game_2026
