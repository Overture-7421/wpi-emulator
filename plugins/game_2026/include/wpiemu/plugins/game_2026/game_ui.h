#pragma once

#include <vector>

#include "wpiemu/sdk/game/ui_descriptors.h"

namespace wpiemu::plugins::game_2026 {

// HUD widgets (fuel meter, hub badges, scores, timers) keyed by the values Game2026State publishes.
std::vector<sdk::HudWidgetDescriptor> hudWidgets();

// Plugin-config panels (field staging, match rules, fuel meter). Pickup and muzzle zones are described by
// zoneKinds() (fuel_zones.h), not by panels.
std::vector<sdk::EditorPanelDescriptor> editorPanels();

}  // namespace wpiemu::plugins::game_2026
