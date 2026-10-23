#pragma once

// Sound cues the 2026 plugin owns (plugins/game_2026/README.md); IGameModule::contributeSoundCues
// defines the shared contract (docs/reference/game-plugin.md). The audio file
// ships with the plugin in plugins/game_2026/assets/sfx/. Cues every season shares (end game, result) are the SDK's.

#include <vector>

#include "wpiemu/sdk/game/sound_cue.h"

namespace wpiemu::plugins::game_2026 {

namespace sound {
// Manual Table 5-4 "ALLIANCE SHIFT starts" ("POWER UP - Linear Popping"): 2:10, 1:45, 1:20 and 0:55.
inline constexpr const char* kShiftChange = "game_2026.shift_change";
}  // namespace sound

std::vector<sdk::SoundCueInfo> soundCueInfos();

}  // namespace wpiemu::plugins::game_2026
