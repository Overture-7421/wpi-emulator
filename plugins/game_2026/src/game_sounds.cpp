#include "wpiemu/plugins/game_2026/game_sounds.h"

namespace wpiemu::plugins::game_2026 {

std::vector<sdk::SoundCueInfo> soundCueInfos() {
    return {{sound::kShiftChange, "Alliance shift", "An ALLIANCE SHIFT starts: the hubs swap active and inactive", "shift_change"}};
}

}  // namespace wpiemu::plugins::game_2026
