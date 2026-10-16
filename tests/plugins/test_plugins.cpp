#include <doctest/doctest.h>

#include "wpiemu/plugins/game_2026/plugin_info.h"

TEST_CASE("game_2026 plugin identifies itself") {
    CHECK(wpiemu::plugins::game_2026::pluginId() == "game_2026");
    CHECK(wpiemu::plugins::game_2026::gameYear() == 2026);
}
