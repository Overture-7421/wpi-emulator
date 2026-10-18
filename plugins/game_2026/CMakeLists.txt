# 2026 (REBUILT) game plugin. May link ONLY wpiemu_sdk (enforced by tools/check_boundaries.py).
add_library(wpiemu_plugin_game_2026 STATIC
    src/config.cpp
    src/field_layout.cpp
    src/fuel_layout.cpp
    src/fuel_pool.cpp
    src/fuel_zones.cpp
    src/game_module.cpp
    src/game_state.cpp
    src/game_ui.cpp
    src/hub_rules.cpp
    src/match_clock.cpp
    src/plugin_info.cpp)
add_library(wpiemu::plugin_game_2026 ALIAS wpiemu_plugin_game_2026)
target_include_directories(wpiemu_plugin_game_2026 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2026 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2026)
