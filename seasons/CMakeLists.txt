# Season packs (docs/reference/seasons.md). Each seasons/<year>/game is an optional game plugin that links only
# wpiemu_sdk; the manifest names it, and builtin_plugins.cpp registers it. Platform-only seasons have no game/ directory.
wpiemu_season_years(_wpiemu_season_years)
set(_wpiemu_plugin_targets "")
set(_wpiemu_plugin_sounds "")
foreach(_year IN LISTS _wpiemu_season_years)
    wpiemu_season_game_plugin(_plugin "${_year}")
    if(NOT _plugin)
        continue()
    endif()
    add_subdirectory("${_year}/game")
    list(APPEND _wpiemu_plugin_targets "wpiemu_plugin_${_plugin}")
    # Sound files a plugin ships (<year>/game/assets/sfx/<asset>.wav), handed to the app through one global property of
    # "<file>|<plugin>/sfx/<asset>.wav" entries so it can embed them without naming a plugin.
    file(GLOB _sounds CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${_year}/game/assets/sfx/*.wav")
    foreach(_file IN LISTS _sounds)
        get_filename_component(_asset "${_file}" NAME)
        list(APPEND _wpiemu_plugin_sounds "${_file}|${_plugin}/sfx/${_asset}")
    endforeach()
endforeach()
set_property(GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_SOUNDS "${_wpiemu_plugin_sounds}")
set_property(GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_TARGETS "${_wpiemu_plugin_targets}")

if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    add_subdirectory(_template/game)
endif()

# Aggregation seam (docs/ARCHITECTURE.md): the single target that links every built-in plugin and
# defines wpiemu::sdk::registerBuiltinGamePlugins(). The app links only this target, never a
# wpiemu_plugin_* target. The template plugin is deliberately NOT included.
add_library(wpiemu_plugins_builtin STATIC builtin_plugins.cpp)
add_library(wpiemu::plugins_builtin ALIAS wpiemu_plugins_builtin)
target_link_libraries(wpiemu_plugins_builtin PUBLIC wpiemu_sdk PRIVATE ${_wpiemu_plugin_targets})
wpiemu_target_defaults(wpiemu_plugins_builtin)
