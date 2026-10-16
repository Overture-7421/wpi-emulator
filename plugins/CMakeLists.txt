add_subdirectory(game_2026)

if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    add_subdirectory(_template)
endif()

# Aggregation seam (ADR 0004/0005): the single target that links every built-in plugin and
# defines wpiemu::sdk::registerBuiltinGamePlugins(). The app links only this target, never a
# wpiemu_plugin_* target. The template plugin is deliberately NOT included.
add_library(wpiemu_plugins_builtin STATIC builtin_plugins.cpp)
add_library(wpiemu::plugins_builtin ALIAS wpiemu_plugins_builtin)
target_link_libraries(wpiemu_plugins_builtin PUBLIC wpiemu_sdk PRIVATE wpiemu_plugin_game_2026)
wpiemu_target_defaults(wpiemu_plugins_builtin)
