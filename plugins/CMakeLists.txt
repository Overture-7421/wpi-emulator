add_subdirectory(game_2026)

if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    add_subdirectory(_template)
endif()

# Sound files the built-in plugins ship (plugins/<name>/assets/sfx/<asset>.wav), handed to the app through one global
# property of "<file>|<name>/sfx/<asset>.wav" entries so it can embed them without naming a plugin. The template ships none.
file(GLOB _wpiemu_plugin_sound_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/assets/sfx/*.wav")
set(_wpiemu_plugin_sounds "")
foreach(_file IN LISTS _wpiemu_plugin_sound_files)
    file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_file}")
    if(NOT _rel MATCHES "^_template/")
        string(REPLACE "/assets/sfx/" "/sfx/" _alias "${_rel}")
        list(APPEND _wpiemu_plugin_sounds "${_file}|${_alias}")
    endif()
endforeach()
set_property(GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_SOUNDS "${_wpiemu_plugin_sounds}")

# Aggregation seam (docs/ARCHITECTURE.md): the single target that links every built-in plugin and
# defines wpiemu::sdk::registerBuiltinGamePlugins(). The app links only this target, never a
# wpiemu_plugin_* target. The template plugin is deliberately NOT included.
add_library(wpiemu_plugins_builtin STATIC builtin_plugins.cpp)
add_library(wpiemu::plugins_builtin ALIAS wpiemu_plugins_builtin)
target_link_libraries(wpiemu_plugins_builtin PUBLIC wpiemu_sdk PRIVATE wpiemu_plugin_game_2026)
wpiemu_target_defaults(wpiemu_plugins_builtin)
