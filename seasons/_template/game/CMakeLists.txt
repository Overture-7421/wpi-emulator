# Skeleton for a new year plugin. Copy seasons/_template to seasons/<year>/ and follow README.md.
# A plugin may link ONLY wpiemu_sdk (enforced by tools/check_boundaries.py): no core, app, ipc, bridges, Qt,
# Box3D or another plugin. Physics goes through sdk::IPhysicsWorld.
#
# Rename on copy:  wpiemu_plugin_template -> wpiemu_plugin_game_<year>   (and the wpiemu::plugin_template alias)
add_library(wpiemu_plugin_template STATIC
    src/plugin_info.cpp
    src/game_module.cpp)
add_library(wpiemu::plugin_template ALIAS wpiemu_plugin_template)
target_include_directories(wpiemu_plugin_template PUBLIC include)
target_link_libraries(wpiemu_plugin_template PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_template)
