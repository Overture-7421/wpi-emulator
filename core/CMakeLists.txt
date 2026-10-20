# Game-agnostic simulation core. Knows nothing about any season.
add_library(wpiemu_core STATIC)
add_library(wpiemu::core ALIAS wpiemu_core)
target_link_libraries(wpiemu_core PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_core)
add_subdirectory(runtime)
add_subdirectory(mechanisms)
add_subdirectory(physics)
