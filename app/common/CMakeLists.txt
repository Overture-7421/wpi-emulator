add_library(wpiemu_app_common STATIC
    app_services.cpp
    app_settings.cpp
    config_repository.cpp
    demo_runtime_host.cpp
    icon_provider.cpp
    key_value_store.cpp
    project_inspector.cpp
    project_prefs.cpp
    project_session.cpp
    recent_projects.cpp
    real_runtime_host.cpp
    robot_basics.cpp
    system_probe.cpp
    runtime_host.h
    theme.cpp
    toolchain_checker.cpp
    ui_helpers.cpp
    ui_resources.cpp
    unit_system.cpp
    # The setup wizard sources live in ../wizard and are compiled into this library.
    ../wizard/project_drop_zone.cpp
    ../wizard/project_page.cpp
    ../wizard/robot_page.cpp
    ../wizard/robot_schematic_view.cpp
    ../wizard/summary_page.cpp
    ../wizard/toolchain_page.cpp
    ../wizard/wizard_dialog.cpp
    ../wizard/wizard_page.h
    ../wizard/wizard_view_model.cpp)
# Includes are rooted at app/ ("common/x.h", "wizard/x.h", "shell/x.h").
target_include_directories(wpiemu_app_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_common
    PUBLIC wpiemu_sdk wpiemu_core wpiemu_app_host Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Svg
    PRIVATE wpiemu_plugins_builtin)
wpiemu_target_defaults(wpiemu_app_common)
# Build-tree fallback of the extension lookup (an installed app finds halsim_wpiemu next to itself first).
if(TARGET halsim_wpiemu)
    add_dependencies(wpiemu_app_common halsim_wpiemu)
    set_property(SOURCE real_runtime_host.cpp APPEND PROPERTY COMPILE_DEFINITIONS
        "WPIEMU_BUILD_EXTENSION_PATH=\"$<TARGET_FILE:halsim_wpiemu>\"")
endif()

# Icons are plain SVG files compiled into the binary (":/wpiemu/icons/<name>.svg").
file(GLOB WPIEMU_APP_ICONS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../resources/icons/*.svg")
qt_add_resources(wpiemu_app_common wpiemu_app_icons
    PREFIX "/wpiemu"
    BASE "${CMAKE_CURRENT_SOURCE_DIR}/../resources"
    FILES ${WPIEMU_APP_ICONS})
