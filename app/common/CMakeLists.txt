add_library(wpiemu_app_common STATIC
    app_services.cpp
    app_settings.cpp
    config_repository.cpp
    demo_runtime_host.cpp
    icon_provider.cpp
    key_value_store.cpp
    project_inspector.cpp
    project_path.cpp
    project_prefs.cpp
    project_session.cpp
    recent_projects.cpp
    real_runtime_host.cpp
    robot_basics.cpp
    system_probe.cpp
    theme.cpp
    toolchain_checker.cpp
    build_tools_installer.cpp
    toolchain_provisioner.cpp
    ui_helpers.cpp
    ui_resources.cpp
    unit_system.cpp
    update_install.cpp
    update_offer.cpp
    update_service.cpp
    update_transport.cpp
    # The setup wizard sources live in ../wizard and are compiled into this library.
    ../wizard/project_drop_zone.cpp
    ../wizard/project_page.cpp
    ../wizard/robot_page.cpp
    ../wizard/robot_schematic_view.cpp
    ../wizard/summary_page.cpp
    ../wizard/toolchain_page.cpp
    ../wizard/wizard_dialog.cpp
    ../wizard/wizard_page.h
    ../wizard/wizard_view_model.cpp
    # The application shell sources live in ../shell and are compiled into this library.
    ../shell/about_dialog.cpp
    ../shell/main_window.cpp
    ../shell/mode_switcher.cpp
    ../shell/page_factory.cpp
    ../shell/settings_dialog.cpp
    ../shell/shell_controller.cpp
    ../shell/update_button.cpp
    ../shell/welcome_page.cpp)
# Includes are rooted at app/ ("common/x.h", "wizard/x.h", "shell/x.h").
target_include_directories(wpiemu_app_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
# The pinned Java downloads, generated from versions.json ("pins/toolchain_pins.h").
wpiemu_write_toolchain_pins("${CMAKE_CURRENT_BINARY_DIR}/generated/pins/toolchain_pins.h")
target_include_directories(wpiemu_app_common PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(WIN32)
    target_link_libraries(wpiemu_app_common PRIVATE version)   # GetFileVersionInfo: the Visual C++ runtime of a JDK
endif()
target_link_libraries(wpiemu_app_common
    PUBLIC wpiemu_sdk wpiemu_core wpiemu_app_host wpiemu_app_ui Qt6::Core Qt6::Gui     Qt6::Widgets Qt6::Svg Qt6::Network
    PRIVATE wpiemu_plugins_builtin)
target_compile_definitions(wpiemu_app_common PRIVATE WPIEMU_APP_VERSION="${PROJECT_VERSION}")
wpiemu_target_defaults(wpiemu_app_common)

# The editor and run view are separate libraries owned by their own streams. The shell adapts the
# application to the editor's backend interface and hands the run view the shared runtime host; a placeholder
# page stands in when a page library is not part of the build.
option(WPIEMU_APP_USE_EDITOR "Embed the robot editor page when its library is part of the build" ON)
option(WPIEMU_APP_USE_RUNVIEW "Embed the run view page when its library is part of the build" ON)
if(WPIEMU_APP_USE_EDITOR AND TARGET wpiemu_app_editor AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../editor/editor_page.h")
    target_sources(wpiemu_app_common PRIVATE ../shell/editor_backend_adapter.cpp)
    target_link_libraries(wpiemu_app_common PUBLIC wpiemu_app_editor)
    target_compile_definitions(wpiemu_app_common PUBLIC WPIEMU_APP_HAS_EDITOR)
endif()
if(WPIEMU_APP_USE_RUNVIEW AND TARGET wpiemu_app_runview AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../runview/run_page.h")
    target_link_libraries(wpiemu_app_common PUBLIC wpiemu_app_runview)
    target_compile_definitions(wpiemu_app_common PUBLIC WPIEMU_APP_HAS_RUNVIEW)
endif()
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
