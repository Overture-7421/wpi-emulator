add_library(wpiemu_app_shell STATIC
    about_dialog.cpp
    main_window.cpp
    mode_switcher.cpp
    page_factory.cpp
    settings_dialog.cpp
    shell_controller.cpp
    welcome_page.cpp)
target_include_directories(wpiemu_app_shell PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_shell PUBLIC wpiemu_app_common)
target_compile_definitions(wpiemu_app_shell PRIVATE WPIEMU_APP_VERSION="${PROJECT_VERSION}")
wpiemu_target_defaults(wpiemu_app_shell)

# The editor and run view are separate libraries owned by their own streams. The shell adapts the
# application to their backend interfaces when they are part of the build and shows a placeholder
# page otherwise.
option(WPIEMU_APP_USE_EDITOR "Embed the robot editor page when its library is part of the build" ON)
option(WPIEMU_APP_USE_RUNVIEW "Embed the run view page when its library is part of the build" ON)
if(WPIEMU_APP_USE_EDITOR AND TARGET wpiemu_app_editor AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../editor/editor_page.h")
    target_sources(wpiemu_app_shell PRIVATE editor_backend_adapter.cpp)
    target_link_libraries(wpiemu_app_shell PUBLIC wpiemu_app_editor)
    target_compile_definitions(wpiemu_app_shell PUBLIC WPIEMU_APP_HAS_EDITOR)
endif()
if(WPIEMU_APP_USE_RUNVIEW AND TARGET wpiemu_app_runview AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../runview/run_page.h")
    target_sources(wpiemu_app_shell PRIVATE run_backend_adapter.cpp)
    target_link_libraries(wpiemu_app_shell PUBLIC wpiemu_app_runview)
    target_compile_definitions(wpiemu_app_shell PUBLIC WPIEMU_APP_HAS_RUNVIEW)
endif()
