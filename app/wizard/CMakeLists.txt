add_library(wpiemu_app_wizard STATIC
    project_drop_zone.cpp
    project_page.cpp
    robot_page.cpp
    robot_schematic_view.cpp
    summary_page.cpp
    toolchain_page.cpp
    wizard_dialog.cpp
    wizard_page.h
    wizard_view_model.cpp)
target_include_directories(wpiemu_app_wizard PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_wizard PUBLIC wpiemu_app_common)
wpiemu_target_defaults(wpiemu_app_wizard)
