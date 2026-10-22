# Unit tests for the application shell (models, view-models) and GUI smoke tests that drive the
# wizard and the main window with QT_QPA_PLATFORM=offscreen.
find_package(Qt6 REQUIRED COMPONENTS Test)
add_executable(wpiemu_app_tests
    main.cpp
    test_app_services.cpp
    test_build_tools_installer.cpp
    test_config_session.cpp
    test_gui_shell.cpp
    test_gui_wizard.cpp
    test_project_inspector.cpp
    test_real_host.cpp
    test_robot_basics.cpp
    test_settings_models.cpp
    test_shell_controller.cpp
    test_toolchain_checker.cpp
    test_toolchain_provisioner.cpp
    test_update.cpp
    test_wizard_view_model.cpp)
target_link_libraries(wpiemu_app_tests PRIVATE wpiemu_app_common Qt6::Test)
target_include_directories(wpiemu_app_tests PRIVATE "${CMAKE_SOURCE_DIR}/tests/support")
target_compile_definitions(wpiemu_app_tests PRIVATE "WPIEMU_SAMPLES_DIR=\"${CMAKE_SOURCE_DIR}/samples\"")
wpiemu_doctest_target(wpiemu_app_tests)

add_test(NAME app.ui_tests COMMAND wpiemu_app_tests)
wpiemu_qt_test_environment(app.ui_tests HEADLESS)
