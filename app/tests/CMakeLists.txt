# Unit tests for the application shell (models, view-models) and GUI smoke tests that drive the
# wizard and the main window with QT_QPA_PLATFORM=offscreen.
find_package(Qt6 REQUIRED COMPONENTS Test)
add_executable(wpiemu_app_tests
    main.cpp
    test_app_services.cpp
    test_config_session.cpp
    test_gui_shell.cpp
    test_gui_wizard.cpp
    test_project_inspector.cpp
    test_real_host.cpp
    test_robot_basics.cpp
    test_settings_models.cpp
    test_shell_controller.cpp
    test_toolchain_checker.cpp
    test_wizard_view_model.cpp)
target_link_libraries(wpiemu_app_tests PRIVATE doctest::doctest wpiemu_app_shell Qt6::Test)
target_include_directories(wpiemu_app_tests PRIVATE "${CMAKE_SOURCE_DIR}/tests/support")
target_compile_definitions(wpiemu_app_tests PRIVATE
    DOCTEST_CONFIG_USE_STD_HEADERS
    "WPIEMU_SAMPLES_DIR=\"${CMAKE_SOURCE_DIR}/samples\"")
wpiemu_target_defaults(wpiemu_app_tests)

add_test(NAME app.ui_tests COMMAND wpiemu_app_tests)
# Qt DLLs are not deployed into the build tree; make them discoverable and run headless.
set(_wpiemu_app_test_env "QT_QPA_PLATFORM=set:offscreen")
if(WIN32)
    get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_RELEASE)
    if(NOT _qt_core_loc)
        get_target_property(_qt_core_loc Qt6::Core LOCATION)
    endif()
    get_filename_component(_qt_bin_dir "${_qt_core_loc}" DIRECTORY)
    list(APPEND _wpiemu_app_test_env "PATH=path_list_prepend:${_qt_bin_dir}")
endif()
set_tests_properties(app.ui_tests PROPERTIES ENVIRONMENT_MODIFICATION "${_wpiemu_app_test_env}")
