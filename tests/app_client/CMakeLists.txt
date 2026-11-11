# Game client tests: doctest model tests (join form, state machine, input mapping, settings, scene feed), an in-process
# end-to-end run over a real MatchHost with encryption on, and GUI smoke tests under QT_QPA_PLATFORM=offscreen.
# Added from app/client/CMakeLists.txt; deliberately excluded from the Qt-free wpiemu_tests glob.
file(GLOB WPIEMU_APP_CLIENT_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
find_package(Qt6 REQUIRED COMPONENTS Test)
get_property(_wpiemu_builtin_plugin_targets GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_TARGETS)
add_executable(wpiemu_app_client_tests ${WPIEMU_APP_CLIENT_TEST_SOURCES})
target_link_libraries(wpiemu_app_client_tests PRIVATE
    wpiemu_app_client wpiemu_app_host wpiemu_match wpiemu_net wpiemu_ipc wpiemu_core wpiemu_sdk
    wpiemu_plugins_builtin ${_wpiemu_builtin_plugin_targets} Qt6::Test)
target_include_directories(wpiemu_app_client_tests PRIVATE "${PROJECT_SOURCE_DIR}/tests" "${PROJECT_SOURCE_DIR}/tests/support")
wpiemu_doctest_target(wpiemu_app_client_tests)

add_test(NAME app_client.ui_tests COMMAND wpiemu_app_client_tests)
wpiemu_qt_test_environment(app_client.ui_tests HEADLESS)
