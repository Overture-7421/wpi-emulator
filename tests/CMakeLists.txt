include(doctest)
include(sdk/header_check.cmake)

# Test sources are globbed so module owners can add tests without touching this file.
file(GLOB_RECURSE WPIEMU_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
if(NOT WPIEMU_BUILD_PLUGIN_TEMPLATE)
    list(FILTER WPIEMU_TEST_SOURCES EXCLUDE REGEX "test_template_plugin\\.cpp$")
endif()
# Qt UI suites with their own executable and event loop (see app/runview/CMakeLists.txt).
list(FILTER WPIEMU_TEST_SOURCES EXCLUDE REGEX "/tests/app_runview/")

add_executable(wpiemu_tests ${WPIEMU_TEST_SOURCES} $<TARGET_OBJECTS:wpiemu_sdk_header_check>)
target_link_libraries(wpiemu_tests PRIVATE
    doctest::doctest
    wpiemu_sdk wpiemu_core wpiemu_ipc
    wpiemu_bridges_hal wpiemu_bridges_vendor
    wpiemu_plugin_game_2026 wpiemu_plugins_builtin)
if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    target_link_libraries(wpiemu_tests PRIVATE wpiemu_plugin_template)
endif()
wpiemu_target_defaults(wpiemu_tests)
target_compile_definitions(wpiemu_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)

doctest_discover_tests(wpiemu_tests)

# Architecture rule: core/app/sdk never see plugins; plugins only see sdk.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME boundaries.check
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_boundaries.py"
            --root "${CMAKE_SOURCE_DIR}")
add_test(NAME boundaries.self_test
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_boundaries.py" --self-test)

if(WPIEMU_BUILD_APP)
    add_test(NAME app.version COMMAND wpiemu_app --version)
    # Qt DLLs are not deployed into the build tree; make them discoverable when running here.
    if(WIN32)
        get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_RELEASE)
        if(NOT _qt_core_loc)
            get_target_property(_qt_core_loc Qt6::Core LOCATION)
        endif()
        get_filename_component(_qt_bin_dir "${_qt_core_loc}" DIRECTORY)
        set_tests_properties(app.version PROPERTIES
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${_qt_bin_dir}")
    endif()
endif()
