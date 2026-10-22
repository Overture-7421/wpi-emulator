include(sdk/header_check.cmake)

# Test sources are globbed so module owners can add tests without touching this file.
file(GLOB_RECURSE WPIEMU_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
if(NOT WPIEMU_BUILD_PLUGIN_TEMPLATE)
    list(FILTER WPIEMU_TEST_SOURCES EXCLUDE REGEX "test_template_plugin\\.cpp$")
endif()
# Qt UI suites with their own executable and event loop (see app/runview/CMakeLists.txt).
list(FILTER WPIEMU_TEST_SOURCES EXCLUDE REGEX "/tests/app_runview/")

file(GLOB WPIEMU_HALSIM_EXT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/halsim_ext/tests/test_*.cpp")
list(APPEND WPIEMU_TEST_SOURCES
    ${WPIEMU_HALSIM_EXT_TEST_SOURCES}
    "${CMAKE_SOURCE_DIR}/app/host/tests/test_session_failures.cpp")

add_executable(wpiemu_tests ${WPIEMU_TEST_SOURCES} $<TARGET_OBJECTS:wpiemu_sdk_header_check>)
target_link_libraries(wpiemu_tests PRIVATE
    wpiemu_sdk wpiemu_core wpiemu_ipc wpiemu_halsim_core wpiemu_app_host
    wpiemu_bridges_hal wpiemu_bridges_vendor wpiemu_bridges_vision
    wpiemu_plugin_game_2026 wpiemu_plugins_builtin)
if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    target_link_libraries(wpiemu_tests PRIVATE wpiemu_plugin_template)
endif()
wpiemu_doctest_target(wpiemu_tests)
target_include_directories(wpiemu_tests PRIVATE support "${CMAKE_SOURCE_DIR}/halsim_ext/tests")
target_compile_definitions(wpiemu_tests PRIVATE "WPIEMU_SAMPLES_DIR=\"${CMAKE_SOURCE_DIR}/samples\""
                                                "WPIEMU_PLUGINS_DIR=\"${CMAKE_SOURCE_DIR}/plugins\"")

doctest_discover_tests(wpiemu_tests)

# Architecture rule: core/app/sdk never see plugins; plugins only see sdk.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME boundaries.check
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_boundaries.py"
            --root "${CMAKE_SOURCE_DIR}")
add_test(NAME boundaries.self_test
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_boundaries.py" --self-test)
add_test(NAME docs.check
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_docs.py"
            --root "${CMAKE_SOURCE_DIR}")
add_test(NAME versions.check
    COMMAND Python3::Interpreter "${CMAKE_SOURCE_DIR}/tools/check_versions.py"
            --root "${CMAKE_SOURCE_DIR}")
add_test(NAME infrastructure.release_tools
    COMMAND Python3::Interpreter -B -m unittest discover
            -s "${CMAKE_SOURCE_DIR}/tools/tests" -v)

if(WPIEMU_BUILD_APP)
    add_test(NAME app.version COMMAND wpiemu_app --version)
    wpiemu_qt_test_environment(app.version)
endif()
