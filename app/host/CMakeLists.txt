# Qt-free orchestration of one robot run (config -> session -> IPC -> launcher). Linked by the Qt shell
# (RealRuntimeHost) and by the opt-in end-to-end test; contains no widgets and no season knowledge.
find_package(Threads REQUIRED)
add_library(wpiemu_app_host STATIC
    robot_session.cpp)
target_include_directories(wpiemu_app_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_host PUBLIC wpiemu_core wpiemu_ipc wpiemu_sdk Threads::Threads)
wpiemu_target_defaults(wpiemu_app_host)

if(WPIEMU_BUILD_TESTS)
    enable_testing()
    include(doctest)

    # Failure handling and state mapping of RobotSession against synthetic projects (fake gradlew scripts): always on.
    add_executable(wpiemu_host_tests
        tests/host_tests_main.cpp
        tests/test_session_failures.cpp)
    target_link_libraries(wpiemu_host_tests PRIVATE doctest::doctest wpiemu_app_host wpiemu_plugins_builtin)
    target_compile_definitions(wpiemu_host_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
    wpiemu_target_defaults(wpiemu_host_tests)
    doctest_discover_tests(wpiemu_host_tests TEST_PREFIX "host.")

    # Opt-in end-to-end test with a real WPILib project, Gradle and the real extension (label e2e, see tests/e2e_test.cpp).
    add_executable(wpiemu_e2e_tests tests/e2e_test.cpp)
    target_link_libraries(wpiemu_e2e_tests PRIVATE doctest::doctest wpiemu_app_host wpiemu_plugins_builtin)
    target_compile_definitions(wpiemu_e2e_tests PRIVATE
        DOCTEST_CONFIG_USE_STD_HEADERS
        "WPIEMU_E2E_DEFAULT_PROJECT=\"${CMAKE_SOURCE_DIR}/../FRC-Shelby-2026OS\"")
    if(TARGET halsim_wpiemu)
        add_dependencies(wpiemu_e2e_tests halsim_wpiemu)
        target_compile_definitions(wpiemu_e2e_tests PRIVATE "WPIEMU_E2E_EXTENSION=\"$<TARGET_FILE:halsim_wpiemu>\"")
    endif()
    wpiemu_target_defaults(wpiemu_e2e_tests)
    add_test(NAME e2e.shelby_2026 COMMAND wpiemu_e2e_tests)
    set_tests_properties(e2e.shelby_2026 PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500)
endif()
