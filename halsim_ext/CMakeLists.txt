# HALSim extension (shared library) loaded inside the robot process (ADR 0001).
#
#   wpiemu_halsim_core   static, HAL-free: lock-step session, IPC link, device table, bridge fan-out.
#                        Unit-tested with fakes (halsim_ext/tests) on every machine.
#   halsim_wpiemu        the shared library: HALSIM_InitExtension + the real HALSIM_* binding. Needs the
#                        WPILib HAL (cmake/WpiEmuWpilib.cmake); without it a stub that fails loudly is
#                        built so the target (and everything that references it) still exists.
include(cmake/WpiEmuWpilib.cmake)

find_package(Threads REQUIRED)

add_library(wpiemu_halsim_core STATIC
    src/bridge_set.cpp
    src/config.cpp
    src/device_table.cpp
    src/extension_runtime.cpp
    src/forwarding_logger.cpp
    src/ipc_link.cpp
    src/os_process.cpp
    src/sim_session.cpp)
target_include_directories(wpiemu_halsim_core PUBLIC include)
target_link_libraries(wpiemu_halsim_core PUBLIC wpiemu_sdk Threads::Threads)
wpiemu_target_defaults(wpiemu_halsim_core)

if(WPIEMU_WPILIB_FOUND)
    # Real HALSIM_* binding (clock, driver station, battery). Also used by the HAL integration test.
    add_library(wpiemu_halsim_hal STATIC src/hal/hal_sim_platform.cpp)
    target_link_libraries(wpiemu_halsim_hal PUBLIC wpiemu_halsim_core wpiemu_wpilib_hal)
    wpiemu_target_defaults(wpiemu_halsim_hal)

    # wpiemu_bridges_builtin is defined in bridges/ (the aggregation seam, ADR 0004 addendum).
    add_library(halsim_wpiemu SHARED src/extension.cpp src/ipc_bindings.cpp)
    target_link_libraries(halsim_wpiemu PRIVATE
        wpiemu_halsim_hal wpiemu_halsim_core wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
else()
    add_library(halsim_wpiemu SHARED src/extension_stub.cpp)
endif()
set_target_properties(halsim_wpiemu PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
wpiemu_target_defaults(halsim_wpiemu)

if(WPIEMU_BUILD_TESTS)
    # The root enables testing only after its subdirectories; do it here so these tests are registered.
    enable_testing()
    # Own test executables: the shared tests/ glob links a fixed library list, and these tests need
    # wpiemu_halsim_core (and, for the HAL test, the WPILib HAL).
    include(doctest)
    add_executable(wpiemu_halsim_ext_tests
        tests/main.cpp
        tests/test_config.cpp
        tests/test_device_table.cpp
        tests/test_delta_filter.cpp
        tests/test_ipc_link.cpp
        tests/test_bridge_set.cpp
        tests/test_handshake.cpp
        tests/test_lockstep.cpp
        tests/test_robustness.cpp
        tests/test_reconnect.cpp
        tests/test_vision_fanout.cpp)
    target_include_directories(wpiemu_halsim_ext_tests PRIVATE tests)
    target_link_libraries(wpiemu_halsim_ext_tests PRIVATE doctest::doctest wpiemu_halsim_core wpiemu_ipc wpiemu_sdk)
    target_compile_definitions(wpiemu_halsim_ext_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
    wpiemu_target_defaults(wpiemu_halsim_ext_tests)
    doctest_discover_tests(wpiemu_halsim_ext_tests TEST_PREFIX "halsim_ext.")

    # Manual end-to-end tool (not a ctest): see tests/manual/README.md.
    add_executable(wpiemu_halsim_fake_emulator tests/manual/fake_emulator.cpp)
    target_link_libraries(wpiemu_halsim_fake_emulator PRIVATE wpiemu_ipc wpiemu_sdk)
    wpiemu_target_defaults(wpiemu_halsim_fake_emulator)

    if(WPIEMU_WPILIB_FOUND)
        # Real sim HAL in-process: stepping, DS, device I/O against a scripted fake emulator.
        add_executable(wpiemu_halsim_ext_hal_tests
            tests/hal/main.cpp
            tests/hal/test_hal_integration.cpp
            tests/hal/test_builtin_bridges.cpp
            tests/hal/test_extension_dll.cpp)
        target_include_directories(wpiemu_halsim_ext_hal_tests PRIVATE tests)
        target_link_libraries(wpiemu_halsim_ext_hal_tests PRIVATE
            doctest::doctest wpiemu_halsim_hal wpiemu_halsim_core wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
        # test_extension_dll loads the real shared library through the HAL's extension loader.
        add_dependencies(wpiemu_halsim_ext_hal_tests halsim_wpiemu)
        target_compile_definitions(wpiemu_halsim_ext_hal_tests PRIVATE
            DOCTEST_CONFIG_USE_STD_HEADERS
            "WPIEMU_EXTENSION_PATH=\"$<TARGET_FILE:halsim_wpiemu>\"")
        wpiemu_target_defaults(wpiemu_halsim_ext_hal_tests)
        # The HAL/wpiutil DLLs are not copied next to the binary: the test finds them through PATH.
        add_test(NAME halsim_ext.hal_integration COMMAND wpiemu_halsim_ext_hal_tests)
        set_tests_properties(halsim_ext.hal_integration PROPERTIES
            TIMEOUT 120
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${WPIEMU_WPILIB_RUNTIME_DIRS}")
    endif()
endif()
