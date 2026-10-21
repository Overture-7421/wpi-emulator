# HALSim extension (shared library) loaded inside the robot process (docs/reference/extension-ipc.md).
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
    # wpiemu_bridges_builtin is defined in bridges/ (the aggregation seam, docs/ARCHITECTURE.md addendum).
    add_library(halsim_wpiemu SHARED
        src/extension.cpp
        src/ipc_bindings.cpp
        src/hal/hal_sim_platform.cpp)
    target_link_libraries(halsim_wpiemu PRIVATE
        wpiemu_halsim_core wpiemu_wpilib_hal wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
else()
    add_library(halsim_wpiemu SHARED src/extension_stub.cpp)
endif()
set_target_properties(halsim_wpiemu PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
wpiemu_target_defaults(halsim_wpiemu)

if(WPIEMU_BUILD_TESTS AND WPIEMU_WPILIB_FOUND)
    # Real sim HAL in-process: stepping, DS, device I/O against a scripted fake emulator.
    add_executable(wpiemu_halsim_ext_hal_tests
        src/hal/hal_sim_platform.cpp
        tests/hal/main.cpp
        tests/hal/test_hal_integration.cpp
        tests/hal/test_builtin_bridges.cpp
        tests/hal/test_extension_dll.cpp)
    target_include_directories(wpiemu_halsim_ext_hal_tests PRIVATE tests "${CMAKE_SOURCE_DIR}/tests/support")
    target_link_libraries(wpiemu_halsim_ext_hal_tests PRIVATE
        wpiemu_halsim_core wpiemu_wpilib_hal wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
    if(TARGET wpiemu_wpilib_ntcore)
        target_sources(wpiemu_halsim_ext_hal_tests PRIVATE
            "${CMAKE_SOURCE_DIR}/bridges/vision/tests/ntcore_smoke_tests.cpp")
        set_property(SOURCE "${CMAKE_SOURCE_DIR}/bridges/vision/tests/ntcore_smoke_tests.cpp"
            APPEND PROPERTY INCLUDE_DIRECTORIES
            "$<TARGET_PROPERTY:wpiemu_bridges_vision,INTERFACE_INCLUDE_DIRECTORIES>")
        target_link_libraries(wpiemu_halsim_ext_hal_tests PRIVATE wpiemu_wpilib_ntcore)
    endif()
    # test_extension_dll loads the real shared library through the HAL's extension loader.
    add_dependencies(wpiemu_halsim_ext_hal_tests halsim_wpiemu)
    target_compile_definitions(wpiemu_halsim_ext_hal_tests PRIVATE
        "WPIEMU_EXTENSION_PATH=\"$<TARGET_FILE:halsim_wpiemu>\"")
    wpiemu_doctest_target(wpiemu_halsim_ext_hal_tests)
    # The HAL/wpiutil DLLs are not copied next to the binary: the test finds them through PATH.
    add_test(NAME halsim_ext.hal_integration COMMAND wpiemu_halsim_ext_hal_tests)
    set_tests_properties(halsim_ext.hal_integration PROPERTIES
        TIMEOUT 120
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${WPIEMU_WPILIB_RUNTIME_DIRS}")
endif()
