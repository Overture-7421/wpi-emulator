# HALSim extension (shared library) loaded inside the robot process (docs/reference/extension-ipc.md).
#
#   wpiemu_halsim_core   static, HAL-free: lock-step session, IPC link, device table, bridge fan-out.
#                        Unit-tested with fakes (halsim_ext/tests) on every machine.
#   halsim_wpiemu        the shared library: HALSIM_InitExtension + the real HALSIM_* binding. Needs the
#                        WPILib HAL (cmake/WpiEmuWpilib.cmake); without it a stub that fails loudly is
#                        built so the target (and everything that references it) still exists.
include(cmake/WpiEmuWpilib.cmake)
if(WPIEMU_REQUIRE_REAL_HAL AND NOT WPIEMU_WPILIB_FOUND)
    message(FATAL_ERROR "WPILib ${WPIEMU_WPILIB_SEASON} HAL artifacts are required; refusing to build a stub")
endif()

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
    # The HAL calls that differ between HAL generations come from src/hal/abi/<abi>/ (docs/reference/seasons.md).
    set(_wpiemu_platform_abi "${CMAKE_CURRENT_SOURCE_DIR}/src/hal/abi/${WPIEMU_HAL_ABI}/hal_platform_abi.cpp")
    if(NOT EXISTS "${_wpiemu_platform_abi}")
        message(FATAL_ERROR "halsim_ext: HAL ABI '${WPIEMU_HAL_ABI}' has no src/hal/abi/${WPIEMU_HAL_ABI}/hal_platform_abi.cpp")
    endif()
    add_library(halsim_wpiemu SHARED
        src/extension.cpp
        src/ipc_bindings.cpp
        src/hal/hal_sim_platform.cpp
        "${_wpiemu_platform_abi}")
    target_include_directories(halsim_wpiemu PRIVATE src/hal)
    target_link_libraries(halsim_wpiemu PRIVATE
        wpiemu_halsim_core wpiemu_wpilib_hal wpiemu_hal_abi wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
else()
    add_library(halsim_wpiemu SHARED src/extension_stub.cpp)
endif()
set_target_properties(halsim_wpiemu PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
wpiemu_target_defaults(halsim_wpiemu)

if(WPIEMU_BUILD_TESTS AND WPIEMU_WPILIB_FOUND)
    # Each HAL ABI has its own real-HAL suites: tests/hal/abi/<abi>/*.cpp.
    file(GLOB _hal_test_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/hal/abi/${WPIEMU_HAL_ABI}/*.cpp")
    # Real sim HAL in-process: stepping, DS, device I/O against a scripted fake emulator.
    add_executable(wpiemu_halsim_ext_hal_tests
        src/hal/hal_sim_platform.cpp
        "${_wpiemu_platform_abi}"
        tests/hal/main.cpp
        ${_hal_test_sources})
    target_include_directories(wpiemu_halsim_ext_hal_tests PRIVATE tests "${CMAKE_SOURCE_DIR}/tests/support")
    target_include_directories(wpiemu_halsim_ext_hal_tests PRIVATE src/hal)
    target_link_libraries(wpiemu_halsim_ext_hal_tests PRIVATE
        wpiemu_halsim_core wpiemu_wpilib_hal wpiemu_hal_abi wpiemu_ipc wpiemu_bridges_builtin wpiemu_sdk)
    # HALSIM binding suites of the ABI, compiled against the real headers: bridges/hal/tests/abi/<abi>/*.cpp.
    file(GLOB _hal_binding_tests CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/bridges/hal/tests/abi/${WPIEMU_HAL_ABI}/*.cpp")
    foreach(_binding_test IN LISTS _hal_binding_tests)
        target_sources(wpiemu_halsim_ext_hal_tests PRIVATE "${_binding_test}")
        set_property(SOURCE "${_binding_test}" APPEND PROPERTY INCLUDE_DIRECTORIES
            "$<TARGET_PROPERTY:wpiemu_bridges_hal,INTERFACE_INCLUDE_DIRECTORIES>")
    endforeach()
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
