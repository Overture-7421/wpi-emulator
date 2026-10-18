# Vision bridges: simulated Limelight / PhotonVision coprocessors presented to the robot through NetworkTables.
# `wpiemu_bridges_vision` depends only on wpiemu_sdk (the NetworkTables seam is INtBackend; tests use
# FakeNtBackend). The real ntcore binding `wpiemu_bridges_vision_nt` is built only when ntcore was found
# (halsim_ext/cmake/WpiEmuWpilib.cmake, WPIEMU_NTCORE_FOUND).
add_library(wpiemu_bridges_vision STATIC
    src/registration.cpp
    src/limelight_bridge.cpp
    src/photon_bridge.cpp
    src/photon_encoding.cpp)
add_library(wpiemu::bridges_vision ALIAS wpiemu_bridges_vision)
target_include_directories(wpiemu_bridges_vision PUBLIC include)
target_link_libraries(wpiemu_bridges_vision PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_bridges_vision)

# Real ntcore binding, only when ntcore (and the HAL) were found by halsim_ext/cmake/WpiEmuWpilib.cmake. It has no
# link-time dependency on ntcore: it attaches to the module the robot process loaded (see nt_binding.h), so only the
# headers are used here. Without ntcore the bridges are registered inert (bridges/builtin_bridges.cpp).
if(TARGET wpiemu_wpilib_ntcore AND TARGET wpiemu_wpilib_hal)
    add_library(wpiemu_bridges_vision_nt STATIC src/ntcore_backend.cpp)
    add_library(wpiemu::bridges_vision_nt ALIAS wpiemu_bridges_vision_nt)
    target_include_directories(wpiemu_bridges_vision_nt PUBLIC include)
    get_target_property(_wpiemu_nt_include wpiemu_wpilib_ntcore INTERFACE_INCLUDE_DIRECTORIES)
    # SYSTEM: WPILib headers are third-party and not warning-clean under /W4.
    target_include_directories(wpiemu_bridges_vision_nt SYSTEM PRIVATE ${_wpiemu_nt_include})
    target_link_libraries(wpiemu_bridges_vision_nt PUBLIC wpiemu_bridges_vision ${CMAKE_DL_LIBS})
    wpiemu_target_defaults(wpiemu_bridges_vision_nt)
    message(STATUS "bridges/vision: ntcore binding enabled")

    if(WPIEMU_BUILD_TESTS)
        # Smoke test of the binding against the real ntcore in-process. It links ntcore (a robot process has it
        # loaded) so the binding attaches to the already loaded module, like in a robot.
        # The root enables testing only after its subdirectories; do it here so this test is registered.
        enable_testing()
        include(doctest)
        add_executable(wpiemu_bridges_vision_nt_tests tests/ntcore_smoke_tests.cpp)
        target_include_directories(wpiemu_bridges_vision_nt_tests SYSTEM PRIVATE ${_wpiemu_nt_include})
        target_link_libraries(wpiemu_bridges_vision_nt_tests PRIVATE
            doctest::doctest wpiemu_bridges_vision_nt wpiemu_wpilib_ntcore wpiemu_sdk)
        target_compile_definitions(wpiemu_bridges_vision_nt_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
        wpiemu_target_defaults(wpiemu_bridges_vision_nt_tests)
        # The ntcore/wpinet/wpiutil DLLs are not copied next to the binary: the test finds them through PATH.
        add_test(NAME bridges_vision.ntcore_smoke COMMAND wpiemu_bridges_vision_nt_tests)
        set_tests_properties(bridges_vision.ntcore_smoke PROPERTIES
            TIMEOUT 120
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:wpiemu_wpilib_ntcore>")
    endif()
endif()
