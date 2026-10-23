# Vision bridges: simulated Limelight / PhotonVision coprocessors presented to the robot through NetworkTables.
# `wpiemu_bridges_vision` depends only on wpiemu_sdk (the NetworkTables seam is INtBackend; tests use
# FakeNtBackend). The real ntcore binding is compiled in when ntcore was found
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
    target_sources(wpiemu_bridges_vision PRIVATE src/ntcore_backend.cpp)
    get_target_property(_wpiemu_nt_include wpiemu_wpilib_ntcore INTERFACE_INCLUDE_DIRECTORIES)
    # SYSTEM: WPILib headers are third-party and not warning-clean under /W4.
    target_include_directories(wpiemu_bridges_vision SYSTEM PRIVATE ${_wpiemu_nt_include})
    target_link_libraries(wpiemu_bridges_vision PRIVATE ${CMAKE_DL_LIBS})
    target_compile_definitions(wpiemu_bridges_vision PUBLIC WPIEMU_HAS_VISION_NT=1 WPIEMU_WPILIB_YEAR=${WPIEMU_WPILIB_YEAR})
    message(STATUS "bridges/vision: ntcore binding enabled")
endif()
