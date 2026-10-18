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
