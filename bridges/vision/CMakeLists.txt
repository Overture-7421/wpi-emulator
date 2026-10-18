# Vision bridge: virtual PhotonVision / Limelight coprocessors published through the robot process' own
# ntcore (ADR 0017). Depends on wpiemu_sdk only. ntcore is attached at run time to the module the robot
# already loaded, so there is no link-time dependency on WPILib; the ntcore headers (when staged) only
# pin the C ABI that the binding is compiled against.
#
# One folder per work stream so independent changes never touch the same file; each folder lists its
# own sources in a sources.cmake next to them:
#   include/.../nt_api.h              INtApi, the NetworkTables seam (+ testing/fake_nt_api.h for tests)
#   include/.../camera_publisher.h    ICameraPublisher / IVisionVendor, the per-vendor seam
#   src/binding/    NtCoreApi: the real run-time binding of the robot's ntcore C API (+ CMake staging)
#   src/core/       VisionBridge: the IDeviceBridge, camera lifecycle, discovery, status, stats
#   src/photon/     PhotonVision profile, packet encoder and publisher
#   src/limelight/  Limelight key tables, conversions and publisher
add_library(wpiemu_bridges_vision STATIC src/vision_bridges.cpp)
add_library(wpiemu::bridges_vision ALIAS wpiemu_bridges_vision)
target_include_directories(wpiemu_bridges_vision PUBLIC include)
target_link_libraries(wpiemu_bridges_vision PUBLIC wpiemu_sdk ${CMAKE_DL_LIBS})
wpiemu_target_defaults(wpiemu_bridges_vision)

include(${CMAKE_CURRENT_SOURCE_DIR}/src/binding/sources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/core/sources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/photon/sources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/src/limelight/sources.cmake)
