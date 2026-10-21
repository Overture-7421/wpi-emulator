add_subdirectory(hal)
add_subdirectory(vendor)
add_subdirectory(vision)

# Aggregation seam (docs/ARCHITECTURE.md addendum, mirrors plugins/): the single target that links every built-in
# bridge module and defines wpiemu::sdk::registerBuiltinBridges() (sdk/devices/builtin_bridges.h). The
# HALSim extension links only this target. Owned by the halsim_ext stream.
add_library(wpiemu_bridges_builtin STATIC builtin_bridges.cpp)
add_library(wpiemu::bridges_builtin ALIAS wpiemu_bridges_builtin)
target_link_libraries(wpiemu_bridges_builtin PUBLIC wpiemu_sdk PRIVATE wpiemu_bridges_hal wpiemu_bridges_vendor wpiemu_bridges_vision)
wpiemu_target_defaults(wpiemu_bridges_builtin)

# The real HALSIM binding of the HAL bridges exists only when the WPILib HAL was found.
if(WPIEMU_HAL_INCLUDE_DIR AND WPIEMU_WPIUTIL_INCLUDE_DIR AND WPIEMU_HAL_LIBRARY)
    target_compile_definitions(wpiemu_bridges_builtin PRIVATE WPIEMU_BUILTIN_HAL_HALSIM WPIEMU_HAS_HALSIM_BINDING=1)
endif()

# The real ntcore binding of the vision bridges exists only when ntcore was found.
if(TARGET wpiemu_wpilib_ntcore AND TARGET wpiemu_wpilib_hal)
    target_compile_definitions(wpiemu_bridges_builtin PRIVATE WPIEMU_BUILTIN_VISION_NT WPIEMU_HAS_VISION_NT=1)
endif()

# The vendor module publishes its registration function in vendor_bridges.h; use it once it exists.
set(_vendor_header "${CMAKE_CURRENT_SOURCE_DIR}/vendor/include/wpiemu/bridges/vendor/vendor_bridges.h")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_vendor_header}")
file(READ "${_vendor_header}" _vendor_text)
if(_vendor_text MATCHES "(register[A-Za-z0-9_]*Bridges)[ \t\r\n]*\\([ \t\r\n]*(sdk::|wpiemu::sdk::)?DeviceBridgeRegistry")
    target_compile_definitions(wpiemu_bridges_builtin PRIVATE WPIEMU_BUILTIN_VENDOR_REGISTER=${CMAKE_MATCH_1})
endif()
