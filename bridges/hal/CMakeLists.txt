# Device bridges for WPILib HAL devices (PWM, DIO, analog, encoders, relay, pneumatics, PD, LEDs, CAN...).
# The bridges depend only on wpiemu_sdk and the IHalApi seam; unit tests use FakeHalApi.
add_library(wpiemu_bridges_hal STATIC
    src/hal_bridges.cpp
    src/registration.cpp
    src/digital_bridges.cpp
    src/sensor_bridges.cpp
    src/pneumatics_bridges.cpp
    src/power_bridges.cpp
    src/led_bridge.cpp
    src/can_bridge.cpp)
add_library(wpiemu::bridges_hal ALIAS wpiemu_bridges_hal)
target_include_directories(wpiemu_bridges_hal PUBLIC include)
target_link_libraries(wpiemu_bridges_hal PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_bridges_hal)

# Real HALSIM_* binding, built only when the selected WPILib HAL is available:
#   -DWPIEMU_WPILIB_ROOT=<dir>   containing the extracted hal-cpp / wpiutil-cpp "headers" zips and the
#                                platform hal-cpp zip (any layout below is searched), e.g.
#       <root>/hal-headers/hal/Ports.h      or  <root>/include/hal/Ports.h
#       <root>/wpiutil-headers/wpi/string.h or  <root>/include/wpi/string.h
#       <root>/hal-bin/windows/x86-64/shared/wpiHal.lib   (or .../linux/x86-64/shared/libwpiHal.so, osx/universal/shared)
# When unset (default) the target is simply not created and halsim_ext must be built without it.
set(WPIEMU_WPILIB_ROOT "" CACHE PATH "Directory with extracted WPILib HAL headers + wpiHal for the real HALSIM binding")
if(WPIEMU_WPILIB_ROOT)
    wpiemu_season_abi_value(_wpiemu_hal_header "${WPIEMU_WPILIB_SEASON}" headers hal)
    wpiemu_season_abi_value(_wpiemu_wpiutil_header "${WPIEMU_WPILIB_SEASON}" headers wpiutil)
    set(_wpi_hints
        "${WPIEMU_WPILIB_ROOT}" "${WPIEMU_WPILIB_ROOT}/include"
        "${WPIEMU_WPILIB_ROOT}/hal-headers" "${WPIEMU_WPILIB_ROOT}/hal-headers/include"
        "${WPIEMU_WPILIB_ROOT}/wpiutil-headers" "${WPIEMU_WPILIB_ROOT}/wpiutil-headers/include")
    unset(WPIEMU_HAL_INCLUDE_DIR CACHE)
    unset(WPIEMU_WPIUTIL_INCLUDE_DIR CACHE)
    unset(WPIEMU_HAL_LIBRARY CACHE)
    find_path(WPIEMU_HAL_INCLUDE_DIR "${_wpiemu_hal_header}" HINTS ${_wpi_hints} NO_DEFAULT_PATH NO_CACHE)
    find_path(WPIEMU_WPIUTIL_INCLUDE_DIR "${_wpiemu_wpiutil_header}" HINTS ${_wpi_hints} NO_DEFAULT_PATH NO_CACHE)
    find_library(WPIEMU_HAL_LIBRARY NAMES wpiHal HINTS
        "${WPIEMU_WPILIB_ROOT}" "${WPIEMU_WPILIB_ROOT}/lib"
        "${WPIEMU_WPILIB_ROOT}/hal-bin/windows/x86-64/shared" "${WPIEMU_WPILIB_ROOT}/hal-bin/linux/x86-64/shared"
        "${WPIEMU_WPILIB_ROOT}/hal-bin/osx/universal/shared"
        "${WPIEMU_WPILIB_ROOT}/windows/x86-64/shared" "${WPIEMU_WPILIB_ROOT}/linux/x86-64/shared"
        "${WPIEMU_WPILIB_ROOT}/osx/universal/shared"
        NO_DEFAULT_PATH NO_CACHE)
    set(WPIEMU_HAL_INCLUDE_DIR "${WPIEMU_HAL_INCLUDE_DIR}" PARENT_SCOPE)
    set(WPIEMU_WPIUTIL_INCLUDE_DIR "${WPIEMU_WPIUTIL_INCLUDE_DIR}" PARENT_SCOPE)
    set(WPIEMU_HAL_LIBRARY "${WPIEMU_HAL_LIBRARY}" PARENT_SCOPE)
    if(WPIEMU_HAL_INCLUDE_DIR AND WPIEMU_WPIUTIL_INCLUDE_DIR AND WPIEMU_HAL_LIBRARY)
        # The families that differ between HAL generations come from abi/<abi>/ (docs/reference/seasons.md).
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/abi/${WPIEMU_HAL_ABI}/hal_sim_abi.cpp")
            message(FATAL_ERROR "bridges/hal: HAL ABI '${WPIEMU_HAL_ABI}' has no src/abi/${WPIEMU_HAL_ABI}/hal_sim_abi.cpp")
        endif()
        target_sources(wpiemu_bridges_hal PRIVATE src/hal_sim_binding.cpp "src/abi/${WPIEMU_HAL_ABI}/hal_sim_abi.cpp")
        target_include_directories(wpiemu_bridges_hal PRIVATE src)
        target_link_libraries(wpiemu_bridges_hal PRIVATE wpiemu_hal_abi)
        # SYSTEM: WPILib headers are third-party and not warning-clean under /W4.
        target_include_directories(wpiemu_bridges_hal SYSTEM PRIVATE
            "${WPIEMU_HAL_INCLUDE_DIR}" "${WPIEMU_WPIUTIL_INCLUDE_DIR}")
        target_link_libraries(wpiemu_bridges_hal PUBLIC "${WPIEMU_HAL_LIBRARY}")
        target_compile_definitions(wpiemu_bridges_hal PUBLIC WPIEMU_HAS_HALSIM_BINDING=1 WPIEMU_HAL_ABI_${WPIEMU_HAL_ABI_ID}=1)
        message(STATUS "wpiemu: HALSIM binding enabled (${WPIEMU_HAL_LIBRARY})")
    else()
        message(WARNING "WPIEMU_WPILIB_ROOT is set but HAL headers/wpiutil headers/wpiHal were not all found "
                        "(hal=${WPIEMU_HAL_INCLUDE_DIR} wpiutil=${WPIEMU_WPIUTIL_INCLUDE_DIR} lib=${WPIEMU_HAL_LIBRARY}); "
                        "the real HALSIM binding is disabled")
    endif()
endif()
