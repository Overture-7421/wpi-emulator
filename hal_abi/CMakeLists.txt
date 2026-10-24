# Per-HAL-ABI headers for the robot-process modules (docs/reference/seasons.md). The profile of the selected season
# (seasons/abi/<id>.json) names the directory; code includes wpiemu/hal_abi/*.h and never a WPILib include root, so a
# season that keeps its HAL generation needs no source change. The headers include WPILib headers: only modules that
# are compiled against the real HAL may include them.
wpiemu_season_abi(_wpiemu_hal_abi "${WPIEMU_WPILIB_SEASON}")
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_wpiemu_hal_abi}/include")
    message(FATAL_ERROR "wpiemu: HAL ABI '${_wpiemu_hal_abi}' has no hal_abi/${_wpiemu_hal_abi}/include directory")
endif()
add_library(wpiemu_hal_abi INTERFACE)
add_library(wpiemu::hal_abi ALIAS wpiemu_hal_abi)
target_include_directories(wpiemu_hal_abi INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/${_wpiemu_hal_abi}/include")
