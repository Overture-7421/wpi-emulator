# Device bridges for vendor devices (Phoenix 6, Talon SRX/Victor, CANivore...). Depends on wpiemu_sdk only.
# HAL SimDevice access is a thin seam (sdk ISimDeviceBackend); the real HAL binding is compiled only when WPILib
# headers are found. WPIEMU_WPILIB_ROOT may be the HAL headers directory (contains hal/SimDevice.h) or a root
# that has it under include/. There is no link-time dependency on WPILib or CTRE: both are resolved at run time
# from the modules the robot process already loaded.
if(NOT DEFINED WPIEMU_WPILIB_ROOT)
    set(WPIEMU_WPILIB_ROOT "" CACHE PATH "Directory with the WPILib HAL headers (hal/simulation/SimDeviceData.h); optional")
endif()

add_library(wpiemu_bridges_vendor STATIC
    src/vendor_bridges.cpp
    src/vendor_sim_bridge.cpp
    src/guarded_backend.cpp
    src/incremental_backend.cpp
    src/ctre_rules.cpp
    src/ctre_native.cpp
    src/symbol_resolver.cpp
    src/hal_sim_device_backend.cpp)
add_library(wpiemu::bridges_vendor ALIAS wpiemu_bridges_vendor)
target_include_directories(wpiemu_bridges_vendor PUBLIC include)
target_link_libraries(wpiemu_bridges_vendor PUBLIC wpiemu_sdk ${CMAKE_DL_LIBS})
wpiemu_target_defaults(wpiemu_bridges_vendor)

if(WPIEMU_WPILIB_ROOT)
    unset(WPIEMU_VENDOR_HAL_INCLUDE_DIR CACHE)
    find_path(WPIEMU_VENDOR_HAL_INCLUDE_DIR hal/simulation/SimDeviceData.h
        HINTS "${WPIEMU_WPILIB_ROOT}" "${WPIEMU_WPILIB_ROOT}/include"
        NO_DEFAULT_PATH)
    if(WPIEMU_VENDOR_HAL_INCLUDE_DIR)
        message(STATUS "bridges/vendor: WPILib HAL binding enabled (${WPIEMU_VENDOR_HAL_INCLUDE_DIR})")
        # PUBLIC so unit tests can build a fake HAL against the same headers.
        target_compile_definitions(wpiemu_bridges_vendor PUBLIC WPIEMU_VENDOR_HAS_WPILIB=1)
        target_include_directories(wpiemu_bridges_vendor SYSTEM PUBLIC "${WPIEMU_VENDOR_HAL_INCLUDE_DIR}")
    else()
        message(WARNING "bridges/vendor: WPIEMU_WPILIB_ROOT='${WPIEMU_WPILIB_ROOT}' has no hal/simulation/SimDeviceData.h; HAL binding disabled")
    endif()
else()
    message(STATUS "bridges/vendor: WPILib headers not provided (WPIEMU_WPILIB_ROOT); HAL binding disabled")
endif()
