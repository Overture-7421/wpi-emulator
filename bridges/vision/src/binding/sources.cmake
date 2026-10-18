# Sources of bridges/vision/src/binding (work stream nt-binding): the run-time binding of the robot's ntcore
# C API behind INtApi (makeNtCoreApi), its staging in CMake and the real-ntcore tests.
#
#   nt_abi.h / nt_functions.h   our declarations of the ntcore C ABI the binding uses and the table of entry points
#   nt_binder.*                 filling the table: from any symbol lookup, and from the ntcore module the robot loaded
#   nt_core_api.*               NtCoreApi, the real INtApi (+ makeNtCoreApi)
#   nt_abi_check.cpp            compile-time proof that the declarations equal the real ntcore headers (when staged)
target_sources(wpiemu_bridges_vision PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/nt_abi_check.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nt_binder.cpp
    ${CMAKE_CURRENT_LIST_DIR}/nt_core_api.cpp)

# The ntcore C headers (ntcore_c.h, ntcore_c_types.h, and wpi/string.h from wpiutil) are staged by
# halsim_ext/cmake/WpiEmuWpilib.cmake next to the HAL headers; WPIEMU_WPILIB_ROOT points at them (either discovery
# route). They are optional: without them the module still builds, but the real binding is not compiled against a
# pinned ABI and makeNtCoreApi() returns an inert API that says so. There is no link-time dependency on ntcore either
# way: the binding attaches at run time to the module the robot process loaded.
if(WPIEMU_WPILIB_ROOT)
    unset(WPIEMU_VISION_NTCORE_INCLUDE_DIR CACHE)
    unset(WPIEMU_VISION_WPIUTIL_INCLUDE_DIR CACHE)
    find_path(WPIEMU_VISION_NTCORE_INCLUDE_DIR ntcore_c.h
        HINTS "${WPIEMU_WPILIB_ROOT}" "${WPIEMU_WPILIB_ROOT}/include"
        NO_DEFAULT_PATH)
    find_path(WPIEMU_VISION_WPIUTIL_INCLUDE_DIR wpi/string.h
        HINTS "${WPIEMU_WPILIB_ROOT}" "${WPIEMU_WPILIB_ROOT}/include"
        NO_DEFAULT_PATH)
    if(WPIEMU_VISION_NTCORE_INCLUDE_DIR AND WPIEMU_VISION_WPIUTIL_INCLUDE_DIR)
        message(STATUS "bridges/vision: ntcore binding enabled (${WPIEMU_VISION_NTCORE_INCLUDE_DIR})")
        target_compile_definitions(wpiemu_bridges_vision PRIVATE WPIEMU_VISION_HAS_NTCORE=1)
        # SYSTEM: WPILib headers are third-party and not warning-clean under /W4.
        target_include_directories(wpiemu_bridges_vision SYSTEM PRIVATE
            "${WPIEMU_VISION_NTCORE_INCLUDE_DIR}" "${WPIEMU_VISION_WPIUTIL_INCLUDE_DIR}")
    else()
        message(STATUS "bridges/vision: no ntcore C headers under WPIEMU_WPILIB_ROOT='${WPIEMU_WPILIB_ROOT}'; "
                       "the ntcore binding is built inert")
    endif()
else()
    message(STATUS "bridges/vision: WPILib headers not provided (WPIEMU_WPILIB_ROOT); the ntcore binding is built "
                   "inert")
endif()
