# Locates the WPILib simulation HAL (hal-cpp), wpiutil and ntcore (headers + desktop shared libraries).
#
# The sim HAL is NOT built by us: ADR 0001 links the extension against the `wpiHal` module that
# the robot process already loaded. We therefore only need headers and an import library / shared
# object. Two input layouts are understood (the same cache variable is shared with bridges/hal and
# bridges/vendor, which expect the "extracted" layout):
#
#   extracted   <root>/include/hal/HAL.h, <root>/include/wpi/string.h,
#               <root>/<platform>/shared/{wpiHal,wpiutil}.*   (platform = windows/x86-64, linux/x86-64,
#               linux/arm64, osx/universal)
#   maven       a WPILib Maven cache (the installer's ~/wpilib/2026/maven, or any mirror) holding
#               edu/wpi/first/{hal/hal-cpp,wpiutil/wpiutil-cpp}/<version>/*-headers.zip and the
#               platform zips. It is extracted into <build>/_wpilib/<version> (extracted layout) and
#               WPIEMU_WPILIB_ROOT is then pointed there for the sibling modules.
#
# ntcore is staged the same way but is OPTIONAL (ADR 0017: the vision bridge binds the robot's ntcore at run time and
# only needs its C headers to pin the ABI, plus the shared libraries for the real-ntcore tests):
#   maven       edu/wpi/first/ntcore/ntcore-cpp/<version>/ntcore-cpp-<version>-{headers,<platform>}.zip and
#               edu/wpi/first/wpinet/wpinet-cpp/<version>/wpinet-cpp-<version>-<platform>.zip (ntcore.dll needs wpinet.dll)
#               land in the same tree: ntcore_c.h next to hal/ and wpi/ in include/, ntcore and wpinet next to wpiHal and
#               wpiutil in <platform>/shared.
#   extracted   <root>/include/ntcore_c.h and <root>/<platform>/shared/{ntcore,wpinet}.* when the root has them.
# A cache or root without ntcore, or a failed download of it, only leaves ntcore out: nothing else changes.
#
# Inputs:
#   WPIEMU_WPILIB_ROOT      either layout (or the year dir containing maven/). Also read from the
#                           environment variable of the same name. Default search for a maven cache:
#                           <PUBLIC|HOME|USERPROFILE>/wpilib/2026/maven.
#   WPIEMU_WPILIB_VERSION   WPILib version of hal-cpp / wpiutil-cpp / ntcore-cpp (default 2026.2.1, ADR 0001).
#   WPIEMU_WPILIB_DOWNLOAD  OFF by default. ON: fetch missing zips from frcmaven (CI on a clean machine).
#
# Outputs:
#   WPIEMU_WPILIB_FOUND            TRUE when headers and the desktop libs were found
#   WPIEMU_WPILIB_RUNTIME_DIRS     directory holding the wpiHal/wpiutil shared libs (put on PATH for tests)
#   WPIEMU_WPILIB_NTCORE_FOUND     TRUE when the ntcore C headers (ntcore_c.h) are in the resolved root
#   WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS   directory holding the ntcore, wpinet and wpiutil shared libs, empty unless all
#                                       are staged (set in the PARENT scope too: the unit tests read it)
#   targets wpiemu_wpilib_hal / wpiemu_wpilib_wpiutil   IMPORTED shared libraries (+ include dirs)
# Nothing is defined (and configure still succeeds) when WPILib is not available.

include_guard(GLOBAL)

set(WPIEMU_WPILIB_ROOT "" CACHE PATH "WPILib HAL root: extracted layout or Maven cache; empty = auto-detect")
set(WPIEMU_WPILIB_VERSION "2026.2.1" CACHE STRING "WPILib version of hal-cpp/wpiutil-cpp to link against")
option(WPIEMU_WPILIB_DOWNLOAD "Download missing WPILib HAL/wpiutil zips from frcmaven" OFF)

set(WPIEMU_WPILIB_FOUND FALSE)
set(WPIEMU_WPILIB_RUNTIME_DIRS "")
set(WPIEMU_WPILIB_NTCORE_FOUND FALSE)
set(WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS "")

# Desktop platform classifier used in the Maven file names and the layout inside the zip.
if(WIN32)
    set(_wpiemu_wpilib_classifier "windowsx86-64")
    set(_wpiemu_wpilib_subdir "windows/x86-64/shared")
elseif(APPLE)
    set(_wpiemu_wpilib_classifier "osxuniversal")
    set(_wpiemu_wpilib_subdir "osx/universal/shared")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_wpiemu_wpilib_classifier "linuxarm64")
    set(_wpiemu_wpilib_subdir "linux/arm64/shared")
else()
    set(_wpiemu_wpilib_classifier "linuxx86-64")
    set(_wpiemu_wpilib_subdir "linux/x86-64/shared")
endif()

set(_wpiemu_extracted_root "")   # root in the "extracted" layout, when we have one

# ---- 1. user-provided root in the extracted layout ---------------------------------------------------
foreach(_root IN ITEMS "${WPIEMU_WPILIB_ROOT}" "$ENV{WPIEMU_WPILIB_ROOT}")
    if(_root AND EXISTS "${_root}/include/hal/HAL.h" AND EXISTS "${_root}/include/wpi/string.h")
        set(_wpiemu_extracted_root "${_root}")
        break()
    endif()
endforeach()

# ---- 2. Maven cache -> stage into the extracted layout --------------------------------------------------
if(NOT _wpiemu_extracted_root)
    set(_wpiemu_wpilib_candidates)
    foreach(_root IN ITEMS "${WPIEMU_WPILIB_ROOT}" "$ENV{WPIEMU_WPILIB_ROOT}")
        if(_root)
            list(APPEND _wpiemu_wpilib_candidates "${_root}" "${_root}/maven")
        endif()
    endforeach()
    foreach(_home IN ITEMS "$ENV{PUBLIC}" "$ENV{HOME}" "$ENV{USERPROFILE}" "C:/Users/Public")
        if(_home)
            list(APPEND _wpiemu_wpilib_candidates "${_home}/wpilib/2026/maven")
        endif()
    endforeach()

    set(_hal_rel "edu/wpi/first/hal/hal-cpp/${WPIEMU_WPILIB_VERSION}/hal-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_util_rel "edu/wpi/first/wpiutil/wpiutil-cpp/${WPIEMU_WPILIB_VERSION}/wpiutil-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_ntcore_rel "edu/wpi/first/ntcore/ntcore-cpp/${WPIEMU_WPILIB_VERSION}/ntcore-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_net_rel "edu/wpi/first/wpinet/wpinet-cpp/${WPIEMU_WPILIB_VERSION}/wpinet-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_wpiemu_maven "")
    foreach(_c IN LISTS _wpiemu_wpilib_candidates)
        if(EXISTS "${_c}/${_hal_rel}-headers.zip" AND EXISTS "${_c}/${_util_rel}-headers.zip")
            set(_wpiemu_maven "${_c}")
            break()
        endif()
    endforeach()

    # Resolves one artifact zip: Maven cache first, optional download second.
    function(_wpiemu_wpilib_fetch rel suffix out_var)
        set(_local "${_wpiemu_maven}/${rel}-${suffix}.zip")
        if(_wpiemu_maven AND EXISTS "${_local}")
            set(${out_var} "${_local}" PARENT_SCOPE)
            return()
        endif()
        set(${out_var} "" PARENT_SCOPE)
        if(NOT WPIEMU_WPILIB_DOWNLOAD)
            return()
        endif()
        get_filename_component(_name "${rel}-${suffix}.zip" NAME)
        set(_dest "${CMAKE_BINARY_DIR}/_wpilib/downloads/${_name}")
        if(NOT EXISTS "${_dest}")
            set(_url "https://frcmaven.wpi.edu/artifactory/release/${rel}-${suffix}.zip")
            message(STATUS "WPILib: downloading ${_url}")
            file(DOWNLOAD "${_url}" "${_dest}" STATUS _st)
            list(GET _st 0 _code)
            if(NOT _code EQUAL 0)
                file(REMOVE "${_dest}")
                message(WARNING "WPILib: download failed (${_st}): ${_url}")
                return()
            endif()
        endif()
        set(${out_var} "${_dest}" PARENT_SCOPE)
    endfunction()

    _wpiemu_wpilib_fetch("${_hal_rel}" "headers" _hal_headers_zip)
    _wpiemu_wpilib_fetch("${_hal_rel}" "${_wpiemu_wpilib_classifier}" _hal_libs_zip)
    _wpiemu_wpilib_fetch("${_util_rel}" "headers" _util_headers_zip)
    _wpiemu_wpilib_fetch("${_util_rel}" "${_wpiemu_wpilib_classifier}" _util_libs_zip)

    if(_hal_headers_zip AND _hal_libs_zip AND _util_headers_zip AND _util_libs_zip)
        set(_stage "${CMAKE_BINARY_DIR}/_wpilib/${WPIEMU_WPILIB_VERSION}")
        if(NOT EXISTS "${_stage}/.extracted")
            file(REMOVE_RECURSE "${_stage}")
            file(ARCHIVE_EXTRACT INPUT "${_hal_headers_zip}" DESTINATION "${_stage}/include")
            file(ARCHIVE_EXTRACT INPUT "${_util_headers_zip}" DESTINATION "${_stage}/include")
            file(ARCHIVE_EXTRACT INPUT "${_hal_libs_zip}" DESTINATION "${_stage}")
            file(ARCHIVE_EXTRACT INPUT "${_util_libs_zip}" DESTINATION "${_stage}")
            file(WRITE "${_stage}/.extracted" "${WPIEMU_WPILIB_VERSION}\n")
        endif()

        # ntcore is optional and has its own markers, so a stage that predates it (or lacked the artifacts at the time)
        # picks it up on the next configure.
        _wpiemu_wpilib_fetch("${_ntcore_rel}" "headers" _ntcore_headers_zip)
        _wpiemu_wpilib_fetch("${_ntcore_rel}" "${_wpiemu_wpilib_classifier}" _ntcore_libs_zip)
        _wpiemu_wpilib_fetch("${_net_rel}" "${_wpiemu_wpilib_classifier}" _net_libs_zip)
        if(_ntcore_headers_zip AND NOT EXISTS "${_stage}/.ntcore-headers")
            file(ARCHIVE_EXTRACT INPUT "${_ntcore_headers_zip}" DESTINATION "${_stage}/include")
            file(WRITE "${_stage}/.ntcore-headers" "${WPIEMU_WPILIB_VERSION}\n")
        endif()
        if(_ntcore_libs_zip AND _net_libs_zip AND NOT EXISTS "${_stage}/.ntcore-runtime")
            file(ARCHIVE_EXTRACT INPUT "${_ntcore_libs_zip}" DESTINATION "${_stage}")
            file(ARCHIVE_EXTRACT INPUT "${_net_libs_zip}" DESTINATION "${_stage}")
            file(WRITE "${_stage}/.ntcore-runtime" "${WPIEMU_WPILIB_VERSION}\n")
        endif()
        set(_wpiemu_extracted_root "${_stage}")
    endif()
endif()

if(_wpiemu_extracted_root)
    # Publish either discovery route to sibling bridges without replacing the user's cache input.
    set(WPIEMU_WPILIB_ROOT "${_wpiemu_extracted_root}" PARENT_SCOPE)
endif()

# ---- 3. imported targets ----------------------------------------------------------------------------------
if(_wpiemu_extracted_root)
    set(_dir "${_wpiemu_extracted_root}/${_wpiemu_wpilib_subdir}")
    if(WIN32)
        set(_hal_loc "${_dir}/wpiHal.dll")
        set(_hal_imp "${_dir}/wpiHal.lib")
        set(_util_loc "${_dir}/wpiutil.dll")
        set(_util_imp "${_dir}/wpiutil.lib")
    elseif(APPLE)
        set(_hal_loc "${_dir}/libwpiHal.dylib")
        set(_util_loc "${_dir}/libwpiutil.dylib")
    else()
        set(_hal_loc "${_dir}/libwpiHal.so")
        set(_util_loc "${_dir}/libwpiutil.so")
    endif()

    if(EXISTS "${_hal_loc}" AND EXISTS "${_util_loc}")
        add_library(wpiemu_wpilib_wpiutil SHARED IMPORTED GLOBAL)
        set_target_properties(wpiemu_wpilib_wpiutil PROPERTIES
            IMPORTED_LOCATION "${_util_loc}"
            INTERFACE_INCLUDE_DIRECTORIES "${_wpiemu_extracted_root}/include")
        add_library(wpiemu_wpilib_hal SHARED IMPORTED GLOBAL)
        set_target_properties(wpiemu_wpilib_hal PROPERTIES
            IMPORTED_LOCATION "${_hal_loc}"
            INTERFACE_INCLUDE_DIRECTORIES "${_wpiemu_extracted_root}/include"
            INTERFACE_LINK_LIBRARIES wpiemu_wpilib_wpiutil
            INTERFACE_COMPILE_DEFINITIONS "WPIEMU_WPILIB_VERSION=\"${WPIEMU_WPILIB_VERSION}\"")
        if(WIN32)
            set_target_properties(wpiemu_wpilib_hal PROPERTIES IMPORTED_IMPLIB "${_hal_imp}")
            set_target_properties(wpiemu_wpilib_wpiutil PROPERTIES IMPORTED_IMPLIB "${_util_imp}")
        endif()
        set(WPIEMU_WPILIB_FOUND TRUE)
        set(WPIEMU_WPILIB_RUNTIME_DIRS "${_dir}")
    endif()
endif()

if(WPIEMU_WPILIB_FOUND)
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: HAL found at ${_wpiemu_extracted_root}; halsim_wpiemu is built with HAL support")
else()
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: NOT found (set -DWPIEMU_WPILIB_ROOT=<.../wpilib/2026/maven> or "
                   "-DWPIEMU_WPILIB_DOWNLOAD=ON); halsim_wpiemu is built as a stub that refuses to load")
endif()

# ---- 4. ntcore (optional) -------------------------------------------------------------------------------------
# The C headers pin the ABI the vision bridge's NetworkTables binding is compiled against (bridges/vision); the shared
# libraries are only needed by the unit tests that play the robot process (ntcore.dll imports wpinet.dll and wpiutil.dll,
# all three sit in the same directory).
if(_wpiemu_extracted_root AND EXISTS "${_wpiemu_extracted_root}/include/ntcore_c.h"
        AND EXISTS "${_wpiemu_extracted_root}/include/ntcore_c_types.h")
    set(WPIEMU_WPILIB_NTCORE_FOUND TRUE)
endif()
if(WPIEMU_WPILIB_FOUND)
    if(WIN32)
        set(_ntcore_loc "${_dir}/ntcore.dll")
        set(_net_loc "${_dir}/wpinet.dll")
    elseif(APPLE)
        set(_ntcore_loc "${_dir}/libntcore.dylib")
        set(_net_loc "${_dir}/libwpinet.dylib")
    else()
        set(_ntcore_loc "${_dir}/libntcore.so")
        set(_net_loc "${_dir}/libwpinet.so")
    endif()
    if(EXISTS "${_ntcore_loc}" AND EXISTS "${_net_loc}")
        set(WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS "${_dir}")
    endif()
    if(WPIEMU_WPILIB_NTCORE_FOUND AND WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS)
        message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore headers and shared libraries staged (${WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS})")
    elseif(WPIEMU_WPILIB_NTCORE_FOUND)
        message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore headers found, shared libraries (ntcore, wpinet) missing; "
                       "the real-ntcore tests are skipped")
    elseif(WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS)
        message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore shared libraries found, headers missing; the vision bridge's "
                       "NetworkTables binding is built inert and the real-ntcore tests are skipped")
    else()
        message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore NOT found (ntcore-cpp / wpinet-cpp artifacts or "
                       "-DWPIEMU_WPILIB_DOWNLOAD=ON); the vision bridge's NetworkTables binding is built inert")
    endif()
endif()
# The unit tests live in a sibling directory: hand the result up like WPIEMU_WPILIB_ROOT.
set(WPIEMU_WPILIB_NTCORE_FOUND "${WPIEMU_WPILIB_NTCORE_FOUND}" PARENT_SCOPE)
set(WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS "${WPIEMU_WPILIB_NTCORE_RUNTIME_DIRS}" PARENT_SCOPE)
