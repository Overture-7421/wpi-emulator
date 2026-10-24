# Locates the WPILib simulation HAL (hal-cpp) and wpiutil (headers + desktop shared libraries).
#
# The sim HAL is NOT built by us: docs/reference/extension-ipc.md links the extension against the `wpiHal` module that
# the robot process already loaded. We therefore only need headers and an import library / shared
# object. Two input layouts are understood (the same cache variable is shared with bridges/hal and
# bridges/vendor, which expect the "extracted" layout):
#
#   extracted   <root>/include/hal/HAL.h, <root>/include/wpi/string.h,
#               <root>/<platform>/shared/{wpiHal,wpiutil}.*   (platform = windows/x86-64, linux/x86-64,
#               linux/arm64, osx/universal)
#   maven       a WPILib Maven cache (the installer's ~/wpilib/<year>/maven, or any mirror) holding
#               edu/wpi/first/{hal/hal-cpp,wpiutil/wpiutil-cpp}/<version>/*-headers.zip and the
#               platform zips. It is extracted into <build>/_wpilib/<version> (extracted layout) and
#               WPIEMU_WPILIB_ROOT is then pointed there for the sibling modules.
#
# Inputs:
#   WPIEMU_WPILIB_ROOT      either layout (or the year dir containing maven/). Also read from the
#                           environment variable of the same name. Default search for a maven cache:
#                           <PUBLIC|HOME|USERPROFILE>/wpilib/<year>/maven.
#   WPIEMU_WPILIB_SEASON    season pack year (default: the pack with "default": true, seasons/<year>/season.json).
#   WPIEMU_WPILIB_VERSION   optional artifact-version override for the selected season profile.
#   WPIEMU_WPILIB_DOWNLOAD  OFF by default. ON: fetch missing zips from frcmaven (CI on a clean machine).
#
# Outputs:
#   WPIEMU_WPILIB_FOUND            TRUE when headers and the desktop libs were found
#   WPIEMU_NTCORE_FOUND            TRUE when ntcore (ntcore-cpp headers + shared lib, and wpinet) was found as well
#   WPIEMU_WPILIB_RUNTIME_DIRS     directory holding the wpiHal/wpiutil/ntcore shared libs (put on PATH for tests)
#   targets wpiemu_wpilib_hal / wpiemu_wpilib_wpiutil   IMPORTED shared libraries (+ include dirs)
#   targets wpiemu_wpilib_ntcore / wpiemu_wpilib_wpinet IMPORTED shared libraries, only when WPIEMU_NTCORE_FOUND.
#                                  The ntcore artifacts (edu/wpi/first/ntcore/ntcore-cpp, wpinet-cpp) are optional;
#                                  the bridges/vision ntcore binding is built only when they are present.
# Nothing is defined (and configure still succeeds) when WPILib is not available.

include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/WpiEmuVersions.cmake")

set(WPIEMU_WPILIB_ROOT "" CACHE PATH "WPILib HAL root: extracted layout or Maven cache; empty = auto-detect")
if(NOT WPIEMU_WPILIB_SEASON)
    wpiemu_default_season(_wpiemu_default_season)
    set(WPIEMU_WPILIB_SEASON "${_wpiemu_default_season}" CACHE STRING "WPILib season year used by this build")
endif()
wpiemu_season_value(_wpiemu_wpilib_pinned "${WPIEMU_WPILIB_SEASON}" wpilib version)
wpiemu_season_value(WPIEMU_WPILIB_INSTALL_FOLDER "${WPIEMU_WPILIB_SEASON}" wpilib installFolder)
wpiemu_season_abi(WPIEMU_HAL_ABI "${WPIEMU_WPILIB_SEASON}")
wpiemu_season_abi_value(_wpiemu_hal_group "${WPIEMU_WPILIB_SEASON}" groups hal)
wpiemu_season_abi_value(_wpiemu_wpiutil_group "${WPIEMU_WPILIB_SEASON}" groups wpiutil)
wpiemu_season_abi_value(_wpiemu_ntcore_group "${WPIEMU_WPILIB_SEASON}" groups ntcore)
wpiemu_season_abi_value(_wpiemu_wpinet_group "${WPIEMU_WPILIB_SEASON}" groups wpinet)
wpiemu_season_abi_value(_wpiemu_datalog_group "${WPIEMU_WPILIB_SEASON}" groups datalog)
wpiemu_season_abi_value(_wpiemu_hal_header "${WPIEMU_WPILIB_SEASON}" headers hal)
wpiemu_season_abi_value(_wpiemu_wpiutil_header "${WPIEMU_WPILIB_SEASON}" headers wpiutil)
wpiemu_season_abi_value(_wpiemu_ntcore_header "${WPIEMU_WPILIB_SEASON}" headers ntcore)
wpiemu_season_abi_value(_wpiemu_ntcore_required "${WPIEMU_WPILIB_SEASON}" ntcoreRequired)
set(WPIEMU_WPILIB_VERSION "${_wpiemu_wpilib_pinned}" CACHE STRING "WPILib version of hal-cpp/wpiutil-cpp to link against")
if(DEFINED WPIEMU_WPILIB_AUTO_VERSION AND WPIEMU_WPILIB_VERSION STREQUAL WPIEMU_WPILIB_AUTO_VERSION)
    set(WPIEMU_WPILIB_VERSION "${_wpiemu_wpilib_pinned}" CACHE STRING
        "WPILib version of hal-cpp/wpiutil-cpp to link against" FORCE)
else()
    # Existing build trees may have another season's pin cached as the default.
    # Recognize a pinned version from another season as an old default.
    wpiemu_season_years(_wpiemu_all_seasons)
    foreach(_year IN LISTS _wpiemu_all_seasons)
        wpiemu_season_value(_version "${_year}" wpilib version)
        if(NOT _year STREQUAL WPIEMU_WPILIB_SEASON AND WPIEMU_WPILIB_VERSION STREQUAL _version)
            set(WPIEMU_WPILIB_VERSION "${_wpiemu_wpilib_pinned}" CACHE STRING
                "WPILib version of hal-cpp/wpiutil-cpp to link against" FORCE)
            break()
        endif()
    endforeach()
endif()
set(WPIEMU_WPILIB_AUTO_VERSION "${_wpiemu_wpilib_pinned}" CACHE INTERNAL
    "Last profile-derived WPILib version" FORCE)
string(SUBSTRING "${WPIEMU_WPILIB_VERSION}" 0 4 _wpiemu_version_year)
if(NOT _wpiemu_version_year STREQUAL WPIEMU_WPILIB_SEASON)
    message(FATAL_ERROR "WPILib version ${WPIEMU_WPILIB_VERSION} does not match selected season "
                        "${WPIEMU_WPILIB_SEASON}; use a version override from that season")
endif()
option(WPIEMU_WPILIB_DOWNLOAD "Download missing WPILib HAL/wpiutil zips from frcmaven" OFF)

set(WPIEMU_WPILIB_FOUND FALSE)
set(WPIEMU_NTCORE_FOUND FALSE)
set(WPIEMU_WPILIB_RUNTIME_DIRS "")

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
    if(_root AND EXISTS "${_root}/include/${_wpiemu_hal_header}" AND EXISTS "${_root}/include/${_wpiemu_wpiutil_header}")
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
            list(APPEND _wpiemu_wpilib_candidates "${_home}/wpilib/${WPIEMU_WPILIB_INSTALL_FOLDER}/maven")
        endif()
    endforeach()

    string(REPLACE "." "/" _hal_group_path "${_wpiemu_hal_group}")
    string(REPLACE "." "/" _wpiemu_wpiutil_group_path "${_wpiemu_wpiutil_group}")
    string(REPLACE "." "/" _wpiemu_ntcore_group_path "${_wpiemu_ntcore_group}")
    string(REPLACE "." "/" _wpiemu_wpinet_group_path "${_wpiemu_wpinet_group}")
    set(_hal_rel "${_hal_group_path}/hal-cpp/${WPIEMU_WPILIB_VERSION}/hal-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_util_rel "${_wpiemu_wpiutil_group_path}/wpiutil-cpp/${WPIEMU_WPILIB_VERSION}/wpiutil-cpp-${WPIEMU_WPILIB_VERSION}")
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
    # ntcore (+ the wpinet it loads) is optional: the vision bridges' ntcore binding is simply not built without it.
    set(_nt_rel "${_wpiemu_ntcore_group_path}/ntcore-cpp/${WPIEMU_WPILIB_VERSION}/ntcore-cpp-${WPIEMU_WPILIB_VERSION}")
    set(_net_rel "${_wpiemu_wpinet_group_path}/wpinet-cpp/${WPIEMU_WPILIB_VERSION}/wpinet-cpp-${WPIEMU_WPILIB_VERSION}")
    _wpiemu_wpilib_fetch("${_nt_rel}" "headers" _nt_headers_zip)
    _wpiemu_wpilib_fetch("${_nt_rel}" "${_wpiemu_wpilib_classifier}" _nt_libs_zip)
    _wpiemu_wpilib_fetch("${_net_rel}" "${_wpiemu_wpilib_classifier}" _net_libs_zip)
    if(_wpiemu_datalog_group)
        string(REPLACE "." "/" _datalog_group_path "${_wpiemu_datalog_group}")
        set(_datalog_rel "${_datalog_group_path}/datalog-cpp/${WPIEMU_WPILIB_VERSION}/datalog-cpp-${WPIEMU_WPILIB_VERSION}")
        _wpiemu_wpilib_fetch("${_datalog_rel}" "${_wpiemu_wpilib_classifier}" _datalog_libs_zip)
    endif()

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
        # ntcore is staged separately (own markers) so a stage extracted before ntcore was supported is completed.
        if(_nt_headers_zip AND _nt_libs_zip AND _net_libs_zip)
            if(NOT EXISTS "${_stage}/.ntcore-headers")
                file(ARCHIVE_EXTRACT INPUT "${_nt_headers_zip}" DESTINATION "${_stage}/include")
                file(WRITE "${_stage}/.ntcore-headers" "${WPIEMU_WPILIB_VERSION}\n")
            endif()
            if(NOT EXISTS "${_stage}/.ntcore-runtime")
                file(ARCHIVE_EXTRACT INPUT "${_nt_libs_zip}" DESTINATION "${_stage}")
                file(ARCHIVE_EXTRACT INPUT "${_net_libs_zip}" DESTINATION "${_stage}")
                file(WRITE "${_stage}/.ntcore-runtime" "${WPIEMU_WPILIB_VERSION}\n")
            endif()
        endif()
        if(_datalog_libs_zip AND NOT EXISTS "${_stage}/.datalog-runtime")
            file(ARCHIVE_EXTRACT INPUT "${_datalog_libs_zip}" DESTINATION "${_stage}")
            file(WRITE "${_stage}/.datalog-runtime" "${WPIEMU_WPILIB_VERSION}\n")
        endif()
        set(_wpiemu_extracted_root "${_stage}")
    endif()
endif()

if(_wpiemu_extracted_root)
    if(NOT EXISTS "${_wpiemu_extracted_root}/include/${_wpiemu_hal_header}" OR
       NOT EXISTS "${_wpiemu_extracted_root}/include/${_wpiemu_wpiutil_header}")
        set(_wpiemu_extracted_root "")
    endif()
endif()

if(_wpiemu_extracted_root)
    # Publish either discovery route to sibling bridges without replacing the user's cache input.
    set(WPIEMU_WPILIB_ROOT "${_wpiemu_extracted_root}" PARENT_SCOPE)
endif()
set(WPIEMU_HAL_ABI "${WPIEMU_HAL_ABI}" PARENT_SCOPE)

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

        # ntcore is optional on top of the HAL: headers + shared library (+ wpinet, which ntcore loads).
        if(WIN32)
            set(_nt_loc "${_dir}/ntcore.dll")
            set(_nt_imp "${_dir}/ntcore.lib")
            set(_net_loc "${_dir}/wpinet.dll")
            set(_net_imp "${_dir}/wpinet.lib")
        elseif(APPLE)
            set(_nt_loc "${_dir}/libntcore.dylib")
            set(_net_loc "${_dir}/libwpinet.dylib")
        else()
            set(_nt_loc "${_dir}/libntcore.so")
            set(_net_loc "${_dir}/libwpinet.so")
        endif()
        if(EXISTS "${_nt_loc}" AND EXISTS "${_net_loc}" AND EXISTS "${_wpiemu_extracted_root}/include/${_wpiemu_ntcore_header}")
            add_library(wpiemu_wpilib_wpinet SHARED IMPORTED GLOBAL)
            set_target_properties(wpiemu_wpilib_wpinet PROPERTIES
                IMPORTED_LOCATION "${_net_loc}"
                INTERFACE_LINK_LIBRARIES wpiemu_wpilib_wpiutil)
            add_library(wpiemu_wpilib_ntcore SHARED IMPORTED GLOBAL)
            set_target_properties(wpiemu_wpilib_ntcore PROPERTIES
                IMPORTED_LOCATION "${_nt_loc}"
                INTERFACE_INCLUDE_DIRECTORIES "${_wpiemu_extracted_root}/include"
                INTERFACE_LINK_LIBRARIES "wpiemu_wpilib_wpinet;wpiemu_wpilib_wpiutil")
            if(WIN32)
                set_target_properties(wpiemu_wpilib_wpinet PROPERTIES IMPORTED_IMPLIB "${_net_imp}")
                set_target_properties(wpiemu_wpilib_ntcore PROPERTIES IMPORTED_IMPLIB "${_nt_imp}")
            endif()
            set(WPIEMU_NTCORE_FOUND TRUE)
        endif()
        if(_wpiemu_ntcore_required AND NOT WPIEMU_NTCORE_FOUND)
            message(FATAL_ERROR "WPILib ${WPIEMU_WPILIB_VERSION} (${WPIEMU_HAL_ABI}) HAL requires ntcore-cpp and wpinet-cpp "
                "runtime artifacts. Complete the extracted WPIEMU_WPILIB_ROOT, or clear it and configure with "
                "WPIEMU_WPILIB_DOWNLOAD=ON.")
        endif()
        if(_wpiemu_datalog_group)
            # ntcore loads the separate datalog library of this ABI.
            if(WIN32)
                set(_datalog_loc "${_dir}/datalog.dll")
            elseif(APPLE)
                set(_datalog_loc "${_dir}/libdatalog.dylib")
            else()
                set(_datalog_loc "${_dir}/libdatalog.so")
            endif()
            if(NOT WPIEMU_NTCORE_FOUND OR NOT EXISTS "${_datalog_loc}")
                message(FATAL_ERROR "WPILib ${WPIEMU_WPILIB_VERSION} ntcore requires the datalog-cpp runtime artifact. "
                    "Complete the extracted WPIEMU_WPILIB_ROOT, or clear it and configure with WPIEMU_WPILIB_DOWNLOAD=ON.")
            endif()
            add_library(wpiemu_wpilib_datalog SHARED IMPORTED GLOBAL)
            set_target_properties(wpiemu_wpilib_datalog PROPERTIES
                IMPORTED_LOCATION "${_datalog_loc}"
                INTERFACE_LINK_LIBRARIES wpiemu_wpilib_wpiutil)
            if(WIN32)
                set_target_properties(wpiemu_wpilib_datalog PROPERTIES IMPORTED_IMPLIB "${_dir}/datalog.lib")
            endif()
            set_property(TARGET wpiemu_wpilib_ntcore APPEND PROPERTY INTERFACE_LINK_LIBRARIES wpiemu_wpilib_datalog)
        endif()
        if(_wpiemu_ntcore_required)
            # The HAL itself loads ntcore.
            set_property(TARGET wpiemu_wpilib_hal APPEND PROPERTY INTERFACE_LINK_LIBRARIES wpiemu_wpilib_ntcore)
        endif()
    endif()
endif()

if(WPIEMU_WPILIB_FOUND)
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: HAL found at ${_wpiemu_extracted_root}; halsim_wpiemu is built with HAL support")
else()
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: NOT found (set -DWPIEMU_WPILIB_ROOT=<.../wpilib/${WPIEMU_WPILIB_INSTALL_FOLDER}/maven> or "
                   "-DWPIEMU_WPILIB_DOWNLOAD=ON); halsim_wpiemu is built as a stub that refuses to load")
endif()

if(WPIEMU_NTCORE_FOUND)
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore found; the vision bridges get their ntcore binding")
elseif(WPIEMU_WPILIB_FOUND)
    message(STATUS "WPILib ${WPIEMU_WPILIB_VERSION}: ntcore NOT found (ntcore-cpp/wpinet-cpp artifacts); the vision bridges "
                   "are built without their ntcore binding and never publish")
endif()

# ---- 4. local WPILib JDK fallback on Windows (when WPIEMU_WPILIB_DOWNLOAD is ON) --------------------------
# If C:/Users/Public/wpilib/<year>/jdk is missing (for example when JAVA_HOME points there on a machine without
# the full WPILib installer), provision it from a local JDK and strip any bundled <14.40 VC++ runtime DLLs
# that would trip WPILib's WPIUtilJNI.checkMsvcRuntime.
if(WIN32 AND WPIEMU_WPILIB_DOWNLOAD)
    string(SUBSTRING "${WPIEMU_WPILIB_VERSION}" 0 4 _wpiemu_year)
    set(_wpiemu_public_jdk "C:/Users/Public/wpilib/${WPIEMU_WPILIB_INSTALL_FOLDER}/jdk")
    if(NOT EXISTS "${_wpiemu_public_jdk}/bin/java.exe")
        file(GLOB _wpiemu_sys_jdks LIST_DIRECTORIES true
            "C:/Program Files/Java/jdk-*"
            "C:/Program Files/Eclipse Adoptium/jdk-*"
            "C:/Program Files/Microsoft/jdk-*")
        foreach(_cand IN LISTS _wpiemu_sys_jdks)
            if(EXISTS "${_cand}/bin/java.exe")
                message(STATUS "wpiemu: provisioning ${_wpiemu_public_jdk} from ${_cand}...")
                file(MAKE_DIRECTORY "C:/Users/Public/wpilib/${WPIEMU_WPILIB_INSTALL_FOLDER}")
                file(COPY "${_cand}/" DESTINATION "${_wpiemu_public_jdk}")
                file(GLOB _old_vc_dlls
                    "${_wpiemu_public_jdk}/bin/msvcp140*.dll"
                    "${_wpiemu_public_jdk}/bin/vcruntime140*.dll")
                if(_old_vc_dlls)
                    file(REMOVE ${_old_vc_dlls})
                endif()
                break()
            endif()
        endforeach()
    endif()
endif()
