# Reads versions.json, the single home of repository version pins; see the season upgrade guide
# (docs/guides/wpilib-upgrade.md).
#
#   wpiemu_pinned_version(<out-var> <key> [<sub-key>...])    e.g. wpiemu_pinned_version(v qt release)
#
# Pins that the build cannot read (workflows, Dockerfile, packaging script, docs) are verified against the same
# file by tools/check_versions.py.
include_guard(GLOBAL)

function(wpiemu_pinned_version out)
    set(_file "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../versions.json")
    cmake_path(NORMAL_PATH _file)
    file(READ "${_file}" _json)
    string(JSON _value GET "${_json}" ${ARGN})
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    set(${out} "${_value}" PARENT_SCOPE)
endfunction()

# ---- season packs (seasons/<year>/season.json, seasons/abi/<id>.json; docs/reference/seasons.md) ----------------------
set(_WPIEMU_SEASONS_DIR "${CMAKE_CURRENT_LIST_DIR}/../seasons")
cmake_path(NORMAL_PATH _WPIEMU_SEASONS_DIR)

# The years of every season pack in ascending order; folders starting with '_' (the template) are not seasons.
function(wpiemu_season_years out)
    file(GLOB _manifests CONFIGURE_DEPENDS "${_WPIEMU_SEASONS_DIR}/*/season.json")
    set(_years "")
    foreach(_manifest IN LISTS _manifests)
        get_filename_component(_dir "${_manifest}" DIRECTORY)
        get_filename_component(_year "${_dir}" NAME)
        if(NOT _year MATCHES "^_")
            list(APPEND _years "${_year}")
        endif()
    endforeach()
    list(SORT _years COMPARE NATURAL)
    set(${out} "${_years}" PARENT_SCOPE)
endfunction()

# wpiemu_season_value(<out> <year> <key>...) reads a member of seasons/<year>/season.json.
function(wpiemu_season_value out year)
    set(_file "${_WPIEMU_SEASONS_DIR}/${year}/season.json")
    if(NOT EXISTS "${_file}")
        message(FATAL_ERROR "wpiemu: there is no season pack for ${year} (${_file})")
    endif()
    file(READ "${_file}" _json)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    string(JSON _value GET "${_json}" ${ARGN})
    set(${out} "${_value}" PARENT_SCOPE)
endfunction()

# wpiemu_abi_value(<out> <abi> <key>...) reads a member of seasons/abi/<abi>.json.
function(wpiemu_abi_value out abi)
    set(_file "${_WPIEMU_SEASONS_DIR}/abi/${abi}.json")
    if(NOT EXISTS "${_file}")
        message(FATAL_ERROR "wpiemu: there is no HAL ABI profile '${abi}' (${_file})")
    endif()
    file(READ "${_file}" _json)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    string(JSON _value ERROR_VARIABLE _error GET "${_json}" ${ARGN})
    if(_error)
        set(_value "")
    endif()
    set(${out} "${_value}" PARENT_SCOPE)
endfunction()

# wpiemu_season_abi(<out> <year>) is the HAL ABI id of a season.
function(wpiemu_season_abi out year)
    wpiemu_season_value(_abi "${year}" platform abi)
    set(${out} "${_abi}" PARENT_SCOPE)
endfunction()

# wpiemu_season_abi_value(<out> <year> <key>...) reads a member of the season's ABI profile ("" when absent).
function(wpiemu_season_abi_value out year)
    wpiemu_season_abi(_abi "${year}")
    wpiemu_abi_value(_value "${_abi}" ${ARGN})
    set(${out} "${_value}" PARENT_SCOPE)
endfunction()

# wpiemu_season_array(<out> <year> <key>...) is the elements of a JSON array member of a season manifest, as a CMake list.
function(wpiemu_season_array out year)
    set(_file "${_WPIEMU_SEASONS_DIR}/${year}/season.json")
    file(READ "${_file}" _json)
    string(JSON _count LENGTH "${_json}" ${ARGN})
    set(_items "")
    if(_count GREATER 0)
        math(EXPR _last "${_count} - 1")
        foreach(_i RANGE ${_last})
            string(JSON _item GET "${_json}" ${ARGN} ${_i})
            list(APPEND _items "${_item}")
        endforeach()
    endif()
    set(${out} "${_items}" PARENT_SCOPE)
endfunction()

# The game plugin id a season names ("" for a platform-only season whose manifest has "game": null).
function(wpiemu_season_game_plugin out year)
    set(_file "${_WPIEMU_SEASONS_DIR}/${year}/season.json")
    file(READ "${_file}" _json)
    string(JSON _type TYPE "${_json}" game)
    set(_plugin "")
    if(NOT _type STREQUAL "NULL")
        string(JSON _plugin GET "${_json}" game plugin)
    endif()
    set(${out} "${_plugin}" PARENT_SCOPE)
endfunction()

# The season that sets "default": true.
function(wpiemu_default_season out)
    wpiemu_season_years(_years)
    foreach(_year IN LISTS _years)
        wpiemu_season_value(_default "${_year}" default)
        if(_default)
            set(${out} "${_year}" PARENT_SCOPE)
            return()
        endif()
    endforeach()
    message(FATAL_ERROR "wpiemu: no season pack sets \"default\": true")
endfunction()

# Writes <header> with the downloadable-Java pins of versions.json (platform assets, major, minimum Visual C++ runtime) for the
# app's toolchain provisioner. The Java major of each season comes from its season pack. Only rewritten when the content changes.
function(wpiemu_write_toolchain_pins header)
    set(_file "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../versions.json")
    cmake_path(NORMAL_PATH _file)
    file(READ "${_file}" _json)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    string(JSON _vendor GET "${_json}" jdk vendor)
    string(JSON _msvc GET "${_json}" msvcRuntimeMin)
    set(_seasons "")
    wpiemu_season_years(_years)
    foreach(_year IN LISTS _years)
        wpiemu_season_value(_season_major "${_year}" toolchain javaMajor)
        string(APPEND _seasons "    {${_year}, ${_season_major}},\n")
    endforeach()
    string(JSON _count LENGTH "${_json}" jdk assets)
    set(_rows "")
    math(EXPR _last "${_count} - 1")
    foreach(_i RANGE ${_last})
        string(JSON _key MEMBER "${_json}" jdk assets ${_i})
        # "<major>-<platform>", e.g. "17-windows-x64"
        string(REGEX MATCH "^([0-9]+)-(.+)$" _unused "${_key}")
        set(_major "${CMAKE_MATCH_1}")
        set(_platform "${CMAKE_MATCH_2}")
        string(JSON _version GET "${_json}" jdk assets ${_key} version)
        string(JSON _url GET "${_json}" jdk assets ${_key} url)
        string(JSON _sha GET "${_json}" jdk assets ${_key} sha256)
        string(JSON _size GET "${_json}" jdk assets ${_key} size)
        string(APPEND _rows "    {${_major}, \"${_platform}\", \"${_version}\", \"${_url}\", \"${_sha}\", ${_size}LL},\n")
    endforeach()
    file(CONFIGURE OUTPUT "${header}" @ONLY CONTENT
"// Generated from versions.json by cmake/WpiEmuVersions.cmake. Do not edit.
#pragma once

namespace wpiemu::app::pins {

struct JdkAsset {
    int major;
    const char* platform;   // \"<windows|macos|linux>-<x64|arm64>\"
    const char* version;
    const char* url;
    const char* sha256;
    long long sizeBytes;
};

// The Java major each WPILib season builds with. A season missing here has no download.
struct JdkSeason {
    int year;
    int major;
};

inline constexpr const char* kJdkVendor = \"@_vendor@\";
inline constexpr const char* kMsvcRuntimeMin = \"@_msvc@\";
inline constexpr JdkSeason kJdkSeasons[] = {
@_seasons@};
inline constexpr JdkAsset kJdkAssets[] = {
@_rows@};

}  // namespace wpiemu::app::pins
")
endfunction()
