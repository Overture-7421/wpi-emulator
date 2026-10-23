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

# Writes <header> with the downloadable-Java pins of versions.json (platform assets, major, minimum Visual C++ runtime) for the
# app's toolchain provisioner. Only rewritten when the content changes.
function(wpiemu_write_toolchain_pins header)
    set(_file "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../versions.json")
    cmake_path(NORMAL_PATH _file)
    file(READ "${_file}" _json)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    string(JSON _vendor GET "${_json}" jdk vendor)
    string(JSON _msvc GET "${_json}" msvcRuntimeMin)
    set(_seasons "")
    string(JSON _season_count LENGTH "${_json}" jdk seasons)
    math(EXPR _season_last "${_season_count} - 1")
    foreach(_i RANGE ${_season_last})
        string(JSON _year MEMBER "${_json}" jdk seasons ${_i})
        string(JSON _season_major GET "${_json}" jdk seasons ${_year})
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
