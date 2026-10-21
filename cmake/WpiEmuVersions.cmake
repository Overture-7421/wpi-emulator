# Reads versions.json, the single home of the repository's version pins (docs/BUILDING.md#version-pins).
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
