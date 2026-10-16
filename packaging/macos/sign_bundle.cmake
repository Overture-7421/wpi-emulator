# CPack pre-build hook (CPACK_PRE_BUILD_SCRIPTS): Developer-ID signs the staged macOS app bundle before the
# DragNDrop generator wraps it in a dmg. Runs on the install tree CPack staged in CPACK_TEMPORARY_DIRECTORY.
#
#   WPIEMU_CODESIGN_IDENTITY   (env) e.g. "Developer ID Application: Name (TEAMID)". Unset/empty: signing is
#                              skipped (the bundle keeps the ad-hoc signature macdeployqt applied).
#   WPIEMU_CODESIGN_KEYCHAIN   (env, optional) keychain that holds the identity (CI uses a temporary one).
#
# No secret is read from the repository; the certificate itself is imported by the CI job / developer machine.
if(NOT APPLE)
    return()
endif()

set(_identity "$ENV{WPIEMU_CODESIGN_IDENTITY}")
if(NOT _identity)
    message(STATUS "wpiemu: WPIEMU_CODESIGN_IDENTITY not set; leaving the app bundle ad-hoc signed")
    return()
endif()

file(GLOB _bundles "${CPACK_TEMPORARY_DIRECTORY}/*.app")
list(LENGTH _bundles _n)
if(NOT _n EQUAL 1)
    message(FATAL_ERROR "wpiemu: expected exactly one .app in ${CPACK_TEMPORARY_DIRECTORY}, found: ${_bundles}")
endif()
list(GET _bundles 0 _app)

get_filename_component(_here "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(_entitlements "${_here}/entitlements.plist")

set(_common --force --timestamp --options runtime --sign "${_identity}")
if(DEFINED ENV{WPIEMU_CODESIGN_KEYCHAIN} AND NOT "$ENV{WPIEMU_CODESIGN_KEYCHAIN}" STREQUAL "")
    list(APPEND _common --keychain "$ENV{WPIEMU_CODESIGN_KEYCHAIN}")
endif()

function(_wpiemu_codesign path)
    execute_process(COMMAND codesign ${_common} ${ARGN} "${path}" RESULT_VARIABLE _rc)
    if(NOT _rc EQUAL 0)
        message(FATAL_ERROR "wpiemu: codesign failed (${_rc}) for ${path}")
    endif()
endfunction()

# Innermost code first: loose libraries / plugins (incl. the halsim extension), then frameworks, then the bundle.
file(GLOB_RECURSE _libs "${_app}/*.dylib" "${_app}/*.so")
foreach(_lib IN LISTS _libs)
    if(NOT IS_SYMLINK "${_lib}")
        _wpiemu_codesign("${_lib}")
    endif()
endforeach()

file(GLOB _frameworks "${_app}/Contents/Frameworks/*.framework")
foreach(_fw IN LISTS _frameworks)
    _wpiemu_codesign("${_fw}")
endforeach()

_wpiemu_codesign("${_app}" --entitlements "${_entitlements}")

execute_process(COMMAND codesign --verify --deep --strict --verbose=2 "${_app}" RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "wpiemu: signature verification failed for ${_app}")
endif()
message(STATUS "wpiemu: signed ${_app}")
