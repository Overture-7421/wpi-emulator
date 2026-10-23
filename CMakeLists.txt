cmake_minimum_required(VERSION 3.25)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/WpiEmuVersions.cmake")

# Set before project()/compiler initialization; presets, releases and the bundle share this default.
if(CMAKE_HOST_APPLE AND NOT DEFINED CMAKE_OSX_DEPLOYMENT_TARGET)
    wpiemu_pinned_version(_wpiemu_macos_deployment macosDeployment)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "${_wpiemu_macos_deployment}" CACHE STRING "Minimum supported macOS version")
endif()

# Qt otherwise forces the deprecated OLD behavior of CMP0156 and warns about it.
if(POLICY CMP0156)
    cmake_policy(SET CMP0156 NEW)
    set(QT_FORCE_CMP0156_TO_NEW ON)
endif()

project(wpi-emulator
    VERSION 0.0.0
    DESCRIPTION "Cross-platform emulator for unmodified WPILib robot projects"
    LANGUAGES C CXX)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

wpiemu_pinned_version(_wpiemu_default_wpilib_season wpilib default)
set(WPIEMU_WPILIB_SEASON "${_wpiemu_default_wpilib_season}" CACHE STRING "WPILib season year used by this build")
set_property(CACHE WPIEMU_WPILIB_SEASON PROPERTY STRINGS "2026" "2027")
if(WPIEMU_WPILIB_SEASON STREQUAL "2027")
    set(CMAKE_CXX_STANDARD 23)
else()
    set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Keep shared libraries / executables together so DLLs resolve when running from the build tree.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

option(WPIEMU_BUILD_APP "Build the Qt application (requires Qt 6)" ON)
option(WPIEMU_BUILD_TESTS "Build unit tests and register CTest checks" ON)
option(WPIEMU_BUILD_PLUGIN_TEMPLATE "Build plugins/_template to keep the template compiling" ON)

include(WpiEmuDefaults)
wpiemu_option_warnings_as_errors(OFF)
include(WpiEmuDependencies)

add_subdirectory(sdk)
add_subdirectory(core)
add_subdirectory(ipc)
add_subdirectory(halsim_ext)
add_subdirectory(bridges)
add_subdirectory(plugins)

if(WPIEMU_BUILD_APP)
    add_subdirectory(app)
else()
    add_subdirectory(app/host)
endif()

if(WPIEMU_BUILD_TESTS)
    add_subdirectory(tests)
endif()

include(WpiEmuInstall)   # install() + CPack rules (docs/releasing.md)
