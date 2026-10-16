# Builds ONLY the robot editor (+ its tests and the demo) against the SDK, without the rest of the
# tree (core, ipc, wizard, run view...). Handy for UI work and for CI jobs that only touch app/editor:
#
#   cmake -S app/editor/standalone -B build/ui-editor -G Ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64
#   cmake --build build/ui-editor && ctest --test-dir build/ui-editor
#
# The regular build (root CMakeLists.txt -> app/ -> editor/) does not use this file.
cmake_minimum_required(VERSION 3.24)

if(POLICY CMP0156)
    cmake_policy(SET CMP0156 NEW)
    set(QT_FORCE_CMP0156_TO_NEW ON)
endif()

project(wpiemu_editor_standalone LANGUAGES C CXX)

get_filename_component(WPIEMU_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
list(PREPEND CMAKE_MODULE_PATH "${WPIEMU_REPO_ROOT}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

option(WPIEMU_BUILD_TESTS "Build the editor unit tests" ON)
option(WPIEMU_WARNINGS_AS_ERRORS "Treat warnings in first-party code as errors" ON)

include(WpiEmuDefaults)
add_subdirectory("${WPIEMU_REPO_ROOT}/sdk" sdk)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Quick Quick3D QuickWidgets)
qt_standard_project_setup()

if(WPIEMU_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  _wpiemu_no_cmake
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(doctest)
    add_library(doctest INTERFACE)
    add_library(doctest::doctest ALIAS doctest)
    target_include_directories(doctest SYSTEM INTERFACE "${doctest_SOURCE_DIR}")
    enable_testing()
endif()

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." editor)
