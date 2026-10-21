# Builds ONLY the robot editor (+ its tests and the demo) against the SDK and app/ui, without the rest of the
# tree (core, ipc, wizard, run view...). Handy for UI work and for CI jobs that only touch app/editor:
#
#   cmake -S app/editor/standalone -B build/ui-editor -G Ninja -DCMAKE_PREFIX_PATH=C:/Qt/<version>/msvc2022_64
#   cmake --build build/ui-editor && ctest --test-dir build/ui-editor
#
# The regular build (root CMakeLists.txt -> app/ -> editor/) does not use this file.
cmake_minimum_required(VERSION 3.25)

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

include(WpiEmuDefaults)
wpiemu_option_warnings_as_errors(ON)
add_subdirectory("${WPIEMU_REPO_ROOT}/sdk" sdk)

include(WpiEmuQt)

if(WPIEMU_BUILD_TESTS)
    include(WpiEmuTests)
endif()

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../ui" ui)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." editor)
