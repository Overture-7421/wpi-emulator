# Test support shared by the full build and the standalone editor build; include it only when tests are built.
#   doctest::doctest, doctest_discover_tests    the one doctest declaration
#   wpiemu_doctest_target(<target>)             settings every doctest executable needs
# Including it also enables CTest for the including project. Do it before adding the subdirectories: a directory added
# earlier would not be part of CTest, and tests registered there would be silently missing.
include(FetchContent)

# doctest is header-only; its own CMakeLists is skipped (SOURCE_SUBDIR) because it targets an obsolete CMake minimum
# version. We define the interface target ourselves.
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
list(APPEND CMAKE_MODULE_PATH "${doctest_SOURCE_DIR}/scripts/cmake")
include(doctest)
enable_testing()

# DOCTEST_CONFIG_USE_STD_HEADERS makes doctest include the standard headers instead of forward-declaring their types.
function(wpiemu_doctest_target target)
    target_link_libraries(${target} PRIVATE doctest::doctest)
    target_compile_definitions(${target} PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
    wpiemu_target_defaults(${target})
endfunction()
