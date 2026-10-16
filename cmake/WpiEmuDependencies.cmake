# Download cache shared by all build trees; keeps clean rebuilds offline-friendly.
include(FetchContent)

# ---- Box3D (physics, C17) -------------------------------------------------
# Pinned tag; consumed only through core/ (behind an IPhysicsWorld adapter).
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(box3d
    GIT_REPOSITORY https://github.com/erincatto/box3d.git
    GIT_TAG        v0.1.0
    GIT_SHALLOW    TRUE
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(box3d)

if(MSVC)
    # Box3D forces a static CRT; first-party code uses the default (dynamic) CRT.
    set_target_properties(box3d PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# ---- doctest (unit tests) -------------------------------------------------
if(WPIEMU_BUILD_TESTS)
    # doctest is header-only; its own CMakeLists is skipped (SOURCE_SUBDIR) because it targets
    # an obsolete CMake minimum version. We define the interface target ourselves.
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
endif()

# ---- Qt 6 (app only) ------------------------------------------------------
# Point CMAKE_PREFIX_PATH at the Qt kit, e.g. C:/Qt/6.8.3/msvc2022_64
if(WPIEMU_BUILD_APP)
    find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Quick Quick3D QuickWidgets)
    qt_standard_project_setup()
endif()
