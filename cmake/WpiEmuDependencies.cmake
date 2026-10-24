# Download cache shared by all build trees; keeps clean rebuilds offline-friendly.
include(FetchContent)

# ---- Box3D (physics, C17) -------------------------------------------------
# Pinned tag; consumed only through core/ (behind an IPhysicsWorld adapter).
if(NOT WPIEMU_EXTENSION_ONLY)
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
endif()

# ---- doctest (unit tests) -------------------------------------------------
if(WPIEMU_BUILD_TESTS)
    include(WpiEmuTests)
endif()

# ---- Qt 6 (app only) ------------------------------------------------------
if(WPIEMU_BUILD_APP)
    include(WpiEmuQt)
endif()
