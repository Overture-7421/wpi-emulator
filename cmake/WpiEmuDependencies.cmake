# Download cache shared by all build trees; keeps clean rebuilds offline-friendly.
include(FetchContent)

# ---- Box3D (physics, C17) -------------------------------------------------
# Pinned tag; consumed only through core/ (behind an IPhysicsWorld adapter).
if(NOT WPIEMU_EXTENSION_ONLY AND NOT WPIEMU_RELAY_ONLY)
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

# ---- Networking libraries (ENet6 + Monocypher, C) ---------------------------
# Pinned tags. Only net/ and relay/ may link wpiemu_third_party_enet / wpiemu_third_party_monocypher; they are
# available in Qt-free and relay-only builds (never in extension-only builds, which have no networking) and are fetched
# only when WPIEMU_FETCH_NET_LIBS is ON.
option(WPIEMU_FETCH_NET_LIBS "Fetch and build ENet6 and Monocypher for the networking modules" ON)
if(WPIEMU_FETCH_NET_LIBS AND NOT WPIEMU_EXTENSION_ONLY)
    # ENet6 (SirLynix/enet6, MIT): dual-stack IPv6 fork of lsalzman/enet. Its CMake builds a static library by default.
    FetchContent_Declare(enet6
        GIT_REPOSITORY https://github.com/SirLynix/enet6.git
        GIT_TAG        v6.1.4
        GIT_SHALLOW    TRUE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(enet6)

    # Monocypher (BSD-2-Clause or CC0-1.0): X25519, BLAKE2b and XChaCha20-Poly1305 live in the core source file.
    # Upstream ships no CMakeLists, so FetchContent only downloads it.
    FetchContent_Declare(monocypher
        GIT_REPOSITORY https://github.com/LoupVaillant/Monocypher.git
        GIT_TAG        4.0.3
        GIT_SHALLOW    TRUE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(monocypher)

    add_library(wpiemu_monocypher STATIC "${monocypher_SOURCE_DIR}/src/monocypher.c")
    set_target_properties(wpiemu_monocypher PROPERTIES
        C_STANDARD 99
        C_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ON)
    target_include_directories(wpiemu_monocypher SYSTEM PUBLIC "${monocypher_SOURCE_DIR}/src")

    set_target_properties(enet6 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    if(MSVC)
        # Keep the dynamic CRT used by first-party code.
        set_target_properties(enet6 wpiemu_monocypher PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
        # Third-party warnings in ENet6 v6.1.4 (sprintf, enum arithmetic, unused local); first-party code stays strict.
        target_compile_options(enet6 PRIVATE /wd4996 /wd5287 /wd4101)
    endif()

    # Include dirs are SYSTEM so third-party headers never trip first-party warnings-as-errors.
    add_library(wpiemu_third_party_enet INTERFACE)
    target_include_directories(wpiemu_third_party_enet SYSTEM INTERFACE "${enet6_SOURCE_DIR}/include")
    target_link_libraries(wpiemu_third_party_enet INTERFACE enet6)
    if(WIN32)
        target_link_libraries(wpiemu_third_party_enet INTERFACE ws2_32 winmm)
    endif()

    add_library(wpiemu_third_party_monocypher INTERFACE)
    target_link_libraries(wpiemu_third_party_monocypher INTERFACE wpiemu_monocypher)

endif()

# ---- doctest (unit tests) -------------------------------------------------
if(WPIEMU_BUILD_TESTS)
    include(WpiEmuTests)
endif()

# ---- Qt 6 (app only) ------------------------------------------------------
if(WPIEMU_BUILD_APP)
    include(WpiEmuQt)
endif()

