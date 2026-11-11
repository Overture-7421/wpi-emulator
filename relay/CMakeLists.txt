# Relay server for practice matches (rooms, limits, ENet event loop), its probe and its tests. Depends on sdk, ipc and
# wpiemu_net only: the relay never links ENet or Monocypher itself (tools/check_boundaries.py).
# Layout: include/wpiemu/relay (public headers), src (wpiemu_relay_core), server (wpiemu_relay), probe (wpiemu_relay_probe),
# tests (wpiemu_relay_tests). Everything is globbed, so adding a file needs no CMake edit.
find_package(Threads REQUIRED)

file(GLOB_RECURSE WPIEMU_RELAY_CORE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE WPIEMU_RELAY_SERVER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/server/*.cpp")
file(GLOB_RECURSE WPIEMU_RELAY_PROBE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/probe/*.cpp")

add_library(wpiemu_relay_core STATIC ${WPIEMU_RELAY_CORE_SOURCES})
add_library(wpiemu::relay_core ALIAS wpiemu_relay_core)
target_include_directories(wpiemu_relay_core PUBLIC include PRIVATE src)
target_link_libraries(wpiemu_relay_core PUBLIC wpiemu_sdk wpiemu_ipc wpiemu_net Threads::Threads)
wpiemu_target_defaults(wpiemu_relay_core)

add_executable(wpiemu_relay ${WPIEMU_RELAY_SERVER_SOURCES})
target_link_libraries(wpiemu_relay PRIVATE wpiemu_relay_core wpiemu_net wpiemu_ipc wpiemu_sdk)
wpiemu_target_defaults(wpiemu_relay)

add_executable(wpiemu_relay_probe ${WPIEMU_RELAY_PROBE_SOURCES})
target_link_libraries(wpiemu_relay_probe PRIVATE wpiemu_relay_core wpiemu_net wpiemu_ipc wpiemu_sdk)
wpiemu_target_defaults(wpiemu_relay_probe)

# The relay image is built from a relay-only tree; the desktop package never ships the server.
if(WPIEMU_RELAY_ONLY)
    # The image's runtime base can ship an older libstdc++ than the compiler that built the binary, so the C++ runtime is
    # linked in; only libc (same Ubuntu release in both stages) stays dynamic.
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        foreach(_relay_exe wpiemu_relay wpiemu_relay_probe)
            target_link_options(${_relay_exe} PRIVATE -static-libstdc++ -static-libgcc)
        endforeach()
    endif()
    install(TARGETS wpiemu_relay wpiemu_relay_probe RUNTIME DESTINATION bin COMPONENT Relay)
endif()

if(WPIEMU_BUILD_TESTS)
    file(GLOB_RECURSE WPIEMU_RELAY_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    add_executable(wpiemu_relay_tests ${WPIEMU_RELAY_TEST_SOURCES})
    target_link_libraries(wpiemu_relay_tests PRIVATE wpiemu_sdk wpiemu_ipc wpiemu_net wpiemu_relay_core)
    target_include_directories(wpiemu_relay_tests PRIVATE tests)
    wpiemu_doctest_target(wpiemu_relay_tests)
    doctest_discover_tests(wpiemu_relay_tests)
endif()
