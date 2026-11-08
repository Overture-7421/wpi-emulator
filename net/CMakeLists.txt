# Networking for practice matches: ENet transports, the secure channel, invite codes and the relay transports.
# Depends on wpiemu_sdk and wpiemu_ipc (the shared BinaryCodec). ENet6 and Monocypher stay private: no public header in
# this directory includes an enet or monocypher header, and only this module may link them (tools/check_boundaries.py).
# Sources and tests are globbed, so adding a file under src/ or tests/ needs no CMake edit.
find_package(Threads REQUIRED)

if(NOT TARGET wpiemu_third_party_enet OR NOT TARGET wpiemu_third_party_monocypher)
    message(FATAL_ERROR "The net module needs ENet6 and Monocypher; configure with WPIEMU_FETCH_NET_LIBS=ON (the default)")
endif()

file(GLOB_RECURSE WPIEMU_NET_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(wpiemu_net STATIC ${WPIEMU_NET_SOURCES})
add_library(wpiemu::net ALIAS wpiemu_net)
target_include_directories(wpiemu_net PUBLIC include PRIVATE src)
target_link_libraries(wpiemu_net
    PUBLIC wpiemu_sdk wpiemu_ipc Threads::Threads
    PRIVATE wpiemu_third_party_enet wpiemu_third_party_monocypher)
wpiemu_target_defaults(wpiemu_net)

if(WPIEMU_BUILD_TESTS)
    file(GLOB_RECURSE WPIEMU_NET_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    add_executable(wpiemu_net_tests ${WPIEMU_NET_TEST_SOURCES})
    target_link_libraries(wpiemu_net_tests PRIVATE wpiemu_sdk wpiemu_ipc wpiemu_net)
    target_include_directories(wpiemu_net_tests PRIVATE tests)
    wpiemu_doctest_target(wpiemu_net_tests)
    doctest_discover_tests(wpiemu_net_tests)
endif()
