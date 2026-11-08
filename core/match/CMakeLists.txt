# Match host and match client session (docs/ARCHITECTURE.md). A separate static library on top of wpiemu_core, which
# already holds the sim runtime (SimulationSession) and the physics (ChassisSimulator, WheelSetpointController).
# It sees the SDK and core only: the transports arrive as sdk ITransport interfaces, so this module never includes net or ipc.
# Sources are globbed, so adding a file under src/ needs no CMake edit. Its unit tests live in tests/match (wpiemu_tests).
file(GLOB_RECURSE WPIEMU_MATCH_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(wpiemu_match STATIC ${WPIEMU_MATCH_SOURCES})
add_library(wpiemu::match ALIAS wpiemu_match)
target_include_directories(wpiemu_match PUBLIC include PRIVATE src)
target_link_libraries(wpiemu_match PUBLIC wpiemu_sdk wpiemu_core)
wpiemu_target_defaults(wpiemu_match)

# Determinism: same floating-point policy as wpiemu_core (see core/physics/CMakeLists.txt).
if(MSVC)
    target_compile_options(wpiemu_match PRIVATE /fp:precise)
else()
    target_compile_options(wpiemu_match PRIVATE -ffp-contract=off)
endif()
