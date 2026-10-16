# Box3D-backed physics, tire models, drivetrains and chassis coupling (ADR 0003, ADR 0008).
# A separate static library so it can be built and tested without the rest of core; wpiemu_core links it.
# Box3D stays private: no public header in this directory includes a box3d header.
add_library(wpiemu_core_physics STATIC
    src/box3d_world.cpp
    src/chassis_builder.cpp
    src/chassis_coupling.cpp
    src/chassis_imu.cpp
    src/chassis_simulator.cpp
    src/drivetrains.cpp
    src/quasi_static_loads.cpp
    src/tire_models.cpp)
add_library(wpiemu::core_physics ALIAS wpiemu_core_physics)
target_include_directories(wpiemu_core_physics PUBLIC include)
target_link_libraries(wpiemu_core_physics PUBLIC wpiemu_sdk PRIVATE box3d)
wpiemu_target_defaults(wpiemu_core_physics)

# Determinism (ADR 0003): our own float code must not be contracted into FMAs (Apple Clang / GCC do by default);
# MSVC's default /fp:precise already forbids it. Never add -ffast-math here.
if(MSVC)
    target_compile_options(wpiemu_core_physics PRIVATE /fp:precise)
else()
    target_compile_options(wpiemu_core_physics PRIVATE -ffp-contract=off)
endif()

target_link_libraries(wpiemu_core PUBLIC wpiemu_core_physics)

# Fast inner loop: only the physics tests (tests/physics) against this library. The same sources are also part
# of wpiemu_tests (globbed), which is what CTest runs.
if(WPIEMU_BUILD_TESTS)
    set(WPIEMU_TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tests")
    file(GLOB WPIEMU_PHYSICS_TEST_SOURCES CONFIGURE_DEPENDS "${WPIEMU_TESTS_DIR}/physics/*.cpp")
    add_executable(wpiemu_physics_tests EXCLUDE_FROM_ALL ${WPIEMU_PHYSICS_TEST_SOURCES} "${WPIEMU_TESTS_DIR}/main.cpp")
    target_link_libraries(wpiemu_physics_tests PRIVATE doctest::doctest wpiemu_core_physics)
    target_compile_definitions(wpiemu_physics_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
    wpiemu_target_defaults(wpiemu_physics_tests)
endif()
