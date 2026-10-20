# Box3D-backed physics, tire models, drivetrains and chassis coupling (docs/reference/physics.md).
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

# Determinism (docs/reference/physics.md): our own float code must not be contracted into FMAs (Apple Clang / GCC do by default);
# MSVC's default /fp:precise already forbids it. Never add -ffast-math here.
if(MSVC)
    target_compile_options(wpiemu_core_physics PRIVATE /fp:precise)
else()
    target_compile_options(wpiemu_core_physics PRIVATE -ffp-contract=off)
endif()

target_link_libraries(wpiemu_core PUBLIC wpiemu_core_physics)
