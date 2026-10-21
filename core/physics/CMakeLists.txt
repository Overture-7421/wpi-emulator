# Box3D-backed physics, tire models, drivetrains and chassis coupling (docs/reference/physics.md).
# Sources are added to wpiemu_core (defined in ../CMakeLists.txt).
# Box3D stays private: no public header in this directory includes a box3d header.
file(GLOB WPIEMU_PHYSICS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

target_sources(wpiemu_core PRIVATE ${WPIEMU_PHYSICS_SOURCES})
target_include_directories(wpiemu_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(wpiemu_core PUBLIC wpiemu_sdk PRIVATE $<BUILD_INTERFACE:box3d>)

# Determinism (docs/reference/physics.md): our own float code must not be contracted into FMAs (Apple Clang / GCC do by default);
# MSVC's default /fp:precise already forbids it. Never add -ffast-math here.
if(MSVC)
    target_compile_options(wpiemu_core PRIVATE /fp:precise)
else()
    target_compile_options(wpiemu_core PRIVATE -ffp-contract=off)
endif()
