# Vision engine: camera model, tag visibility, pose solving, the per-camera frame schedule and the
# AprilTag layout loader (ADR 0017). Game-agnostic and Qt-free. Sources are added to wpiemu_core
# (defined in ../CMakeLists.txt); public headers live under include/wpiemu/core/vision.
file(GLOB WPIEMU_VISION_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

target_sources(wpiemu_core PRIVATE ${WPIEMU_VISION_SOURCES})
target_include_directories(wpiemu_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Determinism (ADR 0003): the solver is plain double math and must not be contracted into FMAs
# (Apple Clang / GCC do by default); MSVC's default /fp:precise already forbids it.
if(NOT MSVC)
    set_source_files_properties(${WPIEMU_VISION_SOURCES}
        PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
endif()
