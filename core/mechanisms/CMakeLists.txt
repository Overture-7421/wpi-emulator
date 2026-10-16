# Motor catalog, 1-DOF plants (WPILib LinearSystemId equivalents), sensor models and the generic
# mechanism templates. Sources are added to wpiemu_core (defined in ../CMakeLists.txt); public
# headers live under include/wpiemu/core/mechanisms, private ones next to the sources.
file(GLOB WPIEMU_MECHANISMS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

target_sources(wpiemu_core PRIVATE ${WPIEMU_MECHANISMS_SOURCES})
target_include_directories(wpiemu_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
