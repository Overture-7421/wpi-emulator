# Sim runtime, config model and robot launcher (no Qt, no season knowledge). Sources are added to
# wpiemu_core (defined in ../CMakeLists.txt); headers live under include/wpiemu/core/{config,runtime,launcher}.
find_package(Threads REQUIRED)

file(GLOB WPIEMU_RUNTIME_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.cpp")
if(WIN32)
    list(FILTER WPIEMU_RUNTIME_SOURCES EXCLUDE REGEX "_posix\\.cpp$")
else()
    list(FILTER WPIEMU_RUNTIME_SOURCES EXCLUDE REGEX "_win\\.cpp$")
endif()

target_sources(wpiemu_core PRIVATE ${WPIEMU_RUNTIME_SOURCES})
target_include_directories(wpiemu_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(wpiemu_core PUBLIC Threads::Threads)
