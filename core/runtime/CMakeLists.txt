# Sim runtime, config model and robot launcher (no Qt, no season knowledge). Sources are added to
# wpiemu_core (defined in ../CMakeLists.txt); headers live under include/wpiemu/core/{config,runtime,launcher}.
find_package(Threads REQUIRED)

# Feature macros do not account for libc++ deployment-target availability.
include(CheckCXXSourceCompiles)
check_cxx_source_compiles([=[
    #include <charconv>
    int main() {
        char buffer[64];
        double value = 0.0;
        const auto parsed = std::from_chars(buffer, buffer, value);
        const auto formatted = std::to_chars(buffer, buffer + sizeof(buffer), value);
        const auto formattedFloat = std::to_chars(buffer, buffer + sizeof(buffer), 0.5f);
        return static_cast<int>(parsed.ec) + static_cast<int>(formatted.ec)
            + static_cast<int>(formattedFloat.ec);
    }
]=] WPIEMU_HAS_FLOAT_CHARCONV)
target_compile_definitions(wpiemu_core PRIVATE
    WPIEMU_HAS_FLOAT_CHARCONV=$<BOOL:${WPIEMU_HAS_FLOAT_CHARCONV}>)

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
