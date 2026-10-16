# IPC transport and wire protocol between the Qt app and the HALSim extension.
# Depends on wpiemu_sdk only (ADR 0004). See ipc/README.md and docs/decisions/0007-ipc-transport.md.
set(WPIEMU_IPC_SOURCES
    src/protocol.cpp
    src/binary_codec.cpp
    src/framing.cpp
    src/framed_transport.cpp
    src/memory_stream.cpp
    src/in_process.cpp
    src/endpoint.cpp
    src/handshake.cpp
    src/local_ipc.cpp)
if(WIN32)
    list(APPEND WPIEMU_IPC_SOURCES src/platform/named_pipe_windows.cpp)
else()
    list(APPEND WPIEMU_IPC_SOURCES src/platform/unix_socket_posix.cpp)
endif()

find_package(Threads REQUIRED)

add_library(wpiemu_ipc STATIC ${WPIEMU_IPC_SOURCES})
add_library(wpiemu::ipc ALIAS wpiemu_ipc)
target_include_directories(wpiemu_ipc PUBLIC include PRIVATE src)
target_link_libraries(wpiemu_ipc PUBLIC wpiemu_sdk Threads::Threads)
if(WIN32)
    target_link_libraries(wpiemu_ipc PRIVATE advapi32)
endif()
# Linked into the halsim_wpiemu shared library.
set_target_properties(wpiemu_ipc PROPERTIES POSITION_INDEPENDENT_CODE ON)
wpiemu_target_defaults(wpiemu_ipc)
