#pragma once

// How the emulator tells the robot process where to connect (docs/reference/extension-ipc.md).
//
// The emulator (server) generates a unique endpoint per session, listens on it, and launches the
// robot with the environment variable WPIEMU_IPC_ENDPOINT=<address>; the HALSim extension (client)
// reads it with ConnectionEndpoint::fromEnvironment() and connects. Addresses:
//   named pipe    \\.\pipe\wpiemu-<random>            (Windows)
//   unix socket   <runtime-dir>/wpiemu-<random>/ipc.sock  (Linux/macOS; the directory is 0700)
//   in-process    inproc:<name>                       (tests / single-process embedding)

#include <string>
#include <string_view>

#include "wpiemu/sdk/core/environment.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::ipc {

inline constexpr std::string_view kNamedPipePrefix = R"(\\.\pipe\)";
inline constexpr std::string_view kInProcessPrefix = "inproc:";

enum class EndpointKind { NamedPipe, UnixSocket, InProcess };

struct EnvironmentEntry {
    std::string name;
    std::string value;
};

class ConnectionEndpoint {
public:
    // A fresh, unguessable endpoint of the local platform's native kind.
    static ConnectionEndpoint generate();
    static ConnectionEndpoint generateInProcess();
    // Syntactic validation only (any kind parses on any OS; the factories check the platform).
    static sdk::Result<ConnectionEndpoint> parse(std::string address);
    static sdk::Result<ConnectionEndpoint> fromEnvironment(const char* variable = sdk::env_names::kIpcEndpoint);

    const std::string& address() const { return address_; }
    EndpointKind kind() const { return kind_; }
    // What to add to the robot process environment.
    EnvironmentEntry environmentEntry() const { return {sdk::env_names::kIpcEndpoint, address_}; }

private:
    ConnectionEndpoint(std::string address, EndpointKind kind) : address_(std::move(address)), kind_(kind) {}

    std::string address_;
    EndpointKind kind_;
};

}  // namespace wpiemu::ipc
