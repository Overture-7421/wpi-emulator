#pragma once

// The process environment and the names of the variables that carry the emulator <-> robot-process contract
// (docs/reference/extension-ipc.md): the emulator sets them when it launches the robot, the HALSim extension reads
// them. Names used by a single module stay with that module.

#include <cstdlib>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace wpiemu::sdk {

using EnvLookup = std::function<std::optional<std::string>(std::string_view name)>;

// A variable of the process environment; unset and empty both read as nullopt.
inline std::optional<std::string> readEnvironment(std::string_view name) {
    const std::string key(name);
    std::string value;
#if defined(_WIN32)
    char* buffer = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&buffer, &length, key.c_str()) != 0 || buffer == nullptr) return std::nullopt;
    value = buffer;
    std::free(buffer);
#else
    const char* raw = std::getenv(key.c_str());  // NOLINT(concurrency-mt-unsafe): read at startup only
    if (raw == nullptr) return std::nullopt;
    value = raw;
#endif
    if (value.empty()) return std::nullopt;
    return value;
}

namespace env_names {
inline constexpr const char* kIpcEndpoint = "WPIEMU_IPC_ENDPOINT";   // authoritative; set by the emulator
inline constexpr const char* kIpcEndpointAlias = "WPIEMU_ENDPOINT";
inline constexpr const char* kIpcConnectTimeoutMs = "WPIEMU_IPC_CONNECT_TIMEOUT_MS";
inline constexpr const char* kIpcWatchdogMs = "WPIEMU_IPC_WATCHDOG_MS";
inline constexpr const char* kClock = "WPIEMU_CLOCK";                // "master" (lock-step) | "realtime"
inline constexpr const char* kProgramName = "WPIEMU_PROGRAM_NAME";
inline constexpr const char* kLogLevel = "WPIEMU_LOG_LEVEL";
inline constexpr const char* kLiveDiscovery = "WPIEMU_LIVE_DISCOVERY";
}  // namespace env_names

}  // namespace wpiemu::sdk
