#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::halsim {

// Environment variable names (documented in docs/reference/extension-ipc.md). The IPC names are the ones
// ipc/endpoint.h publishes (the emulator sets them); the unprefixed spellings are accepted as aliases.
inline constexpr std::string_view kEnvEndpoint = "WPIEMU_IPC_ENDPOINT";
inline constexpr std::string_view kEnvEndpointAlias = "WPIEMU_ENDPOINT";
inline constexpr std::string_view kEnvClock = "WPIEMU_CLOCK";
inline constexpr std::string_view kEnvWatchdogMs = "WPIEMU_IPC_WATCHDOG_MS";
inline constexpr std::string_view kEnvWatchdogMsAlias = "WPIEMU_WATCHDOG_MS";
inline constexpr std::string_view kEnvWatchdog = "WPIEMU_WATCHDOG";
inline constexpr std::string_view kEnvConnectTimeoutMs = "WPIEMU_IPC_CONNECT_TIMEOUT_MS";
inline constexpr std::string_view kEnvConnectTimeoutMsAlias = "WPIEMU_CONNECT_TIMEOUT_MS";
inline constexpr std::string_view kEnvHandshakeTimeoutMs = "WPIEMU_HANDSHAKE_TIMEOUT_MS";
inline constexpr std::string_view kEnvReconnect = "WPIEMU_RECONNECT";
inline constexpr std::string_view kEnvLogLevel = "WPIEMU_LOG_LEVEL";
inline constexpr std::string_view kEnvProgramName = "WPIEMU_PROGRAM_NAME";
inline constexpr std::string_view kEnvDisable = "WPIEMU_DISABLE";
inline constexpr std::string_view kEnvLiveDiscovery = "WPIEMU_LIVE_DISCOVERY";

enum class ClockPreference : std::uint8_t {
    FollowEmulator,  // "master" (default): lock-step unless the emulator's HelloAck says Realtime
    Realtime,        // "realtime": never pause HAL time, whatever the emulator asks
};

struct ExtensionConfig {
    // Empty endpoint = the extension is inert (robot started without the emulator).
    std::string endpoint;
    bool disabled = false;
    ClockPreference clock = ClockPreference::FollowEmulator;
    std::chrono::milliseconds connectTimeout{10000};
    std::chrono::milliseconds handshakeTimeout{5000};
    // Hello::proposedWatchdogMs; 0 = no preference (the emulator's value or the default wins).
    std::uint32_t proposedWatchdogMs = 0;
    // false (WPIEMU_WATCHDOG=off, for debugging with breakpoints): the extension never unblocks the robot by itself.
    bool watchdogEnabled = true;
    bool reconnect = true;
    std::chrono::milliseconds reconnectInterval{1000};
    sdk::LogLevel logLevel = sdk::LogLevel::Info;
    std::string programName;  // overrides the detected executable name
    // A full (non-delta) ActuatorFrame is sent every N completed ticks; 0 = only after a reset.
    std::uint32_t fullFrameInterval = 400;
    // Poll the bridges (discovery) while the robot constructor is still running, so devices show up live.
    // Default off: everything is announced once, when the robot program starts. During construction
    // the robot thread is creating devices concurrently, and polling the Phoenix 6 bridge then crashed
    // the JVM (access violation inside the CTRE native sim) in the first end-to-end run; after program
    // start the robot is quiescent between steps, which is the contract the bridges are written for.
    bool pollWhileConstructing = false;
};

using EnvLookup = std::function<std::optional<std::string>(std::string_view name)>;

// Parses the environment. Malformed values keep the default and add a human-readable entry to
// `warnings` (never throws).
ExtensionConfig parseExtensionConfig(const EnvLookup& env, std::vector<std::string>* warnings = nullptr);

// Reads the process environment (std::getenv).
std::optional<std::string> processEnvironment(std::string_view name);

std::optional<sdk::LogLevel> parseLogLevel(std::string_view text);
std::string_view toString(sdk::LogLevel level);

}  // namespace wpiemu::halsim
