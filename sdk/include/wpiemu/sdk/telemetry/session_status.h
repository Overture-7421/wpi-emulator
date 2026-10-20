#pragma once

#include <cstdint>
#include <string>

#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::sdk {

// Vocabulary of one robot-program run as every layer sees it: the Qt-free session produces it, the
// runtime host republishes it and the UI presents it. Data only; nothing here knows about Qt.

// Run: lock-step from the first tick (refuses an invalid model). Armed: the one long-lived robot instance of the
// app: devices are announced and the robot runs in real time until BeginLockstep (see SimCommand::BeginLockstep);
// an invalid model only warns so devices can still be detected.
enum class SessionMode { Run, Armed };

// State of the robot program process launched by the emulator.
enum class ProcessState {
    Idle,           // never started / cleanly reset
    Starting,       // gradle / robot executable is launching
    Running,        // process alive
    Stopping,       // stop requested, waiting for exit
    Exited,         // ended by itself or by Stop, exit code 0
    Crashed,        // ended unexpectedly (non-zero exit / signal)
    FailedToStart,  // could not be launched (toolchain, build error)
};

// State of the IPC session with the HALSim extension inside the robot process.
enum class LinkState {
    Disconnected,
    Connecting,        // process is up, waiting for Hello / handshake
    Connected,
    HandshakeTimeout,  // the extension never said Hello
    ProtocolMismatch,  // Hello refused (version)
    Stalled,           // lock-step watchdog expired (robot not answering)
};

enum class LogStream { Stdout, Stderr, Emulator };

// Starting, Running or Stopping: a robot process exists and has not ended.
constexpr bool isActive(ProcessState process) {
    return process == ProcessState::Starting || process == ProcessState::Running || process == ProcessState::Stopping;
}

// Robot alive and connected, simulation idle: the state in which devices are detected.
constexpr bool isArmed(ProcessState process, LinkState link, bool simulating) {
    return process == ProcessState::Running && link != LinkState::Disconnected && !simulating;
}

struct SessionStatus {
    SessionMode mode = SessionMode::Run;
    ProcessState process = ProcessState::Idle;
    LinkState link = LinkState::Disconnected;
    int exitCode = 0;
    std::string detail;            // human readable reason / progress
    bool programStarted = false;   // the robot constructor finished (devices announced, lock-step may start)
    bool simulating = false;       // lock-step is running (always true in Run mode once connected; false while armed)

    bool active() const { return isActive(process); }
    bool armed() const { return isArmed(process, link, simulating); }
    bool operator==(const SessionStatus&) const = default;
};

struct SessionLogEntry {
    std::int64_t timestampMs = 0;   // wall clock, ms since the epoch
    LogLevel level = LogLevel::Info;
    LogStream stream = LogStream::Stdout;
    std::string source;
    std::string text;
};

}  // namespace wpiemu::sdk
