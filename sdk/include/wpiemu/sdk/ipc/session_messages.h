#pragma once

// Connection messages: the handshake (Hello/HelloAck), device discovery (DeviceAnnounce/LoopbackSet) and
// diagnostics (Log/Goodbye). Plain structs: the codec in ipc/ decides the serialisation. See messages.h for the flow.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_record.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"

namespace wpiemu::sdk::proto {

enum class RobotLanguage : std::uint8_t { Unknown = 0, Java = 1, Cpp = 2 };
enum class ClockMode : std::uint8_t {
    Lockstep = 0,   // emulator steps HALSIM time; robot loop runs exactly once per its period (default)
    Realtime = 1,   // HALSIM time resumes; frames are best-effort
};
// What the extension does by itself when the watchdog expires or the transport drops.
enum class WatchdogPolicy : std::uint8_t {
    ResumeRealtime = 0,   // HALSIM_ResumeTiming: the robot program keeps running in real time (default)
    Terminate = 1,        // the extension ends the robot process cleanly
};

// ---- handshake --------------------------------------------------------------------------------
struct Hello {
    std::uint32_t protocolVersion = kProtocolVersion;
    RobotLanguage language = RobotLanguage::Unknown;
    std::int32_t wpilibYear = 0;         // e.g. 2026 (from the HAL/WPILib version the robot runs)
    std::string wpilibVersion;           // free-form, e.g. "2026.2.1"
    std::string extensionVersion;
    std::int64_t pid = 0;                // robot process id, so the launcher can kill it (docs/reference/extension-ipc.md)
    std::string programName;
    std::vector<std::pair<std::string, std::int32_t>> halPortCounts;  // "pwm" -> 20, ...
    std::uint32_t proposedWatchdogMs = 0;  // extension's minimum acceptable watchdog; 0 = no preference
    bool supportsArmed =
        false;  // Begin/EndLockstep supported; absent => unsupported
};

struct HelloAck {
    bool accepted = true;
    std::string reason;                  // when !accepted
    std::uint32_t protocolVersion = kProtocolVersion;
    ClockMode clockMode = ClockMode::Lockstep;
    std::uint32_t stepUs = static_cast<std::uint32_t>(kDefaultStepUs);   // must divide the robot period (kRobotPeriodUs) evenly
    std::uint64_t sessionId = 0;
    bool discoveryOnly = false;          // armed start: announce only, no physics, until BeginLockstep
    bool enableL3Hooks = false;          // opt-in CTRE hooks (docs/reference/bridges.md)
    // ---- absent in a HelloAck that does not send them => these defaults ----
    std::uint64_t runEpoch = kFirstRunEpoch;  // epoch in force for this connection (monotonic across reconnects)
    std::uint32_t watchdogTimeoutMs = kDefaultWatchdogTimeoutMs;  // negotiateWatchdogTimeoutMs(...)
    WatchdogPolicy watchdogPolicy = WatchdogPolicy::ResumeRealtime;
};

// ---- discovery ----------------------------------------------------------------------------------
struct DeviceAnnounce : DeviceRecord {
    bool retired = false;                // robot freed the device (descriptor still identifies it)
};

// Devices the extension must loop back itself (unmapped: output copied to input, no IPC hop).
// Absence from the set => mapped. Sent whenever mappings change; always the FULL set.
struct LoopbackSet {
    std::vector<DeviceId> loopback;
};

// ---- misc -----------------------------------------------------------------------------------------
struct Log {
    SimTimeUs simTimeUs = 0;
    LogLevel level = LogLevel::Info;
    std::string source;                  // "extension", "bridge.phoenix6", ...
    std::string message;
};

struct Goodbye {
    std::string reason;
};

}  // namespace wpiemu::sdk::proto
