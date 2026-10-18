#pragma once

// Extension lifecycle signals that travel as plain `Log` messages (no new
// MessageType, so every codec carries them unchanged).
//
// Why: the extension only starts stepping HAL time once the robot program called
// HAL_ObserveUserProgramStarting (ADR 0001); until then it holds the first SensorFrame unanswered.
// The emulator needs to know why it is not being acknowledged, so that it neither fires its
// watchdog nor starts physics before the robot exists. `kLifecycleProgramStarted` is that signal:
// every device the robot constructs in its constructor has been announced BEFORE it is sent.

#include <string>
#include <string_view>

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk::proto {

inline constexpr std::string_view kLifecycleLogSource = "extension.lifecycle";
inline constexpr std::string_view kLifecycleProgramStarted = "program_started";

inline Log makeLifecycleLog(std::string_view event, SimTimeUs simTimeUs = 0) {
    Log log;
    log.simTimeUs = simTimeUs;
    log.level = LogLevel::Info;
    log.source = std::string(kLifecycleLogSource);
    log.message = std::string(event);
    return log;
}

inline bool isLifecycleLog(const Log& log, std::string_view event) {
    return log.source == kLifecycleLogSource && log.message == event;
}

}  // namespace wpiemu::sdk::proto
