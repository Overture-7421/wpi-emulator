#pragma once

// Driver-station state the emulator feeds the robot (DsState), with the station numbering it shares with the game
// plugins and the UI. Plain structs: the codec in ipc/ decides the serialisation. See messages.h for the flow.

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/game/alliance.h"
#include "wpiemu/sdk/ipc/protocol_rules.h"

namespace wpiemu::sdk::proto {

enum class RobotMode : std::uint8_t { Teleop = 0, Autonomous = 1, Test = 2 };
enum class AllianceStation : std::uint8_t { Red1 = 0, Red2, Red3, Blue1, Blue2, Blue3 };

constexpr Alliance allianceOf(AllianceStation s) { return s <= AllianceStation::Red3 ? Alliance::Red : Alliance::Blue; }
// Station `number` (1..3, clamped) of an alliance.
constexpr AllianceStation stationOf(Alliance a, int number) {
    return static_cast<AllianceStation>((a == Alliance::Red ? 0 : 3) + std::clamp(number, 1, 3) - 1);
}
// The 1..3 number within its alliance (inverse of stationOf).
constexpr int stationNumberOf(AllianceStation s) { return static_cast<int>(s) % 3 + 1; }

struct JoystickState {
    bool present = false;
    std::vector<float> axes;             // [-1, 1]
    std::vector<std::int16_t> povs;      // degrees, -1 = released
    std::uint32_t buttons = 0;           // bit i = button i+1
    std::uint8_t buttonCount = 0;
};

struct DsState {
    bool dsAttached = true;
    bool fmsAttached = false;
    bool enabled = false;
    bool eStop = false;
    RobotMode mode = RobotMode::Teleop;
    AllianceStation station = AllianceStation::Red1;
    // Seconds REMAINING in the current period (WPILib's DriverStation.getMatchTime() convention, counting down);
    // -1 = no timed match, consumers use their own integrated clock.
    double matchTimeS = -1.0;
    // FMS "game data" (DriverStation.getGameSpecificMessage()). Empty until the game/FMS has produced it. The runtime
    // overlays IGameState::gameSpecificMessage() on top of whatever the UI requested.
    std::string gameSpecificMessage;
    std::array<JoystickState, 6> joysticks{};
    // identity of the run this state belongs to; a DsState queued before a Reset is dropped after it.
    std::uint64_t sessionId = kUnspecified;
    std::uint64_t runEpoch = kUnspecified;
};

}  // namespace wpiemu::sdk::proto
