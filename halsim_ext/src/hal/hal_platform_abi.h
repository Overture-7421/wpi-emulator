#pragma once

// The HAL calls of the robot-process platform (clock, Driver Station, battery, port counts) whose names, types or units
// differ between HAL generations. Implemented once per ABI in abi/<abi>/hal_platform_abi.cpp; hal_sim_platform.cpp holds
// the shared logic and no ABI conditionals (docs/reference/seasons.md).

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/ipc/driver_station.h"

namespace wpiemu::halsim::abi {

// True while the simulated scheduler has a notifier armed.
bool schedulerArmed();
// Advances HAL time by `deltaUs` microseconds, converting to the HAL's unit; throws std::overflow_error when the step
// does not fit the HAL's range.
void stepTiming(std::uint64_t deltaUs);

void setRobotMode(sdk::proto::RobotMode mode);
void setAllianceStation(sdk::proto::AllianceStation station);
void setGameMessage(const std::string& message);
// How many joysticks the HAL has, and writing one of them (descriptor, axes, POVs and buttons; absent = all zero).
std::size_t maxJoysticks();
void setJoystick(int index, const sdk::proto::JoystickState& state);

// The battery current channel; a no-op where the controller has none.
void setBatteryCurrent(double amps);

// Port counts of the families this HAL generation has beyond the shared ones, in announcement order.
std::vector<std::pair<std::string, std::int32_t>> extraPortCounts();

}  // namespace wpiemu::halsim::abi
