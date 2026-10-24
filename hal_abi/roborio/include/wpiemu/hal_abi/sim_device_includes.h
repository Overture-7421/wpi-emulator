#pragma once

// SimDevice headers for the roborio ABI, and the SimDevice value-direction constants whose names
// differ between HAL generations.

#include <hal/SimDevice.h>
#include <hal/Types.h>
#include <hal/Value.h>
#include <hal/simulation/SimDeviceData.h>

namespace wpiemu::hal_abi {

inline constexpr auto kSimValueInput = HAL_SimValueInput;
inline constexpr auto kSimValueOutput = HAL_SimValueOutput;
inline constexpr auto kSimValueBidir = HAL_SimValueBidir;

}  // namespace wpiemu::hal_abi
