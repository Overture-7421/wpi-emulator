#pragma once

// SimDevice headers for the systemcore ABI, and the SimDevice value-direction constants whose names
// differ between HAL generations.

#include <wpi/hal/SimDevice.h>
#include <wpi/hal/Types.h>
#include <wpi/hal/Value.h>
#include <wpi/hal/simulation/SimDeviceData.h>

namespace wpiemu::hal_abi {

inline constexpr auto kSimValueInput = HAL_SIM_VALUE_INPUT;
inline constexpr auto kSimValueOutput = HAL_SIM_VALUE_OUTPUT;
inline constexpr auto kSimValueBidir = HAL_SIM_VALUE_BIDIR;

}  // namespace wpiemu::hal_abi
