#pragma once

// Real HALSIM_* binding for the HAL bridges. Only compiled/linked when CMake option
// WPIEMU_WPILIB_ROOT points at the WPILib HAL headers + wpiHal (target wpiemu::bridges_hal_halsim);
// the `wpiemu_bridges_hal` library itself never needs WPILib.

#include <memory>

#include "wpiemu/bridges/hal/hal_bridges.h"

namespace wpiemu::bridges::hal {

// Binds every family to the process-wide HALSIM data (the robot's wpiHal instance). Stateless: all
// instances see the same HAL. Written against WPILib/GradleRIO 2026.2.1 (ADR 0001).
std::shared_ptr<IHalApi> makeHalSimApi();

// Convenience for the extension's composition root: registerHalBridges(registry, makeHalSimApi(), options).
sdk::Status registerHalBridges(sdk::DeviceBridgeRegistry& registry);

}  // namespace wpiemu::bridges::hal
