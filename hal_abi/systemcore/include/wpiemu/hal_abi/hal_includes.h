#pragma once

// HAL ABI "systemcore" (WPILib 2027): the headers the robot-process modules share, under the wpi/hal include roots.
// The roborio profile provides the same file with its own roots; code includes this header, never a WPILib path.
// Per-ABI behavior lives in abi/systemcore/ sources next to the module that owns it (docs/reference/seasons.md).

#include <wpi/hal/Extensions.h>
#include <wpi/hal/HAL.h>
#include <wpi/hal/Ports.h>
#include <wpi/hal/REVPH.h>
#include <wpi/hal/simulation/AnalogInData.h>
#include <wpi/hal/simulation/CTREPCMData.h>
#include <wpi/hal/simulation/DIOData.h>
#include <wpi/hal/simulation/DriverStationData.h>
#include <wpi/hal/simulation/DutyCycleData.h>
#include <wpi/hal/simulation/EncoderData.h>
#include <wpi/hal/simulation/MockHooks.h>
#include <wpi/hal/simulation/NotifierData.h>
#include <wpi/hal/simulation/PowerDistributionData.h>
#include <wpi/hal/simulation/REVPHData.h>
#include <wpi/hal/simulation/RoboRioData.h>

namespace wpiemu::hal_abi {

// Initializes the simulation HAL in-process (tests that drive the real HAL without a robot).
inline bool initializeSimHal() { return HAL_Initialize() != 0; }

}  // namespace wpiemu::hal_abi
