#pragma once

// HAL ABI "roborio" (WPILib 2026): the headers the robot-process modules share, under the roboRIO-era include roots.
// The systemcore profile provides the same file with its own roots; code includes this header, never a WPILib path.
// Per-ABI behavior lives in abi/roborio/ sources next to the module that owns it (docs/reference/seasons.md).

#include <hal/Extensions.h>
#include <hal/HAL.h>
#include <hal/Ports.h>
#include <hal/REVPH.h>
#include <hal/simulation/AnalogInData.h>
#include <hal/simulation/CTREPCMData.h>
#include <hal/simulation/DIOData.h>
#include <hal/simulation/DriverStationData.h>
#include <hal/simulation/DutyCycleData.h>
#include <hal/simulation/EncoderData.h>
#include <hal/simulation/MockHooks.h>
#include <hal/simulation/NotifierData.h>
#include <hal/simulation/PowerDistributionData.h>
#include <hal/simulation/REVPHData.h>
#include <hal/simulation/RoboRioData.h>

namespace wpiemu::hal_abi {

// Initializes the simulation HAL in-process (tests that drive the real HAL without a robot).
inline bool initializeSimHal() { return HAL_Initialize(500, 0) != 0; }

}  // namespace wpiemu::hal_abi
