#pragma once

// The HAL families whose HALSIM binding differs between HAL generations. Implemented once per ABI, in
// src/abi/<abi>/hal_sim_abi.cpp (the build compiles the directory of the season's ABI, docs/reference/seasons.md); the
// shared binding in hal_sim_binding.cpp composes them and holds no ABI conditionals.

#include <memory>

#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal::abi {

// HALSIM does not range-check its index argument, so every accessor guards it.
inline bool inRange(int index, int count) { return index >= 0 && index < count; }

std::unique_ptr<IPwmHal> makePwm();
std::unique_ptr<IAddressableLedHal> makeAddressableLed();
std::unique_ptr<IRoboRioHal> makeRoboRio();
std::unique_ptr<ICanHal> makeCan();

// Families only some HAL generations have; null when the ABI has no such device.
std::unique_ptr<IAnalogOutHal> makeAnalogOut();
std::unique_ptr<IRelayHal> makeRelay();
std::unique_ptr<IAccelerometerHal> makeAccelerometer();
std::unique_ptr<IImuHal> makeImu();

// Single calls whose HALSIM name or unit differs.
void setEncoderPeriod(int encoder, double periodS);   // an ABI without a period setter derives its rate from the pulse size
int dutyCycleDigitalChannel(int dutyCycle);           // -1 when the ABI does not expose the channel
bool revPhClosedLoopEnabled(int module);

}  // namespace wpiemu::bridges::hal::abi
