#pragma once

#include <memory>
#include <string_view>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::bridges::hal {

std::string_view bridgeFamily();

// Tunables that the HAL does not expose. Injected through the registration call (no globals).
struct HalBridgeOptions {
    // Encoder counts per mechanism revolution, used to turn the canonical position (rot) / velocity
    // (rps) signals into HAL encoder counts / period. Counts are in "distancePerPulse" units.
    double encoderCountsPerRevolution = 2048.0;
    // Frequency (Hz) written to a DutyCycle input when it is initialised so DutyCycleEncoder::isConnected() is true.
    int dutyCycleDefaultFrequencyHz = 975;
    // The sim HAL has no per-channel "solenoid initialised" flag, so a solenoid channel is announced the
    // first time the robot commands it ON. Set true to announce every channel of every live module instead.
    bool announceAllSolenoidChannels = false;
};

// Registers one bridge per HAL family into `registry`, in a fixed order:
//   hal.pwm hal.dio hal.analog_in hal.analog_out hal.encoder hal.duty_cycle hal.relay
//   hal.ctre_pcm hal.ctre_pcm.solenoid hal.rev_ph hal.rev_ph.solenoid hal.power_distribution
//   hal.addressable_led hal.roborio hal.accelerometer hal.can
// The factories share ownership of `api`. Returns InvalidArgument when `api` is null and
// AlreadyExists when a bridge name is already registered (registration then stops, nothing is rolled back).
sdk::Status registerHalBridges(sdk::DeviceBridgeRegistry& registry, std::shared_ptr<IHalApi> api,
                               HalBridgeOptions options = {});

}  // namespace wpiemu::bridges::hal
