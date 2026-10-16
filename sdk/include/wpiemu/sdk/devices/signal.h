#pragma once

// Signals: the named scalar channels of a device. Announced once per device, then referenced by
// index in per-tick frames (cheap on the wire, no per-tick strings).

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/ids.h"

namespace wpiemu::sdk {

enum class SignalDirection : std::uint8_t {
    Output = 0,         // robot -> sim (we READ it): motor voltage, PWM speed, solenoid state
    Input = 1,          // sim -> robot (we WRITE it): rotor position, DIO value, IMU yaw
    Bidirectional = 2,  // either side may write (e.g. limit switch DIO values)
};

enum class SignalType : std::uint8_t { Double = 0, Bool = 1, Int = 2 };

struct SignalSpec {
    std::string name;  // canonical (devices/signals.h) or vendor-specific
    SignalDirection direction = SignalDirection::Output;
    SignalType type = SignalType::Double;
    std::string unit;  // "V", "A", "rot", "rps", "rad", "m", "" ...  (informational)

    bool operator==(const SignalSpec&) const = default;
};

// One value of one signal. Booleans are 0.0 / 1.0. `signal` indexes the device's announced SignalSpec list.
struct SignalSample {
    DeviceId device;
    std::uint16_t signal = 0;
    double value = 0.0;

    bool operator==(const SignalSample&) const = default;
};

inline const SignalSpec* findSignal(const std::vector<SignalSpec>& signalSpecs, const std::string& name,
                                    std::uint16_t* indexOut = nullptr) {
    for (std::size_t i = 0; i < signalSpecs.size(); ++i) {
        if (signalSpecs[i].name == name) {
            if (indexOut != nullptr) *indexOut = static_cast<std::uint16_t>(i);
            return &signalSpecs[i];
        }
    }
    return nullptr;
}

}  // namespace wpiemu::sdk
