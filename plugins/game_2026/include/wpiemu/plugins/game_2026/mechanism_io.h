#pragma once

// Reading other mechanisms (flywheel speed, hood angle, indexer speed, ...) through the SDK
// directory by id and published value name, converting to SI from NamedValue::unit.

#include <optional>
#include <string>
#include <string_view>

#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::plugins::game_2026 {

// Converts `value` expressed in `unit` to SI (rad, rad/s, m, m/s). Unknown or empty units pass through.
double toSi(double value, std::string_view unit);

struct MechanismReading {
    bool found = false;         // the mechanism exists
    bool hasValue = false;      // it publishes the requested named value
    double value = 0.0;         // SI, valid when hasValue
    bool active = false;        // MechanismState::active
};

MechanismReading readMechanism(sdk::IMechanismDirectory* directory, std::string_view id, std::string_view valueName);

// |value| >= threshold when the value exists, otherwise the mechanism's `active` flag.
bool exceeds(const MechanismReading& reading, double threshold);

}  // namespace wpiemu::plugins::game_2026
