#pragma once

#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

// Names of the values a motor-driven mechanism publishes in NamedValue::name. Zone conditions and plugin references
// (a shooter's flywheel/hood/turret value) read them by name.
namespace value_name {
inline constexpr const char* kPosition = "position";   // m | rad
inline constexpr const char* kVelocity = "velocity";   // m/s | rad/s
inline constexpr const char* kVoltage = "voltage";     // V
inline constexpr const char* kCurrent = "current";     // A
inline constexpr const char* kAtLowerLimit = "atLowerLimit";   // 0 | 1
inline constexpr const char* kAtUpperLimit = "atUpperLimit";   // 0 | 1
}  // namespace value_name

struct NamedValue {
    std::string name;   // value_name::*, or a template's own: "fuelCount"...
    double value = 0.0;
    std::string unit;   // "m", "rad", "rad/s", "A", "" ...
};

// A moving part of a mechanism, in the frame of the mechanism's mount (MechanismConfig::mount).
struct LinkPose {
    std::string name;   // "carriage", "arm", "turret", "flywheel"
    Pose3 pose;
};

// What a mechanism publishes for the UI / snapshot each tick.
struct MechanismState {
    std::string id;     // MechanismConfig::id
    std::string type;
    bool active = false;             // e.g. roller spinning above threshold / intake deployed
    std::vector<LinkPose> links;
    std::vector<NamedValue> values;
};

}  // namespace wpiemu::sdk
