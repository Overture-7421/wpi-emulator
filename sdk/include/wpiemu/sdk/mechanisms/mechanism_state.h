#pragma once

#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

struct NamedValue {
    std::string name;   // "position", "velocity", "current", "fuelCount"...
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
