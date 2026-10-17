#pragma once

#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/dynamics/plant.h"

namespace wpiemu::sdk {

class SimContext;

// Addresses name modeled output axes, not HAL devices or geometry attachment frames.
struct MotionLink {
    std::string id;
    SlotAddress source;
    SlotAddress target;
    double ratio = 1.0;  // target SI units per source SI unit (rad or m)
};

struct MotionAxisSpec {
    std::string name;
    PlantKind kind = PlantKind::Angular;
    std::vector<std::string> motorSlots;
    std::string motionProperty;   // optional property: "linear" selects metres
    std::string enabledProperty;  // optional boolean property
};

struct MotionAxisInfo {
    SlotAddress address;
    PlantKind kind = PlantKind::Angular;
    std::vector<SlotAddress> motorSlots;
};

// Session-local callbacks. Owners outlive the bindings; all calls are on the sim thread.
struct MotionAxisBinding {
    MotionAxisInfo info;
    std::function<PlantState()> state;
    std::function<void(SimContext&, double, std::optional<PlantState>)> advance;
};

inline PlantState linkedMotion(PlantState source, PlantState sourceStart, PlantState targetStart,
                               double ratio) {
    return {targetStart.position + ratio * (source.position - sourceStart.position),
            ratio * source.velocity};
}

inline PlantState clampLinkedMotion(PlantState state, double minimum, double maximum) {
    if (minimum < maximum) {
        if (state.position <= minimum) {
            state.position = minimum;
            if (state.velocity < 0.0) state.velocity = 0.0;
        }
        if (state.position >= maximum) {
            state.position = maximum;
            if (state.velocity > 0.0) state.velocity = 0.0;
        }
    }
    return state;
}

}  // namespace wpiemu::sdk
