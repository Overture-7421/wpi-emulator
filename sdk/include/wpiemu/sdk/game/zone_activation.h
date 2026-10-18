#pragma once

// Read-only evaluation of ZoneActivation (config/zone.h) against the live mechanisms. Header-only,
// stateless, no I/O: call it on the sim thread each tick (or from the UI on a snapshot-consistent directory).

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::sdk {

// Converts `value` expressed in `unit` (NamedValue::unit) to SI (rad, rad/s, m, m/s).
// Unknown or empty units pass through unchanged.
inline double namedValueToSi(double value, std::string_view unit) {
    if (unit == "deg" || unit == "deg/s") return degToRad(value);
    if (unit == "rpm") return rpmToRadPerSec(value);
    if (unit == "rot" || unit == "rev" || unit == "rot/s" || unit == "rps" || unit == "rev/s") return rotToRad(value);
    if (unit == "mm") return value * 1e-3;
    if (unit == "cm") return value * 1e-2;
    if (unit == "in") return inchesToMeters(value);
    return value;
}

struct MechanismReading {
    bool found = false;      // the mechanism exists in the directory
    bool hasValue = false;   // it publishes the requested named value
    bool active = false;     // MechanismState::active
    double value = 0.0;      // SI; valid when hasValue
};

// State of mechanism `mechanismId` and its published `valueName`, converted to SI. A null directory or an
// unknown mechanism yields found == false.
inline MechanismReading readMechanismValue(IMechanismDirectory* directory, const std::string& mechanismId,
                                           const std::string& valueName) {
    MechanismReading r;
    if (directory == nullptr || mechanismId.empty()) return r;
    const IMechanism* mechanism = directory->find(mechanismId);
    if (mechanism == nullptr) return r;
    r.found = true;
    const MechanismState state = mechanism->state();
    r.active = state.active;
    for (const NamedValue& v : state.values)
        if (v.name == valueName) {
            r.hasValue = true;
            r.value = namedValueToSi(v.value, v.unit);
            break;
        }
    return r;
}

// One condition. Unknown mechanism -> false (its id is appended to `missingMechanisms` once, when given).
// A mechanism that does not publish the value falls back to its `active` flag: Above and Within hold when
// it is active, Below when it is not.
inline bool evaluateCondition(const ZoneCondition& condition, IMechanismDirectory* directory,
                              std::vector<std::string>* missingMechanisms = nullptr) {
    const MechanismReading r = readMechanismValue(directory, condition.mechanismId, condition.value);
    if (!r.found) {
        if (missingMechanisms != nullptr &&
            std::find(missingMechanisms->begin(), missingMechanisms->end(), condition.mechanismId) == missingMechanisms->end())
            missingMechanisms->push_back(condition.mechanismId);
        return false;
    }
    if (!r.hasValue) return condition.op == ConditionOp::Below ? !r.active : r.active;
    switch (condition.op) {
        case ConditionOp::Above: return (condition.absolute ? std::fabs(r.value) : r.value) > condition.threshold;
        case ConditionOp::Below: return (condition.absolute ? std::fabs(r.value) : r.value) < condition.threshold;
        case ConditionOp::Within: return std::fabs(r.value - condition.threshold) <= condition.tolerance;
    }
    return false;
}

// All conditions (AND) or any (OR); an empty list is true. Every condition is evaluated, so
// `missingMechanisms` lists all unknown mechanism ids regardless of the outcome.
inline bool evaluateActivation(const ZoneActivation& activation, IMechanismDirectory* directory,
                               std::vector<std::string>* missingMechanisms = nullptr) {
    if (activation.conditions.empty()) return true;
    bool all = true;
    bool any = false;
    for (const ZoneCondition& c : activation.conditions) {
        const bool held = evaluateCondition(c, directory, missingMechanisms);
        all = all && held;
        any = any || held;
    }
    return activation.mode == ActivationMode::All ? all : any;
}

}  // namespace wpiemu::sdk
