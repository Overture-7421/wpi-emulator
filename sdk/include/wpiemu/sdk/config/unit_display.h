#pragma once

// How a PropertyUnit is shown to the user. Persisted values are ALWAYS SI; this maps a PropertyUnit to the shown
// unit (scale, suffix, precision, spin-box step) for the chosen unit system. The wizard, the editor and the
// settings share this one table, so their conversions cannot drift apart.

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

enum class UnitSystem { Imperial, Metric };

// SI value -> shown value. Every conversion is linear.
inline double toDisplay(double si, PropertyUnit unit, UnitSystem system) {
    const bool imperial = system == UnitSystem::Imperial;
    switch (unit) {
        case PropertyUnit::Meters: return imperial ? metersToInches(si) : si * 100.0;
        case PropertyUnit::Kilograms: return imperial ? kgToPounds(si) : si;
        case PropertyUnit::Radians: return radToDeg(si);
        case PropertyUnit::RadiansPerSecond: return radPerSecToRpm(si);
        default: return si;
    }
}

// Shown value -> SI value.
inline double fromDisplay(double shown, PropertyUnit unit, UnitSystem system) {
    const bool imperial = system == UnitSystem::Imperial;
    switch (unit) {
        case PropertyUnit::Meters: return imperial ? inchesToMeters(shown) : shown / 100.0;
        case PropertyUnit::Kilograms: return imperial ? poundsToKg(shown) : shown;
        case PropertyUnit::Radians: return degToRad(shown);
        case PropertyUnit::RadiansPerSecond: return rpmToRadPerSec(shown);
        default: return shown;
    }
}

struct UnitDisplay {
    double factor = 1.0;      // shown = si * factor
    const char* suffix = "";  // UTF-8 literal, shown after the number
    int decimals = 3;
    double step = 0.1;        // spin box single step in shown units
};

inline UnitDisplay unitDisplay(PropertyUnit unit, UnitSystem system) {
    using U = PropertyUnit;
    const bool imperial = system == UnitSystem::Imperial;
    const double factor = toDisplay(1.0, unit, system);
    switch (unit) {
        case U::Meters: return imperial ? UnitDisplay{factor, " in", 2, 0.5} : UnitDisplay{factor, " cm", 1, 1.0};
        case U::Kilograms: return imperial ? UnitDisplay{factor, " lb", 2, 0.5} : UnitDisplay{factor, " kg", 2, 0.25};
        case U::MetersPerSecond: return {factor, " m/s", 2, 0.1};
        case U::Radians: return {factor, " \xC2\xB0", 1, 1.0};
        case U::RadiansPerSecond: return {factor, " rpm", 0, 50.0};
        case U::Seconds: return {factor, " s", 3, 0.05};
        case U::Volts: return {factor, " V", 2, 0.5};
        case U::Amps: return {factor, " A", 1, 5.0};
        case U::Newtons: return {factor, " N", 1, 1.0};
        case U::NewtonMeters: return {factor, " N\xC2\xB7m", 3, 0.05};
        case U::KgM2: return {factor, " kg\xC2\xB7m\xC2\xB2", 5, 0.001};
        case U::Percent: return {factor, " %", 1, 1.0};
        case U::Hertz: return {factor, " Hz", 1, 1.0};
        case U::Ratio: return {factor, "", 3, 0.1};
        case U::Count: return {factor, "", 0, 1.0};
        case U::None: break;
    }
    return {factor, "", 3, 0.1};
}

}  // namespace wpiemu::sdk
