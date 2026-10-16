#pragma once

// Display units for the inspector. Persisted values are ALWAYS SI; this only maps PropertyUnit to a
// shown unit (suffix, scale, step) for the chosen unit system. Pure and Qt-free apart from QString.

#include <QString>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

enum class UnitSystem { Imperial, Metric };

struct UnitDisplay {
    double factor = 1.0;      // shown = si * factor
    const char* suffix = "";  // ASCII/UTF-8 literal, shown after the number
    int decimals = 3;
    double step = 0.1;        // spin box single step in shown units
};

inline UnitDisplay unitDisplay(sdk::PropertyUnit unit, UnitSystem system) {
    using U = sdk::PropertyUnit;
    const bool imperial = system == UnitSystem::Imperial;
    switch (unit) {
        case U::Meters: return imperial ? UnitDisplay{39.37007874, " in", 2, 0.5} : UnitDisplay{100.0, " cm", 1, 1.0};
        case U::Kilograms: return imperial ? UnitDisplay{2.20462262, " lb", 2, 0.5} : UnitDisplay{1.0, " kg", 2, 0.25};
        case U::MetersPerSecond: return {1.0, " m/s", 2, 0.1};
        case U::Radians: return {57.29577951308232, " \xC2\xB0", 1, 1.0};
        case U::RadiansPerSecond: return {9.549296585513721, " rpm", 0, 50.0};
        case U::Seconds: return {1.0, " s", 3, 0.05};
        case U::Volts: return {1.0, " V", 2, 0.5};
        case U::Amps: return {1.0, " A", 1, 5.0};
        case U::Newtons: return {1.0, " N", 1, 1.0};
        case U::NewtonMeters: return {1.0, " N\xC2\xB7m", 3, 0.05};
        case U::KgM2: return {1.0, " kg\xC2\xB7m\xC2\xB2", 5, 0.001};
        case U::Percent: return {1.0, " %", 1, 1.0};
        case U::Hertz: return {1.0, " Hz", 1, 1.0};
        case U::Ratio: return {1.0, "", 3, 0.1};
        case U::Count: return {1.0, "", 0, 1.0};
        case U::None: break;
    }
    return {1.0, "", 3, 0.1};
}

inline double toDisplay(double si, sdk::PropertyUnit unit, UnitSystem system) { return si * unitDisplay(unit, system).factor; }
inline double fromDisplay(double shown, sdk::PropertyUnit unit, UnitSystem system) { return shown / unitDisplay(unit, system).factor; }

inline QString unitSuffix(sdk::PropertyUnit unit, UnitSystem system) { return QString::fromUtf8(unitDisplay(unit, system).suffix); }

}  // namespace wpiemu::app::editor
