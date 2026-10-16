#include "wpiemu/plugins/game_2026/mechanism_io.h"

#include <cmath>

namespace wpiemu::plugins::game_2026 {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double toSi(double value, std::string_view unit) {
    if (unit == "deg") return value * kPi / 180.0;
    if (unit == "deg/s") return value * kPi / 180.0;
    if (unit == "rpm") return value * 2.0 * kPi / 60.0;
    if (unit == "rot" || unit == "rev") return value * 2.0 * kPi;
    if (unit == "rot/s" || unit == "rps" || unit == "rev/s") return value * 2.0 * kPi;
    if (unit == "mm") return value * 1e-3;
    if (unit == "cm") return value * 1e-2;
    if (unit == "in") return value * 0.0254;
    return value;
}

MechanismReading readMechanism(sdk::IMechanismDirectory* directory, std::string_view id, std::string_view valueName) {
    MechanismReading r;
    if (directory == nullptr || id.empty()) return r;
    const sdk::IMechanism* m = directory->find(id);
    if (m == nullptr) return r;
    r.found = true;
    const sdk::MechanismState state = m->state();
    r.active = state.active;
    for (const sdk::NamedValue& v : state.values)
        if (v.name == valueName) {
            r.hasValue = true;
            r.value = toSi(v.value, v.unit);
            break;
        }
    return r;
}

bool exceeds(const MechanismReading& reading, double threshold) {
    if (!reading.found) return false;
    return reading.hasValue ? std::fabs(reading.value) >= threshold : reading.active;
}

}  // namespace wpiemu::plugins::game_2026
