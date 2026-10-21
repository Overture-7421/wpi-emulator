#include <algorithm>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"
#include "wpiemu/sdk/config/sensor.h"

namespace wpiemu::app::editor::ops {

using detail::reservedOwner;
using sdk::RobotConfig;

// ---- sensors ----------------------------------------------------------------------------------
std::string uniqueSensorId(const RobotConfig& c, std::string base) {
    if (base.empty()) base = "sensor";
    auto taken = [&](const std::string& id) {
        return reservedOwner(id) || c.findSensor(id) != nullptr || c.findMechanism(id) != nullptr || c.findZone(id) != nullptr ||
               c.findCamera(id) != nullptr;
    };
    if (!taken(base)) return base;
    for (int n = 2;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) return candidate;
    }
}

std::string sensorDisplayName(const sdk::SensorConfig& s) {
    if (!s.label.empty()) return s.label;
    return sdk::sensorKindInfo(s.kind).displayName + std::string(" (") + s.id + ")";
}

std::string addSensor(RobotConfig& c, sdk::SensorKind kind) {
    sdk::SensorConfig s;
    s.kind = kind;
    s.id = uniqueSensorId(c, sdk::sensorKindKey(kind));
    s.label = sdk::sensorKindInfo(kind).displayName;
    s.properties = sdk::Value::makeObject();
    for (const auto& p : sdk::sensorResponseSchema(kind).properties) s.properties[p.key] = p.defaultValue;
    std::string id = s.id;
    c.sensors.push_back(std::move(s));
    return id;
}

bool removeSensor(RobotConfig& c, std::string_view id) {
    const auto it = std::find_if(c.sensors.begin(), c.sensors.end(), [&](const sdk::SensorConfig& s) { return s.id == id; });
    if (it == c.sensors.end()) return false;
    c.sensors.erase(it);
    return true;
}

std::string duplicateSensor(RobotConfig& c, std::string_view id) {
    const sdk::SensorConfig* original = c.findSensor(id);
    if (original == nullptr) return {};
    sdk::SensorConfig copy = *original;
    copy.id = uniqueSensorId(c, copy.id);
    copy.label = (copy.label.empty() ? original->id : copy.label) + " (copy)";
    copy.mappings.clear();   // a device drives one thing
    std::string newId = copy.id;
    c.sensors.push_back(std::move(copy));
    return newId;
}

sdk::SensorConfig* findSensor(RobotConfig& c, std::string_view id) {
    for (auto& s : c.sensors)
        if (s.id == id) return &s;
    return nullptr;
}

}  // namespace wpiemu::app::editor::ops
