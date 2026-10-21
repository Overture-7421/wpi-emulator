#include <algorithm>

#include "model/config_ops.h"
#include "model/config_ops_internal.h"
#include "wpiemu/sdk/config/sensor.h"

namespace wpiemu::app::editor::ops {

using detail::reservedOwner;
using sdk::RobotConfig;

// ---- sensors ----------------------------------------------------------------------------------
std::string uniqueSensorId(const RobotConfig& c, std::string base) {
    return detail::uniqueEntityId(c, std::move(base), "sensor");
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
    s.properties = sdk::defaultsOf(sdk::sensorResponseSchema(kind));
    std::string id = s.id;
    c.sensors.push_back(std::move(s));
    return id;
}

bool removeSensor(RobotConfig& c, std::string_view id) { return sdk::detail::eraseById(c.sensors, id); }

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

sdk::SensorConfig* findSensor(RobotConfig& c, std::string_view id) { return c.findSensor(id); }

}  // namespace wpiemu::app::editor::ops
