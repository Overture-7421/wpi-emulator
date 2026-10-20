#pragma once

// Sensors: generic, game-fed simulated sensors persisted in RobotConfig::sensors (ADR 0019).
//
// A sensor has no pose: it turns one value published by the selected game plugin (IGameModule::contributeGameValues,
// SensorConfig::source) into a device reading (a CANrange distance, a beam break) through its single device slot
// ("<sensor id>.sensor"). The kind picks the slot type and the response schema; the response says how the value
// maps to the reading. Stored in SI units. There is no migration (ADR 0013).

#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/devices/slot.h"

namespace wpiemu::sdk {

enum class SensorKind {
    Distance,   // CANrange-style: reads metres
    BeamBreak,  // digital input
};

inline constexpr const char* kSensorSlot = "sensor";

struct SensorKindInfo {
    SensorKind kind;
    const char* key;          // persisted
    const char* displayName;
    const char* description;
    SlotType slotType;
};

inline const std::array<SensorKindInfo, 2>& sensorKinds() {
    static const std::array<SensorKindInfo, 2> kinds{{
        {SensorKind::Distance, "distance", "Distance sensor", "Reads a distance in metres (CANrange).", SlotType::DistanceSensor},
        {SensorKind::BeamBreak, "beam_break", "Beam break", "Reads a digital detected / not detected state.", SlotType::BeamBreak},
    }};
    return kinds;
}

inline const SensorKindInfo& sensorKindInfo(SensorKind kind) {
    for (const auto& k : sensorKinds())
        if (k.kind == kind) return k;
    return sensorKinds()[0];
}

inline const char* sensorKindKey(SensorKind kind) { return sensorKindInfo(kind).key; }

inline bool sensorKindFromKey(std::string_view key, SensorKind& out) {
    for (const auto& k : sensorKinds())
        if (key == k.key) {
            out = k.kind;
            return true;
        }
    return false;
}

namespace sensor_prop {
inline constexpr const char* kMode = "mode";  // distance: "threshold" | "linear"
inline constexpr const char* kThreshold = "threshold";
inline constexpr const char* kNearM = "nearM";
inline constexpr const char* kFarM = "farM";
inline constexpr const char* kInputMin = "inputMin";
inline constexpr const char* kInputMax = "inputMax";
inline constexpr const char* kDistanceAtMinM = "distanceAtMinM";
inline constexpr const char* kDistanceAtMaxM = "distanceAtMaxM";
inline constexpr const char* kBrokenValue = "brokenValue";  // beam break: reads true while detected
inline constexpr const char* kModeThreshold = "threshold";
inline constexpr const char* kModeLinear = "linear";
}  // namespace sensor_prop

// How the source value maps to the reading. Units of thresholds / inputs are the source's unit.
inline PropertySchema sensorResponseSchema(SensorKind kind) {
    PropertySchema s;
    auto num = [](const char* key, const char* label, PropertyUnit unit, double def, const char* desc) {
        PropertySpec p;
        p.key = key;
        p.label = label;
        p.description = desc;
        p.type = PropertyType::Double;
        p.unit = unit;
        p.defaultValue = def;
        p.step = 0.0;
        return p;
    };
    if (kind == SensorKind::Distance) {
        PropertySpec mode;
        mode.key = sensor_prop::kMode;
        mode.label = "Mode";
        mode.description = "Threshold: near / far distance. Linear: interpolate between two input values.";
        mode.type = PropertyType::Enum;
        mode.options = {{sensor_prop::kModeThreshold, "Threshold"}, {sensor_prop::kModeLinear, "Linear"}};
        mode.defaultValue = std::string(sensor_prop::kModeThreshold);
        s.properties.push_back(mode);
        auto gated = [&](PropertySpec p, const char* mode) {
            p.visibleWhenKey = sensor_prop::kMode;
            p.visibleWhenEquals = std::string(mode);
            s.properties.push_back(std::move(p));
        };
        gated(num(sensor_prop::kThreshold, "Detect at or above", PropertyUnit::None, 1.0, "The source value at which something is detected."),
              sensor_prop::kModeThreshold);
        gated(num(sensor_prop::kNearM, "Detected distance", PropertyUnit::Meters, 0.1, "Distance reported while detected."),
              sensor_prop::kModeThreshold);
        gated(num(sensor_prop::kFarM, "Idle distance", PropertyUnit::Meters, 1.0, "Distance reported while nothing is detected."),
              sensor_prop::kModeThreshold);
        gated(num(sensor_prop::kInputMin, "Input at start", PropertyUnit::None, 0.0, "Source value mapped to the first distance."),
              sensor_prop::kModeLinear);
        gated(num(sensor_prop::kInputMax, "Input at end", PropertyUnit::None, 1.0, "Source value mapped to the second distance."),
              sensor_prop::kModeLinear);
        gated(num(sensor_prop::kDistanceAtMinM, "Distance at start", PropertyUnit::Meters, 1.0, "Distance reported at the first input."),
              sensor_prop::kModeLinear);
        gated(num(sensor_prop::kDistanceAtMaxM, "Distance at end", PropertyUnit::Meters, 0.1, "Distance reported at the second input."),
              sensor_prop::kModeLinear);
    } else {
        s.properties.push_back(
            num(sensor_prop::kThreshold, "Detect at or above", PropertyUnit::None, 1.0, "The source value at which the beam is broken."));
        PropertySpec broken;
        broken.key = sensor_prop::kBrokenValue;
        broken.label = "Reads true when broken";
        broken.description = "Off for active-low wiring.";
        broken.type = PropertyType::Bool;
        broken.defaultValue = true;
        s.properties.push_back(broken);
    }
    return s;
}

inline SlotSpec sensorSlotSpec(SensorKind kind) {
    const SensorKindInfo& info = sensorKindInfo(kind);
    return {kSensorSlot, info.displayName, info.slotType, false, info.description};
}

struct SensorConfig {
    std::string id;       // unique among sensors, mechanisms, zones and cameras; no '.'
    std::string label;
    SensorKind kind = SensorKind::Distance;
    std::string source;   // a game value id published by the selected plugin; empty = none
    Value properties = Value::makeObject();   // validated against sensorResponseSchema(kind)
    std::vector<SlotMapping> mappings;

    bool operator==(const SensorConfig&) const = default;
};

}  // namespace wpiemu::sdk
