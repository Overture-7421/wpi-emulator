#pragma once

// Metadata that lets the editor render inspectors generically: mechanism templates, drivetrain
// options and game-plugin config all describe their parameters with a PropertySchema; the UI
// never needs to know a concrete mechanism or plugin type. Values travel as `PropertyValue`
// (= Value). A property set is a Value OBJECT keyed by PropertySpec::key.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/config/value_codec.h"

namespace wpiemu::sdk {

using PropertyValue = Value;

enum class PropertyType {
    Bool,
    Int,
    Double,
    String,
    Enum,    // value = one of options[].value (string)
    Vec3,    // see value_codec.h
    Pose3,
    Box3,    // editor shows a gizmo-able volume
    Color,   // "#rrggbb"
};

// Display/edit unit hint. Stored values are ALWAYS SI; the UI may show degrees, inches...
enum class PropertyUnit {
    None, Meters, MetersPerSecond, Radians, RadiansPerSecond, Kilograms, Seconds, Volts, Amps,
    Newtons, NewtonMeters, KgM2, Ratio, Percent, Count, Hertz,
};

struct EnumOption {
    std::string value;
    std::string label;
};

struct PropertySpec {
    std::string key;                 // unique within the schema; stable (persisted)
    std::string label;
    std::string description;
    PropertyType type = PropertyType::Double;
    PropertyUnit unit = PropertyUnit::None;
    PropertyValue defaultValue;
    std::optional<double> min;       // Int/Double
    std::optional<double> max;
    double step = 0.0;               // 0 = UI default
    std::vector<EnumOption> options; // Enum
    std::string group;               // inspector section ("Geometry", "Motor", ...)
    bool advanced = false;           // collapsed by default
    bool required = false;
    // Conditional visibility: shown only when property `visibleWhenKey` equals `visibleWhenEquals`.
    std::string visibleWhenKey;
    PropertyValue visibleWhenEquals;
};

struct PropertySchema {
    std::vector<PropertySpec> properties;

    const PropertySpec* find(std::string_view key) const {
        for (const auto& p : properties)
            if (p.key == key) return &p;
        return nullptr;
    }
};

// Object containing every property's default value.
inline Value defaultsOf(const PropertySchema& schema) {
    Value o = Value::makeObject();
    for (const auto& p : schema.properties) o[p.key] = p.defaultValue;
    return o;
}

// Fills in missing keys from the schema defaults (does not touch present keys; unknown keys stay).
inline void applyDefaults(const PropertySchema& schema, Value& values) {
    if (!values.isObject()) values = Value::makeObject();
    for (const auto& p : schema.properties)
        if (values.find(p.key) == nullptr) values[p.key] = p.defaultValue;
}

// Checks types, ranges, enum membership, required keys. Unknown keys produce a Warning.
inline std::vector<Issue> validateProperties(const PropertySchema& schema, const Value& values,
                                             const std::string& pathPrefix = {}) {
    std::vector<Issue> out;
    auto path = [&](const std::string& k) { return pathPrefix.empty() ? k : pathPrefix + "." + k; };
    if (!values.isObject()) {
        out.push_back({Severity::Error, pathPrefix, "properties must be an object", {}});
        return out;
    }
    for (const auto& p : schema.properties) {
        const Value* v = values.find(p.key);
        if (v == nullptr) {
            if (p.required) out.push_back({Severity::Error, path(p.key), "missing required property", {}});
            continue;
        }
        bool typeOk = true;
        switch (p.type) {
            case PropertyType::Bool: typeOk = v->isBool(); break;
            case PropertyType::Int: typeOk = v->isInt(); break;
            case PropertyType::Double: typeOk = v->isNumber(); break;
            case PropertyType::String:
            case PropertyType::Color: typeOk = v->isString(); break;
            case PropertyType::Enum: {
                typeOk = v->isString() &&
                         std::any_of(p.options.begin(), p.options.end(),
                                     [&](const EnumOption& o) { return o.value == v->asString(); });
                break;
            }
            case PropertyType::Vec3: { Vec3 t; typeOk = fromValue(*v, t); break; }
            case PropertyType::Pose3: { Pose3 t; typeOk = fromValue(*v, t); break; }
            case PropertyType::Box3: { Box3 t; typeOk = fromValue(*v, t); break; }
        }
        if (!typeOk) {
            out.push_back({Severity::Error, path(p.key), "wrong type or invalid value for '" + p.key + "'", {}});
            continue;
        }
        if ((p.type == PropertyType::Int || p.type == PropertyType::Double)) {
            const double d = v->asDouble();
            if (std::isnan(d) || (p.min && d < *p.min) || (p.max && d > *p.max))
                out.push_back({Severity::Error, path(p.key), "'" + p.key + "' out of range", {}});
        }
    }
    for (std::size_t i = 0; i < values.object().size(); ++i)
        if (schema.find(values.object().keyAt(i)) == nullptr)
            out.push_back({Severity::Warning, path(values.object().keyAt(i)), "unknown property", {}});
    return out;
}

}  // namespace wpiemu::sdk
