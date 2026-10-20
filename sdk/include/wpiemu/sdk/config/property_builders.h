#pragma once

// Builders for PropertySpec so a schema reads as one line per property (mechanism templates, game plugins, editor
// fixtures). `group` is the inspector section; `description` is optional help text.

#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"

namespace wpiemu::sdk::props {

// General form: every PropertySpec field a one-line schema entry needs. The typed builders below take
// (key, label, group, ...); this one takes the group after the default, like the inspector tables that use it.
inline PropertySpec makeSpec(std::string key, std::string label, PropertyType type, PropertyUnit unit, PropertyValue def,
                             std::string group, std::string description = {}) {
    PropertySpec p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.group = std::move(group);
    p.type = type;
    p.unit = unit;
    p.defaultValue = std::move(def);
    p.description = std::move(description);
    return p;
}

// Sets the numeric range (and UI step, 0 = default) of an Int/Double spec; returns it for chaining.
inline PropertySpec& withRange(PropertySpec& p, double lo, double hi, double step = 0.0) {
    p.min = lo;
    p.max = hi;
    p.step = step;
    return p;
}

inline PropertySpec integer(std::string key, std::string label, std::string group, int def, int lo, int hi,
                            std::string description = {}) {
    PropertySpec p = makeSpec(std::move(key), std::move(label), PropertyType::Int, PropertyUnit::Count, def, std::move(group),
                              std::move(description));
    p.min = lo;
    p.max = hi;
    return p;
}

inline PropertySpec real(std::string key, std::string label, std::string group, double def, double lo, double hi,
                         PropertyUnit unit, std::string description = {}, double step = 0.0) {
    PropertySpec p = makeSpec(std::move(key), std::move(label), PropertyType::Double, unit, def, std::move(group),
                              std::move(description));
    withRange(p, lo, hi, step);
    return p;
}

inline PropertySpec flag(std::string key, std::string label, std::string group, bool def, std::string description = {}) {
    return makeSpec(std::move(key), std::move(label), PropertyType::Bool, PropertyUnit::None, def, std::move(group),
                    std::move(description));
}

inline PropertySpec choice(std::string key, std::string label, std::string group, std::string def,
                           std::vector<EnumOption> options, std::string description = {}) {
    PropertySpec p = makeSpec(std::move(key), std::move(label), PropertyType::Enum, PropertyUnit::None, std::move(def),
                              std::move(group), std::move(description));
    p.options = std::move(options);
    return p;
}

inline PropertySpec text(std::string key, std::string label, std::string group, std::string def,
                         std::string description = {}) {
    return makeSpec(std::move(key), std::move(label), PropertyType::String, PropertyUnit::None, std::move(def),
                    std::move(group), std::move(description));
}

inline PropertySpec vec3(std::string key, std::string label, std::string group, const Vec3& def,
                         std::string description = {}) {
    return makeSpec(std::move(key), std::move(label), PropertyType::Vec3, PropertyUnit::Meters, toValue(def),
                    std::move(group), std::move(description));
}

inline PropertySpec box(std::string key, std::string label, std::string group, const Box3& def,
                        std::string description = {}) {
    return makeSpec(std::move(key), std::move(label), PropertyType::Box3, PropertyUnit::Meters, toValue(def),
                    std::move(group), std::move(description));
}

}  // namespace wpiemu::sdk::props
