#pragma once

// Small builders for PropertySpec so schemas read as a table.

#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"

namespace wpiemu::plugins::game_2026::props {

inline sdk::PropertySpec makeSpec(std::string key, std::string label, std::string group, sdk::PropertyType type,
                                  sdk::PropertyUnit unit, sdk::PropertyValue def, std::string description) {
    sdk::PropertySpec p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.group = std::move(group);
    p.type = type;
    p.unit = unit;
    p.defaultValue = std::move(def);
    p.description = std::move(description);
    return p;
}

inline sdk::PropertySpec integer(std::string key, std::string label, std::string group, int def, int lo, int hi,
                                 std::string description) {
    sdk::PropertySpec p = makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Int,
                                   sdk::PropertyUnit::Count, def, std::move(description));
    p.min = lo;
    p.max = hi;
    return p;
}

inline sdk::PropertySpec real(std::string key, std::string label, std::string group, double def, double lo, double hi,
                              sdk::PropertyUnit unit, std::string description, bool advanced = false) {
    sdk::PropertySpec p = makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Double, unit,
                                   def, std::move(description));
    p.min = lo;
    p.max = hi;
    p.advanced = advanced;
    return p;
}

inline sdk::PropertySpec flag(std::string key, std::string label, std::string group, bool def, std::string description,
                              bool advanced = false) {
    sdk::PropertySpec p = makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Bool,
                                   sdk::PropertyUnit::None, def, std::move(description));
    p.advanced = advanced;
    return p;
}

inline sdk::PropertySpec choice(std::string key, std::string label, std::string group, std::string def,
                                std::vector<sdk::EnumOption> options, std::string description) {
    sdk::PropertySpec p = makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Enum,
                                   sdk::PropertyUnit::None, std::move(def), std::move(description));
    p.options = std::move(options);
    return p;
}

inline sdk::PropertySpec text(std::string key, std::string label, std::string group, std::string def,
                              std::string description) {
    return makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::String,
                    sdk::PropertyUnit::None, std::move(def), std::move(description));
}

inline sdk::PropertySpec vec3(std::string key, std::string label, std::string group, const sdk::Vec3& def,
                              std::string description) {
    return makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Vec3,
                    sdk::PropertyUnit::Meters, sdk::toValue(def), std::move(description));
}

inline sdk::PropertySpec box(std::string key, std::string label, std::string group, const sdk::Box3& def,
                             std::string description) {
    return makeSpec(std::move(key), std::move(label), std::move(group), sdk::PropertyType::Box3,
                    sdk::PropertyUnit::Meters, sdk::toValue(def), std::move(description));
}

}  // namespace wpiemu::plugins::game_2026::props
