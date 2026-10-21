#pragma once

// A plugin config section declared once: every field is a PropertySpec (key, default, range, group) bound to the
// member of the typed config that stores it. The same table yields the schema, decodes a section Value into the typed
// config and encodes it back, so a key is spelled exactly once. Worked example: plugins/game_2026/src/config.cpp.

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

// `Config` must be default-constructible: its member initializers are the values a missing or invalid key leaves.
template <class Config>
class ConfigBindings {
public:
    // Bool, Int and Double specs bound to a member (or a member of a member). Decoding keeps the field when the key
    // holds another type or NaN, and clamps numbers to the spec's min/max (an Int field reads the number truncated).
    template <class Field>
    ConfigBindings& add(PropertySpec spec, Field Config::* member) {
        static_assert(std::is_arithmetic_v<Field>, "bool, integer and floating-point fields only; use choice()");
        return bind<Field>(std::move(spec), [member](auto& c) -> auto& { return c.*member; });
    }
    template <class Outer, class Field>
    ConfigBindings& add(PropertySpec spec, Outer Config::* outer, Field Outer::* inner) {
        static_assert(std::is_arithmetic_v<Field>, "bool, integer and floating-point fields only; use choice()");
        return bind<Field>(std::move(spec), [outer, inner](auto& c) -> auto& { return (c.*outer).*inner; });
    }

    // Enum specs: values[i] is the field value of spec.options[i]. Decoding keeps the field for any other string.
    template <class Field>
    ConfigBindings& choice(PropertySpec spec, Field Config::* member, std::vector<Field> values) {
        return push(
            std::move(spec),
            [member, values](Config& c, const Value& v, const PropertySpec& s) {
                if (!v.isString()) return;
                const std::string name = v.asString();
                for (std::size_t i = 0; i < values.size() && i < s.options.size(); ++i)
                    if (s.options[i].value == name) c.*member = values[i];
            },
            [member, values](const Config& c, const PropertySpec& s) -> Value {
                for (std::size_t i = 0; i < values.size() && i < s.options.size(); ++i)
                    if (c.*member == values[i]) return s.options[i].value;
                return s.defaultValue;
            });
    }

    PropertySchema schema() const {
        PropertySchema s;
        for (const Entry& e : entries_) s.properties.push_back(e.spec);
        return s;
    }

    // A default Config with every present, valid key applied; unknown keys are ignored (validation reports them).
    Config decode(const Value& data) const {
        Config config;
        for (const Entry& e : entries_)
            if (const Value* v = data.find(e.spec.key)) e.decode(config, *v, e.spec);
        return config;
    }

    // Every key, in declaration order.
    Value encode(const Config& config) const {
        Value out = Value::makeObject();
        for (const Entry& e : entries_) out[e.spec.key] = e.encode(config, e.spec);
        return out;
    }

private:
    using Decode = std::function<void(Config&, const Value&, const PropertySpec&)>;
    using Encode = std::function<Value(const Config&, const PropertySpec&)>;

    struct Entry {
        PropertySpec spec;
        Decode decode;
        Encode encode;
    };

    template <class Field, class Access>
    ConfigBindings& bind(PropertySpec spec, Access access) {
        return push(
            std::move(spec),
            [access](Config& c, const Value& v, [[maybe_unused]] const PropertySpec& s) {
                if constexpr (std::is_same_v<Field, bool>) {
                    if (v.isBool()) access(c) = v.asBool();
                } else {
                    if (!v.isNumber()) return;
                    const double x = s.type == PropertyType::Int ? static_cast<double>(v.asInt()) : v.asDouble();
                    if (std::isnan(x)) return;
                    access(c) = static_cast<Field>(std::clamp(x, s.min.value_or(-1e18), s.max.value_or(1e18)));
                }
            },
            [access](const Config& c, const PropertySpec&) -> Value { return Value(access(c)); });
    }

    ConfigBindings& push(PropertySpec spec, Decode decode, Encode encode) {
        entries_.push_back(Entry{std::move(spec), std::move(decode), std::move(encode)});
        return *this;
    }

    std::vector<Entry> entries_;
};

}  // namespace wpiemu::sdk
