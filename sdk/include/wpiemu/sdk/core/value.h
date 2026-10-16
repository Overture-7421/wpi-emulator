#pragma once

// Small JSON-like dynamic value used for plugin config sections, mechanism properties and HUD
// values. The SDK deliberately has no JSON dependency: core/app translate Value <-> JSON text
// (see config/config_io.h, IValueTextCodec).
//
// Semantics:
//  * Types: null, bool, int64, double, string, array, object.
//  * Objects keep INSERTION ORDER (stable, diff-friendly serialisation) but compare as unordered
//    key/value sets.
//  * Numbers: int64 and double are distinct types, but isNumber()/asDouble() treat both uniformly
//    and operator== treats 1 and 1.0 as equal.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace wpiemu::sdk {

class Value;
using ValueArray = std::vector<Value>;

class ValueObject {
public:
    std::size_t size() const { return keys_.size(); }
    bool empty() const { return keys_.empty(); }
    const std::string& keyAt(std::size_t i) const { return keys_[i]; }
    inline const Value& valueAt(std::size_t i) const;
    inline Value& valueAt(std::size_t i);

    inline const Value* find(std::string_view key) const;
    inline Value* find(std::string_view key);
    bool contains(std::string_view key) const { return indexOf(key) != npos; }
    inline Value& set(std::string_view key, Value v);  // insert or overwrite
    inline bool erase(std::string_view key);
    inline Value& operator[](std::string_view key);  // insert-null if absent

    inline bool operator==(const ValueObject& o) const;

private:
    static constexpr std::size_t npos = static_cast<std::size_t>(-1);
    std::size_t indexOf(std::string_view key) const {
        for (std::size_t i = 0; i < keys_.size(); ++i)
            if (keys_[i] == key) return i;
        return npos;
    }

    std::vector<std::string> keys_;
    std::vector<Value> values_;
};

class Value {
public:
    using Array = ValueArray;
    using Object = ValueObject;

    Value() = default;
    Value(std::nullptr_t) {}  // NOLINT
    Value(bool b) : v_(b) {}  // NOLINT
    Value(int i) : v_(static_cast<std::int64_t>(i)) {}  // NOLINT
    Value(unsigned i) : v_(static_cast<std::int64_t>(i)) {}  // NOLINT
    Value(std::int64_t i) : v_(i) {}  // NOLINT
    Value(float d) : v_(static_cast<double>(d)) {}  // NOLINT
    Value(double d) : v_(d) {}  // NOLINT
    Value(const char* s) : v_(std::string(s)) {}  // NOLINT
    Value(std::string s) : v_(std::move(s)) {}  // NOLINT
    Value(std::string_view s) : v_(std::string(s)) {}  // NOLINT
    Value(Array a) : v_(std::move(a)) {}  // NOLINT
    Value(Object o) : v_(std::move(o)) {}  // NOLINT

    static Value makeArray() { return Value(Array{}); }
    static Value makeObject() { return Value(Object{}); }

    bool isNull() const { return std::holds_alternative<std::monostate>(v_); }
    bool isBool() const { return std::holds_alternative<bool>(v_); }
    bool isInt() const { return std::holds_alternative<std::int64_t>(v_); }
    bool isDouble() const { return std::holds_alternative<double>(v_); }
    bool isNumber() const { return isInt() || isDouble(); }
    bool isString() const { return std::holds_alternative<std::string>(v_); }
    bool isArray() const { return std::holds_alternative<Array>(v_); }
    bool isObject() const { return std::holds_alternative<Object>(v_); }

    // Lenient accessors: return `fallback` when the type does not match.
    bool asBool(bool fallback = false) const { return isBool() ? std::get<bool>(v_) : fallback; }
    std::int64_t asInt(std::int64_t fallback = 0) const {
        if (isInt()) return std::get<std::int64_t>(v_);
        if (isDouble()) return static_cast<std::int64_t>(std::get<double>(v_));
        return fallback;
    }
    double asDouble(double fallback = 0.0) const {
        if (isDouble()) return std::get<double>(v_);
        if (isInt()) return static_cast<double>(std::get<std::int64_t>(v_));
        return fallback;
    }
    std::string asString(std::string_view fallback = {}) const {
        return isString() ? std::get<std::string>(v_) : std::string(fallback);
    }

    // Strict accessors: the value MUST have that type (check is*() first).
    const Array& array() const { return std::get<Array>(v_); }
    Array& array() { return std::get<Array>(v_); }
    const Object& object() const { return std::get<Object>(v_); }
    Object& object() { return std::get<Object>(v_); }

    // Object convenience (no-ops / null for non-objects).
    const Value* find(std::string_view key) const { return isObject() ? object().find(key) : nullptr; }
    // Dotted path through nested objects, e.g. "intake.zone.size".
    const Value* findPath(std::string_view dotted) const {
        const Value* cur = this;
        while (cur != nullptr && !dotted.empty()) {
            const auto dot = dotted.find('.');
            const auto head = dotted.substr(0, dot);
            cur = cur->find(head);
            dotted = dot == std::string_view::npos ? std::string_view{} : dotted.substr(dot + 1);
        }
        return cur;
    }
    // Converts null to an empty object; precondition: null or object.
    Value& operator[](std::string_view key) {
        if (isNull()) v_ = Object{};
        return object()[key];
    }
    Value get(std::string_view key, Value fallback = {}) const {
        const Value* v = find(key);
        return v != nullptr ? *v : std::move(fallback);
    }
    void push_back(Value v) {
        if (isNull()) v_ = Array{};
        array().push_back(std::move(v));
    }

    bool operator==(const Value& o) const {
        if (isNumber() && o.isNumber()) return asDouble() == o.asDouble();
        return v_ == o.v_;
    }

private:
    std::variant<std::monostate, bool, std::int64_t, double, std::string, Array, Object> v_;
};

inline const Value& ValueObject::valueAt(std::size_t i) const { return values_[i]; }
inline Value& ValueObject::valueAt(std::size_t i) { return values_[i]; }
inline const Value* ValueObject::find(std::string_view key) const {
    const auto i = indexOf(key);
    return i == npos ? nullptr : &values_[i];
}
inline Value* ValueObject::find(std::string_view key) {
    const auto i = indexOf(key);
    return i == npos ? nullptr : &values_[i];
}
inline Value& ValueObject::set(std::string_view key, Value v) {
    if (Value* existing = find(key)) {
        *existing = std::move(v);
        return *existing;
    }
    keys_.emplace_back(key);
    values_.push_back(std::move(v));
    return values_.back();
}
inline bool ValueObject::erase(std::string_view key) {
    const auto i = indexOf(key);
    if (i == npos) return false;
    keys_.erase(keys_.begin() + static_cast<std::ptrdiff_t>(i));
    values_.erase(values_.begin() + static_cast<std::ptrdiff_t>(i));
    return true;
}
inline Value& ValueObject::operator[](std::string_view key) {
    if (Value* existing = find(key)) return *existing;
    return set(key, Value{});
}
inline bool ValueObject::operator==(const ValueObject& o) const {
    if (size() != o.size()) return false;
    for (std::size_t i = 0; i < keys_.size(); ++i) {
        const Value* other = o.find(keys_[i]);
        if (other == nullptr || !(*other == values_[i])) return false;
    }
    return true;
}

}  // namespace wpiemu::sdk
