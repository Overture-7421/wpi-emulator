#pragma once

// NetworkTables seam of the vision bridges. The bridges only see this interface (SDK types only); the real
// ntcore implementation lives in the optional `wpiemu_bridges_vision_nt` library and unit tests use
// testing::FakeNtBackend.
//
// Topic paths are absolute ("/limelight/tv"). A topic is created by its first publish() and keeps the type of
// that first value. Timestamps are microseconds in the ROBOT's time base (see nowUs()).
//
// Threading: every call comes from the extension's clock thread.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::bridges::vision {

// Value kinds the bridges publish and read. Always construct the integer alternative explicitly
// (std::int64_t) so it does not collide with bool / double.
using NtBytes = std::vector<std::uint8_t>;
using NtDoubles = std::vector<double>;
using NtValue = std::variant<bool, std::int64_t, double, std::string, NtDoubles, NtBytes>;

inline NtValue ntInt(std::int64_t v) { return NtValue(std::in_place_type<std::int64_t>, v); }
inline NtValue ntDouble(double v) { return NtValue(std::in_place_type<double>, v); }
inline NtValue ntBool(bool v) { return NtValue(std::in_place_type<bool>, v); }
inline NtValue ntString(std::string v) { return NtValue(std::in_place_type<std::string>, std::move(v)); }
inline NtValue ntDoubles(NtDoubles v) { return NtValue(std::in_place_type<NtDoubles>, std::move(v)); }
inline NtValue ntRaw(NtBytes v) { return NtValue(std::in_place_type<NtBytes>, std::move(v)); }

// Numeric view of a robot-written value: integers and doubles read as double (NetworkTables does the same for a
// double subscriber), anything else is nullopt.
inline std::optional<double> ntAsDouble(const std::optional<NtValue>& v) {
    if (!v) return std::nullopt;
    if (const auto* d = std::get_if<double>(&*v)) return *d;
    if (const auto* i = std::get_if<std::int64_t>(&*v)) return static_cast<double>(*i);
    return std::nullopt;
}
inline std::optional<std::int64_t> ntAsInt(const std::optional<NtValue>& v) {
    const std::optional<double> d = ntAsDouble(v);
    if (!d) return std::nullopt;
    return static_cast<std::int64_t>(*d);
}
inline std::optional<bool> ntAsBool(const std::optional<NtValue>& v) {
    if (!v) return std::nullopt;
    if (const auto* b = std::get_if<bool>(&*v)) return *b;
    return std::nullopt;
}

class INtBackend {
    WPIEMU_INTERFACE(INtBackend)
public:
    // False until ntcore is loaded in the robot process (and the robot program started). Cheap; callers poll it
    // every tick and publish nothing while it is false.
    virtual bool available() = 0;

    // The robot-visible time base in microseconds: the value Timer.getFPGATimestamp() / HAL_GetFPGATime report
    // (which is also what ntcore stamps values with in a simulated robot). Only meaningful when available().
    virtual std::int64_t nowUs() = 0;

    // Publishes `value` on `path` with an explicit timestamp. `typeString` is the NetworkTables type string for
    // raw topics (e.g. "photonstruct:PhotonPipelineResult:<uuid>"); ignored for other types. Duplicate values
    // are kept (every call is a new sample). Fails with InvalidState while !available() and with InvalidArgument
    // when `value` has another type than the topic.
    virtual sdk::Status publish(std::string_view path, const NtValue& value, std::int64_t timestampUs,
                                std::string_view typeString = {}) = 0;

    // Latest value of a topic written by the robot (or anybody else), nullopt when it was never written.
    virtual std::optional<NtValue> read(std::string_view path) = 0;

    // Drops every publisher whose path starts with `pathPrefix`.
    virtual void unpublish(std::string_view pathPrefix) = 0;
};

}  // namespace wpiemu::bridges::vision
