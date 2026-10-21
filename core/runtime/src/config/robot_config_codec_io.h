#pragma once

// The helper layer of RobotConfigCodec (internal to core/runtime/src/config): a typed reader over one JSON object, the
// Value builders of the small shared types, and the spelling of the enums the codec names itself. The per-struct decode
// is in robot_config_decode.cpp and the per-struct encode in robot_config_encode.cpp, both on top of this layer.

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::core::config::codec_io {

using sdk::Pose3;
using sdk::Quat;
using sdk::Value;
using sdk::Vec3;

// ---- decode ------------------------------------------------------------------------------------------

struct Ctx {
    std::vector<std::string> errors;
    void err(const std::string& path, const std::string& what) { errors.push_back((path.empty() ? "<root>" : path) + ": " + what); }
};

inline std::string join(const std::string& path, std::string_view key) {
    return path.empty() ? std::string(key) : path + "." + std::string(key);
}

template <typename T>
bool readInteger(const Value& value, T& out) {
    static_assert(std::numeric_limits<T>::is_integer && std::numeric_limits<T>::is_signed);
    if (value.isInt()) {
        const std::int64_t n = value.asInt();
        if (n < std::numeric_limits<T>::min() || n > std::numeric_limits<T>::max()) return false;
        out = static_cast<T>(n);
        return true;
    }
    if (!value.isDouble()) return false;
    const double d = value.asDouble();
    // The signed upper bound is exclusive: INT64_MAX rounds up to 2^63 as a double.
    const double limit = std::ldexp(1.0, std::numeric_limits<T>::digits);
    if (!std::isfinite(d) || std::trunc(d) != d || d < -limit || d >= limit) return false;
    out = static_cast<T>(d);
    return true;
}

// Typed access to one JSON object. Keys nobody asks for are ignored.
class ObjReader {
public:
    ObjReader(const Value& v, std::string path, Ctx& ctx);
    bool valid() const { return obj_ != nullptr; }
    const std::string& path() const { return path_; }

    const Value* get(std::string_view key) const {
        return obj_ != nullptr ? obj_->find(key) : nullptr;
    }

    bool str(std::string_view key, std::string& out);
    bool boolean(std::string_view key, bool& out);
    bool dbl(std::string_view key, double& out);
    bool flt(std::string_view key, float& out);
    bool i64(std::string_view key, std::int64_t& out);
    bool i32(std::string_view key, int& out);
    bool u64(std::string_view key, std::uint64_t& out);
    bool vec3(std::string_view key, Vec3& out);
    bool pose(std::string_view key, Pose3& out);
    // Reads a quaternion stored under `key`; a zero-length quaternion is rejected.
    bool quat(std::string_view key, Quat& out);

private:
    bool typeErr(std::string_view key, const char* expected);

    const Value* obj_ = nullptr;
    std::string path_;
    Ctx& ctx_;
};

// ---- encode ------------------------------------------------------------------------------------------

Value fv(float f);
Value vec3Value(const Vec3& v);
Value quatValue(const Quat& q);
Value poseValue(const Pose3& p);

// ---- enum spelling -----------------------------------------------------------------------------------

bool conditionOpFrom(const std::string& s, sdk::ConditionOp& out);
const char* conditionOpName(sdk::ConditionOp op);

}  // namespace wpiemu::core::config::codec_io
