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
    void attachment(std::string_view key, sdk::MechanismAttachment& out);

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
Value attachmentValue(const sdk::MechanismAttachment& a);

// Counterpart to ObjReader with the same member names so a single `codecFields(codec, value)` template encodes
// and decodes a struct in declaration order without repeating field keys.
class ObjWriter {
public:
    ObjWriter() : out_(Value::makeObject()) {}
    explicit ObjWriter(Value& target) : out_(target) {}

    void str(std::string_view key, const std::string& v) { out_[std::string(key)] = v; }
    void boolean(std::string_view key, bool v) { out_[std::string(key)] = v; }
    void dbl(std::string_view key, double v) { out_[std::string(key)] = v; }
    void flt(std::string_view key, float v) { out_[std::string(key)] = fv(v); }
    void i64(std::string_view key, std::int64_t v) { out_[std::string(key)] = v; }
    void i32(std::string_view key, int v) { out_[std::string(key)] = v; }
    void u64(std::string_view key, std::uint64_t v) { out_[std::string(key)] = static_cast<std::int64_t>(v); }
    void vec3(std::string_view key, const Vec3& v) { out_[std::string(key)] = vec3Value(v); }
    void pose(std::string_view key, const Pose3& v) { out_[std::string(key)] = poseValue(v); }
    void attachment(std::string_view key, const sdk::MechanismAttachment& a) {
        out_[std::string(key)] = attachmentValue(a);
    }
    void set(std::string_view key, Value v) { out_[std::string(key)] = std::move(v); }

    Value take() { return std::move(out_); }

private:
    Value out_;
};

// ---- shared field tables (encode + decode in one place) ----------------------------------------------

template <class Codec, class T>
void materialFields(Codec& c, T& m) {
    c.flt("friction", m.friction);
    c.flt("restitution", m.restitution);
    c.flt("rollingResistance", m.rollingResistance);
}

template <class Codec, class T>
void motorFields(Codec& c, T& m) {
    c.str("name", m.name);
    c.dbl("nominalVoltageV", m.nominalVoltageV);
    c.dbl("stallTorqueNm", m.stallTorqueNm);
    c.dbl("stallCurrentA", m.stallCurrentA);
    c.dbl("freeCurrentA", m.freeCurrentA);
    c.dbl("freeSpeedRadPerSec", m.freeSpeedRadPerSec);
}

template <class Codec, class T>
void tireFields(Codec& c, T& t) {
    c.flt("mu", t.mu);
    c.flt("slideRatio", t.slideRatio);
    c.flt("peakSlipLong", t.peakSlipLong);
    c.flt("peakSlipLat", t.peakSlipLat);
    c.flt("vRefMps", t.vRefMps);
    c.flt("lateralScale", t.lateralScale);
    c.flt("tractionAngleRad", t.tractionAngleRad);
}

template <class Codec, class T>
void primitiveGeometryFields(Codec& c, T& p) {
    c.pose("local", p.local);
    c.vec3("sizeM", p.sizeM);
    c.flt("radiusM", p.radiusM);
    c.flt("heightM", p.heightM);
    c.str("color", p.color);
    c.boolean("collides", p.collides);
    c.flt("massKg", p.massKg);
}

template <class Codec, class T>
void chassisCoreFields(Codec& c, T& ch) {
    c.vec3("sizeM", ch.sizeM);
    c.flt("massKg", ch.massKg);
    c.flt("comHeightM", ch.comHeightM);
    c.vec3("comOffsetM", ch.comOffsetM);
    c.flt("groundClearanceM", ch.groundClearanceM);
    c.vec3("inertiaDiagKgM2", ch.inertiaDiagKgM2);
}

template <class Codec, class T>
void drivetrainCoreFields(Codec& c, T& spec) {
    c.str("kind", spec.kind);
    c.str("tireModel", spec.tireModel);
    c.flt("loadTransferTauS", spec.loadTransferTauS);
}

template <class Codec, class T>
void cameraOpticsFields(Codec& c, T& cam) {
    c.pose("pose", cam.local);
    c.dbl("hFovDeg", cam.hFovDeg);
    c.dbl("vFovDeg", cam.vFovDeg);
    c.dbl("maxRangeM", cam.maxRangeM);
    c.dbl("maxIncidenceDeg", cam.maxIncidenceDeg);
    c.dbl("fps", cam.fps);
    c.dbl("latencyMs", cam.latencyMs);
    c.i32("widthPx", cam.widthPx);
    c.i32("heightPx", cam.heightPx);
    c.dbl("positionNoiseM", cam.positionNoiseM);
    c.dbl("yawNoiseDeg", cam.yawNoiseDeg);
}

template <class Codec, class T>
void projectFields(Codec& c, T& p) {
    c.str("language", p.language);
    c.i32("wpilibYear", p.wpilibYear);
    c.str("gradleRioVersion", p.gradleRioVersion);
}

template <class Codec, class T>
void batteryFields(Codec& c, T& b) {
    c.dbl("nominalVoltageV", b.nominalVoltageV);
    c.dbl("internalResistanceOhm", b.internalResistanceOhm);
    c.dbl("brownoutVoltageV", b.brownoutVoltageV);
}

template <class Codec, class T>
void physicsFields(Codec& c, T& p) {
    c.i32("subSteps", p.subSteps);
    c.boolean("sleeping", p.sleeping);
    c.vec3("gravity", p.gravity);
}

// ---- enum spelling -----------------------------------------------------------------------------------

bool conditionOpFrom(const std::string& s, sdk::ConditionOp& out);
const char* conditionOpName(sdk::ConditionOp op);

}  // namespace wpiemu::core::config::codec_io
