#include "robot_config_codec_io.h"

#include <utility>

#include "wpiemu/core/config/json_codec.h"

namespace wpiemu::core::config::codec_io {

using namespace wpiemu::sdk;

// ---- decode ------------------------------------------------------------------------------------------

ObjReader::ObjReader(const Value& v, std::string path, Ctx& ctx) : path_(std::move(path)), ctx_(ctx) {
    if (v.isObject()) obj_ = &v;
    else ctx_.err(path_, "expected an object");
}

bool ObjReader::str(std::string_view key, std::string& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    if (!v->isString()) return typeErr(key, "a string");
    out = v->asString();
    return true;
}

bool ObjReader::boolean(std::string_view key, bool& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    if (!v->isBool()) return typeErr(key, "a boolean");
    out = v->asBool();
    return true;
}

bool ObjReader::dbl(std::string_view key, double& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    if (!v->isNumber()) return typeErr(key, "a number");
    if (!std::isfinite(v->asDouble())) return typeErr(key, "a finite number");
    out = v->asDouble();
    return true;
}

bool ObjReader::flt(std::string_view key, float& out) {
    double d = 0.0;
    if (!dbl(key, d)) return false;
    if (d < -std::numeric_limits<float>::max() || d > std::numeric_limits<float>::max())
        return typeErr(key, "a number in the float range");
    out = static_cast<float>(d);
    return true;
}

bool ObjReader::i64(std::string_view key, std::int64_t& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    return readInteger(*v, out) || typeErr(key, "an integer in the int64 range");
}

bool ObjReader::i32(std::string_view key, int& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    return readInteger(*v, out) || typeErr(key, "an integer in the int range");
}

bool ObjReader::u64(std::string_view key, std::uint64_t& out) {
    std::int64_t v = 0;
    if (!i64(key, v)) return false;
    // Seeds use the persisted signed int64 representation, including negative high-bit seeds.
    out = static_cast<std::uint64_t>(v);
    return true;
}

bool ObjReader::vec3(std::string_view key, Vec3& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    ObjReader r(*v, join(path_, key), ctx_);
    Vec3 tmp;
    if (!r.flt("x", tmp.x) || !r.flt("y", tmp.y) || !r.flt("z", tmp.z))
        return typeErr(key, "an {x,y,z} object with finite float components");
    out = tmp;
    return true;
}

bool ObjReader::pose(std::string_view key, Pose3& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    ObjReader p(*v, join(path_, key), ctx_);
    if (!p.valid()) return false;
    Pose3 tmp = out;
    p.vec3("position", tmp.position);
    p.quat("rotation", tmp.rotation);
    out = tmp;
    return true;
}

bool ObjReader::quat(std::string_view key, Quat& out) {
    const Value* v = get(key);
    if (v == nullptr) return false;
    ObjReader r(*v, join(path_, key), ctx_);
    Quat q;
    if (!r.flt("x", q.x) || !r.flt("y", q.y) || !r.flt("z", q.z) || !r.flt("w", q.w))
        return typeErr(key, "an {x,y,z,w} object with finite float components");
    const double n = std::sqrt(static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
                               static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w);
    if (!(n > 1e-6)) return typeErr(key, "a non-zero quaternion");
    out = Quat{static_cast<float>(q.x / n), static_cast<float>(q.y / n),
               static_cast<float>(q.z / n), static_cast<float>(q.w / n)};
    return true;
}

bool ObjReader::typeErr(std::string_view key, const char* expected) {
    ctx_.err(join(path_, key), std::string("expected ") + expected);
    return false;
}

// ---- encode ------------------------------------------------------------------------------------------

Value fv(float f) { return Value(floatToDouble(f)); }

Value vec3Value(const Vec3& v) {
    Value o = Value::makeObject();
    o["x"] = fv(v.x);
    o["y"] = fv(v.y);
    o["z"] = fv(v.z);
    return o;
}
Value quatValue(const Quat& q) {
    Value o = Value::makeObject();
    o["x"] = fv(q.x);
    o["y"] = fv(q.y);
    o["z"] = fv(q.z);
    o["w"] = fv(q.w);
    return o;
}
Value poseValue(const Pose3& p) {
    Value o = Value::makeObject();
    o["position"] = vec3Value(p.position);
    o["rotation"] = quatValue(p.rotation);
    return o;
}

// ---- enum spelling -----------------------------------------------------------------------------------

bool conditionOpFrom(const std::string& s, ConditionOp& out) {
    if (s == "above") out = ConditionOp::Above;
    else if (s == "below") out = ConditionOp::Below;
    else if (s == "within") out = ConditionOp::Within;
    else return false;
    return true;
}
const char* conditionOpName(ConditionOp op) {
    switch (op) {
        case ConditionOp::Above: return "above";
        case ConditionOp::Below: return "below";
        case ConditionOp::Within: return "within";
    }
    return "above";
}

}  // namespace wpiemu::core::config::codec_io
