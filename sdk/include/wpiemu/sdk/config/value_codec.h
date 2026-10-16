#pragma once

// Canonical Value encodings of the SDK geometry types (used by property inspectors, plugin
// config sections and the robot.json codec so everybody agrees on the shape):
//   Vec3  -> {"x":..,"y":..,"z":..}                 Quat -> {"x","y","z","w"}
//   Pose3 -> {"position":Vec3,"rotation":Quat}      Box3 -> {"pose":Pose3,"size":Vec3}  (size = full extents, metres)

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

struct Box3 {
    Pose3 pose;
    Vec3 sizeM{1.f, 1.f, 1.f};
    bool operator==(const Box3&) const = default;
};

inline Value toValue(const Vec3& v) {
    Value o = Value::makeObject();
    o["x"] = v.x;
    o["y"] = v.y;
    o["z"] = v.z;
    return o;
}
inline Value toValue(const Quat& q) {
    Value o = Value::makeObject();
    o["x"] = q.x;
    o["y"] = q.y;
    o["z"] = q.z;
    o["w"] = q.w;
    return o;
}
inline Value toValue(const Pose3& p) {
    Value o = Value::makeObject();
    o["position"] = toValue(p.position);
    o["rotation"] = toValue(p.rotation);
    return o;
}
inline Value toValue(const Box3& b) {
    Value o = Value::makeObject();
    o["pose"] = toValue(b.pose);
    o["size"] = toValue(b.sizeM);
    return o;
}

// fromValue overloads return false and leave `out` unchanged when the shape does not match.
inline bool fromValue(const Value& v, Vec3& out) {
    const Value *x = v.find("x"), *y = v.find("y"), *z = v.find("z");
    if (x == nullptr || y == nullptr || z == nullptr || !x->isNumber() || !y->isNumber() || !z->isNumber()) return false;
    out = Vec3{static_cast<float>(x->asDouble()), static_cast<float>(y->asDouble()), static_cast<float>(z->asDouble())};
    return true;
}
inline bool fromValue(const Value& v, Quat& out) {
    const Value *x = v.find("x"), *y = v.find("y"), *z = v.find("z"), *w = v.find("w");
    if (!x || !y || !z || !w || !x->isNumber() || !y->isNumber() || !z->isNumber() || !w->isNumber()) return false;
    out = Quat{static_cast<float>(x->asDouble()), static_cast<float>(y->asDouble()),
               static_cast<float>(z->asDouble()), static_cast<float>(w->asDouble())};
    return true;
}
inline bool fromValue(const Value& v, Pose3& out) {
    const Value *p = v.find("position"), *r = v.find("rotation");
    Pose3 tmp;
    if (p == nullptr || r == nullptr || !fromValue(*p, tmp.position) || !fromValue(*r, tmp.rotation)) return false;
    out = tmp;
    return true;
}
inline bool fromValue(const Value& v, Box3& out) {
    const Value *p = v.find("pose"), *s = v.find("size");
    Box3 tmp;
    if (p == nullptr || s == nullptr || !fromValue(*p, tmp.pose) || !fromValue(*s, tmp.sizeM)) return false;
    out = tmp;
    return true;
}

}  // namespace wpiemu::sdk
