#pragma once

// Basic math types shared by every module.
//
// Conventions (apply to the WHOLE SDK):
//  * Right-handed, Z-up world frame == WPILib NWU field frame: +X toward the opposing alliance
//    wall, +Y to the left, +Z up. Yaw is counter-clockwise (CCW+) seen from above.
//  * Robot/chassis frame: +X forward, +Y left, +Z up, origin at the chassis centre of mass
//    (ADR 0003: body origin == COM).
//  * SI units everywhere (m, kg, s, rad, N, Nm, V, A). Degrees/inches/rpm only at UI/config
//    boundaries (see units.h). Identifiers carry unit suffixes where the type is a plain scalar
//    (`radiusM`, `massKg`, `timeUs`).
//  * Geometry uses float (matches Box3D, ADR 0003); signals, time and electrical quantities use double.
//  * Quaternions are (x, y, z, w), unit length, Hamilton convention.
//  * The Qt Quick 3D renderer is Y-up: conversion happens ONLY inside the UI view layer.

#include <cmath>
#include <cstdint>

namespace wpiemu::sdk {

struct Vec3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    constexpr bool operator==(const Vec3&) const = default;
};

constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }
constexpr float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    const float l = length(v);
    return l > 0.f ? v / l : Vec3{};
}

struct Quat {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float w = 1.f;

    constexpr Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static constexpr Quat identity() { return {}; }

    // `axis` must be unit length.
    static Quat fromAxisAngle(const Vec3& axis, float angleRad) {
        const float h = 0.5f * angleRad;
        const float s = std::sin(h);
        return {axis.x * s, axis.y * s, axis.z * s, std::cos(h)};
    }
    static Quat fromYaw(float yawRad) { return fromAxisAngle({0.f, 0.f, 1.f}, yawRad); }
    // Intrinsic Z-Y-X (yaw, pitch, roll) like aerospace / WPILib Rotation3d.
    static Quat fromEuler(float rollRad, float pitchRad, float yawRad) {
        const float cr = std::cos(rollRad * 0.5f), sr = std::sin(rollRad * 0.5f);
        const float cp = std::cos(pitchRad * 0.5f), sp = std::sin(pitchRad * 0.5f);
        const float cy = std::cos(yawRad * 0.5f), sy = std::sin(yawRad * 0.5f);
        return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
    }

    constexpr Quat conjugate() const { return {-x, -y, -z, w}; }
    constexpr bool operator==(const Quat&) const = default;

    // Hamilton product: (a * b) applies b first, then a.
    constexpr Quat operator*(const Quat& b) const {
        return {w * b.x + x * b.w + y * b.z - z * b.y, w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w, w * b.w - x * b.x - y * b.y - z * b.z};
    }

    Vec3 rotate(const Vec3& v) const {
        const Vec3 q{x, y, z};
        const Vec3 t = 2.f * cross(q, v);
        return v + w * t + cross(q, t);
    }

    Quat normalized() const {
        const float n = std::sqrt(x * x + y * y + z * z + w * w);
        return n > 0.f ? Quat{x / n, y / n, z / n, w / n} : Quat{};
    }

    float yaw() const { return std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z)); }
};

struct Pose3 {
    Vec3 position;
    Quat rotation;

    constexpr Pose3() = default;
    constexpr Pose3(const Vec3& p, const Quat& q) : position(p), rotation(q) {}

    Vec3 transformPoint(const Vec3& local) const { return position + rotation.rotate(local); }
    Vec3 transformVector(const Vec3& local) const { return rotation.rotate(local); }
    Pose3 inverse() const {
        const Quat inv = rotation.conjugate();
        return {-(inv.rotate(position)), inv};
    }
    // (a * b) = pose b expressed in the frame of a.
    Pose3 operator*(const Pose3& b) const { return {transformPoint(b.position), rotation * b.rotation}; }
    constexpr bool operator==(const Pose3&) const = default;
};

}  // namespace wpiemu::sdk
