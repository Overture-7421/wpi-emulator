#pragma once

// Double-precision rigid-body math for the vision engine. The SDK geometry (Vec3 / Quat / Pose3) is
// float, which is right for simulation state but not for pose solving: every vision computation
// runs in double and converts to the SDK types only at the boundary (fromSdk / toSdk).
//
// Frames follow the SDK: right-handed, Z up. Rotations are 3x3 matrices (row-major); a Pose3d maps
// child-frame points to the parent frame: p_parent = R * p_child + t.

#include <array>
#include <cmath>
#include <cstddef>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::core::vision {

struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3d operator-() const { return {-x, -y, -z}; }
    constexpr Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3d operator/(double s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3d& operator+=(const Vec3d& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
};

constexpr Vec3d operator*(double s, const Vec3d& v) { return v * s; }
constexpr double dot(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3d cross(const Vec3d& a, const Vec3d& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3d& v) { return std::sqrt(dot(v, v)); }
inline Vec3d normalized(const Vec3d& v) {
    const double n = norm(v);
    return n > 0.0 ? v / n : Vec3d{};
}

struct Mat3d {
    std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};  // row-major

    constexpr double& operator()(int r, int c) { return m[static_cast<std::size_t>(r * 3 + c)]; }
    constexpr double operator()(int r, int c) const {
        return m[static_cast<std::size_t>(r * 3 + c)];
    }

    static constexpr Mat3d zero() {
        Mat3d z;
        z.m = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return z;
    }
    static constexpr Mat3d fromColumns(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
        Mat3d r;
        r.m = {a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z};
        return r;
    }
    static constexpr Mat3d fromRows(const Vec3d& a, const Vec3d& b, const Vec3d& c) {
        Mat3d r;
        r.m = {a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z};
        return r;
    }
    // The matrix S with S * v == cross(w, v).
    static constexpr Mat3d skew(const Vec3d& w) {
        Mat3d r;
        r.m = {0.0, -w.z, w.y, w.z, 0.0, -w.x, -w.y, w.x, 0.0};
        return r;
    }

    constexpr Vec3d column(int c) const { return {(*this)(0, c), (*this)(1, c), (*this)(2, c)}; }
    constexpr Vec3d row(int r) const { return {(*this)(r, 0), (*this)(r, 1), (*this)(r, 2)}; }
    constexpr Mat3d transposed() const {
        Mat3d t;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) t(r, c) = (*this)(c, r);
        return t;
    }
    constexpr double trace() const { return m[0] + m[4] + m[8]; }
    constexpr double determinant() const { return dot(row(0), cross(row(1), row(2))); }

    constexpr Vec3d operator*(const Vec3d& v) const {
        return {dot(row(0), v), dot(row(1), v), dot(row(2), v)};
    }
    constexpr Mat3d operator*(const Mat3d& o) const {
        Mat3d r = zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) r(i, j) += (*this)(i, k) * o(k, j);
        return r;
    }
    constexpr Mat3d operator+(const Mat3d& o) const {
        Mat3d r;
        for (std::size_t i = 0; i < 9; ++i) r.m[i] = m[i] + o.m[i];
        return r;
    }
    constexpr Mat3d operator-(const Mat3d& o) const {
        Mat3d r;
        for (std::size_t i = 0; i < 9; ++i) r.m[i] = m[i] - o.m[i];
        return r;
    }
    constexpr Mat3d operator*(double s) const {
        Mat3d r;
        for (std::size_t i = 0; i < 9; ++i) r.m[i] = m[i] * s;
        return r;
    }
};

// Rotation by `angleRad` about `axis` (any length; a zero axis gives the identity).
Mat3d rotationFromAxisAngle(const Vec3d& axis, double angleRad);
// exp([omega]x): the rotation with rotation vector `omega` (axis * angle).
Mat3d rotationFromRotationVector(const Vec3d& omega);
// Rotation matrix of a quaternion (x, y, z, w); normalised in double, identity for a zero
// quaternion.
Mat3d rotationFromQuaternion(const sdk::Quat& q);
// Unit quaternion of a rotation matrix, sign-canonicalised to w >= 0.
sdk::Quat quaternionFromRotation(const Mat3d& r);
// Angle in [0, pi] of the rotation `r` (accurate for tiny angles too).
double rotationAngleRad(const Mat3d& r);
// Angle between two rotations: rotationAngleRad(a^T * b).
double rotationDistanceRad(const Mat3d& a, const Mat3d& b);
// Nearest proper rotation to `m` (Gram-Schmidt on the columns): removes accumulated drift.
Mat3d orthonormalized(const Mat3d& m);

struct Pose3d {
    Mat3d R;
    Vec3d t;

    Vec3d apply(const Vec3d& p) const { return R * p + t; }
    Vec3d applyRotation(const Vec3d& v) const { return R * v; }
    Pose3d inverse() const {
        const Mat3d rt = R.transposed();
        return {rt, -(rt * t)};
    }
    // (a * b): pose b expressed in the parent frame of a (apply b first, then a).
    Pose3d operator*(const Pose3d& b) const { return {R * b.R, R * b.t + t}; }
};

Pose3d fromSdk(const sdk::Pose3& pose);
sdk::Pose3 toSdk(const Pose3d& pose);
inline Vec3d fromSdk(const sdk::Vec3& v) {
    return {static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)};
}
inline sdk::Vec3 toSdk(const Vec3d& v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

}  // namespace wpiemu::core::vision
