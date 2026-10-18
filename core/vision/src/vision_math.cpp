#include "wpiemu/core/vision/vision_math.h"

#include <algorithm>

namespace wpiemu::core::vision {

Mat3d rotationFromAxisAngle(const Vec3d& axis, double angleRad) {
    const double n = norm(axis);
    if (n <= 0.0 || angleRad == 0.0) return Mat3d{};
    return rotationFromRotationVector(axis * (angleRad / n));
}

Mat3d rotationFromRotationVector(const Vec3d& omega) {
    const double theta = norm(omega);
    const Mat3d k = Mat3d::skew(omega);
    // Rodrigues: I + sin(t)/t [w]x + (1-cos(t))/t^2 [w]x^2, with the small-angle series near zero.
    double a = 1.0;
    double b = 0.5;
    if (theta > 1e-8) {
        a = std::sin(theta) / theta;
        b = (1.0 - std::cos(theta)) / (theta * theta);
    }
    return Mat3d{} + k * a + (k * k) * b;
}

Mat3d rotationFromQuaternion(const sdk::Quat& q) {
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(n > 0.0)) return Mat3d{};
    const double qx = x / n, qy = y / n, qz = z / n, qw = w / n;
    Mat3d r;
    r.m = {1.0 - 2.0 * (qy * qy + qz * qz),
           2.0 * (qx * qy - qz * qw),
           2.0 * (qx * qz + qy * qw),
           2.0 * (qx * qy + qz * qw),
           1.0 - 2.0 * (qx * qx + qz * qz),
           2.0 * (qy * qz - qx * qw),
           2.0 * (qx * qz - qy * qw),
           2.0 * (qy * qz + qx * qw),
           1.0 - 2.0 * (qx * qx + qy * qy)};
    return r;
}

sdk::Quat quaternionFromRotation(const Mat3d& r) {
    double x, y, z, w;
    const double tr = r.trace();
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s;
        x = (r(2, 1) - r(1, 2)) / s;
        y = (r(0, 2) - r(2, 0)) / s;
        z = (r(1, 0) - r(0, 1)) / s;
    } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
        const double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
        w = (r(2, 1) - r(1, 2)) / s;
        x = 0.25 * s;
        y = (r(0, 1) + r(1, 0)) / s;
        z = (r(0, 2) + r(2, 0)) / s;
    } else if (r(1, 1) > r(2, 2)) {
        const double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
        w = (r(0, 2) - r(2, 0)) / s;
        x = (r(0, 1) + r(1, 0)) / s;
        y = 0.25 * s;
        z = (r(1, 2) + r(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
        w = (r(1, 0) - r(0, 1)) / s;
        x = (r(0, 2) + r(2, 0)) / s;
        y = (r(1, 2) + r(2, 1)) / s;
        z = 0.25 * s;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(n > 0.0)) return sdk::Quat{};
    const double sign = w < 0.0 ? -1.0 : 1.0;
    const double k = sign / n;
    return sdk::Quat{static_cast<float>(x * k), static_cast<float>(y * k),
                     static_cast<float>(z * k), static_cast<float>(w * k)};
}

double rotationAngleRad(const Mat3d& r) {
    const Vec3d v{r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)};
    return std::atan2(0.5 * norm(v), 0.5 * (r.trace() - 1.0));
}

double rotationDistanceRad(const Mat3d& a, const Mat3d& b) {
    return rotationAngleRad(a.transposed() * b);
}

Mat3d orthonormalized(const Mat3d& m) {
    const Vec3d c0 = normalized(m.column(0));
    const Vec3d c2 = normalized(cross(c0, m.column(1)));
    const Vec3d c1 = cross(c2, c0);
    return Mat3d::fromColumns(c0, c1, c2);
}

Pose3d fromSdk(const sdk::Pose3& pose) {
    return {rotationFromQuaternion(pose.rotation), fromSdk(pose.position)};
}

sdk::Pose3 toSdk(const Pose3d& pose) {
    return {toSdk(pose.t), quaternionFromRotation(pose.R)};
}

}  // namespace wpiemu::core::vision
