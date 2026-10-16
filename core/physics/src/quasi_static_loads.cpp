#include "wpiemu/core/physics/quasi_static_loads.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace wpiemu::core::physics {
namespace {

double det3(const double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

}  // namespace

void quasiStaticLoads(std::span<const WheelXY> wheels, float massKg, float comHeightM, float ax, float ay,
                      float gNormal, std::span<float> loads) {
    const std::size_t n = std::min(wheels.size(), loads.size());
    if (n == 0) return;
    const float weight = std::max(0.f, massKg * gNormal);
    const double n0 = static_cast<double>(weight) / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) loads[i] = static_cast<float>(n0);
    if (n < 3) return;

    // Normal equations of the min-norm problem: A lambda = rhs, A = sum r r^T, r = (1, x, y).
    double a[3][3] = {};
    double sx = 0.0, sy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double r[3] = {1.0, wheels[i].x, wheels[i].y};
        for (int p = 0; p < 3; ++p)
            for (int q = 0; q < 3; ++q) a[p][q] += r[p] * r[q];
        sx += wheels[i].x;
        sy += wheels[i].y;
    }
    const double rhs[3] = {0.0, -static_cast<double>(comHeightM) * massKg * ax - n0 * sx,
                           -static_cast<double>(comHeightM) * massKg * ay - n0 * sy};
    const double d = det3(a);
    if (std::fabs(d) < 1e-9) return;  // collinear layout: keep the equal split

    double lambda[3];
    for (int c = 0; c < 3; ++c) {
        double m[3][3];
        for (int p = 0; p < 3; ++p)
            for (int q = 0; q < 3; ++q) m[p][q] = (q == c) ? rhs[p] : a[p][q];
        lambda[c] = det3(m) / d;
    }

    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double v = std::max(0.0, n0 + lambda[0] + lambda[1] * wheels[i].x + lambda[2] * wheels[i].y);
        loads[i] = static_cast<float>(v);
        sum += v;
    }
    if (sum > 1e-6) {
        const double scale = static_cast<double>(weight) / sum;
        for (std::size_t i = 0; i < n; ++i) loads[i] = static_cast<float>(loads[i] * scale);
    }
}

}  // namespace wpiemu::core::physics
