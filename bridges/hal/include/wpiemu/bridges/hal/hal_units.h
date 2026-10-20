#pragma once

// Pure conversion helpers shared by the bridges (and unit-tested directly).

#include <cmath>
#include <cstdint>
#include <limits>

namespace wpiemu::bridges::hal {

// NaN / inf -> 0, otherwise clamped to [-1, 1].
inline double normalizeSpeed(double v) {
    if (!std::isfinite(v)) return 0.0;
    return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v);
}

// Rotations -> encoder counts (saturating to int32, NaN -> 0).
inline int rotationsToCounts(double rotations, double countsPerRevolution) {
    const double c = rotations * countsPerRevolution;
    if (!std::isfinite(c)) return 0;
    constexpr double kMax = static_cast<double>(std::numeric_limits<std::int32_t>::max());
    constexpr double kMin = static_cast<double>(std::numeric_limits<std::int32_t>::min());
    return static_cast<int>(std::llround(c < kMin ? kMin : (c > kMax ? kMax : c)));
}

inline double countsToRotations(int counts, double countsPerRevolution) {
    return countsPerRevolution != 0.0 ? static_cast<double>(counts) / countsPerRevolution : 0.0;
}

// Rotations per second -> HAL encoder period (seconds per count, signed; +inf when stopped).
inline double velocityToPeriod(double rps, double countsPerRevolution) {
    const double countsPerSecond = rps * countsPerRevolution;
    if (!std::isfinite(countsPerSecond) || countsPerSecond == 0.0) return std::numeric_limits<double>::infinity();
    return 1.0 / countsPerSecond;
}

// Absolute position (rotations, any value) -> duty ratio in [0, 1) (wraps once per revolution).
inline double rotationsToDutyRatio(double rotations) {
    if (!std::isfinite(rotations)) return 0.0;
    const double wrapped = rotations - std::floor(rotations);
    return wrapped >= 1.0 ? 0.0 : wrapped;
}

}  // namespace wpiemu::bridges::hal
