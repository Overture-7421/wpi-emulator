#pragma once

// Fixed-point schemes of the match wire fields. Pure functions; every one is total: NaN and infinities map to 0 and
// out-of-range values SATURATE (never wrap, never undefined behaviour), except angles, which are periodic and wrap.
//
//   scheme              type   unit                        range                  max round-trip error
//   angle               i16    2*pi/65536 rad (9.6e-5)     any angle, wraps       pi/65536 rad = 4.8e-5 rad, modulo 2*pi
//   milli (position)    i32    1 mm                        +-2147483 m            0.5 mm
//   milli (velocity)    i16    1 mm/s or 1 mrad/s          +-32.767 m/s or rad/s  0.5 unit
//   milli (zone pose)   i16    1 mm (chassis frame)        +-32.767 m             0.5 mm (see zone_pose_wire.h)
//   centi (entities)    i16    1 cm                        +-327.67 m             0.5 cm
//
// Angle: the i16 is a fraction of a full turn, so dequantizeAngle returns [-pi, pi); pi and -pi are the same angle and
// both give -32768. Compare angles with angleDifferenceRad, not with ==.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk::match {

inline constexpr double kAngleUnitRad = kTwoPi / 65536.0;
inline constexpr double kMaxAngleQuantizationErrorRad = kAngleUnitRad / 2.0;

inline std::int16_t quantizeAngle(double rad) {
    if (!std::isfinite(rad)) return 0;
    double turns = rad / kTwoPi;
    turns -= std::floor(turns);  // [0, 1)
    const auto units = static_cast<std::uint32_t>(std::llround(turns * 65536.0)) & 0xFFFFu;
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(units));
}

inline float dequantizeAngle(std::int16_t q) { return static_cast<float>(static_cast<double>(q) * kAngleUnitRad); }

// Signed shortest difference a - b of two angles, in [-pi, pi].
inline double angleDifferenceRad(double a, double b) {
    double d = std::fmod(a - b, kTwoPi);
    if (d > kPi) d -= kTwoPi;
    if (d < -kPi) d += kTwoPi;
    return d;
}

namespace detail {
template <class Int>
Int saturatingRound(double scaled) {
    if (!std::isfinite(scaled)) return 0;
    constexpr double lo = static_cast<double>(std::numeric_limits<Int>::min());
    constexpr double hi = static_cast<double>(std::numeric_limits<Int>::max());
    return static_cast<Int>(std::clamp(std::round(scaled), lo, hi));
}
}  // namespace detail

// Milli units (x1000): positions as i32 millimetres, velocities as i16 mm/s and mrad/s.
inline std::int32_t quantizeMilliI32(double value) { return detail::saturatingRound<std::int32_t>(value * 1000.0); }
inline std::int16_t quantizeMilliI16(double value) { return detail::saturatingRound<std::int16_t>(value * 1000.0); }
inline double dequantizeMilli(std::int32_t q) { return static_cast<double>(q) / 1000.0; }

// Centi units (x100): entity positions as i16 centimetres.
inline std::int16_t quantizeCentiI16(double value) { return detail::saturatingRound<std::int16_t>(value * 100.0); }
inline double dequantizeCenti(std::int16_t q) { return static_cast<double>(q) / 100.0; }

}  // namespace wpiemu::sdk::match
