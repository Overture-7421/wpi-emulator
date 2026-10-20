#pragma once

// Unit constants and conversions. The SDK stores SI internally; use these ONLY at UI /
// config-file / vendor-API boundaries (e.g. Phoenix works in rotations and rotations/s,
// FRC dimensions are quoted in inches).

#include <cstdint>

namespace wpiemu::sdk {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr float kPiF = static_cast<float>(kPi);
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;
inline constexpr double kStandardGravityMps2 = 9.80665;  // g0 (m/s^2); the simulation default is kDefaultGravityMps2
inline constexpr double kMetersPerInch = 0.0254;
inline constexpr double kKgPerPound = 0.45359237;

// Simulation time is an integer count of microseconds (the HALSim FPGA-time unit) so it is exact.
using SimTimeUs = std::int64_t;

constexpr double usToSeconds(SimTimeUs us) { return static_cast<double>(us) * 1e-6; }
constexpr SimTimeUs secondsToUs(double s) { return static_cast<SimTimeUs>(s * 1e6 + (s >= 0 ? 0.5 : -0.5)); }

constexpr double degToRad(double deg) { return deg * kDegToRad; }
constexpr double radToDeg(double rad) { return rad * kRadToDeg; }
constexpr double rotToRad(double rotations) { return rotations * kTwoPi; }
constexpr double radToRot(double rad) { return rad / kTwoPi; }
constexpr double rpmToRadPerSec(double rpm) { return rpm * (kTwoPi / 60.0); }
constexpr double radPerSecToRpm(double rps) { return rps * (60.0 / kTwoPi); }
constexpr double inchesToMeters(double in) { return in * kMetersPerInch; }
constexpr double metersToInches(double m) { return m / kMetersPerInch; }
constexpr double poundsToKg(double lb) { return lb * kKgPerPound; }
constexpr double kgToPounds(double kg) { return kg / kKgPerPound; }

}  // namespace wpiemu::sdk
