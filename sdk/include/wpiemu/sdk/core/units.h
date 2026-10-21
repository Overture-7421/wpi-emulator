#pragma once

// Unit constants and conversions. The SDK stores SI internally; use these ONLY at UI /
// config-file / vendor-API boundaries (e.g. Phoenix works in rotations and rotations/s,
// FRC dimensions are quoted in inches).

#include <cmath>
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
inline constexpr double kNominalBatteryVoltageV = 12.6;  // a fully charged FRC battery, open circuit

// Simulation time is an integer count of microseconds (the HALSim FPGA-time unit) so it is exact.
using SimTimeUs = std::int64_t;

// The emulator is the clock master: a tick lasts kDefaultStepUs (200 Hz) unless configured, and a configured step
// must divide the robot's kRobotPeriodUs (20 ms) evenly.
inline constexpr SimTimeUs kDefaultStepUs = 5000;
inline constexpr SimTimeUs kRobotPeriodUs = 20000;
inline constexpr double kDefaultStepSeconds = 0.005;   // kDefaultStepUs in seconds
inline constexpr double kDefaultWheelRadiusM = 0.0508;  // a 4 in wheel

constexpr double usToSeconds(SimTimeUs us) { return static_cast<double>(us) * 1e-6; }
constexpr SimTimeUs secondsToUs(double s) { return static_cast<SimTimeUs>(s * 1e6 + (s >= 0 ? 0.5 : -0.5)); }
static_assert(secondsToUs(kDefaultStepSeconds) == kDefaultStepUs);

// Milliseconds and rates <-> integer microseconds. msToUs and frequencyToUs round to the nearest microsecond;
// frequencyToUs needs a positive rate.
constexpr double usToMs(SimTimeUs us) { return static_cast<double>(us) / 1000.0; }
inline SimTimeUs msToUs(double ms) { return static_cast<SimTimeUs>(std::llround(ms * 1000.0)); }
inline SimTimeUs frequencyToUs(double hz) { return static_cast<SimTimeUs>(std::llround(1e6 / hz)); }

constexpr double degToRad(double deg) { return deg * kDegToRad; }
constexpr double radToDeg(double rad) { return rad * kRadToDeg; }
// Float geometry/UI code converts through double so it matches the double overloads above.
constexpr float degToRadF(float deg) { return static_cast<float>(degToRad(static_cast<double>(deg))); }
constexpr float radToDegF(float rad) { return static_cast<float>(radToDeg(static_cast<double>(rad))); }
// Angles wrapped to (-pi, pi] and (-180, 180].
inline double wrapAngleRad(double rad) {
    const double wrapped = std::remainder(rad, kTwoPi);
    return wrapped <= -kPi ? wrapped + kTwoPi : wrapped;
}
inline double wrapAngleDeg(double deg) {
    const double wrapped = std::remainder(deg, 360.0);
    return wrapped <= -180.0 ? wrapped + 360.0 : wrapped;
}
constexpr double rotToRad(double rotations) { return rotations * kTwoPi; }
constexpr double radToRot(double rad) { return rad / kTwoPi; }
constexpr double rpmToRadPerSec(double rpm) { return rpm * (kTwoPi / 60.0); }
constexpr double radPerSecToRpm(double rps) { return rps * (60.0 / kTwoPi); }
constexpr double inchesToMeters(double in) { return in * kMetersPerInch; }
constexpr double metersToInches(double m) { return m / kMetersPerInch; }
constexpr double poundsToKg(double lb) { return lb * kKgPerPound; }
constexpr double kgToPounds(double kg) { return kg / kKgPerPound; }

}  // namespace wpiemu::sdk
