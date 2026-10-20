#pragma once

// Data tables: how CTRE SimDevices map onto canonical signals (ADR 0002 table, re-verified for Phoenix 6 26.3.0
// against the vendor wrapper classes). A vendor bridge is only a table; adding a device = adding a rule.
//
// Units: Phoenix publishes rotations / rotations-per-second for rotary values and DEGREES / degrees-per-second
// for Pigeon 2 angles. Canonical units are rot, rps, rad, rad/s (devices/signals.h); `scale` converts.

#include <string>
#include <vector>

#include "wpiemu/bridges/vendor/guarded_backend.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::vendor::ctre {

inline constexpr const char* kVendor = "ctre";
inline constexpr const char* kFamilyPhoenix6 = "phoenix6";
inline constexpr const char* kFamilyPhoenix5 = "phoenix5";

// DeviceDescriptor::type values.
inline constexpr const char* kTypeTalonFx = "talonfx";  // Kraken X60/X44 and Falcon 500 are all TalonFX
inline constexpr const char* kTypeTalonFxs = "talonfxs";
inline constexpr const char* kTypeCanCoder = "cancoder";
inline constexpr const char* kTypePigeon2 = "pigeon2";
inline constexpr const char* kTypeCanDle = "candle";
inline constexpr const char* kTypeCanRange = "canrange";
inline constexpr const char* kTypeCanDi = "candi";
inline constexpr const char* kTypeTalonSrx = "talonsrx";
inline constexpr const char* kTypeVictorSpx = "victorspx";
inline constexpr const char* kTypePigeonImu = "pigeonimu";

// Vendor-specific (non-canonical) signal names. "...Reported" are the values the robot-visible firmware sim
// publishes back (readonly in the Sim GUI): they include the robot's own setPosition/setYaw offsets.
namespace vsignal {
inline constexpr const char* kTorqueCurrentReported =
    "torqueCurrentReported";  // A, telemetry, not a current request
inline constexpr const char* kRotorPositionReported = "rotorPositionReported";  // rot
inline constexpr const char* kPositionReported = "positionReported";            // rot
inline constexpr const char* kYawReported = "yawReported";                      // rad
inline constexpr const char* kDetectedReported = "detectedReported";            // CANrange isDetected (firmware-derived)
inline constexpr const char* kVBatModulation = "vbatModulation";                // CANdle
inline constexpr const char* kFiveVRail = "fiveVRail";                          // V
inline constexpr const char* kOutputCurrent = "outputCurrent";                  // A
inline constexpr const char* kRequestedOutputCurrent = "requestedOutputCurrent";
inline constexpr const char* kTemperature = "temperature";                      // degC
inline constexpr const char* kDigitalIn2 = "digitalIn2";                        // CANdi S2 (S1 is canonical digitalIn)
inline constexpr const char* kPwm1Position = "pwm1Position";                    // rot
inline constexpr const char* kPwm1Velocity = "pwm1Velocity";                    // rps
inline constexpr const char* kPwm1Connected = "pwm1Connected";
inline constexpr const char* kPwm1RiseToRise = "pwm1RiseToRise";                // s
inline constexpr const char* kPwm1RiseToFall = "pwm1RiseToFall";                // s
inline constexpr const char* kPwm2Position = "pwm2Position";
inline constexpr const char* kPwm2Velocity = "pwm2Velocity";
inline constexpr const char* kPwm2Connected = "pwm2Connected";
inline constexpr const char* kPwm2RiseToRise = "pwm2RiseToRise";
inline constexpr const char* kPwm2RiseToFall = "pwm2RiseToFall";
inline constexpr const char* kSupplyCurrentInput = "supplyCurrentInput";        // Phoenix 5: sim -> robot
inline constexpr const char* kMotorCurrentInput = "motorCurrentInput";          // Phoenix 5: sim -> robot
}  // namespace vsignal

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;

struct Phoenix5Options {
    // Counts per output revolution of the quadrature encoder wired to a Talon SRX. The CTRE Mag Encoder in
    // quadrature mode reports 4096 counts/rev; Phoenix 5 velocity is counts per 100 ms.
    double quadratureCountsPerRev = 4096.0;
};

// Rule tables (all rules: busIsStatic = false, the bus is never fabricated from a SimDevice name).
std::vector<sdk::SimDeviceRule> phoenix6Rules();
std::vector<sdk::SimDeviceRule> phoenix5Rules(const Phoenix5Options& options = {});

// Integrated raw inputs of Phoenix 6 devices that need continuous single writes (see guarded_backend.h).
GuardOptions phoenix6GuardOptions();

}  // namespace wpiemu::bridges::vendor::ctre
