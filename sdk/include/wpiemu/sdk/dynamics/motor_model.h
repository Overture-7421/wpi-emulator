#pragma once

// 1-DOF motor model: data compatible with WPILib's DCMotor (but no WPILib dependency in the SDK;
// core may convert to/from frc::DCMotor).

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::sdk {

struct MotorSpec {
    std::string name = "generic";
    double nominalVoltageV = 12.0;
    double stallTorqueNm = 2.6;
    double stallCurrentA = 105.0;
    double freeCurrentA = 1.8;
    double freeSpeedRadPerSec = rpmToRadPerSec(5676.0);

    // Derived (WPILib DCMotor equations).
    double resistanceOhm() const { return nominalVoltageV / stallCurrentA; }
    double kvRadPerSecPerVolt() const {
        return freeSpeedRadPerSec / (nominalVoltageV - resistanceOhm() * freeCurrentA);
    }
    double ktNmPerAmp() const { return stallTorqueNm / stallCurrentA; }
};

// Voltage that drives `torqueCurrentA` through this motor at shaft speed `omegaRadPerSec`: V = I*R + omega/Kv.
// A controller that regulates current (Phoenix FOC, TorqueCurrentFOC) reports a voltage that only its own motor
// model can interpret; the plant resolves the current against the motor it actually simulates.
inline double voltageForTorqueCurrent(const MotorSpec& m, double torqueCurrentA, double omegaRadPerSec) {
    return torqueCurrentA * m.resistanceOhm() + omegaRadPerSec / m.kvRadPerSecPerVolt();
}

// The hardware a MotorSpec describes, read from its preset name ("Kraken X44 (FOC)", "Falcon 500", ...).
// Vendor bridges use it to tell a vendor's firmware simulation which motor is attached, so that the firmware
// models the same motor the plant does; Unknown leaves the firmware's own default in place. Deliberately
// coarse: the FOC variant of a motor is the same hardware.
enum class MotorFamily : int { Unknown = 0, KrakenX60 = 1, KrakenX44 = 2, Falcon500 = 3, Minion = 4 };

inline MotorFamily motorFamilyOf(const MotorSpec& m) {
    std::string name;
    name.reserve(m.name.size());
    for (const char c : m.name) name.push_back(c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c);
    const auto has = [&name](const char* needle) { return name.find(needle) != std::string::npos; };
    if (has("kraken x60")) return MotorFamily::KrakenX60;
    if (has("kraken x44")) return MotorFamily::KrakenX44;
    if (has("falcon 500")) return MotorFamily::Falcon500;
    if (has("minion")) return MotorFamily::Minion;
    return MotorFamily::Unknown;
}

// One way a MotorSpec is not physical; `field` is the MotorSpec member name.
struct MotorSpecFault {
    const char* field;
    const char* why;
};

// Every fault of `m` (empty when it is valid): the one definition of a physical motor, shared by the motor
// catalog and the configuration validator.
inline std::vector<MotorSpecFault> motorSpecFaults(const MotorSpec& m) {
    std::vector<MotorSpecFault> faults;
    const auto positive = [&](double v, const char* field, const char* why) {
        if (!(std::isfinite(v) && v > 0.0)) faults.push_back({field, why});
    };
    positive(m.nominalVoltageV, "nominalVoltageV", "nominal voltage must be positive and finite");
    positive(m.stallTorqueNm, "stallTorqueNm", "stall torque must be positive and finite");
    positive(m.stallCurrentA, "stallCurrentA", "stall current must be positive and finite");
    positive(m.freeSpeedRadPerSec, "freeSpeedRadPerSec", "free speed must be positive and finite");
    if (!(std::isfinite(m.freeCurrentA) && m.freeCurrentA >= 0.0))
        faults.push_back({"freeCurrentA", "free current must be >= 0 and finite"});
    else if (std::isfinite(m.stallCurrentA) && m.stallCurrentA > 0.0 && m.freeCurrentA >= m.stallCurrentA)
        faults.push_back({"freeCurrentA", "free current must be below the stall current"});
    return faults;
}

inline bool isValidMotorSpec(const MotorSpec& m) { return motorSpecFaults(m).empty(); }

// One motor of the built-in table.
struct MotorPreset {
    std::string key;    // stable snake_case id, e.g. "kraken_x60"; mechanisms persist it as `motorModel`
    std::string label;  // UI text, e.g. "Kraken X60"; also the MotorSpec name of every built-in row
    MotorSpec spec;     // one motor
};

namespace motors {

// Every motor of WPILib's DCMotor factory (allwpilib 2026 DCMotor.java, ONE motor each) in display order. The
// only copy of this data: core's MotorCatalog and every motor selector (wizard, editor) are built from it.
inline const std::vector<MotorPreset>& presets() {
    // (key, label, nominalV, stallTorqueNm, stallCurrentA, freeCurrentA, freeSpeedRpm)
    const auto row = [](const char* key, const char* label, double volts, double stallNm, double stallA, double freeA,
                        double freeRpm) {
        return MotorPreset{key, label, MotorSpec{label, volts, stallNm, stallA, freeA, rpmToRadPerSec(freeRpm)}};
    };
    static const std::vector<MotorPreset> table = {
        row("kraken_x60", "Kraken X60", 12, 7.09, 366, 2, 6000),
        row("kraken_x60_foc", "Kraken X60 (FOC)", 12, 9.37, 483, 2, 5800),
        row("kraken_x44", "Kraken X44", 12, 4.11, 279, 2, 7758),
        row("kraken_x44_foc", "Kraken X44 (FOC)", 12, 5.01, 329, 2, 7368),
        row("falcon_500", "Falcon 500", 12, 4.69, 257, 1.5, 6380),
        row("falcon_500_foc", "Falcon 500 (FOC)", 12, 5.84, 304, 1.5, 6080),
        row("minion", "Minion", 12, 3.17, 211, 2, 7704),
        row("neo", "NEO", 12, 2.6, 105, 1.8, 5676),
        row("neo_550", "NEO 550", 12, 0.97, 100, 1.4, 11000),
        row("neo_vortex", "NEO Vortex", 12, 3.60, 211, 3.6, 6784),
        row("cim", "CIM", 12, 2.42, 133, 2.7, 5310),
        row("mini_cim", "MiniCIM", 12, 1.41, 89, 3, 5840),
        row("bag", "Bag", 12, 0.43, 53, 1.8, 13180),
        row("vex_775pro", "775pro", 12, 0.71, 134, 0.7, 18730),
        row("andymark_rs775_125", "AndyMark RS775-125", 12, 0.28, 18, 1.6, 5800),
        row("banebots_rs775", "Banebots RS775", 12, 0.72, 97, 2.7, 13050),
        row("andymark_9015", "AndyMark 9015", 12, 0.36, 71, 3.7, 14270),
        row("banebots_rs550", "Banebots RS550", 12, 0.38, 84, 0.4, 19000),
        row("romi_built_in", "Romi built-in", 4.5, 0.1765, 1.25, 0.13, 150),
    };
    return table;
}

// The row with this key, or nullptr.
inline const MotorPreset* findPreset(std::string_view key) {
    for (const MotorPreset& p : presets())
        if (p.key == key) return &p;
    return nullptr;
}

// The row whose MotorSpec::name is `name` (how a saved wheel motor is matched to the table), or nullptr for a
// custom motor.
inline const MotorPreset* findPresetByName(std::string_view name) {
    for (const MotorPreset& p : presets())
        if (p.spec.name == name) return &p;
    return nullptr;
}

namespace detail {
inline MotorSpec specOf(std::string_view key) {
    const MotorPreset* p = findPreset(key);
    if (p == nullptr) contractViolation("motors: unknown built-in motor key");
    return p->spec;
}
}  // namespace detail

// Table rows used as defaults by SDK data types (WheelSpec, PlantSpec) and as test fixtures.
inline MotorSpec krakenX60() { return detail::specOf("kraken_x60"); }
inline MotorSpec falcon500() { return detail::specOf("falcon_500"); }
inline MotorSpec neo() { return detail::specOf("neo"); }
inline MotorSpec neo550() { return detail::specOf("neo_550"); }

}  // namespace motors

// Behavioural motor model (swappable: ideal DC, thermal, current-limited...). Pure & stateless:
// `omega` is the motor SHAFT speed (rad/s), torque/current are at the shaft, and a gearbox is
// the caller's job.
class IMotorModel {
    WPIEMU_INTERFACE(IMotorModel)
public:
    virtual const MotorSpec& spec() const = 0;
    virtual double currentA(double voltageV, double omegaRadPerSec) const = 0;
    virtual double torqueNm(double voltageV, double omegaRadPerSec) const = 0;

    double limitedCurrentA(double voltageV, double omegaRadPerSec, double statorLimitA) const {
        const double i = currentA(voltageV, omegaRadPerSec);
        return statorLimitA > 0.0 ? std::clamp(i, -statorLimitA, statorLimitA) : i;
    }
    double limitedTorqueNm(double voltageV, double omegaRadPerSec, double statorLimitA) const {
        return spec().ktNmPerAmp() * limitedCurrentA(voltageV, omegaRadPerSec, statorLimitA);
    }
    double limitedVoltageV(double voltageV, double omegaRadPerSec, double statorLimitA) const {
        if (statorLimitA <= 0.0) return voltageV;
        return omegaRadPerSec / spec().kvRadPerSecPerVolt() +
               spec().resistanceOhm() * limitedCurrentA(voltageV, omegaRadPerSec, statorLimitA);
    }
    double dampingNmS(double voltageV, double omegaRadPerSec, double statorLimitA = 0.0) const {
        if (statorLimitA > 0.0 && std::fabs(currentA(voltageV, omegaRadPerSec)) > statorLimitA) return 0.0;
        const double slope = torqueNm(voltageV, omegaRadPerSec + 1.0) - torqueNm(voltageV, omegaRadPerSec);
        return std::max(0.0, -slope);
    }
    double supplyCurrentA(double voltageV, double omegaRadPerSec, double statorLimitA = 0.0) const {
        const double nominal = std::max(1e-3, spec().nominalVoltageV);
        return std::max(0.0, voltageV * limitedCurrentA(voltageV, omegaRadPerSec, statorLimitA) / nominal);
    }
};

// Ideal brushed-DC model: I = (V - w/Kv) / R, tau = Kt * I.
class DcMotorModel final : public IMotorModel {
public:
    explicit DcMotorModel(MotorSpec spec) : spec_(std::move(spec)) {}
    const MotorSpec& spec() const override { return spec_; }
    double currentA(double voltageV, double omega) const override {
        return (voltageV - omega / spec_.kvRadPerSecPerVolt()) / spec_.resistanceOhm();
    }
    double torqueNm(double voltageV, double omega) const override {
        return spec_.ktNmPerAmp() * currentA(voltageV, omega);
    }

    using IMotorModel::limitedCurrentA;
    using IMotorModel::limitedTorqueNm;
    using IMotorModel::limitedVoltageV;

private:
    MotorSpec spec_;
};

// Output-shaft drive quantities reflected through a gearbox of `gearRatio` and `efficiency`.
struct ReflectedShaftDrive {
    double torqueNm = 0.0;
    double dampingNmS = 0.0;
    double inertiaKgM2 = 0.0;
    double supplyCurrentA = 0.0;
};

inline ReflectedShaftDrive reflectShaftDrive(double motorTorqueSumNm, double motorDampingSumNmS,
                                             double supplyCurrentSumA, int motorCount, double gearRatio,
                                             double efficiency, double rotorInertiaKgM2, double outputInertiaKgM2) {
    const double gear = std::max(gearRatio, 1e-6);
    const double eta = std::clamp(efficiency, 0.01, 1.0);
    return {
        eta * gear * motorTorqueSumNm,
        motorDampingSumNmS * eta * gear * gear,
        static_cast<double>(motorCount) * rotorInertiaKgM2 * gear * gear + outputInertiaKgM2,
        supplyCurrentSumA,
    };
}

inline ReflectedShaftDrive reflectSingleMotorShaftDrive(const IMotorModel& motor, double voltageV,
                                                        double shaftOmegaRadPerSec, double statorLimitA,
                                                        double gearRatio, double efficiency,
                                                        double rotorInertiaKgM2, double outputInertiaKgM2) {
    const double gear = std::max(gearRatio, 1e-6);
    const double motorOmega = gear * shaftOmegaRadPerSec;
    return reflectShaftDrive(motor.limitedTorqueNm(voltageV, motorOmega, statorLimitA),
                             motor.dampingNmS(voltageV, motorOmega, statorLimitA),
                             motor.supplyCurrentA(voltageV, motorOmega, statorLimitA), 1, gear, efficiency,
                             rotorInertiaKgM2, outputInertiaKgM2);
}

}  // namespace wpiemu::sdk
