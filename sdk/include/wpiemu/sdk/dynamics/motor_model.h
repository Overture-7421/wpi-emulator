#pragma once

// 1-DOF motor model: data compatible with WPILib's DCMotor (but no WPILib dependency in the SDK;
// core may convert to/from frc::DCMotor).

#include <cmath>
#include <string>
#include <utility>

#include "wpiemu/sdk/core/interface.h"
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

// Motors with the same data as WPILib's DCMotor catalog (values from allwpilib 2026).
namespace motors {
inline MotorSpec krakenX60() { return {"Kraken X60", 12.0, 7.09, 366.0, 2.0, rpmToRadPerSec(6000.0)}; }
inline MotorSpec falcon500() { return {"Falcon 500", 12.0, 4.69, 257.0, 1.5, rpmToRadPerSec(6380.0)}; }
inline MotorSpec neo() { return {"NEO", 12.0, 2.6, 105.0, 1.8, rpmToRadPerSec(5676.0)}; }
inline MotorSpec neo550() { return {"NEO 550", 12.0, 0.97, 100.0, 1.4, rpmToRadPerSec(11000.0)}; }
inline MotorSpec vortex() { return {"NEO Vortex", 12.0, 3.6, 211.0, 3.6, rpmToRadPerSec(6784.0)}; }
inline MotorSpec cim() { return {"CIM", 12.0, 2.42, 133.0, 2.7, rpmToRadPerSec(5330.0)}; }
inline MotorSpec miniCim() { return {"MiniCIM", 12.0, 1.41, 89.0, 3.0, rpmToRadPerSec(5840.0)}; }
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

private:
    MotorSpec spec_;
};

}  // namespace wpiemu::sdk
