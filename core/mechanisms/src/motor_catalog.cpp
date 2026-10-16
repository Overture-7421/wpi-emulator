#include "wpiemu/core/mechanisms/motor_catalog.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::mechanisms {

namespace {

using sdk::rpmToRadPerSec;

// Rows copy allwpilib 2026 DCMotor.java: (nominalV, stallTorqueNm, stallCurrentA, freeCurrentA, freeSpeedRpm).
MotorCatalogEntry row(const char* key, const char* label, double volts, double stallNm, double stallA,
                      double freeA, double freeRpm) {
    return {key, label, sdk::MotorSpec{label, volts, stallNm, stallA, freeA, rpmToRadPerSec(freeRpm)}};
}

}  // namespace

bool isValidMotorSpec(const sdk::MotorSpec& s) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    return positive(s.nominalVoltageV) && positive(s.stallTorqueNm) && positive(s.stallCurrentA) &&
           positive(s.freeSpeedRadPerSec) && std::isfinite(s.freeCurrentA) && s.freeCurrentA >= 0.0 &&
           s.freeCurrentA * s.resistanceOhm() < s.nominalVoltageV;
}

MotorCatalog MotorCatalog::builtin() {
    MotorCatalog c;
    const MotorCatalogEntry rows[] = {
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
    for (const MotorCatalogEntry& e : rows) (void)c.add(e);
    return c;
}

sdk::Status MotorCatalog::add(MotorCatalogEntry entry) {
    if (entry.key.empty()) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "motor key must not be empty");
    if (!isValidMotorSpec(entry.spec))
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "non-physical motor spec: " + entry.key);
    if (find(entry.key) != nullptr) return sdk::makeError(sdk::ErrorCode::AlreadyExists, "motor exists: " + entry.key);
    entries_.push_back(std::move(entry));
    return sdk::okStatus();
}

const MotorCatalogEntry* MotorCatalog::find(std::string_view key) const {
    for (const MotorCatalogEntry& e : entries_)
        if (e.key == key) return &e;
    return nullptr;
}

const sdk::MotorSpec* MotorCatalog::spec(std::string_view key) const {
    const MotorCatalogEntry* e = find(key);
    return e != nullptr ? &e->spec : nullptr;
}

std::vector<std::string> MotorCatalog::keys() const {
    std::vector<std::string> out;
    out.reserve(entries_.size());
    for (const MotorCatalogEntry& e : entries_) out.push_back(e.key);
    return out;
}

sdk::Result<std::unique_ptr<sdk::IMotorModel>> MotorCatalog::createModel(std::string_view key) const {
    const MotorCatalogEntry* e = find(key);
    if (e == nullptr) return sdk::makeError(sdk::ErrorCode::NotFound, "unknown motor: " + std::string(key));
    return std::unique_ptr<sdk::IMotorModel>(std::make_unique<sdk::DcMotorModel>(e->spec));
}

CurrentLimitedMotorModel::CurrentLimitedMotorModel(std::unique_ptr<sdk::IMotorModel> inner, double limitA)
    : inner_(std::move(inner)), limitA_(limitA) {
    if (!inner_) sdk::contractViolation("CurrentLimitedMotorModel: null inner model");
}

double CurrentLimitedMotorModel::currentA(double voltageV, double omega) const {
    const double i = inner_->currentA(voltageV, omega);
    return limitA_ > 0.0 ? std::clamp(i, -limitA_, limitA_) : i;
}

double CurrentLimitedMotorModel::torqueNm(double voltageV, double omega) const {
    return spec().ktNmPerAmp() * currentA(voltageV, omega);
}

double CurrentLimitedMotorModel::limitedVoltage(double voltageV, double omega) const {
    if (limitA_ <= 0.0) return voltageV;
    const double backEmf = omega / spec().kvRadPerSecPerVolt();
    return backEmf + spec().resistanceOhm() * currentA(voltageV, omega);
}

}  // namespace wpiemu::core::mechanisms
