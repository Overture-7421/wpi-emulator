#pragma once

// Motor data table + motor models.
//
// The catalog carries the same numbers as WPILib's `DCMotor` factory methods (allwpilib 2026,
// wpimath `edu.wpi.first.math.system.plant.DCMotor`, ONE motor each); see docs/mechanisms.md for
// the per-motor source lines. Entries are looked up by a stable snake_case key that is also the
// persisted value of the `motorModel` mechanism property.
//
// A catalog is a plain value (no global registry): the composition root owns one and hands it
// (shared) to the mechanism templates; tests build their own.

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::core::mechanisms {

struct MotorCatalogEntry {
    std::string key;     // stable id, e.g. "kraken_x60"
    std::string label;   // UI text, e.g. "Kraken X60"
    sdk::MotorSpec spec; // one motor
};

class MotorCatalog {
public:
    // Every motor of WPILib's DCMotor factory (see motor_catalog.cpp for the source of each row).
    static MotorCatalog builtin();

    // AlreadyExists for a duplicate key, InvalidArgument for an empty key / non-physical spec.
    sdk::Status add(MotorCatalogEntry entry);

    const MotorCatalogEntry* find(std::string_view key) const;
    // Spec of `key`, or nullptr when unknown.
    const sdk::MotorSpec* spec(std::string_view key) const;
    std::vector<std::string> keys() const;
    const std::vector<MotorCatalogEntry>& entries() const { return entries_; }

    // Ideal DC model (DcMotorModel) of `key`; NotFound when unknown.
    sdk::Result<std::unique_ptr<sdk::IMotorModel>> createModel(std::string_view key) const;

private:
    std::vector<MotorCatalogEntry> entries_;
};

// True when every field is finite and physical (positive current/torque/speed/voltage, free current
// below stall current).
bool isValidMotorSpec(const sdk::MotorSpec& spec);

// Decorator: a stator-current limit on any IMotorModel, the way a smart motor controller limits
// current: |I| <= limitA, the torque follows I (tau = Kt I). limitA <= 0 disables the limit.
class CurrentLimitedMotorModel final : public sdk::IMotorModel {
public:
    CurrentLimitedMotorModel(std::unique_ptr<sdk::IMotorModel> inner, double limitA);

    const sdk::MotorSpec& spec() const override { return inner_->spec(); }
    double currentA(double voltageV, double omegaRadPerSec) const override;
    double torqueNm(double voltageV, double omegaRadPerSec) const override;

    double limitA() const { return limitA_; }
    // The voltage that actually produces the limited current: omega/Kv + R * I_limited.
    double limitedVoltage(double voltageV, double omegaRadPerSec) const;

private:
    std::unique_ptr<sdk::IMotorModel> inner_;
    double limitA_;
};

}  // namespace wpiemu::core::mechanisms
