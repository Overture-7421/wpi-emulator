#pragma once

// Motor data table + motor models.
//
// The built-in rows are the SDK's motor table (sdk::motors::presets()): the same numbers as WPILib's `DCMotor`
// factory methods (allwpilib 2026, ONE motor each), defined once for every module. Entries are looked up by a
// stable snake_case key that is also the persisted value of the `motorModel` mechanism property.
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

// key, UI label and the one-motor spec.
using MotorCatalogEntry = sdk::MotorPreset;

class MotorCatalog {
public:
    // The SDK motor table, in its order.
    static MotorCatalog builtin();

    // AlreadyExists for a duplicate key, InvalidArgument for an empty key / non-physical spec
    // (sdk::isValidMotorSpec).
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
