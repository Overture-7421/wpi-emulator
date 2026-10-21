#include "wpiemu/core/mechanisms/motor_catalog.h"

#include <algorithm>

namespace wpiemu::core::mechanisms {

MotorCatalog MotorCatalog::builtin() {
    MotorCatalog c;
    for (const MotorCatalogEntry& e : sdk::motors::presets()) (void)c.add(e);
    return c;
}

sdk::Status MotorCatalog::add(MotorCatalogEntry entry) {
    if (entry.key.empty()) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "motor key must not be empty");
    if (!sdk::isValidMotorSpec(entry.spec))
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
    return inner_->limitedCurrentA(voltageV, omega, limitA_);
}

double CurrentLimitedMotorModel::torqueNm(double voltageV, double omega) const {
    return inner_->limitedTorqueNm(voltageV, omega, limitA_);
}

double CurrentLimitedMotorModel::limitedVoltage(double voltageV, double omega) const {
    return inner_->limitedVoltageV(voltageV, omega, limitA_);
}

}  // namespace wpiemu::core::mechanisms
