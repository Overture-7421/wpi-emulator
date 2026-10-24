#include "wpiemu/bridges/vendor/ctre_native.h"

#include <bit>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>

#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/sdk/core/strings.h"

namespace wpiemu::bridges::vendor {

std::string_view ctreModelName(CtreDeviceType type) {
    switch (type) {
        case CtreDeviceType::P6_TalonFX: return "talon fx";
        case CtreDeviceType::P6_TalonFXS: return "talon fxs";
        case CtreDeviceType::P6_CANcoder: return "cancoder";
        case CtreDeviceType::P6_Pigeon2: return "pigeon 2";
        case CtreDeviceType::P6_CANdle: return "candle";
        case CtreDeviceType::P6_CANrange: return "canrange";
        case CtreDeviceType::P6_CANdi: return "candi";
        default: return {};
    }
}

std::optional<CtreDeviceType> ctreDeviceTypeFromDescriptorType(std::string_view t) {
    if (t == ctre::kTypeTalonFx) return CtreDeviceType::P6_TalonFX;
    if (t == ctre::kTypeTalonFxs) return CtreDeviceType::P6_TalonFXS;
    if (t == ctre::kTypeCanCoder) return CtreDeviceType::P6_CANcoder;
    if (t == ctre::kTypePigeon2) return CtreDeviceType::P6_Pigeon2;
    if (t == ctre::kTypeCanDle) return CtreDeviceType::P6_CANdle;
    if (t == ctre::kTypeCanRange) return CtreDeviceType::P6_CANrange;
    if (t == ctre::kTypeCanDi) return CtreDeviceType::P6_CANdi;
    if (t == ctre::kTypeTalonSrx) return CtreDeviceType::TalonSRX;
    if (t == ctre::kTypeVictorSpx) return CtreDeviceType::VictorSPX;
    if (t == ctre::kTypePigeonImu) return CtreDeviceType::PigeonIMU;
    return std::nullopt;
}

namespace {

const std::set<int>& floatSpns() {
    static const std::set<int> kFloat{spn::kCancoderMagnetOffset,    spn::kStatorCurrentLimit,
                                      spn::kSupplyCurrentLimit,      spn::kPeakForwardDutyCycle,
                                      spn::kPeakReverseDutyCycle,    spn::kPeakForwardVoltage,
                                      spn::kPeakReverseVoltage,      spn::kSensorToMechanismRatio,
                                      spn::kRotorToSensorRatio};
    return kFloat;
}

}  // namespace

CtreConfigSnapshot CtreConfigSnapshot::parse(std::string_view text) {
    CtreConfigSnapshot snap;
    while (!text.empty()) {
        const auto nl = text.find('\n');
        std::string_view line = sdk::asciiTrim(text.substr(0, nl));
        text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
        const auto comma = line.find(',');
        if (comma == std::string_view::npos) continue;
        const std::optional<int> id = sdk::parseInteger<int>(sdk::asciiTrim(line.substr(0, comma)));
        if (!id) continue;
        snap.raw_[*id] = std::string(sdk::asciiTrim(line.substr(comma + 1)));
    }
    return snap;
}

std::optional<double> CtreConfigSnapshot::number(int spnId) const {
    const auto it = raw_.find(spnId);
    if (it == raw_.end()) return std::nullopt;
    std::string_view token = it->second;
    if (token.starts_with("f_")) {
        token.remove_prefix(2);
        const std::string text(token);  // from_chars(double) is missing in older libc++
        char* end = nullptr;
        const double d = std::strtod(text.c_str(), &end);
        if (end != text.c_str() && *end == '\0') return d;
        return std::nullopt;
    }
    if (token.starts_with("s_")) token.remove_prefix(2);
    if (floatSpns().count(spnId) != 0) {
        std::uint32_t bits = 0;
        if (token.starts_with('-')) {
            const std::optional<std::int32_t> signedBits = sdk::parseInteger<std::int32_t>(token);
            if (!signedBits) return std::nullopt;
            bits = static_cast<std::uint32_t>(*signedBits);
        } else if (const std::optional<std::uint32_t> unsignedBits = sdk::parseInteger<std::uint32_t>(token)) {
            bits = *unsignedBits;
        } else {
            return std::nullopt;
        }
        return static_cast<double>(std::bit_cast<float>(bits));
    }
    if (const std::optional<long long> n = sdk::parseInteger<long long>(token)) return static_cast<double>(*n);
    return std::nullopt;
}

std::optional<int> CtreConfigSnapshot::integer(int spnId) const {
    const auto v = number(spnId);
    if (!v) return std::nullopt;
    return static_cast<int>(*v);
}

std::optional<bool> CtreConfigSnapshot::clockwisePositive() const {
    const auto inverted = integer(spn::kInverted);
    if (inverted) return *inverted == 1;
    const auto direction = integer(spn::kCancoderSensorDirection);
    if (direction) return *direction == 1;
    return std::nullopt;
}

std::string CtreConfigSnapshot::summary() const {
    std::string out;
    const auto add = [&out](const char* label, const std::string& value) {
        if (!out.empty()) out += ' ';
        out += label;
        out += '=';
        out += value;
    };
    const auto fmt = [](double d) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%g", d);
        return std::string(buf);
    };
    if (const auto cw = clockwisePositive()) add("cw+", *cw ? "true" : "false");
    if (const auto v = sensorToMechanismRatio()) add("sensorToMechanism", fmt(*v));
    if (const auto v = rotorToSensorRatio()) add("rotorToSensor", fmt(*v));
    if (const auto v = feedbackSensorSource()) add("feedbackSource", std::to_string(*v));
    if (const auto v = remoteSensorId()) add("remoteSensorId", std::to_string(*v));
    if (const auto v = magnetOffsetRot()) add("magnetOffsetRot", fmt(*v));
    if (const auto v = forwardLimitType()) add("fwdLimitType", std::to_string(*v));
    if (const auto v = reverseLimitType()) add("revLimitType", std::to_string(*v));
    if (out.empty()) out = std::to_string(raw_.size()) + " config entries";
    return out;
}

CtreSimNative::CtreSimNative(std::shared_ptr<const ISymbolResolver> resolver) : resolver_(std::move(resolver)) {
    if (!resolver_) return;
    encodeDevice_ = reinterpret_cast<EncodeDeviceFn>(resolver_->resolve("c_ctre_phoenix6_encode_device"));
    getConfigs_ = reinterpret_cast<GetConfigsNoMallocFn>(resolver_->resolve("c_ctre_phoenix6_get_configs_no_malloc"));
    getPhysics_ = reinterpret_cast<GetPhysicsFn>(resolver_->resolve("c_ctre_phoenix6_platform_sim_get_physics_value"));
    setPhysics_ = reinterpret_cast<SetPhysicsFn>(resolver_->resolve("c_ctre_phoenix6_platform_sim_set_physics_input"));
    getSignal_ = reinterpret_cast<GetSignalSimplifiedFn>(resolver_->resolve("c_ctre_phoenix6_get_signal_simplified"));
}

bool ctreControlModeRegulatesCurrent(int mode) {
    namespace cm = control_mode;
    return (mode >= cm::kDutyCycleFOC && mode <= cm::kMotionMagicDutyCycleFOC) ||
           (mode >= cm::kVoltageFOC && mode <= cm::kMotionMagicTorqueCurrentFOC) ||
           mode == cm::kMotionMagicVelocityDutyCycleFOC || mode == cm::kMotionMagicVelocityVoltageFOC ||
           mode == cm::kMotionMagicVelocityTorqueCurrentFOC || mode == cm::kMotionMagicExpoDutyCycleFOC ||
           mode == cm::kMotionMagicExpoVoltageFOC || mode == cm::kMotionMagicExpoTorqueCurrentFOC;
}

sdk::Result<double> CtreSimNative::readSignal(CtreDeviceType type, int id, std::string_view bus, int spn) const {
    if (!hasSignalRead()) return sdk::makeError(sdk::ErrorCode::Unsupported, "CTRE signal API not available");
    const std::string_view model = ctreModelName(type);
    if (model.empty()) return sdk::makeError(sdk::ErrorCode::Unsupported, "no signal API for this CTRE device type");
    const std::string modelStr(model);
    const std::string busStr(bus.empty() ? std::string_view("rio") : bus);
    std::uint32_t hash = 0;
    const int encoded = encodeDevice_(id, modelStr.c_str(), busStr.c_str(), &hash);
    if (encoded < 0) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "encode_device failed: " + std::to_string(encoded));
    const auto spnId = static_cast<std::uint32_t>(spn);
    double value = 0.0, hwTimestamp = 0.0, swTimestamp = 0.0, ecuTimestamp = 0.0;
    // No wait, zero timeout: the latest value the firmware sim published, never a block on the robot thread.
    const int status = getSignal_(busStr.c_str(), 0, 0.0, 1, &hash, &spnId, &value, &hwTimestamp, &swTimestamp,
                                  &ecuTimestamp);
    if (status < 0) return sdk::makeError(sdk::ErrorCode::IoError, "get_signal failed: " + std::to_string(status));
    return value;
}

sdk::Result<std::string> CtreSimNative::readConfigs(CtreDeviceType type, int id, std::string_view bus) const {
    if (!hasConfigReadback()) return sdk::makeError(sdk::ErrorCode::Unsupported, "CTRE config API not available");
    const std::string_view model = ctreModelName(type);
    if (model.empty()) return sdk::makeError(sdk::ErrorCode::Unsupported, "no config API for this CTRE device type");
    const std::string modelStr(model);
    const std::string busStr(bus.empty() ? std::string_view("rio") : bus);

    std::uint32_t hash = 0;
    const int encoded = encodeDevice_(id, modelStr.c_str(), busStr.c_str(), &hash);
    if (encoded < 0) return sdk::makeError(sdk::ErrorCode::InvalidArgument, "encode_device failed: " + std::to_string(encoded));

    std::string buffer(64 * 1024, '\0');
    const int status = getConfigs_(0, busStr.c_str(), static_cast<int>(hash), configTimeoutSeconds, buffer.data(),
                                   static_cast<std::uint32_t>(buffer.size()), false);
    if (status < 0) return sdk::makeError(sdk::ErrorCode::IoError, "get_configs failed: " + std::to_string(status));
    buffer.resize(std::strlen(buffer.c_str()));
    if (buffer.empty()) return sdk::makeError(sdk::ErrorCode::NotFound, "device returned no configuration");
    return buffer;
}

int CtreSimNative::getPhysicsValue(CtreDeviceType type, int id, const char* key, double* value) const {
    if (getPhysics_ == nullptr) return kCtreNativeUnavailable;
    return getPhysics_(static_cast<int>(type), id, key, value);
}

int CtreSimNative::setPhysicsInput(CtreDeviceType type, int id, const char* key, double value) {
    if (setPhysics_ == nullptr) return kCtreNativeUnavailable;
    return setPhysics_(static_cast<int>(type), id, key, value);
}

std::shared_ptr<ICtreSimNative> makeCtreSimNative(std::shared_ptr<const ISymbolResolver> resolver) {
    if (!resolver) {
        std::unique_ptr<ModuleSymbolResolver> module =
            ModuleSymbolResolver::open(ctreSimModuleName(), ModuleLoadMode::AttachOnly);
        if (!module) return nullptr;
        resolver = std::move(module);
    }
    auto native = std::make_shared<CtreSimNative>(std::move(resolver));
    if (!native->hasConfigReadback() && !native->hasPhysicsApi() && !native->hasSignalRead()) return nullptr;
    return native;
}

}  // namespace wpiemu::bridges::vendor
