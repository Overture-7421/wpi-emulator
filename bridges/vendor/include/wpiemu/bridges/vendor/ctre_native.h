#pragma once

// CTRE native supplement (docs/reference/bridges.md extension seam). Everything here is OPTIONAL:
//  * L2: CTRE's public C sim ABI exported by CTRE_PhoenixTools_Sim (`c_ctre_phoenix6_platform_sim_*`,
//    `c_ctre_phoenix6_get_configs_no_malloc`, `c_ctre_phoenix6_encode_device`). Bound lazily through an
//    ISymbolResolver (attach-only: the module instance the robot already loaded). Every symbol may be absent;
//    all calls then fail soft. Only READ-ONLY config read-back is used by default; the physics get/set calls
//    are exposed for opt-in features but are never invoked by the shipped bridge.
//  * L3: inline hooks on CTRE binaries (bus names, SimCreate for non-SimDevice classes, control requests) are
//    NOT implemented here (CTRE license forbids modifying their binaries; docs/reference/bridges.md). ICtreHookProvider is
//    only the extension interface an out-of-tree/opt-in module can implement to feed observations.

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/bridges/vendor/symbol_resolver.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::bridges::vendor {

// ctre::phoenix::platform::DeviceType (platform/DeviceType.hpp).
enum class CtreDeviceType : int {
    TalonSRX = 0,
    VictorSPX = 1,
    PigeonIMU = 2,
    RibbonPigeonIMU = 3,
    P6_TalonFX = 4,
    P6_CANcoder = 5,
    P6_Pigeon2 = 6,
    P6_TalonFXS = 7,
    P6_CANrange = 8,
    P6_CANdi = 9,
    P6_CANdle = 10,
};

// Model string for c_ctre_phoenix6_encode_device ("talon fx", "cancoder", ...). Empty: no config API for it.
std::string_view ctreModelName(CtreDeviceType type);
// Maps the descriptor type strings used by our rule tables ("talonfx", "pigeon2", ...) to a CTRE device type.
std::optional<CtreDeviceType> ctreDeviceTypeFromDescriptorType(std::string_view descriptorType);

constexpr int kCtreSimDeviceNotFound = 1200;
constexpr int kCtreSimPhysicsTypeNotSupported = 1201;
constexpr int kCtreNativeUnavailable = -1;  // our own code: symbol/module absent

// TalonFXSimState::SetMotorType, as the firmware sim takes it: the physics input `MotorType` with these values
// (sim/TalonFXSimState.hpp, 26.3.0). Without it the firmware sim models a Kraken X60 for every TalonFX, whatever
// is on the robot, and publishes that motor's constants (MotorKV, MotorKT, MotorStallCurrent).
inline constexpr const char* kPhysicsMotorType = "MotorType";
namespace sim_motor_type {
inline constexpr int kKrakenX60 = 0;
inline constexpr int kKrakenX44 = 1;
}  // namespace sim_motor_type

// The firmware sim's motor type for the motor the plant simulates; nullopt when the sim cannot represent it
// (a Falcon 500 or a Minion stays on the firmware's default) or the family is unknown.
std::optional<int> ctreSimMotorTypeFor(sdk::MotorFamily family);

// SPN ids used by the config read-back (ctre/phoenix6/spns/SpnValue.hpp, verified against 26.3.0).
namespace spn {
inline constexpr int kCancoderSensorDirection = 828;
inline constexpr int kCancoderMagnetOffset = 987;
inline constexpr int kInverted = 1901;
inline constexpr int kNeutralMode = 1904;
inline constexpr int kStatorCurrentLimit = 1906;
inline constexpr int kStatorCurrentLimitEnable = 1907;
inline constexpr int kSupplyCurrentLimit = 1908;
inline constexpr int kSupplyCurrentLimitEnable = 1909;
inline constexpr int kPeakForwardDutyCycle = 1910;
inline constexpr int kPeakReverseDutyCycle = 1911;
inline constexpr int kPeakForwardVoltage = 1912;
inline constexpr int kPeakReverseVoltage = 1913;
inline constexpr int kSensorToMechanismRatio = 1918;
inline constexpr int kRotorToSensorRatio = 1919;
inline constexpr int kFeedbackSensorSource = 1920;
inline constexpr int kFeedbackRemoteSensorId = 1921;
inline constexpr int kForwardLimitType = 1928;
inline constexpr int kReverseLimitType = 1934;
}  // namespace spn

// Parsed `<spn>,<token>\n` lines of c_ctre_phoenix6_get_configs. Tokens: `s_<n>` (serialized: IEEE-754 float bits
// for float SPNs, integer otherwise) or `f_<decimal>`. Unknown SPNs stay available as raw text.
class CtreConfigSnapshot {
public:
    static CtreConfigSnapshot parse(std::string_view text);

    bool empty() const { return raw_.empty(); }
    std::size_t size() const { return raw_.size(); }
    const std::map<int, std::string>& raw() const { return raw_; }

    std::optional<double> number(int spnId) const;  // float SPNs decoded, integer/bool SPNs as a double
    std::optional<int> integer(int spnId) const;

    // Typed views. Enumerations stay raw ints (see the CTRE docs): InvertedValue 0 = CCW+, 1 = CW+;
    // SensorDirectionValue 0 = CCW+, 1 = CW+; FeedbackSensorSourceValue 0 = RotorSensor, 1 = RemoteCANcoder, ...
    std::optional<bool> clockwisePositive() const;
    std::optional<double> sensorToMechanismRatio() const { return number(spn::kSensorToMechanismRatio); }
    std::optional<double> rotorToSensorRatio() const { return number(spn::kRotorToSensorRatio); }
    std::optional<int> feedbackSensorSource() const { return integer(spn::kFeedbackSensorSource); }
    std::optional<int> remoteSensorId() const { return integer(spn::kFeedbackRemoteSensorId); }
    std::optional<double> magnetOffsetRot() const { return number(spn::kCancoderMagnetOffset); }
    std::optional<int> forwardLimitType() const { return integer(spn::kForwardLimitType); }  // 0 = NormallyOpen
    std::optional<int> reverseLimitType() const { return integer(spn::kReverseLimitType); }

    // One human-readable line for logs ("inverted=CW+ s2m=1.6667 r2s=1 ...").
    std::string summary() const;

private:
    std::map<int, std::string> raw_;
};

class ICtreSimNative {
    WPIEMU_INTERFACE(ICtreSimNative)
public:
    virtual bool hasConfigReadback() const = 0;
    virtual bool hasPhysicsApi() const = 0;
    // Read-only. Error when the API is absent, the device model is unknown or CTRE reports a failure.
    virtual sdk::Result<std::string> readConfigs(CtreDeviceType type, int id, std::string_view bus) const = 0;
    // CTRE return code (0 = OK, 1200 device missing, 1201 key unsupported) or kCtreNativeUnavailable.
    virtual int getPhysicsValue(CtreDeviceType type, int id, const char* key, double* value) const = 0;
    virtual int setPhysicsInput(CtreDeviceType type, int id, const char* key, double value) = 0;
};

// Binds the exports lazily through an ISymbolResolver. Safe to construct with a null/absent resolver.
class CtreSimNative final : public ICtreSimNative {
public:
    explicit CtreSimNative(std::shared_ptr<const ISymbolResolver> resolver);

    bool hasConfigReadback() const override { return encodeDevice_ != nullptr && getConfigs_ != nullptr; }
    bool hasPhysicsApi() const override { return getPhysics_ != nullptr && setPhysics_ != nullptr; }
    sdk::Result<std::string> readConfigs(CtreDeviceType type, int id, std::string_view bus) const override;
    int getPhysicsValue(CtreDeviceType type, int id, const char* key, double* value) const override;
    int setPhysicsInput(CtreDeviceType type, int id, const char* key, double value) override;

    double configTimeoutSeconds = 0.25;

private:
    using EncodeDeviceFn = int (*)(int, const char*, const char*, std::uint32_t*);
    using GetConfigsNoMallocFn = int (*)(int, const char*, int, double, char*, std::uint32_t, bool);
    using GetPhysicsFn = std::int32_t (*)(int, int, const char*, double*);
    using SetPhysicsFn = std::int32_t (*)(int, int, const char*, double);

    std::shared_ptr<const ISymbolResolver> resolver_;
    EncodeDeviceFn encodeDevice_ = nullptr;
    GetConfigsNoMallocFn getConfigs_ = nullptr;
    GetPhysicsFn getPhysics_ = nullptr;
    SetPhysicsFn setPhysics_ = nullptr;
};

// Attach-only native supplement for the already-loaded CTRE module; absent module => nullptr.
std::shared_ptr<ICtreSimNative> makeCtreSimNative(std::shared_ptr<const ISymbolResolver> resolver = nullptr);

// ---- L3 extension seam (interface only) ---------------------------------------------------------------------

struct CtreDeviceObservation {
    CtreDeviceType type = CtreDeviceType::P6_TalonFX;
    int id = -1;
    std::string bus;  // e.g. "rio", "canivore", "*" (the constructor argument as the robot wrote it)
};

// Receives observations from a hook provider. May be called from ANY thread (robot threads); never blocks.
class ICtreObservationSink {
    WPIEMU_INTERFACE(ICtreObservationSink)
public:
    virtual void onDeviceObserved(const CtreDeviceObservation& observation) = 0;
};

// Implemented by an out-of-tree/opt-in module that is able to observe CTRE constructor calls (e.g. via inline
// hooks on `c_ctre_phoenix6_encode_device`). Installed only when the session enabled optional hooks
// (IBridgeHost::enableOptionalHooks). This repository ships NO implementation (R7).
class ICtreHookProvider {
    WPIEMU_INTERFACE(ICtreHookProvider)
public:
    virtual std::string_view name() const = 0;
    virtual sdk::Status install(ICtreObservationSink& sink) = 0;
    virtual void uninstall() noexcept = 0;
};

}  // namespace wpiemu::bridges::vendor
