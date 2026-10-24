#include "wpiemu/bridges/vendor/vendor_sim_bridge.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::bridges::vendor {

namespace {
constexpr const char* kCategory = "bridge.vendor";
constexpr int kNativeRetryPolls = 50;
constexpr std::size_t kMaxPendingObservations = 256;
}  // namespace

// Forwards to the real host and records what this bridge announced (needed for config read-back, bus
// observations and the time-sync advice).
class VendorSimDeviceBridge::RecordingHost final : public sdk::IBridgeHost {
public:
    RecordingHost(VendorSimDeviceBridge& owner, sdk::IBridgeHost& real) : owner_(&owner), real_(&real) {}

    sdk::DeviceId announce(const sdk::DeviceDescriptor& descriptor, std::vector<sdk::SignalSpec> signalSpecs) override {
        std::optional<std::uint32_t> position, velocity, torqueReported, torqueCommand;
        for (std::size_t i = 0; i < signalSpecs.size(); ++i) {
            if (signalSpecs[i].name == sdk::signal::kPosition)
                position = static_cast<std::uint32_t>(i);
            if (signalSpecs[i].name == sdk::signal::kVelocity)
                velocity = static_cast<std::uint32_t>(i);
            if (signalSpecs[i].name == ctre::vsignal::kTorqueCurrentReported)
                torqueReported = static_cast<std::uint32_t>(i);
            if (signalSpecs[i].name == sdk::signal::kTorqueCurrent)
                torqueCommand = static_cast<std::uint32_t>(i);
        }
        const bool phoenixMotor = descriptor.family == ctre::kFamilyPhoenix6 &&
                                  (descriptor.type == ctre::kTypeTalonFx || descriptor.type == ctre::kTypeTalonFxs);
        if (phoenixMotor && torqueReported && !torqueCommand) {
            // Declared here so the hub can index it; appendTorqueCommands fills it every tick: the firmware's
            // torque current in the FOC control modes, 0 in every other mode (the voltage path).
            sdk::SignalSpec spec;
            spec.name = sdk::signal::kTorqueCurrent;
            spec.direction = sdk::SignalDirection::Output;
            spec.type = sdk::SignalType::Double;
            spec.unit = "A";
            torqueCommand = static_cast<std::uint32_t>(signalSpecs.size());
            signalSpecs.push_back(std::move(spec));
        }
        const sdk::DeviceId id = real_->announce(descriptor, std::move(signalSpecs));
        if (id.valid()) {
            auto& announced = owner_->announced_[id];
            announced.descriptor = descriptor;
            announced.positionSignal = position;
            announced.velocitySignal = velocity;
            announced.torqueReportedSignal = torqueReported;
            announced.torqueCommandSignal = torqueCommand;
            if (descriptor.type == ctre::kTypeCanCoder && !owner_->options_.nativeProvider &&
                !announced.configDone) {
                announced.configDone = true;
                real_->logger().warn(
                    kCategory,
                    descriptor.key().toString() +
                        ": config read-back disabled; encoder inputs remain uncalibrated");
            }
        }
        return id;
    }
    void retire(sdk::DeviceId id) override {
        owner_->announced_.erase(id);
        owner_->configs_.erase(id);
        owner_->disabled_.erase(id);
        real_->retire(id);
    }
    sdk::ILogger& logger() override { return real_->logger(); }
    bool enableOptionalHooks() const override { return real_->enableOptionalHooks(); }
    void observeBus(sdk::DeviceId id, std::string_view busLabel) override { real_->observeBus(id, busLabel); }

private:
    VendorSimDeviceBridge* owner_;
    sdk::IBridgeHost* real_;
};

VendorSimDeviceBridge::VendorSimDeviceBridge(std::string name, std::vector<sdk::SimDeviceRule> rules,
                                             std::shared_ptr<sdk::ISimDeviceBackend> backend,
                                             VendorSimBridgeOptions options)
    : backend_(backend ? std::move(backend) : std::make_shared<NullSimDeviceBackend>()),
      options_(std::move(options)),
      incremental_(*backend_),
      guarded_(incremental_, options_.guard),
      inner_(std::move(name), std::move(rules), guarded_) {}

VendorSimDeviceBridge::~VendorSimDeviceBridge() { stop(); }

sdk::Status VendorSimDeviceBridge::start(sdk::IBridgeHost& host) {
    host_ = &host;
    recording_ = std::make_unique<RecordingHost>(*this, host);
    guarded_.setLogger(&host.logger());
    if (!options_.backendAvailable)
        host.logger().warn(kCategory, std::string(inner_.name()) +
                                          ": no HAL SimDevice backend available (WPILib binding not compiled in); "
                                          "no vendor devices will be discovered");
    sdk::Status st = inner_.start(*recording_);
    if (!st) {
        stop();
        return st;
    }
    if (options_.hooks && host.enableOptionalHooks()) {
        sdk::Status hooked = options_.hooks->install(*this);
        if (hooked) {
            hooksInstalled_ = true;
        } else {
            host.logger().warn(kCategory, "optional CTRE hook provider '" + std::string(options_.hooks->name()) +
                                              "' failed to install: " + hooked.error().message);
        }
    }
    return sdk::okStatus();
}

void VendorSimDeviceBridge::poll() {
    // May run while the robot constructor is still creating devices (live discovery): HAL SimDevice enumeration
    // only. Nothing here may enter CTRE's native sim (see readPendingConfigs).
    if (host_ == nullptr) return;
    inner_.poll();
    drainObservations();
}

void VendorSimDeviceBridge::collectOutputs(std::vector<sdk::SignalSample>& out) {
    const std::size_t first = out.size();
    inner_.collectOutputs(out);
    // Both below enter CTRE's native sim; collectOutputs is only called while the robot is quiescent.
    appendTorqueCommands(out, first);
    readOneConfig();
}

bool VendorSimDeviceBridge::ensureNative() {
    if (native_) return true;
    if (!options_.nativeProvider) return false;
    if (nativeRetryIn_ > 0) {
        --nativeRetryIn_;
        return false;
    }
    native_ = options_.nativeProvider();
    if (!native_) {  // CTRE module not loaded (yet): look again later
        nativeRetryIn_ = kNativeRetryPolls;
        return false;
    }
    return true;
}

// Phoenix 6 FOC. In the FOC control modes the firmware regulates torque current; the voltage it reports is the
// one ITS motor model needs to pass that current at the rotor speed we fed it. Applying that voltage to the
// motor the emulator simulates lets two models disagree about back-EMF, and a swerve steer module ran away to
// free speed while its controller asked for the opposite sign. In those modes the plant gets the firmware's
// torque current (MotorOutput::hasTorqueCurrent, resolved against our own motor); in every other mode the
// canonical torqueCurrent carries 0 and the voltage path stays as documented. The mode is a status signal read
// natively (L2, read-only). Without that API the voltage path is kept and said once, at debug level.
void VendorSimDeviceBridge::appendTorqueCommands(std::vector<sdk::SignalSample>& out, std::size_t first) {
    if (host_ == nullptr) return;
    const bool native = ensureNative();
    for (auto& [id, a] : announced_) {
        if (!a.torqueCommandSignal || !a.torqueReportedSignal) continue;
        if (native && !a.controlModeUnavailable) {
            const auto type = ctreDeviceTypeFromDescriptorType(a.descriptor.type);
            if (!type || !native_->hasSignalRead()) {
                a.controlModeUnavailable = true;
                host_->logger().debug(kCategory, a.descriptor.key().toString() +
                                                     ": control mode unavailable; FOC requests are applied as voltage");
            } else {
                const sdk::Result<double> mode =
                    native_->readSignal(*type, a.descriptor.id, options_.configBus, spn::kTalonFxControlMode);
                if (mode) {
                    a.controlMode = static_cast<int>(mode.value());
                } else if (mode.error().code == sdk::ErrorCode::Unsupported) {
                    a.controlModeUnavailable = true;
                    host_->logger().debug(kCategory, a.descriptor.key().toString() + ": control mode unavailable: " +
                                                         mode.error().message + "; FOC requests are applied as voltage");
                }
                // Any other failure keeps the last mode for this tick.
            }
        }
        double command = 0.0;
        if (ctreControlModeRegulatesCurrent(a.controlMode)) {
            for (std::size_t i = out.size(); i-- > first;) {
                if (out[i].device == id && out[i].signal == *a.torqueReportedSignal) {
                    command = out[i].value;
                    break;
                }
            }
        }
        out.push_back({id, static_cast<std::uint16_t>(*a.torqueCommandSignal), command});
    }
}

std::size_t VendorSimDeviceBridge::readPendingConfigs(std::size_t maxDevices) {
    std::size_t n = 0;
    while (n < maxDevices && readOneConfig()) ++n;
    return n;
}

void VendorSimDeviceBridge::applyInputs(std::span<const sdk::SignalSample> samples) {
    guarded_.beginWrites();
    filtered_.clear();
    for (auto sample : samples) {
        if (disabled_.contains(sample.device)) continue;
        const auto device = announced_.find(sample.device);
        const auto config = configs_.find(sample.device);
        if (device != announced_.end() && device->second.descriptor.type == ctre::kTypeCanCoder &&
            config != configs_.end()) {
            const auto offset = config->second.magnetOffsetRot();
            const auto clockwise = config->second.clockwisePositive();
            if (offset && std::isfinite(*offset) && clockwise) {
                const double sign = *clockwise ? -1.0 : 1.0;
                // Firmware reports sign * raw + magnetOffset. Slots carry the calibrated model
                // frame.
                if (device->second.positionSignal == sample.signal)
                    sample.value = sign * (sample.value - *offset);
                if (device->second.velocitySignal == sample.signal) sample.value *= sign;
            }
        }
        filtered_.push_back(sample);
    }
    inner_.applyInputs(filtered_);
    guarded_.endWrites();
}

void VendorSimDeviceBridge::applyLoopback(sdk::DeviceId id) {
    if (disabled_.count(id) == 0) inner_.applyLoopback(id);
}

void VendorSimDeviceBridge::stop() noexcept {
    if (hooksInstalled_ && options_.hooks) {
        options_.hooks->uninstall();
        hooksInstalled_ = false;
    }
    inner_.stop();
    guarded_.setLogger(nullptr);
    recording_.reset();
    host_ = nullptr;
    announced_.clear();
    configs_.clear();
    disabled_.clear();
    std::lock_guard lock(observationMutex_);
    observations_.clear();
}

TimeSyncAdvice VendorSimDeviceBridge::timeSyncAdvice() const {
    if (options_.timeSync == TimeSyncPolicy::Ignore || announced_.empty()) return {};
    TimeSyncAdvice advice;
    advice.paceToWallClock = true;
    advice.maxRealtimeFactor = 1.0;
    advice.pauseUnsafe = true;
    advice.reason =
        "CTRE Phoenix firmware simulation runs on the wall clock, not on HAL sim time (docs/reference/bridges.md): pace lock-step "
        "ticks to at most 1x real time while Phoenix devices are live";
    return advice;
}

void VendorSimDeviceBridge::onDeviceObserved(const CtreDeviceObservation& observation) {
    std::lock_guard lock(observationMutex_);
    if (observations_.size() < kMaxPendingObservations) observations_.push_back(observation);
}

void VendorSimDeviceBridge::setInputWritesEnabled(sdk::DeviceId id, bool enabled) {
    if (enabled) {
        disabled_.erase(id);
    } else {
        disabled_.insert(id);
    }
}

const CtreConfigSnapshot* VendorSimDeviceBridge::configOf(sdk::DeviceId id) const {
    const auto it = configs_.find(id);
    return it == configs_.end() ? nullptr : &it->second;
}

void VendorSimDeviceBridge::drainObservations() {
    std::vector<CtreDeviceObservation> batch;
    {
        std::lock_guard lock(observationMutex_);
        batch.swap(observations_);
    }
    if (batch.empty()) return;
    std::vector<CtreDeviceObservation> unresolved;
    for (auto& obs : batch) {
        bool matched = false;
        for (const auto& [id, info] : announced_) {
            const auto type = ctreDeviceTypeFromDescriptorType(info.descriptor.type);
            if (type && *type == obs.type && info.descriptor.id == obs.id) {
                inner_.observeBus(id, obs.bus);
                matched = true;
                break;
            }
        }
        if (!matched) unresolved.push_back(std::move(obs));  // the device may be announced by a later poll
    }
    if (unresolved.empty()) return;
    std::lock_guard lock(observationMutex_);
    for (auto& obs : unresolved)
        if (observations_.size() < kMaxPendingObservations) observations_.push_back(std::move(obs));
}

bool VendorSimDeviceBridge::readOneConfig() {
    if (host_ == nullptr || !options_.nativeProvider || announced_.empty()) return false;
    const auto wantsConfig = [this](const Announced& a) {
        return !a.configDone && a.descriptor.family == ctre::kFamilyPhoenix6 && a.configAttempts < options_.configReadAttempts;
    };
    const auto pending = std::find_if(announced_.begin(), announced_.end(),
                                      [&](const auto& kv) { return wantsConfig(kv.second); });
    if (pending == announced_.end()) return false;
    if (!ensureNative()) return false;

    Announced& a = pending->second;
    const sdk::DeviceId id = pending->first;
    const auto type = ctreDeviceTypeFromDescriptorType(a.descriptor.type);
    if (!type || !native_->hasConfigReadback()) {
        a.configDone = true;
        if (a.descriptor.type == ctre::kTypeCanCoder)
            host_->logger().warn(
                kCategory,
                a.descriptor.key().toString() +
                    ": config read-back unavailable; encoder inputs remain uncalibrated");
        return false;
    }
    ++a.configAttempts;
    sdk::Result<std::string> text = native_->readConfigs(*type, a.descriptor.id, options_.configBus);
    if (!text) {
        if (text.error().code == sdk::ErrorCode::Unsupported || a.configAttempts >= options_.configReadAttempts) {
            a.configDone = true;
            const auto message = a.descriptor.key().toString() +
                                 ": config read-back unavailable: " + text.error().message;
            if (a.descriptor.type == ctre::kTypeCanCoder)
                host_->logger().warn(kCategory, message + "; encoder inputs remain uncalibrated");
            else
                host_->logger().debug(kCategory, message);
        }
        return false;
    }
    CtreConfigSnapshot snapshot = CtreConfigSnapshot::parse(text.value());
    if (a.descriptor.type == ctre::kTypeCanCoder &&
        (!snapshot.magnetOffsetRot() || !std::isfinite(*snapshot.magnetOffsetRot()) ||
         !snapshot.clockwisePositive()))
        host_->logger().warn(
            kCategory, a.descriptor.key().toString() +
                           ": incomplete CANcoder calibration; encoder inputs remain uncalibrated");
    a.configDone = true;
    host_->logger().info(kCategory, a.descriptor.key().toString() + " config: " + snapshot.summary());
    const CtreConfigSnapshot& stored = configs_[id] = std::move(snapshot);
    if (options_.onConfig) options_.onConfig(id, a.descriptor, stored);
    return true;
}

}  // namespace wpiemu::bridges::vendor
