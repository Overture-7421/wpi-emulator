#pragma once

// Scriptable IDeviceHub for isolated mechanism / drivetrain / plugin tests.
//  * Inputs (what the "robot" commands): setMotorOutput / setDigitalOutput.
//  * Outputs (what the model wrote back): lastMotorFeedback / lastEncoder / lastDigital / ...
//  * A slot only records/reads data when it is BOUND (like the real hub); use autoBind() to
//    create a fake device and bind it in one call.

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/devices/device_hub.h"
#include "wpiemu/sdk/devices/device_identity.h"

namespace wpiemu::sdk::testing {

class FakeDeviceHub final : public IDeviceHub {
public:
    // ---- scripting ----------------------------------------------------------------------------
    DeviceId addDevice(DeviceDescriptor descriptor, std::vector<SignalSpec> signalSpecs = {}) {
        DiscoveredDevice d;
        d.id = DeviceId{nextId_++};
        d.descriptor = std::move(descriptor);
        d.signalSpecs = std::move(signalSpecs);
        devices_.push_back(std::move(d));
        refreshMapped();  // a device announced after its mapping was saved becomes mapped immediately
        return devices_.back().id;
    }
    // Creates a device with the capabilities `type` needs, and binds it to `slot`.
    DeviceId autoBind(const SlotAddress& slot, SlotType type, int id = -1) {
        DeviceDescriptor desc;
        desc.family = "fake";
        desc.type = "fake";
        desc.id = id >= 0 ? id : static_cast<int>(nextId_);
        desc.capabilities = requiredCapabilities(type);
        const DeviceId did = addDevice(desc);
        (void)bind(slot, desc.key());
        return did;
    }
    void setMotorOutput(const SlotAddress& slot, const MotorOutput& out) { motorOut_[slot] = out; }
    void setDigitalOutput(const SlotAddress& slot, bool v) { digitalOut_[slot] = v; }
    void setBatteryVoltage(double v) { battery_ = v; }
    // Scripts a robot-declared address that resolves to `device` without being mapped (an alias the
    // robot program or an L3 hook reported). Two such aliases + one mapping => AliasState::Aliased.
    void declareAlias(DeviceId device, const DeviceKey& declaredAddress) {
        auto& keys = declared_[device];
        if (std::find(keys.begin(), keys.end(), declaredAddress) == keys.end()) keys.push_back(declaredAddress);
        refreshMapped();
    }
    // Mirrors IDeviceHubIngress::observeBus (L3 observation).
    void observeBus(DeviceId device, std::string_view busLabel) {
        for (auto& d : devices_)
            if (d.id == device) d.observedBus = std::string(busLabel);
        refreshMapped();
    }

    std::optional<MotorFeedback> lastMotorFeedback(const SlotAddress& s) const { return get(motorFb_, s); }
    std::optional<EncoderFeedback> lastEncoder(const SlotAddress& s) const { return get(encoder_, s); }
    std::optional<bool> lastDigital(const SlotAddress& s) const { return get(digitalIn_, s); }
    std::optional<double> lastAnalog(const SlotAddress& s) const { return get(analog_, s); }
    std::optional<ImuFeedback> lastImu(const SlotAddress& s) const { return get(imu_, s); }

    // ---- IActuatorReader ----------------------------------------------------------------------
    MotorOutput motorOutput(const SlotAddress& slot) const override {
        if (!isBound(slot)) return {};
        const auto it = motorOut_.find(slot);
        if (it == motorOut_.end()) return {};
        MotorOutput out = it->second;
        out.valid = true;
        return out;
    }
    bool digitalOutput(const SlotAddress& slot) const override {
        if (!isBound(slot)) return false;
        const auto it = digitalOut_.find(slot);
        return it != digitalOut_.end() && it->second;
    }

    // ---- ISensorWriter ------------------------------------------------------------------------
    void writeMotorFeedback(const SlotAddress& s, const MotorFeedback& f) override { if (isBound(s)) motorFb_[s] = f; }
    void writeEncoder(const SlotAddress& s, const EncoderFeedback& f) override { if (isBound(s)) encoder_[s] = f; }
    void writeDigital(const SlotAddress& s, bool v) override { if (isBound(s)) digitalIn_[s] = v; }
    void writeAnalog(const SlotAddress& s, double v) override { if (isBound(s)) analog_[s] = v; }
    void writeImu(const SlotAddress& s, const ImuFeedback& f) override { if (isBound(s)) imu_[s] = f; }

    // ---- IDeviceDirectory ---------------------------------------------------------------------
    std::span<const DiscoveredDevice> discovered() const override { return devices_; }
    const DiscoveredDevice* find(DeviceId id) const override {
        for (const auto& d : devices_)
            if (d.id == id) return &d;
        return nullptr;
    }
    const DiscoveredDevice* find(const DeviceKey& key) const override {
        const PhysicalDeviceId physical = key.physical();  // the bus is not part of the match
        for (const auto& d : devices_)
            if (d.physicalId() == physical) return &d;
        return nullptr;
    }
    Status bind(const SlotAddress& slot, const DeviceKey& device) override {
        for (const auto& [s, k] : bindings_)
            if (k == device && !(s == slot)) return makeError(ErrorCode::AlreadyExists, "device already bound: " + device.toString());
        bindings_[slot] = device;
        refreshMapped();
        return okStatus();
    }
    void unbind(const SlotAddress& slot) override {
        bindings_.erase(slot);
        refreshMapped();
    }
    std::optional<DeviceKey> boundDevice(const SlotAddress& slot) const override {
        const auto it = bindings_.find(slot);
        if (it == bindings_.end()) return std::nullopt;
        return it->second;
    }
    std::optional<DeviceId> resolve(const SlotAddress& slot) const override {
        if (!isBound(slot)) return std::nullopt;  // unbound, or disabled by an alias conflict
        const auto key = boundDevice(slot);
        const DiscoveredDevice* d = find(*key);
        if (d == nullptr) return std::nullopt;
        return d->id;
    }
    bool isMapped(DeviceId id) const override {
        const DiscoveredDevice* d = find(id);
        return d != nullptr && d->mapped;
    }
    std::span<const HubWarning> warnings() const override { return warnings_; }

    double batteryVoltage() const override { return battery_; }

private:
    template <class M>
    static auto get(const M& m, const SlotAddress& s) -> std::optional<typename M::mapped_type> {
        const auto it = m.find(s);
        if (it == m.end()) return std::nullopt;
        return it->second;
    }
    bool isBound(const SlotAddress& s) const {
        const auto it = bindings_.find(s);
        // a slot whose device is in AliasState::Conflict is disabled (neutral reads, writes ignored)
        return it != bindings_.end() && conflicted_.find(it->second.physical()) == conflicted_.end();
    }
    void refreshMapped() {
        std::map<PhysicalDeviceId, std::vector<DeviceKey>> bound;
        for (const auto& [slot, key] : bindings_) {
            auto& keys = bound[key.physical()];
            if (std::find(keys.begin(), keys.end(), key) == keys.end()) keys.push_back(key);
        }
        conflicted_.clear();
        std::erase_if(warnings_, [](const HubWarning& w) {
            return w.code == validation_code::kDeviceAliasConflict || w.code == validation_code::kDeviceAliased ||
                   w.code == validation_code::kDeviceBusMismatch;
        });
        for (auto& d : devices_) {
            const PhysicalDeviceId physical = d.physicalId();
            std::vector<DeviceKey> declared = declared_[d.id];
            const auto it = bound.find(physical);
            const std::size_t independent = it == bound.end() ? 0 : it->second.size();
            if (it != bound.end())
                for (const DeviceKey& k : it->second)
                    if (std::find(declared.begin(), declared.end(), k) == declared.end()) declared.push_back(k);
            d.aliases = declared;
            d.aliasState = classifyAliasState(declared.size(), independent);
            if (d.aliasState == AliasState::Conflict) conflicted_.insert(physical);
            d.mapped = independent > 0 && d.aliasState != AliasState::Conflict;
            if (d.aliasState == AliasState::Conflict)
                warnings_.push_back({Severity::Error, d.id, validation_code::kDeviceAliasConflict,
                                     "device " + physical.toString() + " is mapped through several addresses; disabled"});
            else if (d.aliasState == AliasState::Aliased)
                warnings_.push_back({Severity::Warning, d.id, validation_code::kDeviceAliased,
                                     "device " + physical.toString() + " is reachable through several declared addresses"});
            if (!d.observedBus.empty())
                for (const DeviceKey& k : d.aliases)
                    if (k.bus != d.observedBus) {
                        warnings_.push_back({Severity::Warning, d.id, validation_code::kDeviceBusMismatch,
                                             "declared " + k.toString() + " but observed on bus '" + d.observedBus + "'"});
                        break;
                    }
        }
    }

    std::uint32_t nextId_ = 1;
    double battery_ = 12.0;
    std::vector<DiscoveredDevice> devices_;
    std::vector<HubWarning> warnings_;
    std::map<DeviceId, std::vector<DeviceKey>> declared_;
    std::set<PhysicalDeviceId> conflicted_;
    std::map<SlotAddress, DeviceKey> bindings_;
    std::map<SlotAddress, MotorOutput> motorOut_;
    std::map<SlotAddress, bool> digitalOut_;
    std::map<SlotAddress, MotorFeedback> motorFb_;
    std::map<SlotAddress, EncoderFeedback> encoder_;
    std::map<SlotAddress, bool> digitalIn_;
    std::map<SlotAddress, double> analog_;
    std::map<SlotAddress, ImuFeedback> imu_;
};

}  // namespace wpiemu::sdk::testing
