#pragma once

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
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/devices/device_hub.h"

namespace wpiemu::sdk::testing {

class FakeDeviceHub final : public IDeviceHub {
public:
    DeviceId addDevice(DeviceDescriptor descriptor) {
        const DeviceId id{nextId_++};
        DiscoveredDevice device;
        device.id = id;
        device.descriptor = std::move(descriptor);
        devices_.push_back(std::move(device));
        refresh();
        return id;
    }

    DeviceId autoBind(const SlotAddress& slot, SlotType type, int autoId = 0) {
        DeviceDescriptor descriptor;
        descriptor.family = "fake";
        descriptor.type = type == SlotType::Motor ? "motor" : "device";
        descriptor.vendor = "test";
        descriptor.id = autoId > 0 ? autoId : static_cast<int>(nextId_);
        switch (type) {
            case SlotType::Motor:
                descriptor.capabilities = Capability::MotorOutput;
                break;
            case SlotType::LimitSwitch:
            case SlotType::BeamBreak:
                descriptor.capabilities = Capability::DigitalInput;
                break;
            case SlotType::Solenoid:
                descriptor.capabilities = Capability::Solenoid | Capability::DigitalOutput;
                break;
            case SlotType::PositionSensor:
                descriptor.capabilities = Capability::PositionSensor;
                break;
            case SlotType::VelocitySensor:
                descriptor.capabilities = Capability::VelocitySensor;
                break;
            default:
                break;
        }
        const DeviceId id = addDevice(std::move(descriptor));
        (void)bind(slot, devices_.back().descriptor.key());
        return id;
    }

    void setMotorOutput(const SlotAddress& slot, const MotorOutput& output) {
        MotorOutput validOutput = output;
        validOutput.valid = true;
        motorOutputs_[slot] = validOutput;
    }
    void setDigitalOutput(const SlotAddress& slot, bool output) { digitalOutputs_[slot] = output; }
    void setBatteryVoltage(double volts) { batteryVoltage_ = volts; }

    std::optional<MotorFeedback> lastMotorFeedback(const SlotAddress& slot) const { return lookup(motorFeedback_, slot); }
    std::optional<EncoderFeedback> lastEncoder(const SlotAddress& slot) const { return lookup(encoders_, slot); }
    std::optional<bool> lastDigital(const SlotAddress& slot) const { return lookup(digitals_, slot); }
    std::optional<double> lastAnalog(const SlotAddress& slot) const { return lookup(analogs_, slot); }
    std::optional<ImuFeedback> lastImu(const SlotAddress& slot) const { return lookup(imus_, slot); }

    void declareAlias(DeviceId id, const DeviceKey& key) {
        if (find(id) != nullptr && !key.bus.empty()) declaredAliases_[id].insert(key);
        refresh();
    }

    void observeBus(DeviceId id, std::string_view bus) {
        if (DiscoveredDevice* device = mutableFind(id); device != nullptr) device->observedBus = std::string(bus);
        refresh();
    }

    MotorOutput motorOutput(const SlotAddress& slot) const override {
        const auto it = motorOutputs_.find(slot);
        MotorOutput output = it == motorOutputs_.end() ? MotorOutput{} : it->second;
        if (!resolve(slot)) output.valid = false;
        return output;
    }

    bool digitalOutput(const SlotAddress& slot) const override {
        if (!resolve(slot)) return false;
        const auto it = digitalOutputs_.find(slot);
        return it != digitalOutputs_.end() && it->second;
    }

    void writeMotorFeedback(const SlotAddress& slot, const MotorFeedback& feedback) override {
        if (resolve(slot)) motorFeedback_[slot] = feedback;
    }
    void writeEncoder(const SlotAddress& slot, const EncoderFeedback& feedback) override {
        if (resolve(slot)) encoders_[slot] = feedback;
    }
    void writeDigital(const SlotAddress& slot, bool value) override {
        if (resolve(slot)) digitals_[slot] = value;
    }
    void writeAnalog(const SlotAddress& slot, double volts) override {
        if (resolve(slot)) analogs_[slot] = volts;
    }
    void writeImu(const SlotAddress& slot, const ImuFeedback& feedback) override {
        if (resolve(slot)) imus_[slot] = feedback;
    }

    std::span<const DiscoveredDevice> discovered() const override { return devices_; }
    const DiscoveredDevice* find(DeviceId id) const override {
        const auto it = std::find_if(devices_.begin(), devices_.end(), [id](const DiscoveredDevice& d) { return d.id == id; });
        return it == devices_.end() ? nullptr : &*it;
    }
    const DiscoveredDevice* find(const DeviceKey& key) const override { return findPhysical(key.physical()); }
    const DiscoveredDevice* findPhysical(const PhysicalDeviceId& physical) const override {
        const auto it = std::find_if(devices_.begin(), devices_.end(),
                                     [&physical](const DiscoveredDevice& d) { return d.physicalId() == physical; });
        return it == devices_.end() ? nullptr : &*it;
    }

    Status bind(const SlotAddress& slot, const DeviceKey& device) override {
        if (slot.owner.empty() || slot.slot.empty())
            return makeError(ErrorCode::InvalidArgument, "slot owner and name must not be empty");
        for (const auto& [otherSlot, otherDevice] : bindings_)
            if (otherSlot != slot && otherDevice == device)
                return makeError(ErrorCode::AlreadyExists, "device address is already bound to another slot");
        bindings_[slot] = device;
        if (DiscoveredDevice* discovered = mutableFind(device.physical()); discovered != nullptr) addAlias(*discovered, device);
        refresh();
        return okStatus();
    }

    void unbind(const SlotAddress& slot) override {
        bindings_.erase(slot);
        refresh();
    }

    std::optional<DeviceKey> boundDevice(const SlotAddress& slot) const override {
        const auto it = bindings_.find(slot);
        return it == bindings_.end() ? std::nullopt : std::optional<DeviceKey>{it->second};
    }

    std::optional<DeviceId> resolve(const SlotAddress& slot) const override {
        const auto bound = bindings_.find(slot);
        if (bound == bindings_.end()) return std::nullopt;
        const DiscoveredDevice* device = findPhysical(bound->second.physical());
        return device != nullptr && device->live && device->aliasState != AliasState::Conflict
                   ? std::optional<DeviceId>{device->id}
                   : std::nullopt;
    }

    bool isMapped(DeviceId id) const override {
        const DiscoveredDevice* device = find(id);
        return device != nullptr && device->live && device->mapped;
    }
    std::span<const HubWarning> warnings() const override { return warnings_; }
    double batteryVoltage() const override { return batteryVoltage_; }

private:
    template <class T>
    static std::optional<T> lookup(const std::map<SlotAddress, T>& values, const SlotAddress& slot) {
        const auto it = values.find(slot);
        return it == values.end() ? std::nullopt : std::optional<T>{it->second};
    }

    DiscoveredDevice* mutableFind(DeviceId id) {
        const auto it = std::find_if(devices_.begin(), devices_.end(), [id](const DiscoveredDevice& d) { return d.id == id; });
        return it == devices_.end() ? nullptr : &*it;
    }
    DiscoveredDevice* mutableFind(const PhysicalDeviceId& physical) {
        const auto it = std::find_if(devices_.begin(), devices_.end(),
                                     [&physical](const DiscoveredDevice& d) { return d.physicalId() == physical; });
        return it == devices_.end() ? nullptr : &*it;
    }
    static void addAlias(DiscoveredDevice& device, const DeviceKey& key) {
        if (!key.bus.empty() && std::find(device.aliases.begin(), device.aliases.end(), key) == device.aliases.end())
            device.aliases.push_back(key);
    }
    void refresh() {
        warnings_.clear();
        for (DiscoveredDevice& device : devices_) {
            device.aliases.clear();
            if (!device.descriptor.key().bus.empty()) addAlias(device, device.descriptor.key());
            if (const auto it = declaredAliases_.find(device.id); it != declaredAliases_.end())
                for (const DeviceKey& key : it->second) addAlias(device, key);
            for (const auto& [slot, key] : bindings_)
                if (key.physical() == device.physicalId()) addAlias(device, key);
            std::set<DeviceKey> mappedKeys;
            for (const auto& [slot, key] : bindings_)
                if (key.physical() == device.physicalId()) mappedKeys.insert(key);
            device.aliasState = classifyAliasState(device.aliases.size(), mappedKeys.size());
            device.mapped = !mappedKeys.empty() && device.aliasState != AliasState::Conflict;
            if (device.aliasState == AliasState::Aliased)
                warnings_.push_back({Severity::Warning, device.id, validation_code::kDeviceAliased, "device has declared bus aliases"});
            else if (device.aliasState == AliasState::Conflict)
                warnings_.push_back({Severity::Error, device.id, validation_code::kDeviceAliasConflict,
                                     "one physical device is mapped through multiple bus aliases"});
            if (!device.observedBus.empty()) {
                for (const DeviceKey& alias : device.aliases)
                    if (!alias.bus.empty() && alias.bus != device.observedBus) {
                        warnings_.push_back({Severity::Warning, device.id, validation_code::kDeviceBusMismatch,
                                             "observed bus differs from a declared device address"});
                        break;
                    }
            }
        }
    }

    std::uint32_t nextId_ = 1;
    std::vector<DiscoveredDevice> devices_;
    std::map<SlotAddress, DeviceKey> bindings_;
    std::map<DeviceId, std::set<DeviceKey>> declaredAliases_;
    std::map<SlotAddress, MotorOutput> motorOutputs_;
    std::map<SlotAddress, bool> digitalOutputs_;
    std::map<SlotAddress, MotorFeedback> motorFeedback_;
    std::map<SlotAddress, EncoderFeedback> encoders_;
    std::map<SlotAddress, bool> digitals_;
    std::map<SlotAddress, double> analogs_;
    std::map<SlotAddress, ImuFeedback> imus_;
    std::vector<HubWarning> warnings_;
    double batteryVoltage_ = 12.0;
};

}  // namespace wpiemu::sdk::testing
