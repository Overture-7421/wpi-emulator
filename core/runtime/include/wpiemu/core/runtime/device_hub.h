#pragma once

#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/devices/device_hub.h"

namespace wpiemu::core::runtime {

// The emulator-side device hub (sdk::IDeviceHub + sdk::IDeviceHubIngress): discovery, slot mapping, the typed
// data plane between mechanisms and the raw per-device signals of the IPC frames.
//
// Threading: SIM THREAD ONLY (sdk contract). The IPC thread never touches the hub: it posts messages into a
// MessageInbox (robot_link.h) which the sim thread drains into onAnnounce / onActuatorFrame at the start of a tick.
// Everything the UI needs is copied into the immutable ISimSnapshot.
//
// Behaviour (ARCHITECTURE 4, device_identity.h):
//  * A declared address resolves to the discovered device with the same PhysicalDeviceId (bus ignored).
//  * live && !mapped devices are in loopbackSet() and raise `unmapped_loopback` (not for devices no slot type can
//    take, such as the roboRIO); devices in AliasState::Conflict
//    are treated as unmapped (neutral reads, ignored writes, `device_alias_conflict` Error).
//  * Slots with no (usable) device read neutral data and ignore writes.
//  * Warnings are DERIVED from the current state on every change, so they are always de-duplicated and clear
//    themselves when the cause goes away (device mapped, conflict resolved...).
//  * Bindings are keyed by DeviceKey and survive clearDevices(); loadMappings()/exportMappings() persist them.
class DeviceHub final : public sdk::IDeviceHub, public sdk::IDeviceHubIngress {
public:
    explicit DeviceHub(sdk::ILogger* logger = nullptr) : logger_(logger) {}

    // ---- slot knowledge (used for compatibility warnings; optional) ----
    void declareSlots(const std::string& owner, const std::vector<sdk::SlotSpec>& specs);
    void clearSlotDeclarations();

    // ---- persisted mappings ----
    // Binds every mapping; the first failure is returned but the remaining ones are still applied.
    sdk::Status loadMappings(std::span<const sdk::SlotMapping> mappings);
    std::vector<sdk::SlotMapping> exportMappings() const;
    // A reversed motor slot reads its command and receives its rotor feedback negated (mechanism frame).
    void setReversed(const sdk::SlotAddress& slot, bool reversed);
    bool isReversed(const sdk::SlotAddress& slot) const { return reversed_.count(slot) != 0; }
    // Increments whenever loopbackSet() content may have changed: tells the session to resend it.
    std::uint64_t mappingRevision() const { return revision_; }
    // After the robot program started (first actuator frame): bound-but-undiscovered slots raise `slot_missing_device`.
    void markDiscoveryComplete();
    bool discoveryComplete() const { return discoveryComplete_; }
    std::uint64_t droppedSamples() const { return droppedSamples_; }
    // Keep discovery/bindings, invalidate actuation, and neutralize old feedback for the next full frame.
    void resetSignalHistory();

    // ---- sdk::IActuatorReader ----
    sdk::MotorOutput motorOutput(const sdk::SlotAddress& slot) const override;
    bool digitalOutput(const sdk::SlotAddress& slot) const override;

    // ---- sdk::ISensorWriter ----
    void writeMotorFeedback(const sdk::SlotAddress& slot, const sdk::MotorFeedback& feedback) override;
    void writeEncoder(const sdk::SlotAddress& slot, const sdk::EncoderFeedback& feedback) override;
    void writeDigital(const sdk::SlotAddress& slot, bool value) override;
    void writeAnalog(const sdk::SlotAddress& slot, double volts) override;
    void writeDistance(const sdk::SlotAddress& slot, double metres) override;
    void writeImu(const sdk::SlotAddress& slot, const sdk::ImuFeedback& feedback) override;

    // ---- sdk::IDeviceDirectory ----
    std::span<const sdk::DiscoveredDevice> discovered() const override { return infos_; }
    const sdk::DiscoveredDevice* find(sdk::DeviceId id) const override;
    const sdk::DiscoveredDevice* find(const sdk::DeviceKey& key) const override;
    const sdk::DiscoveredDevice* findPhysical(const sdk::PhysicalDeviceId& physical) const override;
    sdk::Status bind(const sdk::SlotAddress& slot, const sdk::DeviceKey& device) override;
    void unbind(const sdk::SlotAddress& slot) override;
    std::optional<sdk::DeviceKey> boundDevice(const sdk::SlotAddress& slot) const override;
    std::optional<sdk::DeviceId> resolve(const sdk::SlotAddress& slot) const override;
    bool isMapped(sdk::DeviceId id) const override;
    std::span<const sdk::HubWarning> warnings() const override { return warnings_; }

    // ---- sdk::IDeviceHub ----
    double batteryVoltage() const override { return battery_; }

    // ---- sdk::IDeviceHubIngress ----
    void onAnnounce(const sdk::proto::DeviceAnnounce& announce) override;
    void onActuatorFrame(const sdk::proto::ActuatorFrame& frame) override;
    void setBatteryVoltage(double volts) override { battery_ = volts; }
    void buildSensorFrame(sdk::proto::SensorFrame& out, bool full) override;
    sdk::proto::LoopbackSet loopbackSet() const override;
    void clearDevices() override;
    void observeBus(sdk::DeviceId device, std::string_view busLabel) override;

private:
    struct Extra {
        std::map<std::string, std::uint16_t, std::less<>> byName;
        std::vector<double> value;
        std::vector<std::uint8_t> dirty;
        std::vector<std::uint8_t> written;
        std::vector<sdk::DeviceKey> declared;  // addresses the robot program itself declared (announce keys with a known bus)
        bool hasOutput = false;
    };

    std::optional<std::size_t> indexOf(sdk::DeviceId id) const;
    // Device behind a bound slot, or nullopt (unbound, undiscovered, retired, or in AliasState::Conflict).
    std::optional<std::size_t> usable(const sdk::SlotAddress& slot) const;
    int outputIndex(const Extra& e, const sdk::DiscoveredDevice& d, std::string_view name) const;
    int inputIndex(const Extra& e, const sdk::DiscoveredDevice& d, std::string_view name) const;
    void setInput(std::size_t device, std::string_view name, double value);
    bool setFirstInput(std::size_t device, std::initializer_list<std::string_view> names, double value);
    double outputValue(const Extra& e, int index) const { return index >= 0 ? e.value[static_cast<std::size_t>(index)] : 0.0; }
    void refresh();

    sdk::ILogger* logger_;
    std::vector<sdk::DiscoveredDevice> infos_;
    std::vector<Extra> extras_;
    std::unordered_map<sdk::DeviceId, std::size_t> byId_;
    std::map<sdk::SlotAddress, sdk::DeviceKey> bindings_;
    std::set<sdk::SlotAddress> reversed_;
    std::map<sdk::SlotAddress, sdk::SlotSpec> slotSpecs_;
    std::set<sdk::PhysicalDeviceId> conflicted_;
    std::vector<sdk::HubWarning> warnings_;
    std::vector<sdk::DeviceId> lastLoopback_;
    std::uint64_t revision_ = 0;
    std::uint64_t droppedSamples_ = 0;
    double battery_ = 12.0;
    bool discoveryComplete_ = false;
    bool fullFramePending_ = false;
};

}  // namespace wpiemu::core::runtime
