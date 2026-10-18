#pragma once

// The editor's view of discovered devices plus the helpers shared by the device list, the tree and
// the slot rows (resolution by PHYSICAL id, drag-and-drop payloads).

#include <QByteArray>
#include <QMimeData>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/devices/slot.h"

namespace wpiemu::app::editor {

// Declared addresses resolve to the discovered device with the same PhysicalDeviceId (bus ignored).
class DeviceDirectory {
public:
    void setDevices(std::vector<sdk::DiscoveredDevice> devices) { devices_ = std::move(devices); }

    // Returns true when the device was not known before.
    bool upsert(const sdk::DiscoveredDevice& device) {
        for (auto& d : devices_)
            if (d.descriptor.physicalId() == device.descriptor.physicalId()) {
                d = device;
                return false;
            }
        devices_.push_back(device);
        return true;
    }

    const std::vector<sdk::DiscoveredDevice>& all() const { return devices_; }
    bool empty() const { return devices_.empty(); }

    const sdk::DiscoveredDevice* find(const sdk::PhysicalDeviceId& id) const {
        for (const auto& d : devices_)
            if (d.descriptor.physicalId() == id) return &d;
        return nullptr;
    }
    const sdk::DiscoveredDevice* find(const sdk::DeviceKey& key) const { return find(key.physical()); }

private:
    std::vector<sdk::DiscoveredDevice> devices_;
};

// "talonfx" -> "TalonFX"; unknown types are shown as-is.
inline QString prettyType(const std::string& type) {
    static const std::pair<const char*, const char*> known[] = {
        {"talonfx", "TalonFX"}, {"cancoder", "CANcoder"}, {"pigeon2", "Pigeon 2"}, {"candle", "CANdle"}, {"talonsrx", "Talon SRX"},
        {"victorspx", "Victor SPX"}, {"dio", "DIO"}, {"pwm", "PWM"}, {"analog", "Analog in"}, {"relay", "Relay"}, {"solenoid", "Solenoid"}};
    for (const auto& [raw, nice] : known)
        if (type == raw) return QString::fromLatin1(nice);
    return QString::fromStdString(type);
}

inline QString deviceLabel(const sdk::DeviceDescriptor& d) {
    if (!d.displayName.empty()) return QString::fromStdString(d.displayName);
    QString t = prettyType(d.type);
    if (d.id >= 0) t += QStringLiteral(" %1").arg(d.id);
    if (d.channel >= 0) t += QStringLiteral(" ch %1").arg(d.channel);
    return t;
}

inline QString deviceLabel(const sdk::DeviceKey& k) {
    QString t = prettyType(k.type);
    if (k.id >= 0) t += QStringLiteral(" %1").arg(k.id);
    if (k.channel >= 0) t += QStringLiteral(" ch %1").arg(k.channel);
    return t;
}

inline QString busLabel(const std::string& bus) { return bus.empty() ? QStringLiteral("\xE2\x80\x94") : QString::fromStdString(bus); }

// ---- drag-and-drop payloads -------------------------------------------------------------------
inline constexpr const char* kDeviceMime = "application/x-wpiemu-device";
inline constexpr const char* kMechanismMime = "application/x-wpiemu-mechanism";
inline constexpr const char* kZoneMime = "application/x-wpiemu-zone";

struct DevicePayload {
    sdk::DeviceKey key;
    sdk::CapabilitySet capabilities;
};

// The address to propose when mapping a discovered device: the observed bus wins over the (often unknown) static one.
inline sdk::DeviceKey proposedKey(const sdk::DiscoveredDevice& d) {
    sdk::DeviceKey k = d.descriptor.key();
    if (!d.observedBus.empty()) k.bus = d.observedBus;
    return k;
}

inline QByteArray encodePayload(const DevicePayload& payload) {
    const sdk::DeviceKey& k = payload.key;
    const QStringList parts{QString::fromStdString(k.family), QString::fromStdString(k.type), QString::number(k.id),
                            QString::fromStdString(k.bus), QString::number(k.channel), QString::number(payload.capabilities.bits)};
    return parts.join(QLatin1Char('|')).toUtf8();
}

inline QByteArray encodeDevice(const sdk::DiscoveredDevice& device) {
    return encodePayload({proposedKey(device), device.descriptor.capabilities});
}

inline std::optional<DevicePayload> decodeDevice(const QByteArray& data) {
    const QStringList p = QString::fromUtf8(data).split(QLatin1Char('|'));
    if (p.size() != 6) return std::nullopt;
    DevicePayload out;
    out.key.family = p[0].toStdString();
    out.key.type = p[1].toStdString();
    out.key.id = p[2].toInt();
    out.key.bus = p[3].toStdString();
    out.key.channel = p[4].toInt();
    out.capabilities = sdk::CapabilitySet{p[5].toUInt()};
    return out;
}

inline std::optional<DevicePayload> devicePayload(const QMimeData* mime) {
    if (mime == nullptr || !mime->hasFormat(QString::fromLatin1(kDeviceMime))) return std::nullopt;
    return decodeDevice(mime->data(QString::fromLatin1(kDeviceMime)));
}

inline bool slotAccepts(const sdk::SlotSpec& spec, sdk::CapabilitySet caps) {
    return caps.containsAll(sdk::requiredCapabilities(spec.type));
}

}  // namespace wpiemu::app::editor
