#include "can_bridge.h"

#include <array>
#include <string_view>

#include "bridge_support.h"

namespace wpiemu::bridges::hal {

using namespace detail;
using sdk::SignalDirection;
using sdk::SignalType;

namespace {
// Mirrors HAL_CANManufacturer / HAL_CANDeviceType (hal/CANAPITypes.h, 2026.2.1).
constexpr std::array<std::string_view, 21> kManufacturers{
    "broadcast", "ni", "lm", "deka", "ctre", "rev", "grapple", "mindsensors", "team_use", "kauai_labs", "copperforge",
    "playing_with_fusion", "studica", "thrifty_bot", "redux", "andymark", "vivid_hosting", "vertos", "swyft",
    "lumyn", "brushland"};
constexpr std::array<std::string_view, 14> kDeviceTypes{
    "broadcast", "robot_controller", "motor_controller", "relay_controller", "gyro", "accelerometer", "distance_sensor",
    "encoder", "power_distribution", "pneumatics", "miscellaneous", "io_breakout", "servo_controller", "color_sensor"};

constexpr std::size_t kSignalCount = 5;
}  // namespace

std::string canManufacturerName(int manufacturer) {
    if (manufacturer >= 0 && static_cast<std::size_t>(manufacturer) < kManufacturers.size())
        return std::string(kManufacturers[static_cast<std::size_t>(manufacturer)]);
    return "mfr" + std::to_string(manufacturer);
}

std::string canDeviceTypeName(int deviceType) {
    if (deviceType >= 0 && static_cast<std::size_t>(deviceType) < kDeviceTypes.size())
        return std::string(kDeviceTypes[static_cast<std::size_t>(deviceType)]);
    if (deviceType == 31) return "firmware_update";
    return "type" + std::to_string(deviceType);
}

CanFrameBridge::CanFrameBridge(std::shared_ptr<ICanHal> hal) : hal_(std::move(hal)) {}

CanFrameBridge::~CanFrameBridge() { stop(); }

sdk::Status CanFrameBridge::start(sdk::IBridgeHost& host) {
    host_ = &host;
    subscription_ = hal_->subscribe(*this);
    if (!subscription_) {
        host_ = nullptr;
        return sdk::makeError(sdk::ErrorCode::Unsupported, "HAL CAN send callback could not be registered");
    }
    return sdk::okStatus();
}

void CanFrameBridge::onFrame(const CanFrame& frame) {
    const CanArbitrationId id = CanArbitrationId::decode(frame.arbitrationId);
    if (id.deviceType == 0 && id.manufacturer == 0) return;  // broadcast message, not a device

    const Key key{id.manufacturer, id.deviceType, id.deviceId};
    std::uint32_t low = 0;
    std::uint32_t high = 0;
    for (std::size_t i = 0; i < 8 && i < frame.size; ++i) {
        std::uint32_t& word = i < 4 ? low : high;
        word |= static_cast<std::uint32_t>(frame.data[i]) << (8 * (i % 4));
    }

    std::lock_guard lock(mutex_);
    auto it = devices_.find(key);
    if (it == devices_.end()) {
        if (devices_.size() >= kMaxDevices) return;
        it = devices_.emplace(key, State{}).first;
        pending_.push_back(key);
    }
    State& s = it->second;
    ++s.frameCount;
    s.lastApi = id.apiId;
    s.lastPeriodMs = frame.periodMs;
    s.lastLow = low;
    s.lastHigh = high;
}

void CanFrameBridge::poll() {
    if (host_ == nullptr) return;
    std::vector<Key> pending;
    {
        std::lock_guard lock(mutex_);
        pending.swap(pending_);
    }
    for (const Key& key : pending) {
        const auto [mfr, devType, devId] = key;
        const std::string type = "can_" + canManufacturerName(mfr) + "_" + canDeviceTypeName(devType);
        const sdk::DeviceDescriptor d = makeDescriptor(
            type, devId, -1, sdk::CapabilitySet{},
            "HALSIM CAN " + std::to_string(mfr) + ":" + std::to_string(devType) + ":" + std::to_string(devId),
            "CAN " + canManufacturerName(mfr) + " " + canDeviceTypeName(devType) + " " + std::to_string(devId));
        const sdk::DeviceId id = host_->announce(
            d, {sig("frameCount", SignalDirection::Output, SignalType::Int),
                sig("lastApiId", SignalDirection::Output, SignalType::Int),
                sig("lastPeriodMs", SignalDirection::Output, SignalType::Int, "ms"),
                sig("lastDataLow", SignalDirection::Output, SignalType::Int),
                sig("lastDataHigh", SignalDirection::Output, SignalType::Int)});
        std::lock_guard lock(mutex_);
        State& s = devices_[key];
        s.id = id;
        s.announced = true;
    }
}

void CanFrameBridge::collectOutputs(std::vector<sdk::SignalSample>& out) {
    std::lock_guard lock(mutex_);
    for (const auto& entry : devices_) {
        const State& s = entry.second;
        if (!s.announced) continue;
        const std::array<double, kSignalCount> values{static_cast<double>(s.frameCount), static_cast<double>(s.lastApi),
                                                     static_cast<double>(s.lastPeriodMs), static_cast<double>(s.lastLow),
                                                     static_cast<double>(s.lastHigh)};
        for (std::size_t i = 0; i < values.size(); ++i) out.push_back({s.id, static_cast<std::uint16_t>(i), values[i]});
    }
}

bool CanFrameBridge::owns(sdk::DeviceId id) const {
    std::lock_guard lock(mutex_);
    for (const auto& entry : devices_)
        if (entry.second.announced && entry.second.id == id) return true;
    return false;
}

void CanFrameBridge::stop() noexcept {
    subscription_.reset();  // after this no robot thread is inside onFrame
    host_ = nullptr;
    std::lock_guard lock(mutex_);
    devices_.clear();
    pending_.clear();
}

}  // namespace wpiemu::bridges::hal
