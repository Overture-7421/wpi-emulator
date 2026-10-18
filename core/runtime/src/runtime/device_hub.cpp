#include "wpiemu/core/runtime/device_hub.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/devices/signals.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

namespace {
bool isInputDirection(SignalDirection d) { return d == SignalDirection::Input || d == SignalDirection::Bidirectional; }
bool isOutputDirection(SignalDirection d) { return d == SignalDirection::Output || d == SignalDirection::Bidirectional; }
}  // namespace

void DeviceHub::declareSlots(const std::string& owner, const std::vector<SlotSpec>& specs) {
    for (auto it = slotSpecs_.begin(); it != slotSpecs_.end();)
        it = it->first.owner == owner ? slotSpecs_.erase(it) : std::next(it);
    for (const SlotSpec& s : specs) slotSpecs_[SlotAddress{owner, s.name}] = s;
    refresh();
}

void DeviceHub::clearSlotDeclarations() {
    slotSpecs_.clear();
    refresh();
}

Status DeviceHub::loadMappings(std::span<const SlotMapping> mappings) {
    std::optional<Error> first;
    for (const SlotMapping& m : mappings) {
        Status s = bind(m.slot, m.device);
        if (!s && !first) first = s.error();
        if (s) setReversed(m.slot, m.reversed);
    }
    if (first) return *first;
    return okStatus();
}

std::vector<SlotMapping> DeviceHub::exportMappings() const {
    std::vector<SlotMapping> out;
    out.reserve(bindings_.size());
    for (const auto& [slot, key] : bindings_) out.push_back({slot, key, isReversed(slot)});
    return out;
}

void DeviceHub::setReversed(const SlotAddress& slot, bool reversed) {
    if (reversed)
        reversed_.insert(slot);
    else
        reversed_.erase(slot);
}

void DeviceHub::markDiscoveryComplete() {
    if (discoveryComplete_) return;
    discoveryComplete_ = true;
    refresh();
}

std::optional<std::size_t> DeviceHub::indexOf(DeviceId id) const {
    const auto it = byId_.find(id);
    if (it == byId_.end()) return std::nullopt;
    return it->second;
}

const DiscoveredDevice* DeviceHub::find(DeviceId id) const {
    const auto i = indexOf(id);
    return i ? &infos_[*i] : nullptr;
}

const DiscoveredDevice* DeviceHub::findPhysical(const PhysicalDeviceId& physical) const {
    for (const DiscoveredDevice& d : infos_)
        if (d.physicalId() == physical) return &d;
    return nullptr;
}

const DiscoveredDevice* DeviceHub::find(const DeviceKey& key) const { return findPhysical(key.physical()); }

Status DeviceHub::bind(const SlotAddress& slot, const DeviceKey& device) {
    if (slot.owner.empty() || slot.slot.empty()) return makeError(ErrorCode::InvalidArgument, "slot address needs an owner and a slot name");
    for (const auto& [s, k] : bindings_)
        if (k == device && !(s == slot)) return makeError(ErrorCode::AlreadyExists, "device already bound to " + s.toString() + ": " + device.toString());
    const auto existing = bindings_.find(slot);
    if (existing != bindings_.end() && existing->second == device) return okStatus();
    bindings_[slot] = device;
    refresh();
    return okStatus();
}

void DeviceHub::unbind(const SlotAddress& slot) {
    reversed_.erase(slot);
    if (bindings_.erase(slot) != 0) refresh();
}

std::optional<DeviceKey> DeviceHub::boundDevice(const SlotAddress& slot) const {
    const auto it = bindings_.find(slot);
    if (it == bindings_.end()) return std::nullopt;
    return it->second;
}

std::optional<std::size_t> DeviceHub::usable(const SlotAddress& slot) const {
    const auto it = bindings_.find(slot);
    if (it == bindings_.end()) return std::nullopt;
    const PhysicalDeviceId physical = it->second.physical();
    if (conflicted_.count(physical) != 0) return std::nullopt;
    for (std::size_t i = 0; i < infos_.size(); ++i)
        if (infos_[i].live && infos_[i].physicalId() == physical) return i;
    return std::nullopt;
}

std::optional<DeviceId> DeviceHub::resolve(const SlotAddress& slot) const {
    const auto i = usable(slot);
    if (!i) return std::nullopt;
    return infos_[*i].id;
}

bool DeviceHub::isMapped(DeviceId id) const {
    const DiscoveredDevice* d = find(id);
    return d != nullptr && d->mapped;
}

int DeviceHub::outputIndex(const Extra& e, const DiscoveredDevice& d, std::string_view name) const {
    const auto it = e.byName.find(name);
    if (it == e.byName.end() || !isOutputDirection(d.signalSpecs[it->second].direction)) return -1;
    return it->second;
}

int DeviceHub::inputIndex(const Extra& e, const DiscoveredDevice& d, std::string_view name) const {
    const auto it = e.byName.find(name);
    if (it == e.byName.end() || !isInputDirection(d.signalSpecs[it->second].direction)) return -1;
    return it->second;
}

void DeviceHub::setInput(std::size_t device, std::string_view name, double value) {
    Extra& e = extras_[device];
    const int idx = inputIndex(e, infos_[device], name);
    if (idx < 0) return;
    const auto i = static_cast<std::size_t>(idx);
    if (!e.written[i] || e.value[i] != value) e.dirty[i] = 1;
    e.written[i] = 1;
    e.value[i] = value;
}

bool DeviceHub::setFirstInput(std::size_t device, std::initializer_list<std::string_view> names, double value) {
    for (const std::string_view name : names) {
        if (inputIndex(extras_[device], infos_[device], name) >= 0) {
            setInput(device, name, value);
            return true;
        }
    }
    return false;
}

// ---- reads ------------------------------------------------------------------------------------------------

MotorOutput DeviceHub::motorOutput(const SlotAddress& slot) const {
    const auto i = usable(slot);
    if (!i) return {};
    const Extra& e = extras_[*i];
    const DiscoveredDevice& d = infos_[*i];
    if (!e.hasOutput) return {};

    MotorOutput out;
    out.valid = true;
    int duty = outputIndex(e, d, signal::kDutyCycle);
    if (duty < 0) duty = outputIndex(e, d, signal::kPwmSpeed);
    out.dutyCycle = outputValue(e, duty);
    if (const int volt = outputIndex(e, d, signal::kMotorVoltage); volt >= 0) out.voltageV = outputValue(e, volt);
    else out.voltageV = out.dutyCycle * battery_;
    if (const int torque = outputIndex(e, d, signal::kTorqueCurrent); torque >= 0) {
        out.torqueCurrentA = outputValue(e, torque);
        out.hasTorqueCurrent = std::abs(out.torqueCurrentA) > 1e-9;
    }
    if (isReversed(slot)) {
        out.dutyCycle = -out.dutyCycle;
        out.voltageV = -out.voltageV;
        out.torqueCurrentA = -out.torqueCurrentA;
    }
    return out;
}

bool DeviceHub::digitalOutput(const SlotAddress& slot) const {
    const auto i = usable(slot);
    if (!i) return false;
    const Extra& e = extras_[*i];
    const DiscoveredDevice& d = infos_[*i];
    int idx = outputIndex(e, d, signal::kDigitalOut);
    if (idx < 0) idx = outputIndex(e, d, signal::kSolenoid);
    return idx >= 0 && outputValue(e, idx) != 0.0;
}

// ---- writes -----------------------------------------------------------------------------------------------

void DeviceHub::writeMotorFeedback(const SlotAddress& slot, const MotorFeedback& f) {
    const auto i = usable(slot);
    if (!i) return;
    const double s = isReversed(slot) ? -1.0 : 1.0;
    if (!setFirstInput(*i, {signal::kRotorPosition, signal::kPosition}, s * f.rotorPositionRot)) return;
    setFirstInput(*i, {signal::kRotorVelocity, signal::kVelocity}, s * f.rotorVelocityRps);
    setInput(*i, signal::kRotorAcceleration, s * f.rotorAccelerationRps2);
    if (f.forwardLimit) setInput(*i, signal::kForwardLimit, *f.forwardLimit ? 1.0 : 0.0);
    if (f.reverseLimit) setInput(*i, signal::kReverseLimit, *f.reverseLimit ? 1.0 : 0.0);
}

void DeviceHub::writeEncoder(const SlotAddress& slot, const EncoderFeedback& f) {
    const auto i = usable(slot);
    if (!i) return;
    setFirstInput(*i, {signal::kPosition, signal::kRotorPosition}, f.positionRot);
    setFirstInput(*i, {signal::kVelocity, signal::kRotorVelocity}, f.velocityRps);
}

void DeviceHub::writeDigital(const SlotAddress& slot, bool value) {
    const auto i = usable(slot);
    if (!i) return;
    setFirstInput(*i, {signal::kDigitalIn, signal::kDetected}, value ? 1.0 : 0.0);
}

void DeviceHub::writeAnalog(const SlotAddress& slot, double volts) {
    const auto i = usable(slot);
    if (!i) return;
    setFirstInput(*i, {signal::kAnalogVoltage, signal::kDistance}, volts);
}

void DeviceHub::writeImu(const SlotAddress& slot, const ImuFeedback& f) {
    const auto i = usable(slot);
    if (!i) return;
    setInput(*i, signal::kYaw, f.yawRad);
    setInput(*i, signal::kPitch, f.pitchRad);
    setInput(*i, signal::kRoll, f.rollRad);
    setInput(*i, signal::kAngularVelX, f.angularVelocityRadPerSec.x);
    setInput(*i, signal::kAngularVelY, f.angularVelocityRadPerSec.y);
    setInput(*i, signal::kAngularVelZ, f.angularVelocityRadPerSec.z);
    setInput(*i, signal::kAccelX, f.accelerationMps2.x);
    setInput(*i, signal::kAccelY, f.accelerationMps2.y);
    setInput(*i, signal::kAccelZ, f.accelerationMps2.z);
}

// ---- ingress ----------------------------------------------------------------------------------------------

void DeviceHub::onAnnounce(const proto::DeviceAnnounce& a) {
    std::size_t index = infos_.size();
    // idempotent: same id, or a re-announce of the same physical device under a fresh session-local id
    if (const auto byId = indexOf(a.id)) index = *byId;
    else
        for (std::size_t i = 0; i < infos_.size(); ++i)
            if (infos_[i].physicalId() == a.descriptor.physicalId()) {
                index = i;
                break;
            }
    const bool created = index == infos_.size();
    if (created) {
        infos_.emplace_back();
        extras_.emplace_back();
    } else {
        byId_.erase(infos_[index].id);
    }

    DiscoveredDevice& d = infos_[index];
    Extra& e = extras_[index];
    d.id = a.id;
    d.descriptor = a.descriptor;
    d.live = !a.retired;
    if (created || d.signalSpecs != a.signalSpecs) {
        d.signalSpecs = a.signalSpecs;
        e.byName.clear();
        for (std::size_t s = 0; s < a.signalSpecs.size(); ++s) e.byName.emplace(a.signalSpecs[s].name, static_cast<std::uint16_t>(s));
        e.value.assign(a.signalSpecs.size(), 0.0);
        e.dirty.assign(a.signalSpecs.size(), 0);
        e.written.assign(a.signalSpecs.size(), 0);
        e.hasOutput = false;
    }
    if (!a.observedBus.empty()) d.observedBus = a.observedBus;
    e.declared.clear();
    if (a.descriptor.bus != kBusUnknown) e.declared.push_back(a.descriptor.key());
    byId_[a.id] = index;
    if (created && logger_ != nullptr) logger_->debug("hub", "discovered " + d.descriptor.key().toString());
    refresh();
}

void DeviceHub::onActuatorFrame(const proto::ActuatorFrame& frame) {
    for (const SignalSample& s : frame.samples) {
        const auto di = indexOf(s.device);
        if (!di || s.signal >= extras_[*di].value.size()) {
            ++droppedSamples_;
            continue;
        }
        if (!isOutputDirection(infos_[*di].signalSpecs[s.signal].direction)) {
            ++droppedSamples_;
            continue;
        }
        extras_[*di].value[s.signal] = s.value;
        extras_[*di].hasOutput = true;
    }
}

void DeviceHub::buildSensorFrame(proto::SensorFrame& out, bool full) {
    full = full || fullFramePending_;
    fullFramePending_ = false;
    out.samples.clear();
    out.full = full;
    out.batteryVoltageV = battery_;
    for (std::size_t i = 0; i < infos_.size(); ++i) {
        if (!infos_[i].live || !infos_[i].mapped) continue;
        Extra& e = extras_[i];
        for (std::size_t s = 0; s < e.value.size(); ++s) {
            if (!isInputDirection(infos_[i].signalSpecs[s].direction) || !e.written[s]) continue;
            if (full || e.dirty[s]) out.samples.push_back({infos_[i].id, static_cast<std::uint16_t>(s), e.value[s]});
            e.dirty[s] = 0;
        }
    }
}

void DeviceHub::resetSignalHistory() {
    for (Extra& e : extras_) {
        std::fill(e.value.begin(), e.value.end(), 0.0);
        std::fill(e.dirty.begin(), e.dirty.end(), std::uint8_t{0});
        // Keep the input mask so even optional feedback not rewritten this epoch is neutralized.
        e.hasOutput = false;
    }
    fullFramePending_ = true;
}

proto::LoopbackSet DeviceHub::loopbackSet() const {
    proto::LoopbackSet set;
    for (const DiscoveredDevice& d : infos_)
        if (d.live && !d.mapped) set.loopback.push_back(d.id);
    std::sort(set.loopback.begin(), set.loopback.end());
    return set;
}

void DeviceHub::clearDevices() {
    infos_.clear();
    extras_.clear();
    byId_.clear();
    discoveryComplete_ = false;
    refresh();
}

void DeviceHub::observeBus(DeviceId device, std::string_view busLabel) {
    const auto i = indexOf(device);
    if (!i) return;
    infos_[*i].observedBus = std::string(busLabel);
    refresh();
}

// ---- derived state ----------------------------------------------------------------------------------------

void DeviceHub::refresh() {
    std::map<PhysicalDeviceId, std::vector<DeviceKey>> bound;
    std::map<PhysicalDeviceId, std::vector<SlotAddress>> boundSlots;
    for (const auto& [slot, key] : bindings_) {
        auto& keys = bound[key.physical()];
        if (std::find(keys.begin(), keys.end(), key) == keys.end()) keys.push_back(key);
        boundSlots[key.physical()].push_back(slot);
    }

    conflicted_.clear();
    warnings_.clear();
    for (std::size_t i = 0; i < infos_.size(); ++i) {
        DiscoveredDevice& d = infos_[i];
        const PhysicalDeviceId physical = d.physicalId();
        std::vector<DeviceKey> declared = extras_[i].declared;
        const auto it = bound.find(physical);
        const std::size_t independent = it == bound.end() ? 0 : it->second.size();
        if (it != bound.end())
            for (const DeviceKey& k : it->second)
                if (std::find(declared.begin(), declared.end(), k) == declared.end()) declared.push_back(k);
        d.aliases = declared;
        d.aliasState = classifyAliasState(declared.size(), independent);
        if (d.aliasState == AliasState::Conflict) conflicted_.insert(physical);
        d.mapped = independent > 0 && d.aliasState != AliasState::Conflict;

        if (d.aliasState == AliasState::Conflict) {
            warnings_.push_back({Severity::Error, d.id, validation_code::kDeviceAliasConflict,
                                 "device " + physical.toString() + " is mapped through several addresses; it is disabled (loopback) until the mappings are fixed"});
        } else if (d.aliasState == AliasState::Aliased) {
            warnings_.push_back({Severity::Warning, d.id, validation_code::kDeviceAliased,
                                 "device " + physical.toString() + " is reachable through several declared addresses"});
        }
        if (!d.observedBus.empty())
            for (const DeviceKey& k : d.aliases)
                if (k.bus != d.observedBus) {
                    warnings_.push_back({Severity::Warning, d.id, validation_code::kDeviceBusMismatch,
                                         "declared " + k.toString() + " but observed on bus '" + d.observedBus + "'"});
                    break;
                }
        if (d.live && !d.mapped && d.aliasState != AliasState::Conflict && isMappable(d.descriptor))
            warnings_.push_back({Severity::Warning, d.id, "unmapped_loopback",
                                 "device " + d.descriptor.key().toString() + " is not mapped to any slot: its output is looped back"});
        if (d.mapped) {
            for (const SlotAddress& slot : boundSlots[physical]) {
                const auto spec = slotSpecs_.find(slot);
                if (spec != slotSpecs_.end() && !isCompatible(spec->second, d.descriptor))
                    warnings_.push_back({Severity::Warning, d.id, "slot_incompatible",
                                         "device " + d.descriptor.key().toString() + " lacks the capabilities slot '" + slot.toString() + "' needs"});
            }
        }
    }
    if (discoveryComplete_) {
        for (const auto& [slot, key] : bindings_) {
            const bool found = std::any_of(infos_.begin(), infos_.end(), [&](const DiscoveredDevice& d) { return d.live && d.physicalId() == key.physical(); });
            if (!found)
                warnings_.push_back({Severity::Warning, DeviceId{}, "slot_missing_device",
                                     "slot '" + slot.toString() + "' is mapped to " + key.toString() + " but the robot program never created it"});
        }
    }

    proto::LoopbackSet now = loopbackSet();
    if (now.loopback != lastLoopback_) {
        lastLoopback_ = std::move(now.loopback);
        ++revision_;
    }
}

}  // namespace wpiemu::core::runtime
