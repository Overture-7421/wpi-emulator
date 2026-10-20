#pragma once

// Generic, table-driven bridge for devices exposed as HAL SimDevices (docs/reference/bridges.md): Phoenix 6 /
// Phoenix 5 wrapper classes, NavX, ADIS, ... A vendor bridge is just a TABLE of SimDeviceRule on
// top of this class; no HAL/vendor headers are needed here. The only HAL-touching piece is the
// ISimDeviceBackend the extension (or a vendor bridge) implements over HAL_*/HALSIM_* SimDevice calls.

#include <cstdint>
#include <map>
#include <optional>
#include <regex>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::sdk {

using SimDeviceHandle = StrongId<struct SimDeviceHandleTag>;
using SimValueHandle = StrongId<struct SimValueHandleTag>;

struct SimDeviceInfo {
    SimDeviceHandle handle;
    std::string name;  // e.g. "CANMotor:Talon FX (v6)[4]"
};

struct SimValueInfo {
    SimValueHandle handle;
    std::string name;  // e.g. "motorVoltage"
    SignalDirection direction = SignalDirection::Output;  // HAL direction seen from the SIMULATOR
    SignalType type = SignalType::Double;
};

// Thin seam over the HAL SimDevice C API. All values are exchanged as double (bool: 0/1).
// Called on the extension clock thread only.
class ISimDeviceBackend {
    WPIEMU_INTERFACE(ISimDeviceBackend)
public:
    // Increments whenever a SimDevice is created/freed (set from HAL created-callbacks, which
    // run on robot threads: make it atomic). Lets the bridge skip enumeration when idle.
    virtual std::uint64_t generation() const = 0;
    virtual void listDevices(std::vector<SimDeviceInfo>& out) const = 0;
    virtual void listValues(SimDeviceHandle device, std::vector<SimValueInfo>& out) const = 0;
    virtual double getValue(SimValueHandle value) const = 0;
    virtual void setValue(SimValueHandle value, double v) = 0;
};

// Maps one SimDevice value onto a canonical signal.
struct SimValueBinding {
    std::string child;      // "" for the root SimDevice, else the regex child group ("Rotor Sensor", "Fwd Limit")
    std::string simValue;   // HAL value name
    std::string signal;     // canonical/vendor signal name
    double scale = 1.0;     // canonical = raw * scale + offset (inverse applied on write)
    double offset = 0.0;
    std::optional<SignalDirection> direction;  // override HAL's direction
    std::string unit;
    std::string loopbackTo;  // for Output signals: Input signal (same device) that receives the value when unmapped
};

struct SimDeviceRule {
    std::string name;                 // rule id for diagnostics
    std::string family;               // DeviceDescriptor::family
    std::string type;
    std::string vendor;
    std::string bus = "rio";
    // Full-match ECMAScript regex over the SimDevice name. Group `idGroup` is the device id,
    // `childGroup` (0 = none) is the sub-device path; children are folded into the root's descriptor.
    std::string nameRegex;
    int idGroup = 1;
    int childGroup = 0;
    CapabilitySet capabilities;
    // Empty => auto-bind every value as "<child>/<value>" (or "<value>") with HAL's direction.
    std::vector<SimValueBinding> bindings;
    // true: `bus` is a FACT of the family (e.g. rio-only HAL devices) and is copied to the descriptor.
    // false: the SimDevice name carries no bus (Phoenix 6 / CAN vendors): the descriptor's bus stays
    // kBusUnknown and may only be filled later by an observation (SimDeviceBridge::observeBus, L3).
    // CAN-vendor rules SHOULD set this to false so no per-device bus is fabricated (docs/ARCHITECTURE.md).
    bool busIsStatic = true;
};

class SimDeviceBridge : public IDeviceBridge {
public:
    SimDeviceBridge(std::string name, std::vector<SimDeviceRule> rules, ISimDeviceBackend& backend)
        : name_(std::move(name)), backend_(&backend) {
        compiled_.reserve(rules.size());
        for (auto& r : rules) {
            Compiled c;
            try {
                c.re = std::regex(r.nameRegex, std::regex::ECMAScript);
            } catch (const std::regex_error&) {
                c.valid = false;
            }
            c.rule = std::move(r);
            compiled_.push_back(std::move(c));
        }
    }

    std::string_view name() const override { return name_; }

    Status start(IBridgeHost& host) override {
        for (const auto& c : compiled_)
            if (!c.valid)
                return makeError(ErrorCode::InvalidArgument, "bad SimDeviceRule regex in rule '" + c.rule.name + "'");
        host_ = &host;
        seenGeneration_ = ~std::uint64_t{0};
        return okStatus();
    }

    void poll() override {
        if (host_ == nullptr) return;
        const std::uint64_t gen = backend_->generation();
        if (gen == seenGeneration_) return;
        seenGeneration_ = gen;

        listing_.clear();
        backend_->listDevices(listing_);

        // Drop members whose SimDevice vanished.
        std::vector<Entry*> touched;
        for (auto& [key, e] : entries_) {
            const auto before = e.members.size();
            std::erase_if(e.members, [&](const Member& m) {
                for (const auto& d : listing_)
                    if (d.handle == m.handle) return false;
                return true;
            });
            if (e.members.size() != before) touched.push_back(&e);
        }

        for (const auto& dev : listing_) {
            if (isKnown(dev.handle)) continue;
            for (const auto& c : compiled_) {
                std::smatch m;
                if (!std::regex_match(dev.name, m, c.re)) continue;
                const int id = idFromMatch(m, c.rule.idGroup);
                const std::string child = c.rule.childGroup > 0 && c.rule.childGroup < static_cast<int>(m.size())
                                              ? m[static_cast<std::size_t>(c.rule.childGroup)].str()
                                              : std::string{};
                const std::string ruleBus = c.rule.busIsStatic ? c.rule.bus : std::string(kBusUnknown);
                DeviceKey key{c.rule.family, c.rule.type, id, ruleBus, -1};
                Entry& e = entries_[key];
                if (e.descriptor.type.empty()) {
                    e.descriptor.family = c.rule.family;
                    e.descriptor.type = c.rule.type;
                    e.descriptor.vendor = c.rule.vendor;
                    e.descriptor.id = id;
                    e.descriptor.bus = ruleBus;
                    e.descriptor.capabilities = c.rule.capabilities;
                }
                if (child.empty()) e.descriptor.nativeName = dev.name;
                bindMember(e, c.rule, dev, child);
                touched.push_back(&e);
                break;
            }
        }

        for (Entry* e : touched) {
            if (e->members.empty()) {
                if (e->id.valid()) host_->retire(e->id);
                e->retired = true;
                continue;
            }
            if (!e->dirty && e->id.valid() && !e->retired) continue;
            std::vector<SignalSpec> specs;
            specs.reserve(e->signalSpecs.size());
            for (const auto& s : e->signalSpecs) specs.push_back(s.spec);
            e->id = host_->announce(e->descriptor, std::move(specs));
            e->dirty = false;
            e->retired = false;
            byId_[e->id] = e;
        }
        std::erase_if(entries_, [&](const auto& kv) { return kv.second.retired && kv.second.members.empty(); });
        rebuildIndex();
    }

    void collectOutputs(std::vector<SignalSample>& out) override {
        for (const auto& [key, e] : entries_) {
            if (!e.id.valid() || e.retired) continue;
            for (std::size_t i = 0; i < e.signalSpecs.size(); ++i) {
                const auto& s = e.signalSpecs[i];
                if (s.spec.direction == SignalDirection::Input) continue;
                out.push_back({e.id, static_cast<std::uint16_t>(i), backend_->getValue(s.value) * s.scale + s.offset});
            }
        }
    }

    void applyInputs(std::span<const SignalSample> samples) override {
        for (const auto& sample : samples) {
            Entry* e = entryFor(sample.device);
            if (e == nullptr || sample.signal >= e->signalSpecs.size()) continue;
            const auto& s = e->signalSpecs[sample.signal];
            if (s.spec.direction == SignalDirection::Output) continue;
            backend_->setValue(s.value, (sample.value - s.offset) / s.scale);
        }
    }

    void applyLoopback(DeviceId id) override {
        Entry* e = entryFor(id);
        if (e == nullptr) return;
        for (const auto& s : e->signalSpecs) {
            if (s.loopbackTo.empty() || s.spec.direction == SignalDirection::Input) continue;
            for (const auto& target : e->signalSpecs) {
                if (target.spec.name != s.loopbackTo || target.spec.direction == SignalDirection::Output) continue;
                const double canonical = backend_->getValue(s.value) * s.scale + s.offset;
                backend_->setValue(target.value, (canonical - target.offset) / target.scale);
            }
        }
    }

    bool owns(DeviceId id) const override { return byId_.find(id) != byId_.end(); }

    // L3 feed (e.g. the CTRE constructor hook): records the bus a device was seen on and forwards it to
    // the host. Returns false if `id` is not one of this bridge's devices. The static rule never
    // supplies a bus; only this call does. Does not change the device's key or physical identity.
    bool observeBus(DeviceId id, std::string_view busLabel) {
        Entry* e = entryFor(id);
        if (e == nullptr || host_ == nullptr) return false;
        e->observedBus = std::string(busLabel);
        host_->observeBus(id, busLabel);
        return true;
    }
    std::string observedBus(DeviceId id) const {
        const auto it = byId_.find(id);
        return it == byId_.end() ? std::string{} : it->second->observedBus;
    }

    void stop() noexcept override {
        host_ = nullptr;
        entries_.clear();
        byId_.clear();
        listing_.clear();
    }

private:
    struct Member {
        SimDeviceHandle handle;
        std::string child;
    };
    struct LiveSignal {
        SignalSpec spec;
        SimValueHandle value;
        double scale = 1.0;
        double offset = 0.0;
        std::string loopbackTo;
    };
    struct Entry {
        DeviceId id;
        DeviceDescriptor descriptor;
        std::vector<Member> members;
        std::vector<LiveSignal> signalSpecs;
        std::string observedBus;
        bool dirty = true;
        bool retired = false;
    };
    struct Compiled {
        SimDeviceRule rule;
        std::regex re;
        bool valid = true;
    };

    static int idFromMatch(const std::smatch& m, int group) {
        if (group <= 0 || group >= static_cast<int>(m.size())) return -1;
        try {
            return std::stoi(m[static_cast<std::size_t>(group)].str());
        } catch (...) {
            return -1;
        }
    }

    bool isKnown(SimDeviceHandle h) const {
        for (const auto& [key, e] : entries_)
            for (const auto& m : e.members)
                if (m.handle == h) return true;
        return false;
    }

    void bindMember(Entry& e, const SimDeviceRule& rule, const SimDeviceInfo& dev, const std::string& child) {
        e.members.push_back({dev.handle, child});
        values_.clear();
        backend_->listValues(dev.handle, values_);
        for (const auto& v : values_) {
            const SimValueBinding* b = nullptr;
            for (const auto& cand : rule.bindings)
                if (cand.child == child && cand.simValue == v.name) b = &cand;
            if (!rule.bindings.empty() && b == nullptr) continue;
            LiveSignal ls;
            ls.value = v.handle;
            ls.spec.type = v.type;
            ls.spec.direction = v.direction;
            ls.spec.name = child.empty() ? v.name : child + "/" + v.name;
            if (b != nullptr) {
                ls.spec.name = b->signal;
                if (b->direction) ls.spec.direction = *b->direction;
                ls.spec.unit = b->unit;
                ls.scale = b->scale;
                ls.offset = b->offset;
                ls.loopbackTo = b->loopbackTo;
            }
            e.signalSpecs.push_back(std::move(ls));  // append-only: indices never change
            e.dirty = true;
        }
    }

    Entry* entryFor(DeviceId id) {
        auto it = byId_.find(id);
        return it == byId_.end() ? nullptr : it->second;
    }
    void rebuildIndex() {
        byId_.clear();
        for (auto& [key, e] : entries_)
            if (e.id.valid()) byId_[e.id] = &e;
    }

    std::string name_;
    ISimDeviceBackend* backend_;
    IBridgeHost* host_ = nullptr;
    std::vector<Compiled> compiled_;
    std::map<DeviceKey, Entry> entries_;  // node-based: Entry addresses are stable until erased
    std::map<DeviceId, Entry*> byId_;
    std::uint64_t seenGeneration_ = ~std::uint64_t{0};
    std::vector<SimDeviceInfo> listing_;
    std::vector<SimValueInfo> values_;
};

}  // namespace wpiemu::sdk
