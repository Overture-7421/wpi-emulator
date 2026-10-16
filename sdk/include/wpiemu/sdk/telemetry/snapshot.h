#pragma once

// View-model the UI consumes. The UI NEVER touches core internals, the physics world or the hub
// from its own thread: the sim thread builds an immutable ISimSnapshot (SnapshotBuilder) at the
// end of a tick (or every N ticks) and publishes it; the UI thread picks up the latest one.
//
// Ownership: snapshots are shared_ptr<const ISimSnapshot>, immutable after build(), safe to read
// from any thread, and stay valid as long as the pointer is held.

#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/devices/discovered_device.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/mechanisms/mechanism_state.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk {

struct BodySample {
    std::string name;           // "chassis", plugin bodies ...
    Pose3 pose;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
};

struct WheelSample {
    std::string name;
    Vec3 contactPoint;
    double steerAngleRad = 0.0;
    double spinAngleRad = 0.0;   // for wheel rotation rendering
    float normalLoadN = 0.f;
    float slipRatio = 0.f;
    float slipAngleTan = 0.f;
    bool airborne = false;
};

// A homogeneous set of simple entities drawn with instancing (fuel balls, ...).
struct EntityGroupSample {
    std::string group;             // "fuel"
    std::string shape = "sphere";  // "sphere" | "box"
    float radiusM = 0.1f;          // sphere
    Vec3 halfExtentsM;             // box
    std::string color = "#ffd000";
    std::vector<Vec3> positions;
};

struct HudValue {
    std::string key;   // matches HudWidgetDescriptor::valueKey
    Value value;
};

class ISimSnapshot {
    WPIEMU_INTERFACE(ISimSnapshot)
public:
    virtual std::uint64_t tick() const = 0;
    virtual SimTimeUs simTimeUs() const = 0;
    virtual bool paused() const = 0;
    virtual bool robotConnected() const = 0;

    virtual const BodySample& chassis() const = 0;
    virtual std::span<const BodySample> bodies() const = 0;
    virtual std::span<const WheelSample> wheels() const = 0;
    virtual std::span<const EntityGroupSample> entityGroups() const = 0;
    virtual std::span<const MechanismState> mechanisms() const = 0;
    virtual std::span<const HudValue> hud() const = 0;
    virtual std::span<const DiscoveredDevice> devices() const = 0;
    virtual std::span<const HubWarning> warnings() const = 0;

    virtual double batteryVoltageV() const = 0;
    virtual const proto::DsState& driverStation() const = 0;
    virtual const PhysicsStats& physicsStats() const = 0;

    // Convenience (non-virtual): HUD lookup by key, nullptr if absent.
    const Value* hudValue(std::string_view key) const {
        for (const auto& h : hud())
            if (h.key == key) return &h.value;
        return nullptr;
    }
};

// Mutable plain-data form; also directly usable as a test fixture.
struct SimSnapshotData {
    std::uint64_t tick = 0;
    SimTimeUs simTimeUs = 0;
    bool paused = false;
    bool robotConnected = false;
    BodySample chassis;
    std::vector<BodySample> bodies;
    std::vector<WheelSample> wheels;
    std::vector<EntityGroupSample> entityGroups;
    std::vector<MechanismState> mechanisms;
    std::vector<HudValue> hud;
    std::vector<DiscoveredDevice> devices;
    std::vector<HubWarning> warnings;
    double batteryVoltageV = 12.6;
    proto::DsState driverStation;
    PhysicsStats physicsStats;
};

class SimSnapshot final : public ISimSnapshot {
public:
    explicit SimSnapshot(SimSnapshotData data) : d_(std::move(data)) {}

    std::uint64_t tick() const override { return d_.tick; }
    SimTimeUs simTimeUs() const override { return d_.simTimeUs; }
    bool paused() const override { return d_.paused; }
    bool robotConnected() const override { return d_.robotConnected; }
    const BodySample& chassis() const override { return d_.chassis; }
    std::span<const BodySample> bodies() const override { return d_.bodies; }
    std::span<const WheelSample> wheels() const override { return d_.wheels; }
    std::span<const EntityGroupSample> entityGroups() const override { return d_.entityGroups; }
    std::span<const MechanismState> mechanisms() const override { return d_.mechanisms; }
    std::span<const HudValue> hud() const override { return d_.hud; }
    std::span<const DiscoveredDevice> devices() const override { return d_.devices; }
    std::span<const HubWarning> warnings() const override { return d_.warnings; }
    double batteryVoltageV() const override { return d_.batteryVoltageV; }
    const proto::DsState& driverStation() const override { return d_.driverStation; }
    const PhysicsStats& physicsStats() const override { return d_.physicsStats; }

private:
    SimSnapshotData d_;
};

// Handed to IMechanism / IGameState so they contribute to the snapshot without knowing its layout.
class SnapshotBuilder {
public:
    SimSnapshotData& data() { return data_; }

    void addBody(BodySample b) { data_.bodies.push_back(std::move(b)); }
    void addMechanism(MechanismState m) { data_.mechanisms.push_back(std::move(m)); }
    EntityGroupSample& addEntityGroup(EntityGroupSample g) {
        data_.entityGroups.push_back(std::move(g));
        return data_.entityGroups.back();
    }
    // Keys are plugin-namespaced by convention ("game_2026.fuel.count").
    void setHud(std::string key, Value value) {
        for (auto& h : data_.hud)
            if (h.key == key) { h.value = std::move(value); return; }
        data_.hud.push_back({std::move(key), std::move(value)});
    }

    std::shared_ptr<const ISimSnapshot> build() { return std::make_shared<const SimSnapshot>(std::move(data_)); }

private:
    SimSnapshotData data_;
};

// Latest-value handoff from the sim thread to the UI thread.
class ISnapshotSink {
    WPIEMU_INTERFACE(ISnapshotSink)
public:
    virtual void publish(std::shared_ptr<const ISimSnapshot> snapshot) = 0;  // sim thread
};

class ISnapshotSource {
    WPIEMU_INTERFACE(ISnapshotSource)
public:
    virtual std::shared_ptr<const ISimSnapshot> latest() const = 0;          // any thread; may be null before the first tick
};

// Thread-safe single-slot mailbox implementing both ends.
class LatestSnapshotSlot final : public ISnapshotSink, public ISnapshotSource {
public:
    void publish(std::shared_ptr<const ISimSnapshot> snapshot) override {
        std::lock_guard lock(mutex_);
        snapshot_ = std::move(snapshot);
    }
    std::shared_ptr<const ISimSnapshot> latest() const override {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const ISimSnapshot> snapshot_;
};

}  // namespace wpiemu::sdk
