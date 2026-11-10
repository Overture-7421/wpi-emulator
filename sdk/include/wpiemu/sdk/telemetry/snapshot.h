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
#include <optional>
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
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
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
    // OPTIONAL stable id of each entity, parallel to `positions` (an id never changes for the life of the entity
    // within a match, and is unique within the group). Empty = unidentified (the group cannot be delta-encoded for
    // a match). A non-empty list must have exactly positions.size() entries: see entityIdsConsistent.
    std::vector<std::uint32_t> ids;
};

// The invariant of EntityGroupSample::ids: absent, or one id per position.
inline bool entityIdsConsistent(const EntityGroupSample& group) {
    return group.ids.empty() || group.ids.size() == group.positions.size();
}

struct HudValue {
    std::string key;   // matches HudWidgetDescriptor::valueKey
    Value value;
};

// Runtime state of one robot zone (RobotConfig::zones): whether its activation currently holds.
struct ZoneState {
    std::string id;       // ZoneConfig::id
    bool active = false;
    // Where the zone is, in the chassis frame of its robot: the host robot's zones follow their mechanisms, a remote
    // robot's are the effective pose its client commanded (clamped). Unset by runtimes that do not report it.
    std::optional<Pose3> poseInChassis;
};

// Runtime state of one simulated camera (RobotConfig::cameras).
struct CameraState {
    std::string id;                 // CameraConfig::id
    Pose3 worldPose;                // camera frame in the field frame (+X forward) at the last capture
    bool tagsVisible = false;       // the last captured frame saw at least one tag
    std::vector<int> visibleTagIds; // tag ids in the last captured frame
};

// Runtime state of one robot sensor (RobotConfig::sensors).
struct SensorState {
    std::string id;                      // SensorConfig::id
    std::optional<double> sourceValue;   // the game value it read this tick; nullopt = source unavailable
    double reading = 0.0;                // what the robot sees: metres (Distance) or 1/0 digital level (BeamBreak)
};

// One REMOTE robot of a practice match (the host's own robot is the chassis()/wheels()/zones()/hud() of the snapshot).
// Same pieces the host robot has, so the UI renders it with the same rig code.
struct RobotSample {
    match::ParticipantId participantId = match::kHostParticipantId;
    std::string name;
    proto::AllianceStation station = proto::AllianceStation::Red1;
    BodySample body;                 // the chassis: pose and velocities in the field frame
    std::vector<WheelSample> wheels;
    std::vector<ZoneState> zones;    // in the robot's own zone order (its Join spec)
    std::vector<HudValue> hud;       // per-robot HUD (IGameState::publishRobot)
    bool isHost = false;             // true only if a runtime chooses to describe the host robot this way
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
    // Activation state of the robot's zones (empty when the game defines none).
    virtual std::span<const ZoneState> zones() const { return {}; }
    // State of the robot's simulated cameras (empty when none are configured).
    virtual std::span<const CameraState> cameras() const { return {}; }
    // State of the robot's sensors (empty when none are configured).
    virtual std::span<const SensorState> sensors() const { return {}; }
    // The OTHER robots of a practice match (never the host's own robot, which is chassis()/wheels()/zones()/hud()).
    // Empty outside a match.
    virtual std::span<const RobotSample> remoteRobots() const { return {}; }
    // State of the practice match this snapshot belongs to; nullptr = no match session. Valid as long as the snapshot.
    virtual const match::MatchInfo* matchInfo() const { return nullptr; }
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
    std::vector<ZoneState> zones;
    std::vector<CameraState> cameras;
    std::vector<SensorState> sensors;
    std::vector<DiscoveredDevice> devices;
    std::vector<HubWarning> warnings;
    double batteryVoltageV = 12.6;
    proto::DsState driverStation;
    PhysicsStats physicsStats;
    std::vector<RobotSample> remoteRobots;
    std::optional<match::MatchInfo> matchInfo;   // nullopt = no match session
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
    std::span<const ZoneState> zones() const override { return d_.zones; }
    std::span<const CameraState> cameras() const override { return d_.cameras; }
    std::span<const SensorState> sensors() const override { return d_.sensors; }
    std::span<const RobotSample> remoteRobots() const override { return d_.remoteRobots; }
    const match::MatchInfo* matchInfo() const override { return d_.matchInfo ? &*d_.matchInfo : nullptr; }
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
    void addZone(ZoneState z) { data_.zones.push_back(std::move(z)); }
    void addCamera(CameraState c) { data_.cameras.push_back(std::move(c)); }
    void addSensor(SensorState s) { data_.sensors.push_back(std::move(s)); }
    // Remote robots of a practice match (the runtime adds them before IGameState::publishRobot runs).
    void addRobot(RobotSample r) { data_.remoteRobots.push_back(std::move(r)); }
    void setMatchInfo(match::MatchInfo info) { data_.matchInfo = std::move(info); }
    // Sets HUD value `key` of the remote robot `participant` (what IGameState::publishRobot uses). Returns false, and
    // does nothing, when no robot with that id was added.
    bool setRobotHud(match::ParticipantId participant, std::string key, Value value) {
        for (RobotSample& r : data_.remoteRobots) {
            if (r.participantId != participant) continue;
            for (HudValue& h : r.hud)
                if (h.key == key) { h.value = std::move(value); return true; }
            r.hud.push_back({std::move(key), std::move(value)});
            return true;
        }
        return false;
    }
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
