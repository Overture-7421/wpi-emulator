#pragma once

// Test scaffolding for the 2026 plugin with several robots, over g26test::Session: a FakeRobotDirectory bound to the
// session's context, remote robots as kinematic bodies of the fake world and helpers that announce roster changes to the
// game state the way the runtime does (SimulationSession::addRemoteRobot / removeRemoteRobot).

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "test_support.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_robot_directory.h"

namespace g26test {

class Roster {
public:
    // `s` must have been start()ed: the host view uses its chassis body.
    explicit Roster(Session& s) : s_(s) {
        s.env.ctx.bindRobots(&dir);
        sdk::RobotView host;
        host.participantId = sdk::match::kHostParticipantId;
        host.name = "Host";
        host.station = s.env.driverStation.station;
        host.body = s.chassis;
        host.enabled = true;
        host.isHost = true;
        dir.add(host);
    }

    // A remote robot: a kinematic body at `position`/`yaw` and the given zones; announced to the game state.
    void add(sdk::ParticipantId id, sdk::proto::AllianceStation station, std::vector<sdk::ZoneConfig> zones,
             const sdk::Vec3& position, float yaw = 0.f, bool enabled = true) {
        sdk::BodyDesc desc;
        desc.type = sdk::BodyType::Kinematic;
        desc.pose = sdk::Pose3{position, sdk::Quat::fromYaw(yaw)};
        sdk::RobotView view;
        view.participantId = id;
        view.name = "Remote" + std::to_string(id);
        view.station = station;
        view.body = s_.env.physics.createBody(desc);
        zones_[id] = std::make_unique<std::vector<sdk::ZoneConfig>>(std::move(zones));
        view.zones = zones_[id].get();
        view.enabled = enabled;
        dir.add(view);
        s_.game->onRobotAdded(s_.env.ctx, *dir.list.rbegin());
    }
    void remove(sdk::ParticipantId id) {
        dir.remove(id);
        s_.game->onRobotRemoved(s_.env.ctx, id);
    }

    sdk::RobotView& view(sdk::ParticipantId id) {
        for (sdk::RobotView& r : dir.list)
            if (r.participantId == id) return r;
        FAIL("no robot " << id);
        return dir.list.front();
    }
    void setFlags(sdk::ParticipantId id, std::uint32_t flags) { view(id).zoneFlags = flags; }
    void setEnabled(sdk::ParticipantId id, bool enabled) { view(id).enabled = enabled; }
    void place(sdk::ParticipantId id, const sdk::Vec3& position, float yaw = 0.f) {
        s_.env.physics.setPose(view(id).body, sdk::Pose3{position, sdk::Quat::fromYaw(yaw)});
    }

    sdk::testing::FakeRobotDirectory dir;

private:
    Session& s_;
    std::map<sdk::ParticipantId, std::unique_ptr<std::vector<sdk::ZoneConfig>>> zones_;
};

// Pickup zone of a remote robot: no activation conditions are read, the client's flag decides.
inline sdk::ZoneConfig remotePickup(const std::string& id, double rateHz = 0.0) {
    sdk::Value p = sdk::Value::makeObject();
    p["intakeRateHz"] = rateHz;
    return makeZone(id, plugins::game_2026::kPickupKind, p);
}

inline sdk::ZoneConfig remoteShooter(const std::string& id, sdk::Value props = sdk::Value::makeObject()) {
    return makeZone(id, plugins::game_2026::kShooterKind, std::move(props));
}

// The per-robot HUD value `key` of remote robot `id` in a snapshot built by publish + publishRobot, or nullptr.
inline const sdk::Value* robotHud(const sdk::ISimSnapshot& snap, sdk::ParticipantId id, std::string_view key) {
    for (const sdk::RobotSample& r : snap.remoteRobots()) {
        if (r.participantId != id) continue;
        for (const sdk::HudValue& h : r.hud)
            if (h.key == key) return &h.value;
    }
    return nullptr;
}

// What the runtime's publish pass does: the game's own publish, one RobotSample per remote robot, then publishRobot.
inline std::shared_ptr<const sdk::ISimSnapshot> publishAll(Session& s, Roster& roster) {
    sdk::SnapshotBuilder out;
    s.stateOwner->publish(out);
    for (const sdk::RobotView& r : roster.dir.list) {
        if (r.isHost) continue;
        sdk::RobotSample sample;
        sample.participantId = r.participantId;
        sample.name = r.name;
        sample.station = r.station;
        out.addRobot(std::move(sample));
    }
    for (const sdk::RobotView& r : roster.dir.list)
        if (!r.isHost) s.stateOwner->publishRobot(out, r);
    return out.build();
}

}  // namespace g26test
