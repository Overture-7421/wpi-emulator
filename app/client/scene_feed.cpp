#include "client/scene_feed.h"

#include <algorithm>

#include "runview/remote_rigs.h"

namespace wpiemu::app::client {
namespace {

namespace mc = core::match;

sdk::BodySample bodyOf(const char* name, const sdk::Pose3& pose, const sdk::Vec3& linear, const sdk::Vec3& angular) {
    sdk::BodySample body;
    body.name = name;
    body.pose = pose;
    body.linearVelocity = linear;
    body.angularVelocity = angular;
    return body;
}

std::vector<sdk::WheelSample> wheelsOf(const std::vector<mc::WheelView>& wheels) {
    std::vector<sdk::WheelSample> out;
    out.reserve(wheels.size());
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        sdk::WheelSample sample;
        sample.name = "wheel" + std::to_string(i);   // the names robotConfigOfAppearance gives the wheels
        sample.steerAngleRad = wheels[i].steerRad;
        sample.spinAngleRad = wheels[i].spinRad;
        out.push_back(std::move(sample));
    }
    return out;
}

std::vector<sdk::ZoneState> zonesOf(std::uint32_t flags, const std::vector<sdk::Pose3>& poses, const std::vector<sdk::ZoneConfig>* ids) {
    std::vector<sdk::ZoneState> out;
    out.reserve(poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
        sdk::ZoneState zone;
        zone.id = ids != nullptr && i < ids->size() ? (*ids)[i].id : "zone" + std::to_string(i);
        zone.active = i < 32 && ((flags >> i) & 1u) != 0;
        zone.poseInChassis = poses[i];
        out.push_back(std::move(zone));
    }
    return out;
}

void addHud(std::vector<sdk::HudValue>& out, const std::vector<mc::HudEntry>& entries) {
    for (const mc::HudEntry& e : entries) {
        if (e.key.empty()) continue;
        const auto at = std::find_if(out.begin(), out.end(), [&](const sdk::HudValue& v) { return v.key == e.key; });
        if (at != out.end()) at->value = e.value;
        else out.push_back({e.key, e.value});
    }
}

}  // namespace

std::shared_ptr<const sdk::RobotConfig> sceneConfigOf(const mc::OwnRobotView& own) {
    if (!own.appearance || own.appearance->empty()) return nullptr;
    return std::make_shared<const sdk::RobotConfig>(runview::robotConfigOfAppearance(*own.appearance));
}

HostScene makeClientScene(const sdk::IGameModule* game, std::shared_ptr<const sdk::FieldDescription> field,
                          std::shared_ptr<const sdk::RobotConfig> ownConfig, sdk::proto::AllianceStation station) {
    HostProject project;
    project.config = std::move(ownConfig);
    project.game = game;
    project.defaultStation = station;
    HostScene scene = makeHostScene(project);
    if (field) scene.field = std::move(field);
    return scene;
}

std::shared_ptr<const MatchRoster> rosterOf(const mc::MatchClientView& view) {
    auto roster = std::make_shared<MatchRoster>();
    roster->reserve(view.remotes.size());
    for (const mc::RemoteRobotView& r : view.remotes) {
        RemoteParticipant p;
        p.id = r.id;
        p.name = r.name;
        p.station = r.station;
        p.appearance = r.appearance;
        roster->push_back(std::move(p));
    }
    return roster;
}

std::shared_ptr<const sdk::ISimSnapshot> snapshotOf(const mc::MatchClientView& view, const sdk::RobotConfig* ownConfig,
                                                    const std::string& phaseHudKey) {
    if (!view.hasOwn) return nullptr;
    sdk::SimSnapshotData data;
    data.tick = view.revision;
    data.simTimeUs = view.renderSimTimeUs;
    data.paused = view.hud.state == sdk::match::MatchState::Paused;
    data.robotConnected = view.connection.state == mc::ConnectionState::Live || view.connection.state == mc::ConnectionState::Lobby ||
                          view.connection.state == mc::ConnectionState::Paused;

    const mc::OwnRobotView& own = view.own;
    data.chassis = bodyOf("chassis", own.pose, own.linearVelocity, own.angularVelocity);
    data.wheels = wheelsOf(own.wheels);
    data.zones = zonesOf(own.zoneFlags, own.zonePoses, ownConfig != nullptr ? &ownConfig->zones : nullptr);

    addHud(data.hud, view.hud.game);
    addHud(data.hud, own.hud);
    if (!phaseHudKey.empty() && !view.hud.phaseKey.empty()) data.hud.push_back({phaseHudKey, view.hud.phaseKey});

    for (const mc::EntityGroupView& group : view.entities) {
        sdk::EntityGroupSample sample;
        sample.group = "group" + std::to_string(group.groupId);
        sample.radiusM = kEntityRadiusM;
        sample.color = kEntityColor;
        sample.positions.reserve(group.entities.size());
        sample.ids.reserve(group.entities.size());
        for (const mc::EntityView& e : group.entities) {
            sample.positions.push_back({e.x, e.y, e.z});
            sample.ids.push_back(e.id);
        }
        data.entityGroups.push_back(std::move(sample));
    }

    for (const mc::RemoteRobotView& r : view.remotes) {
        sdk::RobotSample sample;
        sample.participantId = r.id;
        sample.name = r.name;
        sample.station = r.station;
        sample.body = bodyOf("chassis", r.pose, r.linearVelocity, r.angularVelocity);
        sample.wheels = wheelsOf(r.wheels);
        sample.zones = zonesOf(r.zoneFlags, r.zonePoses, nullptr);
        addHud(sample.hud, r.hud);
        sample.isHost = false;   // the remote rigs draw the host's robot like any other
        data.remoteRobots.push_back(std::move(sample));
    }

    data.driverStation.enabled = view.hud.enabled;
    data.driverStation.eStop = view.hud.eStop;
    data.driverStation.mode = view.hud.mode;
    data.driverStation.station = own.station;
    data.driverStation.matchTimeS = view.hud.matchTimeS;
    return std::make_shared<const sdk::SimSnapshot>(std::move(data));
}

}  // namespace wpiemu::app::client
