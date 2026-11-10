#include "wpiemu/core/match/wire_frame.h"

#include <algorithm>
#include <cmath>

namespace wpiemu::core::match {

namespace {

namespace wire = sdk::match;

void fillBody(wire::RobotWireState& out, const sdk::BodySample& body) {
    out.xMm = wire::quantizeMilliI32(body.pose.position.x);
    out.yMm = wire::quantizeMilliI32(body.pose.position.y);
    out.yawQ = wire::quantizeAngle(body.pose.rotation.yaw());
    out.vxMmps = wire::quantizeMilliI16(body.linearVelocity.x);
    out.vyMmps = wire::quantizeMilliI16(body.linearVelocity.y);
    out.omegaMradps = wire::quantizeMilliI16(body.angularVelocity.z);
}

void fillWheels(wire::RobotWireState& out, std::span<const sdk::WheelSample> wheels) {
    const std::size_t n = std::min(wheels.size(), wire::kMaxWheelsPerRobot);
    out.wheels.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        out.wheels.push_back({wire::quantizeAngle(wheels[i].steerAngleRad), wire::quantizeAngle(wheels[i].spinAngleRad)});
}

void fillZones(wire::RobotWireState& out, std::span<const sdk::ZoneState> zones) {
    for (std::size_t i = 0; i < zones.size() && i < wire::kMaxZonesPerRobot; ++i)
        if (zones[i].active) out.zoneFlags |= wire::zoneFlagBit(i);
}

void fillZonePoses(wire::RobotWireState& out, std::span<const sdk::ZoneState> zones, const ZoneRestPoses& restPoses) {
    const auto rest = restPoses.find(out.participantId);
    if (rest == restPoses.end() || rest->second.empty()) return;
    std::vector<sdk::Pose3> effective(rest->second.begin(), rest->second.end());
    for (std::size_t i = 0; i < zones.size() && i < effective.size(); ++i)
        if (zones[i].poseInChassis) effective[i] = *zones[i].poseInChassis;
    out.zonePoses = wire::selectSnapshotZonePoses(rest->second, effective, out.zoneFlags);
}

void fillHud(std::vector<wire::HudValue>& out, std::span<const sdk::HudValue> hud, const HudKeyTable& keys, std::size_t cap,
             std::size_t& unknown) {
    for (const sdk::HudValue& h : hud) {
        const std::optional<float> v = hudWireValue(h.value);
        if (!v) continue;
        const std::optional<std::uint16_t> id = keys.idOf(h.key);
        if (!id) {
            ++unknown;
            continue;
        }
        out.push_back({*id, *v});
    }
    std::sort(out.begin(), out.end(), [](const wire::HudValue& a, const wire::HudValue& b) { return a.keyId < b.keyId; });
    out.erase(std::unique(out.begin(), out.end(), [](const wire::HudValue& a, const wire::HudValue& b) { return a.keyId == b.keyId; }),
              out.end());
    if (out.size() > cap) out.resize(cap);
}

}  // namespace

std::optional<float> hudWireValue(const sdk::Value& value) {
    if (value.isBool()) return value.asBool() ? 1.f : 0.f;
    if (!value.isNumber()) return std::nullopt;
    const double d = value.asDouble();
    if (!std::isfinite(d)) return 0.f;
    return static_cast<float>(d);
}

WireFrame buildWireFrame(const sdk::ISimSnapshot& snapshot, const HudKeyTable& hudKeys, const wire::MatchInfo& info,
                         const ZoneRestPoses& zoneRestPoses, EntityMotionTracker* motion) {
    WireFrame frame;
    frame.tick = snapshot.tick();
    frame.simTimeUs = snapshot.simTimeUs();
    frame.info = info;

    wire::RobotWireState host;
    host.participantId = wire::kHostParticipantId;
    fillBody(host, snapshot.chassis());
    fillWheels(host, snapshot.wheels());
    fillZones(host, snapshot.zones());
    fillZonePoses(host, snapshot.zones(), zoneRestPoses);
    frame.robots.push_back(std::move(host));

    std::vector<const sdk::RobotSample*> remotes;
    for (const sdk::RobotSample& r : snapshot.remoteRobots()) remotes.push_back(&r);
    std::sort(remotes.begin(), remotes.end(),
              [](const sdk::RobotSample* a, const sdk::RobotSample* b) { return a->participantId < b->participantId; });
    for (const sdk::RobotSample* r : remotes) {
        if (frame.robots.size() >= wire::kMaxRobotsPerMatch) break;
        wire::RobotWireState out;
        out.participantId = r->participantId;
        fillBody(out, r->body);
        fillWheels(out, r->wheels);
        fillZones(out, r->zones);
        fillZonePoses(out, r->zones, zoneRestPoses);
        fillHud(out.hud, r->hud, hudKeys, wire::kMaxHudValuesPerRobot, frame.hudKeysUnknown);
        frame.robots.push_back(std::move(out));
    }
    for (const wire::RobotWireState& r : frame.robots)
        frame.anchors.push_back({static_cast<std::int32_t>(std::lround(r.xMm / 10.0)), static_cast<std::int32_t>(std::lround(r.yMm / 10.0))});

    fillHud(frame.gameHud, snapshot.hud(), hudKeys, wire::kMaxGameHudValues, frame.hudKeysUnknown);

    const std::span<const sdk::EntityGroupSample> groups = snapshot.entityGroups();
    for (std::size_t g = 0; g < groups.size() && g < wire::kMaxEntityGroups; ++g) {
        const sdk::EntityGroupSample& group = groups[g];
        if (group.ids.empty() || group.ids.size() != group.positions.size()) {
            ++frame.groupsWithoutIds;
            continue;
        }
        EntityGroupFrame out;
        out.groupId = static_cast<std::uint16_t>(g);
        // Ascending id, duplicates dropped; the raw positions (metres) travel with their entity for the motion tracker.
        std::vector<std::size_t> order(group.ids.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return group.ids[a] < group.ids[b]; });
        order.erase(std::unique(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return group.ids[a] == group.ids[b]; }),
                    order.end());
        std::vector<sdk::Vec3> raw;
        out.entities.reserve(order.size());
        raw.reserve(order.size());
        for (const std::size_t i : order) {
            wire::EntityState e;
            e.id = group.ids[i];
            e.xCm = wire::quantizeCentiI16(group.positions[i].x);
            e.yCm = wire::quantizeCentiI16(group.positions[i].y);
            e.zCm = wire::quantizeCentiI16(group.positions[i].z);
            out.entities.push_back(e);
            raw.push_back(group.positions[i]);
        }
        if (motion != nullptr) motion->observe(out.groupId, frame.simTimeUs, raw, out.entities, out.unsettled);
        frame.groups.push_back(std::move(out));
    }
    return frame;
}

}  // namespace wpiemu::core::match
