#include "remote_rigs.h"

#include <algorithm>

namespace wpiemu::app::runview {
namespace {

constexpr float kMarkerShare = 0.6f;       // of the frame box's footprint
constexpr float kMarkerThicknessM = 0.03f;
constexpr float kLabelClearanceM = 0.25f;  // above the frame box

bool sameIdentity(const RemoteRig& rig, const RemoteParticipant& p) {
    return rig.name() == p.name && rig.station() == p.station && rig.connected() == p.connected;
}

}  // namespace

sdk::RobotConfig robotConfigOfAppearance(const sdk::match::RobotAppearance& appearance) {
    sdk::RobotConfig config;
    config.chassis = sdk::match::toChassisSpec(appearance);
    config.drivetrain.spec.wheels = sdk::match::toWheelSpecs(appearance);
    for (const sdk::match::AppearanceZone& zone : appearance.zones) {
        sdk::ZoneConfig z = sdk::match::toZoneConfig(zone);
        // A remote robot has no mechanisms: its zones sit on the chassis at their rest pose.
        z.attachment = {};
        z.local = sdk::match::appearanceZoneRestPose(zone);
        config.zones.push_back(std::move(z));
    }
    return config;
}

RemoteRig::RemoteRig(const RemoteParticipant& participant, const std::vector<sdk::ZoneKindInfo>& kinds) {
    setIdentity(participant);
    id_ = participant.id;
    appearance_ = participant.appearance;
    const auto config = std::make_shared<const sdk::RobotConfig>(robotConfigOfAppearance(*appearance_));
    robot_.setConfig(config);
    zones_.setScene(config, {}, kinds);

    const sdk::match::RobotAppearance& a = *appearance_;
    const sdk::Vec3 size = config->chassis.sizeM;
    const sdk::Vec3 centre{sdk::match::appearanceMetres(a.centreXMm), sdk::match::appearanceMetres(a.centreYMm),
                           sdk::match::appearanceMetres(a.centreZMm)};
    marker_.name = "alliance";
    marker_.size = {size.x * kMarkerShare, size.y * kMarkerShare, kMarkerThicknessM};
    marker_.pose.position = {centre.x, centre.y, centre.z + 0.5f * size.z + 0.5f * kMarkerThicknessM};
    labelAnchor_ = {centre.x, centre.y, centre.z + 0.5f * size.z + kLabelClearanceM};
}

void RemoteRig::setIdentity(const RemoteParticipant& participant) {
    name_ = participant.name;
    station_ = participant.station;
    connected_ = participant.connected;
}

RemoteRig::Changes RemoteRig::update(const sdk::RobotSample* sample) {
    Changes changes;
    const bool present = sample != nullptr;
    changes.presence = present != present_;
    present_ = present;
    if (!present) return changes;

    if (!(sample->body.pose == pose_)) {
        pose_ = sample->body.pose;
        changes.pose = true;
    }
    RobotRig::Input robotInput;
    robotInput.chassis = &sample->body;
    robotInput.wheels = sample->wheels;
    robotInput.wheelsByIndex = true;
    changes.robotParts = robot_.update(robotInput);

    ZoneRig::Input zoneInput;
    zoneInput.zones = sample->zones;
    zoneInput.byIndex = true;
    zoneInput.reportedPoses = true;
    changes.zoneParts = zones_.update(zoneInput);
    return changes;
}

RemoteRigSet::RosterDiff RemoteRigSet::setZoneKinds(std::vector<sdk::ZoneKindInfo> kinds) {
    kinds_ = std::move(kinds);
    RosterDiff diff;
    for (std::size_t i = 0; i < rigs_.size(); ++i) diff.removed.push_back(i);
    rigs_.clear();
    changes_.clear();
    for (const RemoteParticipant& p : roster_ ? *roster_ : MatchRoster{}) {
        if (!p.appearance || p.appearance->empty()) continue;
        diff.added.push_back(rigs_.size());
        rigs_.push_back(std::make_unique<RemoteRig>(p, kinds_));
    }
    changes_.assign(rigs_.size(), {});
    return diff;
}

RemoteRigSet::RosterDiff RemoteRigSet::setRoster(std::shared_ptr<const MatchRoster> roster) {
    roster_ = std::move(roster);
    RosterDiff diff;

    std::vector<std::unique_ptr<RemoteRig>> next;
    std::vector<bool> kept(rigs_.size(), false);
    if (roster_) {
        for (const RemoteParticipant& p : *roster_) {
            if (!p.appearance || p.appearance->empty()) continue;
            const auto old = std::find_if(rigs_.begin(), rigs_.end(), [&](const auto& r) {
                return r && r->id() == p.id && (r->appearance() == p.appearance || *r->appearance() == *p.appearance);
            });
            if (old != rigs_.end()) {
                kept[static_cast<std::size_t>(old - rigs_.begin())] = true;
                if (!sameIdentity(**old, p)) {
                    (*old)->setIdentity(p);
                    diff.identityChanged = true;
                }
                next.push_back(std::move(*old));
            } else {
                diff.added.push_back(next.size());
                next.push_back(std::make_unique<RemoteRig>(p, kinds_));
            }
        }
    }
    for (std::size_t i = 0; i < kept.size(); ++i)
        if (!kept[i]) diff.removed.push_back(i);

    // A reordering without joins or leaves (the roster is in join order, so only a replaced rig can move) is not tracked:
    // rows are matched by position.
    rigs_ = std::move(next);
    changes_.assign(rigs_.size(), {});
    return diff;
}

bool RemoteRigSet::update(std::span<const sdk::RobotSample> samples) {
    changes_.assign(rigs_.size(), {});
    bool any = false;
    for (std::size_t i = 0; i < rigs_.size(); ++i) {
        const auto sample = std::find_if(samples.begin(), samples.end(),
                                         [&](const sdk::RobotSample& s) { return s.participantId == rigs_[i]->id() && !s.isHost; });
        changes_[i] = rigs_[i]->update(sample != samples.end() ? &*sample : nullptr);
        any = any || changes_[i].any();
    }
    return any;
}

const RemoteRig* RemoteRigSet::find(sdk::match::ParticipantId id) const {
    for (const auto& r : rigs_)
        if (r->id() == id) return r.get();
    return nullptr;
}

}  // namespace wpiemu::app::runview
