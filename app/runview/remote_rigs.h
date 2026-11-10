#pragma once

// Qt-free drawing state of the OTHER robots of a hosted practice match: one rig per remote participant, built from the
// appearance the host sent (chassis box and primitives, wheels, zone volumes) and driven by the RobotSample of that
// participant in the snapshot the run view already consumes. Reuses RobotRig and ZoneRig, so a remote robot is drawn
// exactly like the local one; coordinates stay SDK (Z-up NWU) and the view layer converts.

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "contract/match_roster.h"
#include "scene_parts.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::runview {

// The robot description a remote appearance stands for: the frame box and primitives, the wheels (named wheel0...) and the
// zones at their rest poses on the chassis. Everything a RobotRig and a ZoneRig need, nothing a simulation would.
sdk::RobotConfig robotConfigOfAppearance(const sdk::match::RobotAppearance& appearance);

class RemoteRig {
public:
    RemoteRig(const RemoteParticipant& participant, const std::vector<sdk::ZoneKindInfo>& kinds);

    sdk::match::ParticipantId id() const { return id_; }
    const std::string& name() const { return name_; }
    sdk::proto::AllianceStation station() const { return station_; }
    bool connected() const { return connected_; }
    const std::shared_ptr<const sdk::match::RobotAppearance>& appearance() const { return appearance_; }

    // True while the latest snapshot described this robot; otherwise it is not drawn (it keeps its last pose).
    bool present() const { return present_; }
    // The chassis in the field frame, as of the last update.
    const sdk::Pose3& pose() const { return pose_; }

    const RobotRig& robot() const { return robot_; }
    const ZoneRig& zones() const { return zones_; }
    // A flat plate on top of the frame box in the chassis frame, for the alliance colour.
    const ScenePart& marker() const { return marker_; }
    // Where a name label hangs, in the chassis frame (above the frame box).
    const sdk::Vec3& labelAnchor() const { return labelAnchor_; }

    // What the last update changed.
    struct Changes {
        bool presence = false;
        bool pose = false;
        bool robotParts = false;
        bool zoneParts = false;
        bool any() const { return presence || pose || robotParts || zoneParts; }
    };
    Changes update(const sdk::RobotSample* sample);

    // The roster may rename a participant or move it to another station without changing the robot.
    void setIdentity(const RemoteParticipant& participant);

private:
    sdk::match::ParticipantId id_ = sdk::match::kNoParticipant;
    std::string name_;
    sdk::proto::AllianceStation station_ = sdk::proto::AllianceStation::Red1;
    bool connected_ = true;
    std::shared_ptr<const sdk::match::RobotAppearance> appearance_;
    bool present_ = false;
    sdk::Pose3 pose_;
    RobotRig robot_;
    ZoneRig zones_;
    ScenePart marker_;
    sdk::Vec3 labelAnchor_;
};

// The rigs of a match, one per remote participant whose appearance is known, in roster order.
class RemoteRigSet {
public:
    // Which rows a roster change removed (indexes before the change, ascending) and added (indexes after it, ascending).
    // A participant whose appearance changed is in both.
    struct RosterDiff {
        std::vector<std::size_t> removed;
        std::vector<std::size_t> added;
        bool identityChanged = false;   // a name, station or connection state changed in place
        bool any() const { return !removed.empty() || !added.empty() || identityChanged; }
    };

    // Zone kinds style the zone volumes (colour, volume or muzzle); a zone whose kind is unknown is not drawn. A change
    // rebuilds every rig (all rows removed and added).
    RosterDiff setZoneKinds(std::vector<sdk::ZoneKindInfo> kinds);
    // nullptr = nobody. Participants without an appearance are listed by the roster but have no rig.
    RosterDiff setRoster(std::shared_ptr<const MatchRoster> roster);
    // Pairs each rig with its sample (by participant id). A rig without a sample is not present.
    // Returns true when anything visible changed.
    bool update(std::span<const sdk::RobotSample> samples);

    std::size_t size() const { return rigs_.size(); }
    RemoteRig& at(std::size_t row) { return *rigs_[row]; }
    const RemoteRig& at(std::size_t row) const { return *rigs_[row]; }
    const RemoteRig* find(sdk::match::ParticipantId id) const;
    // What the last update() changed in each row.
    const std::vector<RemoteRig::Changes>& changes() const { return changes_; }

private:
    std::shared_ptr<const MatchRoster> roster_;
    std::vector<sdk::ZoneKindInfo> kinds_;
    std::vector<std::unique_ptr<RemoteRig>> rigs_;
    std::vector<RemoteRig::Changes> changes_;
};

}  // namespace wpiemu::app::runview
