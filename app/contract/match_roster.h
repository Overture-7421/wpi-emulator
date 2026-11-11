#pragma once

// The participants of a hosted practice match as the run view needs them to draw the other robots: who they are and how
// their robot looks. SDK types only (the shell adapts its hosting status to this), so the run view stays free of the host
// and of core. Where each robot is arrives separately, in ISimSnapshot::remoteRobots(), matched by participant id.

#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::app {

struct RemoteParticipant {
    sdk::match::ParticipantId id = sdk::match::kNoParticipant;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    bool connected = true;   // false: dropped, resumable; its robot stays on the field
    // Null or empty until the host knows how the robot looks (such a participant is listed but not drawn).
    std::shared_ptr<const sdk::match::RobotAppearance> appearance;

    bool operator==(const RemoteParticipant&) const = default;
};

// The remote participants (never the host's own robot), in join order. Immutable once handed out.
using MatchRoster = std::vector<RemoteParticipant>;

}  // namespace wpiemu::app
