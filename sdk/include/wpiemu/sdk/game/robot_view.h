#pragma once

// What a game plugin can see of the robots in a session. A practice match has the host's own robot plus up to
// match::kMaxRobotsPerMatch - 1 remote robots; a plain session has exactly the host robot or none at all.
//
// The runtime (core) owns the robots and implements IRobotDirectory; the game state reads it through
// SimContext::robots() and is told about roster changes through IGameState::onRobotAdded / onRobotRemoved
// (game/game_state.h). Everything here is SIM THREAD only and is valid until the next tick or roster change:
// do not keep a RobotView, the span or the zones pointer across ticks, copy what you need.
//
// A robot's ZONES describe where it picks things up or shoots from (config/zone.h):
//   - local host robot: `zones` is null (use SimContext::config().zones) and the activation of each zone is
//     evaluated from the live mechanisms (evaluateActivation, game/zone_activation.h). `zoneFlags` is unused (0).
//   - remote robot: `zones` points at the zone configs of its accepted Join spec and `zoneFlags` holds the flags the
//     client sent with its latest input (bit i = zone i active). The runtime already forces the flags to 0 while the
//     robot is disabled, e-stopped or the match is paused. There are no mechanism instances: resolve a zone's pose
//     with restPoseOfZone (game/zone_pose.h).

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/ids.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/game/alliance.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk {

using match::ParticipantId;

struct RobotView {
    ParticipantId participantId = match::kHostParticipantId;
    std::string name;
    proto::AllianceStation station = proto::AllianceStation::Red1;
    BodyId body;                                // the robot's rigid body in SimContext::physics()
    const std::vector<ZoneConfig>* zones = nullptr;  // see the file comment; null = none (host: use the config)
    std::uint32_t zoneFlags = 0;                // remote robots only: bit i = zone i active
    bool enabled = false;                       // the robot may act (match enabled, not e-stopped, not paused)
    bool isHost = false;                        // the host's own WPILib robot (participantId == kHostParticipantId)

    Alliance alliance() const { return proto::allianceOf(station); }
    std::size_t zoneCount() const { return zones != nullptr ? zones->size() : 0; }
    // Remote robots: the client's flag for zone `index`; false when out of range or past the 32 flag bits.
    bool zoneFlagActive(std::size_t index) const {
        return index < 32 && index < zoneCount() && ((zoneFlags >> index) & 1u) != 0;
    }
};

class IRobotDirectory {
    WPIEMU_INTERFACE(IRobotDirectory)
public:
    // Every robot of the session, host first, then remote robots in join order. Stable order between roster changes.
    virtual std::span<const RobotView> robots() const = 0;
    // nullptr when no robot has this participant id.
    virtual const RobotView* find(ParticipantId id) const {
        for (const RobotView& r : robots())
            if (r.participantId == id) return &r;
        return nullptr;
    }
};

// The directory of a session that has no roster: always empty.
class NullRobotDirectory final : public IRobotDirectory {
public:
    std::span<const RobotView> robots() const override { return {}; }
};

}  // namespace wpiemu::sdk
