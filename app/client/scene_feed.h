#pragma once

// The adapter between what a game client knows (MatchClientView: predicted own robot, interpolated remote robots, entities, HUD)
// and what the run view's scene already draws (ISimSnapshot, HostScene, MatchRoster). The own robot becomes the snapshot's
// chassis, wheels, zones and HUD, every other robot (the host's included) a RobotSample, so the client renders with the same
// RobotRig, ZoneRig, remote-rig and HUD code as the desktop app and nothing is drawn twice. Pure data conversion, no Qt widgets.

#include <memory>
#include <string>

#include "client/client_services.h"
#include "contract/match_roster.h"
#include "contract/runtime_host.h"
#include "wpiemu/core/match/client_view.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::client {

// What the wire does not say about an entity: it carries positions only. Every group is drawn as this sphere.
inline constexpr float kEntityRadiusM = 0.075f;
inline constexpr const char* kEntityColor = "#ffd000";

// The robot as the scene draws it: the appearance the host sent (the accepted, clamped spec), without mechanisms. Null before
// the roster carried it.
std::shared_ptr<const sdk::RobotConfig> sceneConfigOf(const core::match::OwnRobotView& own);

// The static scene: the game's HUD, zone kinds, tags and sounds, the field the prediction world holds and the own robot.
// `field` and `ownConfig` may be null early in a join (a bare floor is drawn).
HostScene makeClientScene(const sdk::IGameModule* game, std::shared_ptr<const sdk::FieldDescription> field,
                          std::shared_ptr<const sdk::RobotConfig> ownConfig, sdk::proto::AllianceStation station);

// Every other participant of the match (the host's robot included), in roster order, with how to draw them.
std::shared_ptr<const MatchRoster> rosterOf(const core::match::MatchClientView& view);

// The picture of one view. `phaseHudKey` is the game's phase HUD key (phaseHudKeyOf(descriptors)): the host's phase text is
// published under it so the scoreboard shows it. Null when there is no own robot yet.
std::shared_ptr<const sdk::ISimSnapshot> snapshotOf(const core::match::MatchClientView& view, const sdk::RobotConfig* ownConfig,
                                                    const std::string& phaseHudKey);

}  // namespace wpiemu::app::client
