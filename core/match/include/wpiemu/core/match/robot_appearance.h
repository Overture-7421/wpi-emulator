#pragma once

// The roster appearance (sdk/match/robot_appearance.h) of a robot config. Built from a config the host has already
// accepted and clamped (spec_intake.h) or from the host's own trusted config, never from a client's JSON.

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::core::match {

// The drawable robot: chassis frame box, primitives, wheel hubs (physics::wheelHubLocal, the position the simulation
// uses) and zone rest poses, each list cut to its wire cap (see makeRobotAppearance).
sdk::match::RobotAppearance appearanceOf(const sdk::RobotConfig& config);

}  // namespace wpiemu::core::match
