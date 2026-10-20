#pragma once

// Start positions: named field poses the user can put the robot back on from the run view
// (RobotConfig::startPositions, docs/reference/runtime.md). Stored in SI units, field frame (Z-up NWU).

#include <string>

#include "wpiemu/sdk/game/alliance.h"

namespace wpiemu::sdk {

// Field-frame chassis origin and yaw (counter-clockwise from +X).
struct RobotStartPose {
    float xM = 0.f;
    float yM = 0.f;
    float headingRad = 0.f;

    bool operator==(const RobotStartPose&) const = default;
};

struct StartPosition {
    std::string id;       // unique among start positions
    std::string label;
    Alliance alliance = Alliance::Blue;
    RobotStartPose pose;

    bool operator==(const StartPosition&) const = default;
};

}  // namespace wpiemu::sdk
