#pragma once

// Start positions: named field poses the user can put the robot back on from the run view
// (RobotConfig::startPositions, ADR 0020). Stored in SI units, field frame (Z-up NWU).

#include <string>
#include <string_view>

namespace wpiemu::sdk {

// Field-frame chassis origin and yaw (counter-clockwise from +X).
struct RobotStartPose {
    float xM = 0.f;
    float yM = 0.f;
    float headingRad = 0.f;

    bool operator==(const RobotStartPose&) const = default;
};

enum class Alliance { Blue, Red };

constexpr Alliance otherAlliance(Alliance a) { return a == Alliance::Blue ? Alliance::Red : Alliance::Blue; }
constexpr std::string_view allianceKey(Alliance a) { return a == Alliance::Blue ? "blue" : "red"; }

struct StartPosition {
    std::string id;       // unique among start positions
    std::string label;
    Alliance alliance = Alliance::Blue;
    RobotStartPose pose;

    bool operator==(const StartPosition&) const = default;
};

}  // namespace wpiemu::sdk
