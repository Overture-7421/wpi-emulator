#pragma once

// Field AprilTag layout. Season data belongs to the game plugin (IGameModule::aprilTagLayout); core and the
// vision bridges only consume it. Poses are in the field frame (right-handed Z-up NWU, +X toward the opposing
// wall); a tag's +X axis points OUT of its face (WPILib AprilTagFieldLayout convention).

#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

// Side length of the black square (6.5 in) of every FRC field tag: the same each season, not season data.
inline constexpr double kFrcAprilTagSizeM = 0.1651;

struct AprilTag {
    int id = 0;
    Pose3 pose;               // field frame, centre of the tag; +X out of the face
    double sizeM = kFrcAprilTagSizeM;

    bool operator==(const AprilTag&) const = default;
};

struct AprilTagLayout {
    std::string name;          // e.g. "2026-rebuilt-welded"
    float fieldLengthM = 0.f;
    float fieldWidthM = 0.f;
    std::vector<AprilTag> tags;

    const AprilTag* find(int id) const {
        for (const AprilTag& t : tags)
            if (t.id == id) return &t;
        return nullptr;
    }
    bool empty() const { return tags.empty(); }
};

}  // namespace wpiemu::sdk
