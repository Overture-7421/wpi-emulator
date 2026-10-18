#pragma once

// AprilTag field layouts: where the fiducials are on the field. Game plugins contribute them
// through IGameModule::contributeFiducialLayouts() (the first one is the plugin default); a WPILib
// AprilTagFieldLayout JSON file (VisionConfig::layoutFile) can replace them for custom or
// off-season fields.
//
// Poses are in the SDK world frame (Z-up NWU = WPILib blue origin) and follow the WPILib tag-frame
// convention: the origin is the tag centre, +X is the outward normal (out of the tag face), +Z up
// and +Y the viewer's right when looking at the face. The four corners are always listed BL, BR,
// TR, TL as seen from the front (vision_types.h).

#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

struct FiducialTag {
    int id = 0;
    Pose3 pose;          // tag frame in the world frame
    double sizeM = 0.0;  // edge length of the black square; 0 => FiducialLayout::tagSizeM

    bool operator==(const FiducialTag&) const = default;
};

struct FiducialLayout {
    std::string id;  // "game_2026.welded"
    std::string displayName;
    double tagSizeM = 0.1651;  // default tag edge length (6.5 in)
    float fieldLengthM = 0.f;
    float fieldWidthM = 0.f;
    std::vector<FiducialTag> tags;

    const FiducialTag* find(int tagId) const {
        for (const FiducialTag& t : tags)
            if (t.id == tagId) return &t;
        return nullptr;
    }

    bool operator==(const FiducialLayout&) const = default;
};

}  // namespace wpiemu::sdk
