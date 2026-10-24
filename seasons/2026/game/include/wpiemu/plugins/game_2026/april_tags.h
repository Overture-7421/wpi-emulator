#pragma once

// REBUILT (2026) AprilTag layout: the official WPILib "2026-rebuilt-welded" layout (32 tags, 36h11 family).

#include "wpiemu/sdk/vision/apriltag.h"

namespace wpiemu::plugins::game_2026 {

// Poses are in the SDK field frame (the blue-origin WPILib frame, +X toward the red wall); a tag's +X points out
// of its face. Tag size is 6.5 in (0.1651 m).
sdk::AprilTagLayout game2026AprilTagLayout();

}  // namespace wpiemu::plugins::game_2026
