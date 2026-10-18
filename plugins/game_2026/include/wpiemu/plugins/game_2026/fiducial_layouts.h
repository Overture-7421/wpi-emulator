#pragma once

// The 2026 (REBUILT) AprilTag layouts as data. The tag tables are generated from WPILib's layout
// resources by tools/gen_fiducial_layout.py into src/fiducial_layouts.cpp (provenance in its header
// comment, how to regenerate in docs/game_2026.md); the module hands them out through
// IGameModule::contributeFiducialLayouts().

#include <vector>

#include "wpiemu/sdk/vision/fiducial_layout.h"

namespace wpiemu::plugins::game_2026 {

// "game_2026.welded" first (the plugin default), then "game_2026.andymark". Poses are in the SDK
// field frame, which is WPILib's blue-origin frame, so the WPILib values are used unchanged.
std::vector<sdk::FiducialLayout> buildFiducialLayouts();

}  // namespace wpiemu::plugins::game_2026
