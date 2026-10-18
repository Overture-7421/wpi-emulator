#pragma once

// AprilTag layouts for the vision engine: reading a WPILib AprilTagFieldLayout JSON file and
// choosing the layout a project uses (ADR 0017).
//
// WPILib format (poses in the blue-origin field frame, which is the SDK world frame):
//   {"tags":[{"ID":1,"pose":{"translation":{"x":..,"y":..,"z":..},
//             "rotation":{"quaternion":{"W":..,"X":..,"Y":..,"Z":..}}}}, ...],
//    "field":{"length":16.541,"width":8.069}}
// The format carries no tag size, so a loaded layout uses the default FiducialLayout::tagSizeM.
//
// All failures are returned by value (no exceptions): ParseError for malformed or invalid content,
// IoError / NotFound for the file, NotFound for an unknown layout id.

#include <filesystem>
#include <string_view>

#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/fiducial_layout.h"

namespace wpiemu::core::vision {

// Parses and validates a layout: unique non-negative integer ids, finite numbers, at least one
// tag; quaternions are normalised (a zero quaternion is an error). Tag order is the file's. The
// layout id is "custom".
sdk::Result<sdk::FiducialLayout> parseFiducialLayout(std::string_view jsonText,
                                                     const sdk::IValueTextCodec& codec);

// Reads and parses `path`; the layout id is "file:<file name>" and the display name the file stem.
sdk::Result<sdk::FiducialLayout> loadFiducialLayoutFile(const std::filesystem::path& path,
                                                        const sdk::IValueTextCodec& codec);

// The layout a project uses. Precedence: VisionConfig::layoutFile (a relative path is resolved
// against `baseDir` when that is not empty) > VisionConfig::layoutId among the plugin's layouts
// (unknown id is an error) > the plugin's first layout > error.
sdk::Result<sdk::FiducialLayout> resolveFiducialLayout(const sdk::VisionConfig& config,
                                                       const sdk::IGameModule* module,
                                                       const sdk::IValueTextCodec& codec,
                                                       const std::filesystem::path& baseDir = {});

}  // namespace wpiemu::core::vision
