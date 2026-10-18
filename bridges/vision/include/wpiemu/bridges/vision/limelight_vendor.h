#pragma once

// Limelight "virtual coprocessor": publishes the /limelight[-name]/... table exactly as a Limelight does
// (tv/tx/ty/..., botpose arrays, rawfiducials, state echoes) and reads the robot's writes
// (robot_orientation_set, camerapose_robotspace_set, pipeline, ledMode, ...). Work stream
// limelight-publisher; contract in the session spec section F/F.1.

#include <memory>

#include "wpiemu/bridges/vision/camera_publisher.h"

namespace wpiemu::bridges::vision {

std::unique_ptr<IVisionVendor> makeLimelightVendor();

}  // namespace wpiemu::bridges::vision
