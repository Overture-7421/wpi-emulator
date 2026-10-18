#pragma once

// PhotonVision "virtual coprocessor": publishes /photonvision/<camera>/... exactly as the PhotonVision
// coprocessor does (photonstruct result stream, heartbeat, state echoes, intrinsics) and reads the
// robot's PhotonLib requests. Work stream photon-publisher; contract in the session spec section F/F.1.

#include <memory>

#include "wpiemu/bridges/vision/camera_publisher.h"

namespace wpiemu::bridges::vision {

std::unique_ptr<IVisionVendor> makePhotonVendor();

}  // namespace wpiemu::bridges::vision
