#pragma once

// PhotonVision "virtual coprocessor": publishes /photonvision/<camera>/... exactly as the PhotonVision
// coprocessor does (photonstruct result stream, heartbeat, state echoes, intrinsics) and reads the
// robot's PhotonLib requests. Work stream photon-publisher; contract in the session spec section F/F.1.
//
//   photon_profile.h   the message definitions (schema texts + UUID) the bridge can write: data
//   photon_codec.h     the one generic encoder/decoder that walks them
//   photon_mapper.h    sdk::VisionFrame -> PhotonVision result values, and the proof that a profile is fully covered
//   (this header)      the vendor the bridge core asks for one publisher per enabled PhotonVision camera
//
// createPublisher() gives every camera its own publisher and shares what PhotonVision keeps once per coprocessor
// (/photonvision/version, /photonvision/ledModeState and the robot's LED request) between them. A camera whose
// message profile is unknown (CameraSetup::photonMessageUuid without a profile), unusable, or whose result topic a
// robot-side PhotonCameraSim already publishes, is inert: it publishes nothing, reports camera_status
// kPhotonUuidUnsupported / kForeignPublisher, and still reads the robot's requests. It returns nullptr only for a
// setup of another vendor. discoverRobotCameras() names the cameras whose *Request topics the robot created.
//
// What the robot's requests do: pipelineIndexRequest, fpsLimitRequest and ledModeRequest are echoed into their state
// topics (and reported in CameraRobotState); driverModeRequest is echoed too, and while it is on every frame is
// published as an EMPTY result, which is what PhotonVision's driver-mode pipeline reports. The fps limit does not
// thin the frames out (the engine owns the frame schedule).

#include <memory>

#include "wpiemu/bridges/vision/camera_publisher.h"

namespace wpiemu::bridges::vision {

std::unique_ptr<IVisionVendor> makePhotonVendor();

}  // namespace wpiemu::bridges::vision
