#pragma once

// Entry point of the vision bridge module (ADR 0017): registers the "vision" IDeviceBridge, which turns
// the emulator's vendor-neutral camera frames (SensorFrame::vision) into the NetworkTables entries a real
// PhotonVision or Limelight coprocessor would publish, inside the robot process.

#include "wpiemu/bridges/vision/vision_bridge.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::bridges::vision {

// Registers the vision bridge, named "vision" (BridgeInfo family "vision"), wired to the robot's own ntcore
// (makeNtCoreApi) and the shipped vendors (PhotonVision, Limelight). Cheap and inert until the emulator configured
// cameras (HelloAck::vision) and the robot process has loaded ntcore.
sdk::Status registerVisionBridges(sdk::DeviceBridgeRegistry& registry);
// Same, with the NetworkTables seam and/or the vendors replaced (tests, builds without a vendor).
sdk::Status registerVisionBridges(sdk::DeviceBridgeRegistry& registry, VisionBridgeOptions options);

}  // namespace wpiemu::bridges::vision
