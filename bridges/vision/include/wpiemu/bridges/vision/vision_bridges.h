#pragma once

// Entry point of the vision bridge module (ADR 0017): registers the "vision" IDeviceBridge, which turns
// the emulator's vendor-neutral camera frames (SensorFrame::vision) into the NetworkTables entries a real
// PhotonVision or Limelight coprocessor would publish, inside the robot process.

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace wpiemu::bridges::vision {

// Registers the vision bridge. Cheap and inert until the emulator configured cameras (HelloAck::vision)
// and the robot process has loaded ntcore.
sdk::Status registerVisionBridges(sdk::DeviceBridgeRegistry& registry);

}  // namespace wpiemu::bridges::vision
