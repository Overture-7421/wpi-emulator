#pragma once

// Vision bridges run INSIDE THE ROBOT PROCESS (owned by the halsim_wpiemu extension) and present the simulated
// cameras to the robot code exactly like the real coprocessor would (Limelight / PhotonVision over NetworkTables).
// They depend only on the SDK (+ ntcore in the real binding, which the SDK never sees).
//
// Threading: configure / poll / apply / reset run on the extension's single CLOCK THREAD while the robot is
// quiescent (between HALSIM_StepTiming calls), like IDeviceBridge::applyInputs. Never call them from HAL callbacks.
// Errors: no exceptions; failures are logged and the affected camera is skipped.

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/vision/camera_frame.h"

namespace wpiemu::sdk {

class IVisionBridge {
    WPIEMU_INTERFACE(IVisionBridge)
public:
    virtual std::string_view name() const = 0;   // unique: "vision.limelight", "vision.photonvision"

    // Bind the backend (e.g. resolve ntcore). Once, before the robot's user code runs.
    virtual Status start(ILogger& logger) = 0;
    // (Re)create the topics of the cameras this bridge serves (the CameraKind it understands). Idempotent; called on
    // every new connection with the emulator's current camera list. Cameras of other kinds are ignored.
    virtual void configure(const VisionSetup& setup) = 0;
    // Mirror robot-written settings (pipeline index, driver mode, LED mode) and heartbeats. Every tick.
    virtual void poll(SimTimeUs simTimeUs) = 0;
    // Publish the frames due this tick (CameraFrame::publishTimeUs <= simTimeUs). Frames of other kinds are ignored.
    virtual void apply(std::span<const CameraFrame> frames, SimTimeUs simTimeUs) = 0;
    // New connection or simulation epoch: drop published state so stale results are not seen.
    virtual void reset() {}
    // Idempotent; releases topics and backend.
    virtual void stop() noexcept = 0;
};

// One registry for vision bridges; each module exposes `registerVisionBridges(VisionBridgeRegistry&)`. A distinct type
// (not an alias) so builtin_bridges.h can forward-declare it.
class VisionBridgeRegistry : public Registry<BridgeInfo, IVisionBridge> {};

}  // namespace wpiemu::sdk
