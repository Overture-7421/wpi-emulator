#pragma once

#include <optional>

#include "wpiemu/sdk/config/start_position.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::sdk {

// Commands the UI sends to the running simulation. All calls are THREAD-SAFE and non-blocking:
// they enqueue work for the sim thread (applied at the start of the next tick) and return
// immediately. Results of mapping changes show up as snapshot warnings.
class ISimController {
    WPIEMU_INTERFACE(ISimController)
public:
    virtual void setDriverStation(const proto::DsState& ds) = 0;
    virtual void control(const proto::SimControl& command) = 0;
    // Live remapping while connected (the editor edits RobotConfig; this applies it to the hub).
    virtual void bindSlot(const SlotAddress& slot, const DeviceKey& device) = 0;
    virtual void unbindSlot(const SlotAddress& slot) = 0;
    // The chassis placement every Reset restores until changed again; nullopt = the configured start pose. Applied in
    // order with the other commands, so setStartPlacement followed by a Reset control resets onto the new placement. It
    // does not move the robot by itself.
    virtual void setStartPlacement(const std::optional<RobotStartPose>& pose) = 0;
};

}  // namespace wpiemu::sdk
