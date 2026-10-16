#pragma once

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
};

}  // namespace wpiemu::sdk
