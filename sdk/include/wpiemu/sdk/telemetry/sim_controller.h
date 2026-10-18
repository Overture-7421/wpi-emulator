#pragma once

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/math.h"
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
    // Teleports the chassis to `pose` (world frame, the body origin = the COM; callers that only know x, y and yaw take the
    // height from the latest snapshot's chassis pose and pass an upright rotation, see toPose3). Applied before the next
    // tick, also while paused or before the robot connected: linear and angular velocity are zeroed and the drivetrain is
    // reset; mechanisms, game state, the clock and the RNG keep running. A non-finite pose is ignored. Reset still returns
    // to the configured start pose.
    virtual void setRobotPose(const Pose3& pose) = 0;
};

}  // namespace wpiemu::sdk
