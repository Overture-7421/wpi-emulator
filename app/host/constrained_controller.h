#pragma once

// The simulation controller the UI talks to while a practice match is hosted: it forwards everything to the runner, except
// the two things clients cannot follow while they are connected. A time scale other than 1x is held at 1x (the scale the
// user asked for is kept and comes back when the last client leaves) and Step is dropped. Constraints are applied by the
// match host from the sim thread (apply); every method is thread-safe.

#include <mutex>
#include <optional>

#include "wpiemu/sdk/telemetry/sim_controller.h"

namespace wpiemu::app::host {

class ConstrainedController final : public sdk::ISimController {
public:
    explicit ConstrainedController(sdk::ISimController& inner, double initialTimeScale = 1.0)
        : inner_(inner), requestedScale_(initialTimeScale) {}

    // The host's constraints changed (HostConstraints::lockTimeScale / stepDisallowed). Engaging the lock sends 1x to the
    // runner; releasing it sends the scale the user last asked for.
    void apply(bool lockTimeScale, bool stepDisallowed);
    bool timeScaleLocked() const;
    bool stepDisabled() const;

    void setDriverStation(const sdk::proto::DsState& ds) override { inner_.setDriverStation(ds); }
    void control(const sdk::proto::SimControl& command) override;
    void bindSlot(const sdk::SlotAddress& slot, const sdk::DeviceKey& device) override { inner_.bindSlot(slot, device); }
    void unbindSlot(const sdk::SlotAddress& slot) override { inner_.unbindSlot(slot); }
    void setStartPlacement(const std::optional<sdk::RobotStartPose>& pose) override { inner_.setStartPlacement(pose); }

private:
    void sendTimeScale(double scale);

    sdk::ISimController& inner_;
    mutable std::mutex mutex_;
    bool lockTimeScale_ = false;
    bool stepDisallowed_ = false;
    double requestedScale_ = 1.0;   // what the user last asked for
};

}  // namespace wpiemu::app::host
