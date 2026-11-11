#include "host/constrained_controller.h"

namespace wpiemu::app::host {

void ConstrainedController::sendTimeScale(double scale) {
    sdk::proto::SimControl c;
    c.command = sdk::proto::SimCommand::SetTimeScale;
    c.timeScale = scale;
    inner_.control(c);
}

void ConstrainedController::apply(bool lockTimeScale, bool stepDisallowed) {
    std::lock_guard lock(mutex_);
    const bool wasLocked = lockTimeScale_;
    lockTimeScale_ = lockTimeScale;
    stepDisallowed_ = stepDisallowed;
    if (lockTimeScale && !wasLocked && requestedScale_ != 1.0) sendTimeScale(1.0);
    else if (!lockTimeScale && wasLocked && requestedScale_ != 1.0) sendTimeScale(requestedScale_);
}

bool ConstrainedController::timeScaleLocked() const {
    std::lock_guard lock(mutex_);
    return lockTimeScale_;
}

bool ConstrainedController::stepDisabled() const {
    std::lock_guard lock(mutex_);
    return stepDisallowed_;
}

void ConstrainedController::control(const sdk::proto::SimControl& command) {
    std::lock_guard lock(mutex_);
    switch (command.command) {
        case sdk::proto::SimCommand::SetTimeScale:
            requestedScale_ = command.timeScale;
            if (lockTimeScale_) return;
            break;
        case sdk::proto::SimCommand::Step:
            if (stepDisallowed_) return;
            break;
        default: break;
    }
    inner_.control(command);
}

}  // namespace wpiemu::app::host
