#include "wpiemu/halsim_ext/sim_session.h"

#include <algorithm>
#include <string>
#include <utility>
#include <variant>

#include "wpiemu/sdk/ipc/lifecycle.h"

namespace wpiemu::halsim {

namespace proto = sdk::proto;

namespace {
constexpr const char* kLog = "extension";
constexpr std::chrono::milliseconds kStartPoll{5};
constexpr std::chrono::milliseconds kSchedulerSettle{500};
constexpr std::chrono::milliseconds kLockstepIdle{20};
constexpr std::chrono::milliseconds kFreeIdle{5};
constexpr std::chrono::milliseconds kGoodbyeFlush{500};

std::uint32_t clampWatchdog(std::uint32_t ms) {
    return std::clamp(ms, proto::kMinWatchdogTimeoutMs, proto::kMaxWatchdogTimeoutMs);
}
}  // namespace

std::int64_t SimSession::nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

SimSession::SimSession(SessionServices services, std::unique_ptr<IpcLink> link, const proto::HelloAck& ack)
    : s_(services), link_(std::move(link)), ack_(ack) {
    lockstep_ = ack.clockMode == proto::ClockMode::Lockstep && !ack.discoveryOnly &&
                s_.config.clock == ClockPreference::FollowEmulator;
    watchdogMs_ = clampWatchdog(ack.watchdogTimeoutMs);
    follower_.begin(ack.sessionId, ack.runEpoch);
    sessionId_ = ack.sessionId;
    epoch_ = ack.runEpoch;
    pendingDs_ = proto::DsState{};  // a DS is attached (disabled) until the emulator says otherwise
}

SimSession::~SimSession() {
    shutdown_ = true;
    {
        std::lock_guard lock(watchdogMutex_);
        watchdogStop_ = true;
    }
    watchdogCv_.notify_all();
    if (watchdog_.joinable()) watchdog_.join();
    s_.devices.setSink({});
    s_.logger.attachRemote({});
    link_->close("session ended");
}

void SimSession::begin() {
    link_->start();
    s_.devices.setSink([this](proto::DeviceAnnounce&& announce) {
        if (announce.retired) {
            std::lock_guard lock(retiredMutex_);
            retired_.push_back(announce.id);
        }
        s_.stats.announces.fetch_add(1, std::memory_order_relaxed);
        link_->post(proto::Message(std::move(announce)));
    });
    s_.logger.attachRemote([this](proto::Log&& log) { link_->postDroppable(proto::Message(std::move(log))); });
    // Devices announced before this connection (reconnect) are re-announced; repeats are idempotent.
    for (proto::DeviceAnnounce& announce : s_.devices.snapshot()) {
        s_.stats.announces.fetch_add(1, std::memory_order_relaxed);
        link_->post(proto::Message(std::move(announce)));
    }
    touch();
}

void SimSession::requestShutdown() {
    shutdown_ = true;
    s_.ports.clock.resumeTiming();  // never leave the robot frozen on the way out
    watchdogCv_.notify_all();
}

void SimSession::touch() { lastActivityNs_.store(nowNs(), std::memory_order_relaxed); }

void SimSession::post(proto::Message message) { link_->post(std::move(message)); }

SessionEnd SimSession::run() {
    const bool lockstepCapable = ack_.clockMode == proto::ClockMode::Lockstep && s_.config.clock == ClockPreference::FollowEmulator;
    if (lockstepCapable && s_.config.watchdogEnabled) watchdog_ = std::thread([this] { watchdogLoop(); });

    SessionEnd end = SessionEnd::Shutdown;
    if (const std::optional<SessionEnd> early = waitForProgramStart()) end = *early;
    else end = serve();

    {
        std::lock_guard lock(watchdogMutex_);
        watchdogStop_ = true;
    }
    watchdogCv_.notify_all();
    if (watchdog_.joinable()) watchdog_.join();

    switch (end) {
        case SessionEnd::Disconnected:
            s_.logger.warn(kLog, "emulator connection lost: " + link_->closeReason());
            if (!unblocked_.exchange(true)) applyPolicy("transport lost");
            break;
        case SessionEnd::Stopped:
        case SessionEnd::Shutdown:
            s_.ports.clock.resumeTiming();
            link_->post(proto::Message(proto::Goodbye{end == SessionEnd::Stopped ? "stopped by emulator" : "robot shutting down"}));
            link_->flush(kGoodbyeFlush);
            break;
    }
    s_.devices.setSink({});
    s_.logger.attachRemote({});
    link_->close("session ended");
    return end;
}

std::optional<SessionEnd> SimSession::waitForProgramStart() {
    while (!shutdown_) {
        proto::Message message;
        switch (link_->pop(message, kStartPoll)) {
            case IpcLink::PopResult::Message:
                touch();
                if (const std::optional<SessionEnd> end = dispatch(std::move(message))) return end;
                break;
            case IpcLink::PopResult::Closed: return SessionEnd::Disconnected;
            case IpcLink::PopResult::Timeout: break;
        }
        if (s_.ports.clock.programStarted()) break;
        if (s_.config.pollWhileConstructing) s_.bridges.poll();
    }
    if (shutdown_) return SessionEnd::Shutdown;

    // TimedRobot arms its first alarm right after observing program start; give it a moment so the
    // first step lands on a scheduled loop (robots without a notifier simply pay the timeout once).
    const auto settleDeadline = Clock::now() + kSchedulerSettle;
    while (lockstep_ && !shutdown_ && !s_.ports.clock.schedulerArmed() && Clock::now() < settleDeadline) {
        proto::Message message;
        switch (link_->pop(message, std::chrono::milliseconds(2))) {
            case IpcLink::PopResult::Message:
                touch();
                if (const std::optional<SessionEnd> end = dispatch(std::move(message))) return end;
                break;
            case IpcLink::PopResult::Closed: return SessionEnd::Disconnected;
            case IpcLink::PopResult::Timeout: break;
        }
    }
    if (shutdown_) return SessionEnd::Shutdown;

    programStarted_ = true;
    s_.bridges.poll();  // everything the robot constructor created is announced before the signal below
    drainRetired();
    post(proto::Message(proto::makeLifecycleLog(proto::kLifecycleProgramStarted)));
    s_.logger.info(kLog, "robot program started; " + std::to_string(s_.devices.liveCount()) + " device(s) announced");
    applyPendingDriverStation();
    armed_ = lockstep_;
    touch();
    return std::nullopt;
}

SessionEnd SimSession::serve() {
    if (deferredFrame_) {
        proto::SensorFrame frame = std::move(*deferredFrame_);
        deferredFrame_.reset();
        onSensorFrame(std::move(frame));
    }
    for (;;) {
        if (shutdown_) return SessionEnd::Shutdown;
        const std::chrono::milliseconds idleTimeout = lockstep_ ? kLockstepIdle : kFreeIdle;
        proto::Message message;
        switch (link_->pop(message, idleTimeout)) {
            case IpcLink::PopResult::Message:
                touch();
                if (const std::optional<SessionEnd> end = dispatch(std::move(message))) return *end;
                break;
            case IpcLink::PopResult::Timeout: idle(); break;
            case IpcLink::PopResult::Closed: return shutdown_ ? SessionEnd::Shutdown : SessionEnd::Disconnected;
        }
    }
}

std::optional<SessionEnd> SimSession::dispatch(proto::Message&& message) {
    if (auto* frame = std::get_if<proto::SensorFrame>(&message)) {
        onSensorFrame(std::move(*frame));
    } else if (auto* ds = std::get_if<proto::DsState>(&message)) {
        onDsState(std::move(*ds));
    } else if (auto* set = std::get_if<proto::LoopbackSet>(&message)) {
        loopback_ = std::move(set->loopback);
        std::sort(loopback_.begin(), loopback_.end());
    } else if (auto* control = std::get_if<proto::SimControl>(&message)) {
        return onControl(*control);
    } else if (std::holds_alternative<proto::Goodbye>(message)) {
        s_.logger.info(kLog, "emulator said Goodbye: " + std::get<proto::Goodbye>(message).reason);
        return SessionEnd::Disconnected;
    } else {
        s_.logger.debug(kLog, "ignoring unexpected message type " +
                                  std::to_string(static_cast<int>(proto::typeOf(message))));
    }
    return std::nullopt;
}

void SimSession::onSensorFrame(proto::SensorFrame&& frame) {
    if (!programStarted_) {
        // Hold it: stepping before the robot program exists would desynchronise sim time (ADR 0001).
        if (deferredFrame_) {
            s_.stats.protocolErrors.fetch_add(1, std::memory_order_relaxed);
            post(proto::Message(follower_.makeAck(frame, proto::TickResult::ProtocolError)));
        } else {
            deferredFrame_ = std::move(frame);
        }
        return;
    }
    if (const Degraded why = degraded_.load(); why != Degraded::None) {
        post(proto::Message(follower_.makeAck(
            frame, why == Degraded::Stalled ? proto::TickResult::RobotStalled : proto::TickResult::Stopped)));
        return;
    }

    switch (follower_.onSensorFrame(frame)) {
        case proto::FollowerAction::Drop:
            s_.stats.framesDropped.fetch_add(1, std::memory_order_relaxed);
            return;
        case proto::FollowerAction::ProtocolError:
            s_.stats.protocolErrors.fetch_add(1, std::memory_order_relaxed);
            post(proto::Message(follower_.makeAck(frame, proto::TickResult::ProtocolError)));
            return;
        case proto::FollowerAction::Step: break;
    }
    epoch_ = follower_.runEpoch();
    s_.logger.setSimTimeUs(frame.simTimeUs);

    drainRetired();
    applyPendingDriverStation();
    if (frame.batteryVoltageV != lastBatteryV_ || frame.batteryCurrentA != lastBatteryA_) {
        lastBatteryV_ = frame.batteryVoltageV;
        lastBatteryA_ = frame.batteryCurrentA;
        s_.ports.power.setBattery(lastBatteryV_, lastBatteryA_);
    }
    s_.bridges.applyInputs(frame.samples);

    if (lockstep_) {
        const std::uint64_t stepUs = frame.stepUs != 0 ? frame.stepUs : ack_.stepUs;
        {
            std::lock_guard lock(stepMutex_);
            stepStartNs_ = nowNs();
            stepAbandoned_ = false;
            inflightTick_ = frame.tick;
            inflightSimTimeUs_ = frame.simTimeUs;
        }
        s_.ports.clock.stepTiming(stepUs);
        std::lock_guard lock(stepMutex_);
        const bool abandoned = stepAbandoned_;
        stepStartNs_ = 0;
        if (abandoned) return;  // the watchdog already answered this tick with RobotStalled
    }

    // Announce first so that the emulator knows a device before it sees samples for it.
    s_.bridges.poll();
    drainRetired();
    outputs_.clear();
    s_.bridges.collectOutputs(outputs_);
    std::erase_if(outputs_, [this](const sdk::SignalSample& s) {
        return std::binary_search(loopback_.begin(), loopback_.end(), s.device);
    });
    s_.bridges.applyLoopback(loopback_);

    const bool forceFull = s_.config.fullFrameInterval != 0 && ++ticksSinceFull_ >= s_.config.fullFrameInterval;
    const bool full = delta_.filter(outputs_, forceFull);
    if (full) ticksSinceFull_ = 0;

    proto::ActuatorFrame ack = follower_.makeAck(frame, proto::TickResult::Completed);
    ack.full = full;
    ack.samples = outputs_;
    ack.robotLoopCount = s_.ports.clock.robotLoopCount();
    post(proto::Message(std::move(ack)));
    s_.stats.ticksCompleted.fetch_add(1, std::memory_order_relaxed);
    touch();
}

void SimSession::onDsState(proto::DsState&& state) {
    const proto::FrameDisposition d =
        proto::classifyFrame(state.sessionId, state.runEpoch, follower_.sessionId(), follower_.runEpoch());
    if (d == proto::FrameDisposition::DropStaleEpoch || d == proto::FrameDisposition::DropForeignSession) {
        s_.stats.framesDropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    pendingDs_ = std::move(state);
    // Lock-step applies it at the next tick boundary; otherwise nothing steps, so apply right away.
    if (programStarted_ && !lockstep_) applyPendingDriverStation();
}

std::optional<SessionEnd> SimSession::onControl(const proto::SimControl& control) {
    auto rejectWith = [&](proto::ControlResult result, const char* detail) {
        if (control.commandId == proto::kUnspecified) return;
        proto::SimControlAck ack;
        ack.commandId = control.commandId;
        ack.command = control.command;
        ack.result = result;
        ack.sessionId = follower_.sessionId();
        ack.runEpoch = follower_.runEpoch();
        ack.tick = follower_.lastCompletedTick();
        ack.detail = detail;
        post(proto::Message(std::move(ack)));
    };
    if (control.command == proto::SimCommand::Step && !paused_) {
        rejectWith(proto::ControlResult::Rejected, "Step while running");
        return std::nullopt;
    }
    if (control.command == proto::SimCommand::SetTimeScale && !lockstep_) {
        rejectWith(proto::ControlResult::Unsupported, "time scale is only meaningful in lock-step");
        return std::nullopt;
    }

    if (control.command == proto::SimCommand::BeginLockstep) {
        if (lockstep_) return rejectWith(proto::ControlResult::Rejected, "already in lock-step"), std::nullopt;
        if (ack_.clockMode != proto::ClockMode::Lockstep || s_.config.clock != ClockPreference::FollowEmulator)
            return rejectWith(proto::ControlResult::Rejected, "this session does not follow the emulator's clock"), std::nullopt;
        if (!programStarted_) return rejectWith(proto::ControlResult::Rejected, "the robot program has not started"), std::nullopt;
    }
    if (control.command == proto::SimCommand::EndLockstep && !lockstep_) {
        rejectWith(proto::ControlResult::Rejected, "not in lock-step");
        return std::nullopt;
    }

    proto::ControlDecision decision = follower_.onControl(control);
    epoch_ = follower_.runEpoch();
    if (decision.apply) {
        switch (control.command) {
            case proto::SimCommand::Pause: paused_ = true; break;
            case proto::SimCommand::Resume: paused_ = false; break;
            case proto::SimCommand::Step:
            case proto::SimCommand::SetTimeScale: break;  // pacing is the emulator's business
            case proto::SimCommand::Reset:
                delta_.reset();  // the next ActuatorFrame is a full one in the new epoch
                ticksSinceFull_ = 0;
                if (lockstep_ && unblocked_.exchange(false)) {  // re-engage lock-step after a stall
                    degraded_ = Degraded::None;
                    s_.ports.clock.pauseTiming();
                    s_.logger.info(kLog, "lock-step re-engaged by Reset");
                }
                break;
            case proto::SimCommand::BeginLockstep:
                delta_.reset();
                ticksSinceFull_ = 0;
                paused_ = false;
                unblocked_ = false;
                degraded_ = Degraded::None;
                s_.ports.clock.pauseTiming();
                lockstep_ = true;
                touch();
                armed_ = true;
                s_.logger.info(kLog, "lock-step started by the emulator");
                break;
            case proto::SimCommand::EndLockstep:
                armed_ = false;
                paused_ = false;
                lockstep_ = false;
                unblocked_ = false;
                degraded_ = Degraded::None;
                s_.ports.clock.resumeTiming();
                s_.logger.info(kLog, "lock-step ended by the emulator; the robot keeps running in real time");
                break;
            case proto::SimCommand::Stop: armed_ = false; break;
        }
    }
    if (decision.sendAck) post(proto::Message(std::move(decision.ack)));
    if (decision.apply && control.command == proto::SimCommand::Stop) {
        s_.ports.clock.resumeTiming();
        return SessionEnd::Stopped;
    }
    return std::nullopt;
}

void SimSession::applyPendingDriverStation() {
    if (!pendingDs_) return;
    s_.ports.driverStation.apply(*pendingDs_);
    pendingDs_.reset();
}

void SimSession::idle() {
    s_.bridges.poll();
    drainRetired();
    if (!lockstep_ && programStarted_) {
        applyPendingDriverStation();
        s_.bridges.applyLoopback(loopback_);
    }
}

void SimSession::drainRetired() {
    std::vector<sdk::DeviceId> retired;
    {
        std::lock_guard lock(retiredMutex_);
        retired.swap(retired_);
    }
    for (const sdk::DeviceId id : retired) delta_.forget(id);
}

void SimSession::watchdogLoop() {
    const auto period = std::clamp(std::chrono::milliseconds(watchdogMs_ / 5), std::chrono::milliseconds(10),
                                   std::chrono::milliseconds(100));
    std::unique_lock lock(watchdogMutex_);
    while (!watchdogStop_) {
        watchdogCv_.wait_for(lock, period, [&] { return watchdogStop_; });
        if (watchdogStop_) break;
        lock.unlock();
        checkWatchdog();
        lock.lock();
    }
}

void SimSession::checkWatchdog() {
    if (!armed_ || degraded_.load() != Degraded::None) return;
    const std::int64_t timeoutNs = static_cast<std::int64_t>(watchdogMs_) * 1'000'000;
    const std::int64_t now = nowNs();

    bool stalled = false;
    bool lost = false;
    proto::ActuatorFrame stallAck;
    {
        std::lock_guard lock(stepMutex_);
        if (stepStartNs_ != 0) {
            if (!stepAbandoned_ && now - stepStartNs_ > timeoutNs) {
                stepAbandoned_ = true;
                stalled = true;
                stallAck.tick = inflightTick_;
                stallAck.ackTick = inflightTick_;
                stallAck.simTimeUs = inflightSimTimeUs_;
                stallAck.sessionId = sessionId_.load();
                stallAck.runEpoch = epoch_.load();
                stallAck.result = proto::TickResult::RobotStalled;
            }
        } else if (!paused_ && now - lastActivityNs_.load(std::memory_order_relaxed) > timeoutNs) {
            lost = true;
        }
    }
    if (stalled) {
        post(proto::Message(std::move(stallAck)));
        unblockRobot(Degraded::Stalled, "the robot did not finish the tick within the watchdog timeout");
    } else if (lost) {
        unblockRobot(Degraded::EmulatorLost, "no message from the emulator within the watchdog timeout");
    }
}

void SimSession::unblockRobot(Degraded why, const char* reason) {
    if (unblocked_.exchange(true)) return;
    degraded_ = why;
    (why == Degraded::Stalled ? s_.stats.stalls : s_.stats.emulatorLost).fetch_add(1, std::memory_order_relaxed);
    s_.logger.error(kLog, std::string("watchdog: ") + reason + " (" + std::to_string(watchdogMs_) + " ms)");
    applyPolicy(reason);
}

void SimSession::applyPolicy(const char* reason) {
    if (ack_.watchdogPolicy == proto::WatchdogPolicy::Terminate) {
        s_.logger.error(kLog, std::string("terminating the robot process: ") + reason);
        s_.ports.process.terminate(0);
        return;
    }
    s_.logger.warn(kLog, std::string("resuming real-time HAL timing: ") + reason);
    s_.ports.clock.resumeTiming();
}

}  // namespace wpiemu::halsim
