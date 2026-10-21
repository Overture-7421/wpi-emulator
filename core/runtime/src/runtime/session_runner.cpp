#include "wpiemu/core/runtime/session_runner.h"

#include <algorithm>

#include "wpiemu/sdk/ipc/lifecycle.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;
using Clock = std::chrono::steady_clock;

SessionRunner::SessionRunner(SimulationSession& session, MessageInbox& inbox, ISnapshotSink& sink, ILogger& logger, RunnerOptions options,
                             const IWallClock* clock)
    : session_(session),
      inbox_(inbox),
      sink_(sink),
      logger_(logger),
      options_(options),
      clock_(clock != nullptr ? *clock : defaultClock_),
      paused_(options.startPaused),
      timeScale_(std::max(0.0, options.timeScale)),
      requestedDs_(session.driverStation()) {
    nextTickAt_ = clock_.now();
    publishSnapshot(clock_.now(), true);
}

SessionRunner::~SessionRunner() {
    stop();
    closeLink();
}

// ---- thread-safe controller side ----------------------------------------------------------------------------

void SessionRunner::enqueue(Command command) {
    {
        std::lock_guard lock(commandMutex_);
        commands_.push_back(std::move(command));
    }
    inbox_.wake();
}

void SessionRunner::setDriverStation(const proto::DsState& ds) { enqueue(SetDs{ds}); }
void SessionRunner::control(const proto::SimControl& command) { enqueue(Control{command}); }
void SessionRunner::bindSlot(const SlotAddress& slot, const DeviceKey& device) { enqueue(Bind{slot, device}); }
void SessionRunner::unbindSlot(const SlotAddress& slot) { enqueue(Unbind{slot}); }
void SessionRunner::setStartPlacement(const std::optional<RobotStartPose>& pose) { enqueue(SetStart{pose}); }

std::optional<proto::Hello> SessionRunner::robotHello() const {
    std::lock_guard lock(helloMutex_);
    return hello_;
}

std::string SessionRunner::refusal() const {
    std::lock_guard lock(helloMutex_);
    return refusal_;
}

// ---- thread -------------------------------------------------------------------------------------------------

void SessionRunner::start() {
    if (thread_.joinable()) return;
    thread_.resetStop();
    thread_.start([this](const StopToken& stop) {
        while (!stop.stopRequested() && !finished_) poll(std::chrono::milliseconds(10));
    });
}

void SessionRunner::stop() {
    thread_.requestStop();
    inbox_.wake();
    thread_.join();
}

// ---- the iteration ------------------------------------------------------------------------------------------

void SessionRunner::poll(std::chrono::milliseconds maxWait) {
    std::vector<Command> batch;
    {
        std::lock_guard lock(commandMutex_);
        batch.swap(commands_);
    }
    applyCommands(batch);

    if (!finished_) {
        InboundEvent ev;
        if (inbox_.waitPop(ev, waitHint(maxWait, clock_.now()))) {
            handle(ev);
            while (inbox_.tryPop(ev)) handle(ev);
        }
        checkWatchdog();
        const auto now = clock_.now();
        if (shouldTick(now)) doTick(now);
    }
    refreshState();
    publishSnapshot(clock_.now(), false);
}

std::chrono::milliseconds SessionRunner::waitHint(std::chrono::milliseconds maxWait, Clock::time_point now) const {
    using ms = std::chrono::milliseconds;
    auto until = [&](Clock::time_point t) {
        if (t <= now) return ms(0);
        return std::min(maxWait, std::chrono::duration_cast<ms>(t - now) + ms(1));
    };
    if (shouldTick(now)) return ms(0);
    if (link_ && helloDone_ && !finished_) {
        if (window_.outstanding()) return until(tickDeadline_);
        if (awaitingResetAck_ || stopping_) return until(ackDeadline_);
        if (!paused_ && !window_.stalled() && !armed_ && programStarted_ && timeScale_ > 0.0) return until(nextTickAt_);
    }
    return maxWait;
}

bool SessionRunner::shouldTick(Clock::time_point now) const {
    if (finished_ || stopping_ || !link_ || !helloDone_ || !programStarted_ || armed_ || awaitingResetAck_) return false;
    if (!window_.canSend()) return false;  // one outstanding tick, or stalled
    if (paused_) return stepCredits_ > 0;
    return timeScale_ <= 0.0 || now >= nextTickAt_;
}

void SessionRunner::doTick(Clock::time_point now) {
    proto::SensorFrame frame;
    frame.sessionId = window_.sessionId();
    frame.runEpoch = window_.runEpoch();
    const bool firstOfEpoch = needFull_;
    session_.tick(frame, needFull_);
    needFull_ = false;

    if (Status s = window_.onSent(frame); !s) {
        logger_.error("runner", "lock-step violation: " + s.error().message);
        return;
    }
    if (firstOfEpoch) {
        sendVisionSetup();  // the extension (re)creates its camera state before the first frame of every epoch
        if (!link_) return;
    }
    if (Status s = link_->send(frame); !s) {
        handleDisconnect("send failed: " + s.error().message);
        return;
    }
    ++ticksSent_;
    tickDeadline_ = now + std::chrono::milliseconds(watchdogMs_);
    if (paused_) {
        --stepCredits_;
    } else if (timeScale_ > 0.0) {
        const auto interval = std::chrono::microseconds(static_cast<std::int64_t>(static_cast<double>(session_.clock().fixedStepUs()) / timeScale_));
        nextTickAt_ += interval;
        if (now - nextTickAt_ > std::chrono::milliseconds(100)) nextTickAt_ = now + interval;  // fell behind: resync, no catch-up burst
    }

    if (const bool browned = session_.battery().brownedOut(); browned != brownoutSent_) {
        brownoutSent_ = browned;
        logger_.warn("runner", browned ? "brownout: robot disabled" : "brownout cleared");
        sendDriverStation();
    } else if (session_.gameSpecificMessage() != gameMessageSent_) {
        sendDriverStation();  // the game plugin latched (or cleared) its FMS game data during this tick
    }
    if (++ticksSinceSnapshot_ >= std::max<std::uint32_t>(1, options_.snapshotEveryTicks)) {
        ticksSinceSnapshot_ = 0;
        snapshotDirty_ = true;
    }
}

void SessionRunner::checkWatchdog() {
    const auto now = clock_.now();
    if (window_.outstanding() && now >= tickDeadline_) {
        window_.onWatchdogExpired();
        logger_.error("runner", "robot stalled: no ActuatorFrame within " + std::to_string(watchdogMs_) + " ms; lock-step suspended");
        snapshotDirty_ = true;
    }
    if (awaitingResetAck_ && now >= ackDeadline_) {
        awaitingResetAck_ = false;
        const bool wasBegin = awaitingBeginAck_;
        if (wasBegin) {
            awaitingBeginAck_ = false;
            armed_ = true;
        }
        window_.onWatchdogExpired();
        logger_.error("runner", std::string("robot did not acknowledge ") +
                                    (wasBegin ? "BeginLockstep" : "Reset") + " within " +
                                    std::to_string(watchdogMs_) + " ms");
        snapshotDirty_ = true;
    }
    if (stopping_ && now >= ackDeadline_) {
        logger_.warn("runner", "robot did not acknowledge Stop; closing the link");
        finish();
    }
}

// ---- commands -------------------------------------------------------------------------------------------------

void SessionRunner::applyCommands(std::vector<Command>& batch) {
    for (Command& c : batch) {
        if (auto* ds = std::get_if<SetDs>(&c)) {
            requestedDs_ = ds->ds;
            sendDriverStation();
        } else if (auto* ctl = std::get_if<Control>(&c)) {
            applyControl(ctl->command);
        } else if (auto* bind = std::get_if<Bind>(&c)) {
            if (Status s = session_.bindSlot(bind->slot, bind->device); !s)
                logger_.warn("runner", "bind " + bind->slot.toString() + " failed: " + s.error().message);
            sendLoopbackIfChanged(false);
        } else if (auto* start = std::get_if<SetStart>(&c)) {
            session_.setStartPlacement(start->pose);
        } else if (auto* unbind = std::get_if<Unbind>(&c)) {
            session_.hub().unbind(unbind->slot);
            sendLoopbackIfChanged(false);
        }
        snapshotDirty_ = true;
    }
}

void SessionRunner::applyControl(const proto::SimControl& c) {
    switch (c.command) {
        case proto::SimCommand::Pause:
            paused_ = true;
            sendPauseState();
            break;
        case proto::SimCommand::Resume:
            paused_ = false;
            nextTickAt_ = clock_.now();
            sendPauseState();
            break;
        case proto::SimCommand::Step:
            if (paused_) stepCredits_ = std::min(options_.maxStepCredits, stepCredits_ + std::max<std::uint32_t>(1, c.stepCount));
            else logger_.debug("runner", "Step ignored: the simulation is running");
            break;
        case proto::SimCommand::SetTimeScale:
            timeScale_ = std::max(0.0, c.timeScale);
            nextTickAt_ = clock_.now();
            break;
        case proto::SimCommand::Reset: {
            beginEpoch();
            if (link_ && helloDone_) {
                proto::SimControl reset;
                reset.command = proto::SimCommand::Reset;
                reset.commandId = ++commandCounter_;
                reset.sessionId = window_.sessionId();
                reset.runEpoch = window_.runEpoch();
                resetCommandId_ = reset.commandId;
                awaitingResetAck_ = true;
                ackDeadline_ = clock_.now() + std::chrono::milliseconds(watchdogMs_);
                sendToRobot(reset);
                sendDriverStation();
            }
            break;
        }
        case proto::SimCommand::BeginLockstep: {
            if (!peerSupportsArmed_) {
                logger_.warn("runner",
                             "BeginLockstep unsupported: peer did not advertise Armed capability");
                break;
            }
            if (!link_ || !helloDone_ || !armed_ || !programStarted_ || awaitingBeginAck_) {
                logger_.warn("runner", "BeginLockstep ignored: not armed with a started robot program");
                break;
            }
            beginEpoch();
            proto::SimControl begin;
            begin.command = proto::SimCommand::BeginLockstep;
            begin.commandId = ++commandCounter_;
            begin.sessionId = window_.sessionId();
            begin.runEpoch = window_.runEpoch();
            resetCommandId_ = begin.commandId;
            awaitingResetAck_ = true;
            awaitingBeginAck_ = true;
            ackDeadline_ = clock_.now() + std::chrono::milliseconds(watchdogMs_);
            sendToRobot(begin);
            armed_ = false;  // ticks start once the ack arrives (shouldTick waits for awaitingResetAck_)
            sendDriverStation();
            break;
        }
        case proto::SimCommand::EndLockstep: {
            if (!peerSupportsArmed_) {
                logger_.warn("runner",
                             "EndLockstep unsupported: peer did not advertise Armed capability");
                break;
            }
            if (!link_ || !helloDone_ || armed_) {
                logger_.warn("runner", "EndLockstep ignored: not in lock-step");
                break;
            }
            beginEpoch();
            proto::SimControl end;
            end.command = proto::SimCommand::EndLockstep;
            end.commandId = ++commandCounter_;
            end.sessionId = window_.sessionId();
            end.runEpoch = window_.runEpoch();
            awaitingResetAck_ = false;
            awaitingBeginAck_ = false;
            armed_ = true;
            paused_ = false;
            sendToRobot(end);
            sendDriverStation();
            break;
        }
        case proto::SimCommand::Stop: {
            if (link_ && helloDone_ && !stopping_) {
                proto::SimControl stop;
                stop.command = proto::SimCommand::Stop;
                stop.commandId = ++commandCounter_;
                stop.sessionId = window_.sessionId();
                stop.runEpoch = window_.runEpoch();
                stopCommandId_ = stop.commandId;
                stopping_ = true;
                ackDeadline_ = clock_.now() + std::chrono::milliseconds(watchdogMs_);
                sendToRobot(stop);
            } else if (!stopping_) {
                finish();
            }
            break;
        }
    }
}

void SessionRunner::beginEpoch() {
    epoch_ = ++epochCounter_;
    session_.reset();
    window_.begin(window_.sessionId(), epoch_.load());
    needFull_ = true;
    stepCredits_ = 0;
    nextTickAt_ = clock_.now();
    ticksSinceSnapshot_ = 0;
    brownoutSent_ = false;
    snapshotDirty_ = true;
}

// ---- inbound ----------------------------------------------------------------------------------------------------

void SessionRunner::handle(InboundEvent& ev) {
    switch (ev.kind) {
        case InboundEvent::Kind::Connected:
            if (link_) {
                logger_.info("runner", "a new robot connection replaces the previous one");
                handleDisconnect("replaced by a new connection");
            }
            link_ = std::move(ev.link);
            linkId_ = ev.linkId;
            helloDone_ = false;
            peerSupportsArmed_ = false;
            snapshotDirty_ = true;
            break;
        case InboundEvent::Kind::Disconnected:
            if (ev.linkId == linkId_ && link_) handleDisconnect(ev.detail);
            break;
        case InboundEvent::Kind::Message:
            if (ev.linkId != linkId_ || !link_) {
                ++dropped_;  // late message of a replaced / closed link
                break;
            }
            handleMessage(ev.message);
            break;
    }
}

void SessionRunner::handleMessage(proto::Message& message) {
    if (const auto* hello = std::get_if<proto::Hello>(&message)) {
        handleHello(*hello);
        return;
    }
    if (!helloDone_) {
        logger_.warn("runner", "message before Hello ignored");
        ++dropped_;
        return;
    }
    if (const auto* announce = std::get_if<proto::DeviceAnnounce>(&message)) {
        session_.hub().onAnnounce(*announce);
        sendLoopbackIfChanged(false);
        snapshotDirty_ = true;
    } else if (const auto* actuator = std::get_if<proto::ActuatorFrame>(&message)) {
        handleActuator(*actuator);
    } else if (const auto* ack = std::get_if<proto::SimControlAck>(&message)) {
        if (ack->commandId == resetCommandId_ && awaitingResetAck_) {
            awaitingResetAck_ = false;
            if (awaitingBeginAck_) {
                awaitingBeginAck_ = false;
                if (ack->result != proto::ControlResult::Applied) {
                    armed_ = true;
                    logger_.warn("runner", "the robot refused to start lock-step: " + ack->detail);
                }
            } else if (ack->result != proto::ControlResult::Applied) {
                logger_.warn("runner", "Reset was not applied by the robot: " + ack->detail);
            }
        } else if (ack->commandId == stopCommandId_ && stopping_) {
            finish();
        } else if (ack->commandId == pauseCommandId_ && ack->result != proto::ControlResult::Applied) {
            logger_.warn("runner", "the robot did not apply Pause/Resume: " + ack->detail);
        }
    } else if (const auto* log = std::get_if<proto::Log>(&message)) {
        if (proto::isLifecycleLog(*log, proto::kLifecycleProgramStarted)) {
            if (!programStarted_.exchange(true)) {
                nextTickAt_ = clock_.now();
                snapshotDirty_ = true;
                logger_.info("runner", "robot program started");
            }
            return;
        }
        logger_.log(log->level, "robot." + log->source, log->message);
    } else if (const auto* bye = std::get_if<proto::Goodbye>(&message)) {
        handleDisconnect("robot said goodbye: " + bye->reason);
    } else {
        logger_.warn("runner", "unexpected message type from the robot ignored");
        ++dropped_;
    }
}

void SessionRunner::handleHello(const proto::Hello& hello) {
    if (helloDone_) {
        logger_.warn("runner", "duplicate Hello ignored");
        return;
    }
    proto::HelloAck ack;
    if (hello.protocolVersion < proto::kMinCompatibleProtocolVersion ||
        (options_.discoveryOnly && !hello.supportsArmed)) {
        ack.accepted = false;
        ack.reason =
            hello.protocolVersion < proto::kMinCompatibleProtocolVersion
                ? "protocol version " + std::to_string(hello.protocolVersion) + " is too old"
                : "Armed operation requires the extension's explicit supportsArmed capability";
        {
            std::lock_guard lock(helloMutex_);
            refusal_ = ack.reason;
        }
        sendToRobot(ack);
        logger_.error("runner", "robot refused: " + ack.reason);
        closeLink();
        return;
    }
    {
        std::lock_guard lock(helloMutex_);
        hello_ = hello;
        refusal_.clear();
    }
    programStarted_ = !options_.awaitProgramStart;
    peerSupportsArmed_ = hello.supportsArmed;
    session_.hub().clearDevices();
    watchdogMs_ = proto::negotiateWatchdogTimeoutMs(hello.proposedWatchdogMs, options_.preferredWatchdogMs);
    window_.begin(++sessionCounter_, window_.runEpoch());
    beginEpoch();

    ack.accepted = true;
    ack.clockMode = proto::ClockMode::Lockstep;
    ack.stepUs = static_cast<std::uint32_t>(session_.clock().fixedStepUs());
    ack.sessionId = window_.sessionId();
    armed_ = options_.discoveryOnly;
    awaitingBeginAck_ = false;
    ack.discoveryOnly = armed_;
    ack.enableL3Hooks = options_.enableL3Hooks;
    ack.runEpoch = window_.runEpoch();
    ack.watchdogTimeoutMs = watchdogMs_;
    ack.watchdogPolicy = options_.watchdogPolicy;
    sendToRobot(ack);
    helloDone_ = true;
    firstActuator_ = true;
    sendVisionSetup();
    logger_.info("runner", "robot connected: pid " + std::to_string(hello.pid) + ", " + hello.programName + ", WPILib " + hello.wpilibVersion +
                               ", session " + std::to_string(window_.sessionId()) + ", epoch " + std::to_string(window_.runEpoch()));
    sendLoopbackIfChanged(true);
    sendDriverStation();
    if (paused_) sendPauseState();  // startPaused, or a pause that outlived the previous connection
}

void SessionRunner::handleActuator(const proto::ActuatorFrame& frame) {
    if (window_.stalled()) {  // a late answer after the watchdog fired: lock-step stays suspended until Reset/reconnect
        ++dropped_;
        return;
    }
    switch (window_.onAck(frame)) {
        case proto::AckVerdict::Completed:
            robotLoops_ = frame.robotLoopCount;
            session_.hub().onActuatorFrame(frame);
            if (firstActuator_) {
                firstActuator_ = false;
                session_.hub().markDiscoveryComplete();
            }
            sendLoopbackIfChanged(false);
            break;
        case proto::AckVerdict::Dropped: ++dropped_; break;
        case proto::AckVerdict::Stalled:
            logger_.error("runner", "the robot reported a stall; lock-step suspended (Reset to recover)");
            snapshotDirty_ = true;
            break;
        case proto::AckVerdict::Stopped:
            logger_.info("runner", "the robot left lock-step (Stopped)");
            finish();
            break;
        case proto::AckVerdict::ProtocolError:
            logger_.error("runner", "protocol error from the robot; closing the connection");
            handleDisconnect("protocol error");
            break;
    }
}

void SessionRunner::handleDisconnect(const std::string& why) {
    if (!link_) return;
    logger_.warn("runner", "robot disconnected: " + why);
    closeLink();
    helloDone_ = false;
    programStarted_ = false;
    awaitingResetAck_ = false;
    stopping_ = false;
    sentLoopbackRevision_ = ~std::uint64_t{0};
    window_.begin(window_.sessionId(), window_.runEpoch());
    session_.hub().clearDevices();
    snapshotDirty_ = true;
}

void SessionRunner::finish() {
    closeLink();
    helloDone_ = false;
    stopping_ = false;
    finished_ = true;
    snapshotDirty_ = true;
    refreshState();
    publishSnapshot(clock_.now(), true);
}

// ---- outbound ---------------------------------------------------------------------------------------------------

void SessionRunner::sendToRobot(const proto::Message& message) {
    if (!link_) return;
    if (Status s = link_->send(message); !s) handleDisconnect("send failed: " + s.error().message);
}

proto::DsState SessionRunner::effectiveDs() const {
    proto::DsState ds = requestedDs_;
    if (session_.battery().brownedOut()) ds.enabled = false;
    // The game's FMS data wins over the DS panel's manual text once the game has produced it.
    if (auto message = session_.gameSpecificMessage()) ds.gameSpecificMessage = std::move(*message);
    ds.sessionId = window_.sessionId();
    ds.runEpoch = window_.runEpoch();
    return ds;
}

void SessionRunner::sendDriverStation() {
    const proto::DsState ds = effectiveDs();
    gameMessageSent_ = session_.gameSpecificMessage();
    session_.setDriverStation(ds);
    if (link_ && helloDone_) sendToRobot(ds);
}

void SessionRunner::sendVisionSetup() {
    if (!link_ || !helloDone_) return;
    sdk::VisionSetup setup = session_.vision().setup();
    setup.photonVersion = options_.photonVersion;
    sendToRobot(setup);
}

void SessionRunner::sendLoopbackIfChanged(bool force) {
    if (!link_ || !helloDone_) return;
    const std::uint64_t revision = session_.hub().mappingRevision();
    if (!force && revision == sentLoopbackRevision_) return;
    sentLoopbackRevision_ = revision;
    sendToRobot(session_.hub().loopbackSet());
}

void SessionRunner::sendPauseState() {
    if (!link_ || !helloDone_) return;
    proto::SimControl control;
    control.command = paused_ ? proto::SimCommand::Pause : proto::SimCommand::Resume;
    control.commandId = ++commandCounter_;
    control.sessionId = window_.sessionId();
    control.runEpoch = window_.runEpoch();
    pauseCommandId_ = control.commandId;
    sendToRobot(control);
}

void SessionRunner::closeLink() {
    if (link_) link_->close();
    link_.reset();
    linkId_ = 0;
}

// ---- state / snapshot -------------------------------------------------------------------------------------------

void SessionRunner::refreshState() {
    RunnerState s = RunnerState::Running;
    if (finished_) s = RunnerState::Stopped;
    else if (!link_ || !helloDone_) s = RunnerState::WaitingForRobot;
    else if (window_.stalled()) s = RunnerState::RobotStalled;
    else if (paused_) s = RunnerState::Paused;
    if (state_.exchange(s) != s) snapshotDirty_ = true;
}

void SessionRunner::publishSnapshot(Clock::time_point now, bool force) {
    if (!force && (!snapshotDirty_ || now - lastPublish_ < options_.snapshotMinInterval)) return;
    std::vector<HubWarning> extra;
    if (window_.stalled() && link_)
        extra.push_back({Severity::Error, DeviceId{}, "robot_stalled", "the robot stopped answering; Reset or restart it"});
    sink_.publish(session_.buildSnapshot(paused_, link_ != nullptr && helloDone_, extra));
    snapshotDirty_ = false;
    lastPublish_ = now;
}

}  // namespace wpiemu::core::runtime
