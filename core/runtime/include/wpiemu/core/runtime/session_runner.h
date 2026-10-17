#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>
#include <vector>

#include "wpiemu/core/runtime/robot_link.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/telemetry/sim_controller.h"

namespace wpiemu::core::runtime {

// Monotonic wall clock; injectable so the watchdog and pacing are testable without sleeping.
class IWallClock {
    WPIEMU_INTERFACE(IWallClock)
public:
    virtual std::chrono::steady_clock::time_point now() const = 0;
};

class SteadyWallClock final : public IWallClock {
public:
    std::chrono::steady_clock::time_point now() const override { return std::chrono::steady_clock::now(); }
};

struct RunnerOptions {
    double timeScale = 1.0;                     // 1 = real time, 0 = as fast as the robot allows, 0.5 = half speed
    std::uint32_t preferredWatchdogMs = 0;      // 0 = no preference (negotiated with the extension's proposal)
    sdk::proto::WatchdogPolicy watchdogPolicy = sdk::proto::WatchdogPolicy::ResumeRealtime;
    // Start ARMED: announce devices and keep the robot running in real time without stepping until
    // SimCommand::BeginLockstep (and again after EndLockstep). One robot process serves detection and the run.
    bool discoveryOnly = false;
    bool enableL3Hooks = false;
    bool startPaused = false;
    // Hold the first SensorFrame (and the watchdog) until the extension reports `program_started` (lifecycle Log, sdk/ipc/lifecycle.h):
    // a robot constructor may legitimately take longer than the watchdog. False = tick right after the handshake (peers without the signal).
    bool awaitProgramStart = false;
    std::uint32_t snapshotEveryTicks = 4;       // 50 Hz at 200 Hz
    std::chrono::milliseconds snapshotMinInterval{20};
    std::uint64_t maxStepCredits = 100000;
};

enum class RunnerState {
    WaitingForRobot,   // no (handshaken) connection
    Running,
    Paused,
    RobotStalled,      // the extension reported a stall or the watchdog expired; Reset or reconnect to recover
    Stopped,           // session over (Stop command, extension Stopped); the runner is finished
};

// The lock-step driver around a SimulationSession (ARCHITECTURE 3, 4.1). Runs on the sim thread; the UI talks to it
// only through the thread-safe ISimController side. Responsibilities:
//  * handshake (Hello -> HelloAck with sessionId / runEpoch / negotiated watchdog), reconnect = new epoch + reset;
//  * exactly one outstanding tick (LockstepWindow), stale/foreign frames dropped, ack verdicts handled;
//  * commands: Pause / Resume / Step (local, the emulator is the clock master), Reset (epoch bump, acked by the
//    extension), Stop (acked), SetTimeScale (pacing), DS state, live bind/unbind (LoopbackSet resent);
//  * stall watchdog: tick outstanding longer than the negotiated timeout -> RobotStalled, no more frames;
//  * brownout: the DS state sent to the robot is forced disabled while the battery is browned out;
//  * snapshot publishing to the ISnapshotSink (every N ticks and on state changes).
// poll() performs one bounded iteration (tests drive it deterministically); start() runs it on a thread.
class SessionRunner final : public sdk::ISimController {
public:
    SessionRunner(SimulationSession& session, MessageInbox& inbox, sdk::ISnapshotSink& sink, sdk::ILogger& logger, RunnerOptions options = {},
                  const IWallClock* clock = nullptr);
    ~SessionRunner() override;

    // ---- sdk::ISimController (thread-safe, non-blocking) ----
    void setDriverStation(const sdk::proto::DsState& ds) override;
    void control(const sdk::proto::SimControl& command) override;
    void bindSlot(const sdk::SlotAddress& slot, const sdk::DeviceKey& device) override;
    void unbindSlot(const sdk::SlotAddress& slot) override;

    // ---- driving ----
    // One iteration: apply queued commands, drain inbound events (waiting at most `maxWait` when there is nothing
    // to do), enforce the watchdog and send the next SensorFrame when due.
    void poll(std::chrono::milliseconds maxWait);
    void start();  // poll() loop on an own thread until stop() or the session ends
    void stop();   // joins the thread; does not send Stop to the robot (use control(Stop))

    // ---- observation (any thread) ----
    RunnerState state() const { return state_.load(); }
    std::uint64_t ticksSent() const { return ticksSent_.load(); }
    std::uint64_t runEpoch() const { return epoch_.load(); }
    std::uint64_t droppedFrames() const { return dropped_.load(); }
    std::optional<sdk::proto::Hello> robotHello() const;
    // True once the robot program started (always true after the handshake unless RunnerOptions::awaitProgramStart).
    bool programStarted() const { return programStarted_.load(); }
    // True while connected and not stepping (armed start, before BeginLockstep or after EndLockstep).
    bool armed() const { return armed_.load(); }
    // Periodic iterations the robot reported in its last ActuatorFrame (diagnostics; resets with the robot process).
    std::uint64_t robotLoopCount() const { return robotLoops_.load(); }
    // Why the last connection was refused in the handshake (e.g. protocol too old); empty when none was.
    std::string refusal() const;

private:
    struct SetDs { sdk::proto::DsState ds; };
    struct Control { sdk::proto::SimControl command; };
    struct Bind { sdk::SlotAddress slot; sdk::DeviceKey device; };
    struct Unbind { sdk::SlotAddress slot; };
    using Command = std::variant<SetDs, Control, Bind, Unbind>;

    void enqueue(Command command);
    void applyCommands(std::vector<Command>& batch);
    void applyControl(const sdk::proto::SimControl& command);
    void handle(InboundEvent& event);
    void handleMessage(sdk::proto::Message& message);
    void handleHello(const sdk::proto::Hello& hello);
    void handleActuator(const sdk::proto::ActuatorFrame& frame);
    void handleDisconnect(const std::string& why);
    void checkWatchdog();
    bool shouldTick(std::chrono::steady_clock::time_point now) const;
    std::chrono::milliseconds waitHint(std::chrono::milliseconds maxWait, std::chrono::steady_clock::time_point now) const;
    void doTick(std::chrono::steady_clock::time_point now);
    void beginEpoch();
    void sendDriverStation();
    void sendLoopbackIfChanged(bool force);
    void sendPauseState();
    void sendToRobot(const sdk::proto::Message& message);
    void closeLink();
    void refreshState();
    void finish();
    void publishSnapshot(std::chrono::steady_clock::time_point now, bool force);
    sdk::proto::DsState effectiveDs() const;

    SimulationSession& session_;
    MessageInbox& inbox_;
    sdk::ISnapshotSink& sink_;
    sdk::ILogger& logger_;
    RunnerOptions options_;
    SteadyWallClock defaultClock_;
    const IWallClock& clock_;

    // command queue (UI thread -> sim thread)
    std::mutex commandMutex_;
    std::vector<Command> commands_;

    // sim-thread state
    std::shared_ptr<IRobotLink> link_;
    std::uint64_t linkId_ = 0;
    bool helloDone_ = false;
    bool peerSupportsArmed_ = false;
    bool paused_ = false;
    bool finished_ = false;
    bool awaitingResetAck_ = false;
    bool stopping_ = false;
    bool needFull_ = true;
    bool firstActuator_ = true;
    bool brownoutSent_ = false;
    bool snapshotDirty_ = true;
    std::uint64_t stepCredits_ = 0;
    double timeScale_ = 1.0;
    sdk::proto::LockstepWindow window_;
    std::uint64_t sessionCounter_ = 0;
    std::uint64_t epochCounter_ = 0;
    std::uint64_t commandCounter_ = 0;
    std::uint64_t resetCommandId_ = 0;
    std::uint64_t stopCommandId_ = 0;
    std::uint64_t pauseCommandId_ = 0;  // last Pause/Resume forwarded; the extension disarms its watchdog while paused
    std::uint32_t watchdogMs_ = sdk::proto::kDefaultWatchdogTimeoutMs;
    std::uint64_t sentLoopbackRevision_ = ~std::uint64_t{0};
    sdk::proto::DsState requestedDs_;
    std::optional<std::string> gameMessageSent_;  // the game's FMS message as of the last DsState sent
    std::chrono::steady_clock::time_point tickDeadline_{};
    std::chrono::steady_clock::time_point ackDeadline_{};
    std::chrono::steady_clock::time_point nextTickAt_{};
    std::chrono::steady_clock::time_point lastPublish_{};
    std::uint64_t ticksSinceSnapshot_ = 0;

    mutable std::mutex helloMutex_;
    std::optional<sdk::proto::Hello> hello_;
    std::string refusal_;
    std::atomic<bool> programStarted_{false};
    std::atomic<bool> armed_{false};
    bool awaitingBeginAck_ = false;
    std::atomic<std::uint64_t> robotLoops_{0};
    std::atomic<RunnerState> state_{RunnerState::WaitingForRobot};
    std::atomic<std::uint64_t> ticksSent_{0};
    std::atomic<std::uint64_t> epoch_{0};
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<bool> stopThread_{false};
    std::thread thread_;
};

}  // namespace wpiemu::core::runtime
