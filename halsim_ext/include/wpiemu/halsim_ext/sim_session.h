#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "wpiemu/halsim_ext/bridge_set.h"
#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/halsim_ext/device_table.h"
#include "wpiemu/halsim_ext/forwarding_logger.h"
#include "wpiemu/halsim_ext/hal_ports.h"
#include "wpiemu/halsim_ext/ipc_link.h"
#include "wpiemu/halsim_ext/runtime_stats.h"
#include "wpiemu/halsim_ext/signal_delta_filter.h"
#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::halsim {

enum class SessionEnd : std::uint8_t {
    Disconnected,  // transport lost or the emulator said Goodbye: watchdog policy applied
    Stopped,       // the emulator sent Stop: timing resumed, no reconnect
    Shutdown,      // requestShutdown(): the robot process is going away
};

// What the session shares with the runtime that owns it (non-owning; all outlive the session).
struct SessionServices {
    const ExtensionConfig& config;
    HalPorts ports;
    BridgeSet& bridges;
    DeviceTable& devices;
    ForwardingLogger& logger;
    AtomicRuntimeStats& stats;
};

// One connection to the emulator, from the accepted HelloAck to the transport going away. Runs the
// per-tick protocol of ARCHITECTURE section 4 on the extension's clock thread, plus a watchdog
// thread that unblocks the robot by itself (SDK 1.1) when the emulator or the robot stops answering.
class SimSession {
public:
    SimSession(SessionServices services, std::unique_ptr<IpcLink> link, const sdk::proto::HelloAck& ack);
    ~SimSession();
    SimSession(const SimSession&) = delete;
    SimSession& operator=(const SimSession&) = delete;

    // Starts the link threads, re-announces known devices and routes logs to the emulator.
    void begin();
    // Blocks the calling (clock) thread until the session ends.
    SessionEnd run();
    // Any thread: ends run() promptly and resumes HAL timing.
    void requestShutdown();

    bool lockstep() const { return lockstep_; }
    sdk::proto::WatchdogPolicy watchdogPolicy() const { return ack_.watchdogPolicy; }
    std::chrono::milliseconds watchdogTimeout() const { return std::chrono::milliseconds(watchdogMs_); }

private:
    enum class Degraded : std::uint8_t { None, Stalled, EmulatorLost };
    using Clock = std::chrono::steady_clock;

    static std::int64_t nowNs();

    // ---- clock thread ----
    std::optional<SessionEnd> waitForProgramStart();
    SessionEnd serve();
    std::optional<SessionEnd> dispatch(sdk::proto::Message&& message);
    void onSensorFrame(sdk::proto::SensorFrame&& frame);
    void onDsState(sdk::proto::DsState&& state);
    std::optional<SessionEnd> onControl(const sdk::proto::SimControl& control);
    void applyPendingDriverStation();
    void idle();
    void drainRetired();
    void post(sdk::proto::Message message);
    void touch();

    // ---- any thread ----
    void watchdogLoop();
    void checkWatchdog();
    void unblockRobot(Degraded why, const char* reason);
    void applyPolicy(const char* reason);

    SessionServices s_;
    std::unique_ptr<IpcLink> link_;
    sdk::proto::HelloAck ack_;
    bool lockstep_ = false;
    std::uint32_t watchdogMs_ = sdk::proto::kDefaultWatchdogTimeoutMs;

    // Clock-thread state.
    sdk::proto::LockstepFollower follower_;
    SignalDeltaFilter delta_;
    std::vector<sdk::DeviceId> loopback_;
    std::vector<sdk::SignalSample> outputs_;
    std::optional<sdk::proto::DsState> pendingDs_;
    std::optional<sdk::proto::SensorFrame> deferredFrame_;
    std::uint64_t ticksSinceFull_ = 0;
    double lastBatteryV_ = -1.0;
    double lastBatteryA_ = -1.0;
    bool programStarted_ = false;

    // Shared with the watchdog.
    std::atomic<std::uint64_t> sessionId_{0};
    std::atomic<std::uint64_t> epoch_{0};
    std::atomic<bool> armed_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> shutdown_{false};
    std::atomic<bool> unblocked_{false};
    std::atomic<Degraded> degraded_{Degraded::None};
    std::atomic<std::int64_t> lastActivityNs_{0};

    std::mutex stepMutex_;
    std::int64_t stepStartNs_ = 0;  // 0 = no step in flight
    bool stepAbandoned_ = false;
    std::uint64_t inflightTick_ = 0;
    sdk::SimTimeUs inflightSimTimeUs_ = 0;

    std::mutex retiredMutex_;
    std::vector<sdk::DeviceId> retired_;

    std::mutex watchdogMutex_;
    std::condition_variable watchdogCv_;
    bool watchdogStop_ = false;
    std::thread watchdog_;
};

}  // namespace wpiemu::halsim
