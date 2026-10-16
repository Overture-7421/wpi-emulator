#pragma once

// One run of the robot project, Qt-free: the real composition root of the product. It turns a RobotConfig into a
// simulation session (game plugin by year, Box3D world, registries, mechanisms), serves it over the local IPC
// transport with a SessionRunner and launches the unmodified robot project (Gradle) with the HALSim extension.
//
// Used by the Qt shell (RealRuntimeHost: the UI thread polls status()/takeLogs()/latestSnapshot()) and by the opt-in
// end-to-end test. Every method is thread-safe unless noted.

#include <atomic>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/launcher/robot_launcher.h"
#include "wpiemu/core/runtime/session_runner.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app::host {

enum class Mode { Run, Discovery };
enum class ProcessState { Idle, Starting, Running, Stopping, Exited, Crashed, FailedToStart };
enum class LinkState { Disconnected, Connecting, Connected, HandshakeTimeout, ProtocolMismatch, Stalled };
enum class LogStream { Stdout, Stderr, Emulator };

struct LogEntry {
    std::int64_t timestampMs = 0;   // wall clock, ms since the epoch
    sdk::LogLevel level = sdk::LogLevel::Info;
    LogStream stream = LogStream::Stdout;
    std::string source;
    std::string text;
};

struct SessionStatus {
    Mode mode = Mode::Run;
    ProcessState process = ProcessState::Idle;
    LinkState link = LinkState::Disconnected;
    int exitCode = 0;
    std::string detail;            // human readable reason / progress
    bool programStarted = false;   // the robot constructor finished (devices announced, lock-step may start)

    bool active() const { return process == ProcessState::Starting || process == ProcessState::Running || process == ProcessState::Stopping; }
    bool operator==(const SessionStatus&) const = default;
};

// Collaborators owned by the composition root (non-owning; they outlive every session).
struct Registries {
    const sdk::GamePluginRegistry* plugins = nullptr;
    sdk::MechanismFactory* mechanisms = nullptr;
    const sdk::DrivetrainRegistry* drivetrains = nullptr;
    const sdk::TireModelRegistry* tireModels = nullptr;
    core::config::PluginRegistrar* registrar = nullptr;
    const core::config::ConfigPipeline* pipeline = nullptr;
    const sdk::IValueTextCodec* json = nullptr;
};

struct Options {
    Mode mode = Mode::Run;
    std::filesystem::path projectDir;
    std::filesystem::path extensionLibrary;               // halsim_wpiemu.{dll,so,dylib}
    core::launcher::LaunchMode launchMode = core::launcher::LaunchMode::GradleInitScript;
    bool release = true;                                  // ExternalDirect: Release vs Debug variant
    std::filesystem::path scratchDir;                     // generated init script; empty = <temp>/wpiemu
    std::filesystem::path javaHome;                       // empty = the WPILib JDK of the project year, else JAVA_HOME / PATH
    bool gradleOffline = false;
    std::vector<std::string> extraGradleArgs;
    std::map<std::string, std::string> extraEnv;          // robot-process environment (wins over everything)
    bool preStartPolling = false;                         // WPIEMU_LIVE_DISCOVERY: poll the bridges during the robot constructor (crashes the JVM with CTRE)
    double timeScale = 1.0;
    bool startPaused = false;
    sdk::proto::DsState driverStation;
    std::uint32_t watchdogMs = 0;                         // 0 = negotiated default
    std::chrono::milliseconds handshakeTimeout{std::chrono::minutes(5)};  // robot started but the extension never connected
    std::chrono::milliseconds stopGrace{500};             // how long the robot may leave by itself before its tree is killed
};

// Static field the plugin would build (for drawing it before a run). Null when `module` is null or fails.
std::shared_ptr<const sdk::FieldDescription> describeField(sdk::IGameModule* module);

// Where the robot starts: inside the plugin's field when it has one, else at the origin.
struct StartPlacement {
    float xM = 0.f, yM = 0.f;
};
StartPlacement startPlacementFor(const sdk::FieldDescription* field);

class RobotSession {
public:
    // Validates the config (Run: errors refuse the start; Discovery: reported as warnings), builds the session and
    // starts the IPC server, the runner thread and the robot launch. Errors are user-presentable.
    static sdk::Result<std::unique_ptr<RobotSession>> start(const Registries& registries, sdk::RobotConfig config, Options options);

    // Synchronous teardown: the whole robot process tree is killed. Never leaves an orphan.
    ~RobotSession();
    RobotSession(const RobotSession&) = delete;
    RobotSession& operator=(const RobotSession&) = delete;

    // Recomputes and returns the state (call periodically).
    SessionStatus status();
    std::vector<LogEntry> takeLogs(std::size_t maxEntries = 2000);
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const { return slot_.latest(); }

    sdk::ISimController& controller() { return *runner_; }
    core::runtime::SessionRunner& runner() { return *runner_; }
    const sdk::IGameModule* gameModule() const { return module_; }
    const std::string& endpoint() const { return endpoint_; }
    bool coreOnly() const { return module_ == nullptr; }
    const std::vector<sdk::Issue>& configIssues() const { return issues_; }

    // Non-blocking: sends the protocol Stop and tears everything down on a worker. finished() turns true at the end.
    void requestStop();
    bool stopRequested() const { return stopRequested_.load(); }
    bool finished() const { return teardownDone_.load(); }
    void waitUntilFinished();   // joins the teardown worker

private:
    class LogQueue;
    class OutputSink;
    class QueueLogger;

    RobotSession();
    void beginTeardown(bool protocolStop);
    void teardown();
    std::string outputTail() const;
    void fail(ProcessState process, LinkState link, std::string detail);

    Options options_;
    std::unique_ptr<LogQueue> logs_;
    std::unique_ptr<QueueLogger> logger_;
    std::unique_ptr<OutputSink> sink_;
    sdk::LatestSnapshotSlot slot_;
    std::unique_ptr<core::runtime::SimulationSession> session_;
    const sdk::IGameModule* module_ = nullptr;
    std::vector<sdk::Issue> issues_;
    std::string endpoint_;

    core::runtime::MessageInbox inbox_;
    std::unique_ptr<core::runtime::RobotLinkAcceptor> acceptor_;
    std::unique_ptr<core::runtime::SessionRunner> runner_;
    std::unique_ptr<core::launcher::RobotLauncher> launcher_;

    std::chrono::steady_clock::time_point startedAt_;
    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> teardownStarted_{false};
    std::atomic<bool> teardownDone_{false};
    std::thread teardownThread_;
    std::mutex statusMutex_;
    SessionStatus status_;
    bool seenConnection_ = false;
    bool failed_ = false;
};

}  // namespace wpiemu::app::host
