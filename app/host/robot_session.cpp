#include "host/robot_session.h"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <sstream>

#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/launcher/output_level.h"
#include "wpiemu/core/launcher/project_inspector.h"
#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/core/runtime/physics_wiring.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/local_ipc.h"

namespace wpiemu::app::host {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;
namespace launcher = core::launcher;
namespace runtime = core::runtime;

namespace {

constexpr bool kWindows = fs::path::preferred_separator == '\\';
constexpr std::size_t kMaxQueuedLogs = 50000;
constexpr std::size_t kTailLines = 40;

std::int64_t wallMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string describeIssues(const std::vector<Issue>& issues, std::size_t limit) {
    std::ostringstream out;
    std::size_t shown = 0, errors = 0;
    for (const Issue& i : issues)
        if (i.severity == Severity::Error) ++errors;
    for (const Issue& i : issues) {
        if (i.severity != Severity::Error) continue;
        if (shown++ >= limit) break;
        out << "\n  - " << (i.path.empty() ? "" : i.path + ": ") << i.message;
    }
    if (errors > shown) out << "\n  - ... and " << (errors - shown) << " more";
    return out.str();
}

std::string detectLanguage(const fs::path& dir, const std::string& configured) {
    if (configured == "java" || configured == "cpp") return configured;
    std::error_code ec;
    if (fs::exists(dir / "src" / "main" / "cpp", ec) && !fs::exists(dir / "src" / "main" / "java", ec)) return "cpp";
    return "java";
}

// The JDK Gradle runs with: the first of launcher::kJdkSources that is present. Empty means `java` on PATH.
fs::path pickJdkHome(const Options& options, int wpilibYear) {
    std::optional<launcher::WpilibInstall> install;
    if (!options.wpilibHome.empty()) install = launcher::wpilibInstallAt(options.wpilibHome, wpilibYear);
    else if (wpilibYear > 0) install = launcher::findWpilibInstall(wpilibYear, {});
    const std::optional<std::string> javaHomeEnv = sdk::readEnvironment("JAVA_HOME");
    for (const launcher::JdkSource source : launcher::kJdkSources) {
        switch (source) {
            case launcher::JdkSource::Setting:
                if (!options.javaHome.empty()) return options.javaHome;
                break;
            case launcher::JdkSource::WpilibInstall:
                if (install && !install->jdkHome.empty()) return install->jdkHome;
                break;
            case launcher::JdkSource::JavaHome:
                if (javaHomeEnv && !javaHomeEnv->empty()) {
                    std::error_code ec;
                    const fs::path candidate(*javaHomeEnv);
                    if (fs::is_regular_file(candidate / "bin" / (kWindows ? "java.exe" : "java"), ec) ||
                        fs::is_regular_file(candidate / "bin" / (kWindows ? "java" : "java.exe"), ec))
                        return candidate;
                }
                break;
            case launcher::JdkSource::Path: break;
        }
    }
    return {};
}

}  // namespace

// ---- log plumbing ----------------------------------------------------------------------------------------------------------------

class RobotSession::LogQueue {
public:
    void add(SessionLogEntry entry) {
        std::lock_guard lock(mutex_);
        if (entry.stream != LogStream::Emulator) {
            tail_.push_back(entry.text);
            if (tail_.size() > kTailLines) tail_.pop_front();
        }
        queue_.push_back(std::move(entry));
        if (queue_.size() > kMaxQueuedLogs) {
            queue_.pop_front();
            ++dropped_;
        }
    }
    std::vector<SessionLogEntry> take(std::size_t max) {
        std::lock_guard lock(mutex_);
        std::vector<SessionLogEntry> out;
        const std::size_t n = std::min(max, queue_.size());
        out.reserve(n + 1);
        if (dropped_ > 0) {
            SessionLogEntry note;
            note.timestampMs = wallMs();
            note.level = LogLevel::Warn;
            note.stream = LogStream::Emulator;
            note.source = "log";
            note.text = std::to_string(dropped_) + " console line(s) were dropped (the robot logs faster than the UI reads)";
            out.push_back(std::move(note));
            dropped_ = 0;
        }
        for (std::size_t i = 0; i < n; ++i) {
            out.push_back(std::move(queue_.front()));
            queue_.pop_front();
        }
        return out;
    }
    std::string tail() const {
        std::lock_guard lock(mutex_);
        std::string out;
        for (const std::string& l : tail_) out += "\n  " + l;
        return out;
    }

private:
    mutable std::mutex mutex_;
    std::deque<SessionLogEntry> queue_;
    std::deque<std::string> tail_;
    std::size_t dropped_ = 0;
};

class RobotSession::OutputSink final : public launcher::ILogSink {
public:
    explicit OutputSink(LogQueue& queue) : queue_(queue) {}
    void line(launcher::OutputStream stream, std::string_view text) override {
        SessionLogEntry e;
        e.timestampMs = wallMs();
        e.stream = stream == launcher::OutputStream::Stderr ? LogStream::Stderr : LogStream::Stdout;
        e.level = launcher::classifyOutputLine(stream, text);
        e.source = stream == launcher::OutputStream::Stderr ? "robot.stderr" : "robot";
        e.text = std::string(text);
        queue_.add(std::move(e));
    }

private:
    LogQueue& queue_;
};

class RobotSession::QueueLogger final : public ILogger {
public:
    explicit QueueLogger(LogQueue& queue) : queue_(queue) {}
    void log(LogLevel level, std::string_view category, std::string_view message) override {
        if (level == LogLevel::Trace) return;
        SessionLogEntry e;
        e.timestampMs = wallMs();
        e.level = level;
        e.stream = LogStream::Emulator;
        e.source = std::string(category);
        e.text = std::string(message);
        queue_.add(std::move(e));
    }

private:
    LogQueue& queue_;
};

// ---- field -----------------------------------------------------------------------------------------------------------------------

std::shared_ptr<const FieldDescription> describeField(IGameModule* module) {
    if (module == nullptr) return nullptr;
    auto world = core::physics::createBox3dWorld(PhysicsConfig{});
    if (!world) return nullptr;
    auto field = module->createField(*world);
    if (!field) return nullptr;
    auto description = std::make_shared<const FieldDescription>(field.value()->description());
    return description;   // `field` is destroyed before `world`
}

StartPlacement startPlacementFor(const FieldDescription* field,
                                 const std::optional<RobotStartPose>& configured) {
    if (configured) return {configured->xM, configured->yM, configured->headingRad};
    if (field == nullptr || field->lengthM <= 0.f || field->widthM <= 0.f) return {};
    return {std::min(2.0f, field->lengthM * 0.15f), field->widthM * 0.5f};
}

// ---- construction -----------------------------------------------------------------------------------------------------------------

RobotSession::RobotSession() {
    logs_ = std::make_unique<LogQueue>();
    logger_ = std::make_unique<QueueLogger>(*logs_);
    sink_ = std::make_unique<OutputSink>(*logs_);
}

namespace {

Result<std::unique_ptr<runtime::SimulationSession>> buildSession(const Registries& reg, RobotConfig config, ILogger& logger, IGameModule* module,
                                                                  const FieldDescription* field) {
    runtime::SessionParts parts;
    parts.world = core::physics::createBox3dWorld(config.sim.physics);
    if (!parts.world) return makeError(ErrorCode::Internal, "the physics world could not be created (invalid physics settings)");
    if (module == nullptr) {
        // Core-only mode (no game plugin for this season): the robot still needs a floor to drive on.
        BodyDesc floor;
        floor.type = BodyType::Static;
        floor.pose.position = Vec3{0.f, 0.f, -0.5f};
        const BodyId body = parts.world->createBody(floor);
        ShapeDesc slab;
        slab.geometry = BoxShape{Vec3{50.f, 50.f, 0.5f}};
        slab.material = Material{0.8f, 0.f, 0.f};
        slab.category = layer::Field;
        if (!body.valid() || !parts.world->addShape(body, slab).valid()) return makeError(ErrorCode::Internal, "could not create the floor");
    }
    const StartPlacement start = startPlacementFor(field, config.startPose);
    core::physics::ChassisPlacement placement;
    placement.xM = start.xM;
    placement.yM = start.yM;
    placement.yawRad = start.headingRad;
    parts.config = std::move(config);
    parts.mechanisms = reg.mechanisms;
    parts.drivetrains = reg.drivetrains;
    parts.tireModels = reg.tireModels;
    parts.plugins = reg.plugins;
    parts.logger = &logger;
    parts.chassisBuilder = runtime::physicsChassisBuilder(placement);
    parts.coupling = std::make_unique<runtime::PhysicsCouplingDriver>();
    return runtime::SessionBuilder::build(std::move(parts));
}

}  // namespace

Result<std::unique_ptr<RobotSession>> RobotSession::start(const Registries& reg, RobotConfig config, Options options) {
    if (reg.plugins == nullptr || reg.mechanisms == nullptr || reg.drivetrains == nullptr || reg.tireModels == nullptr || reg.registrar == nullptr ||
        reg.pipeline == nullptr || reg.json == nullptr)
        return makeError(ErrorCode::InvalidArgument, "incomplete registries");

    std::unique_ptr<RobotSession> self(new RobotSession());
    ILogger& log = *self->logger_;
    self->options_ = options;
    self->status_.mode = options.mode;

    // ---- project, toolchain and extension: cheap checks first, with actionable messages ----
    std::error_code ec;
    if (options.projectDir.empty() || !fs::is_directory(options.projectDir, ec))
        return makeError(ErrorCode::NotFound, "The robot project folder does not exist: " + options.projectDir.string());
    const fs::path wrapper = launcher::gradleWrapperPath(options.projectDir, kWindows);
    if (!fs::exists(wrapper, ec))
        return makeError(ErrorCode::NotFound, "This project has no Gradle wrapper (" + wrapper.filename().string() +
                                                 "), so the emulator cannot launch it. Use a standard WPILib project (it contains gradlew and gradlew.bat).");
    if (options.extensionLibrary.empty() || !fs::exists(options.extensionLibrary, ec))
        return makeError(ErrorCode::NotFound, "The HALSim extension (" + options.extensionLibrary.string() +
                                                 ") was not found. Reinstall the emulator, build the halsim_wpiemu target, or set WPIEMU_EXTENSION_PATH.");

    // ---- config: defaults, validation ----
    auto loaded = reg.pipeline->process(std::move(config));
    if (!loaded) return makeError(loaded.error().code, "The robot configuration cannot be used: " + loaded.error().message);
    self->issues_ = loaded.value().issues;
    RobotConfig cfg = std::move(loaded.value().config);
    if (!loaded.value().valid()) {
        if (options.mode == SessionMode::Run)
            return makeError(ErrorCode::InvalidState, "The robot configuration has errors that must be fixed before the robot can run:" +
                                                          describeIssues(self->issues_, 5));
        log.warn("config", "the robot configuration has errors; discovery runs anyway:" + describeIssues(self->issues_, 5));
    }

    // ---- game plugin by season ----
    IGameModule* module = core::config::selectPlugin(cfg, *reg.plugins);
    if (module != nullptr) {
        if (Status s = reg.registrar->ensureRegistered(*module); !s) return makeError(s.error().code, "game plugin: " + s.error().message);
        log.info("session", "game plugin: " + module->metadata().displayName + " (" + module->metadata().id + ")");
    } else {
        log.info("session", cfg.project.wpilibYear > 0
                                ? "no game plugin for season " + std::to_string(cfg.project.wpilibYear) + ": core features only (chassis, drivetrain, mechanisms)"
                                : "no game plugin selected: core features only");
    }
    self->module_ = module;

    // ---- simulation session ----
    const std::shared_ptr<const FieldDescription> field = describeField(module);
    auto built = buildSession(reg, cfg, log, module, field.get());
    if (!built && options.mode == SessionMode::Armed) {
        // An armed session only needs the device hub until the simulation starts: fall back to the bare chassis when the full model cannot be built.
        log.warn("session", "the robot model could not be built (" + built.error().message + "); discovering with an empty model");
        RobotConfig bare = cfg;
        bare.mechanisms.clear();
        bare.drivetrain.spec.wheels.clear();
        bare.drivetrain.mappings.clear();
        bare.chassis = sdk::ChassisSpec{};
        built = buildSession(reg, std::move(bare), log, module, field.get());
    }
    if (!built) return makeError(built.error().code, "The simulation could not be built: " + built.error().message);
    self->session_ = std::move(built).value();
    self->session_->setStartPlacement(options.startPose);
    self->session_->setDriverStation(options.driverStation);

    // ---- IPC server + sim thread ----
    const ipc::ConnectionEndpoint endpoint = ipc::ConnectionEndpoint::generate();
    auto server = ipc::makeLocalServer(endpoint);
    if (!server) return makeError(server.error().code, "cannot open the IPC endpoint: " + server.error().message);
    self->acceptor_ = std::make_unique<runtime::RobotLinkAcceptor>(std::move(server).value(), std::make_shared<ipc::BinaryMessageCodec>(), self->inbox_, &log);
    if (Status s = self->acceptor_->start(); !s) return makeError(s.error().code, "cannot listen for the robot: " + s.error().message);
    self->endpoint_ = self->acceptor_->endpoint();

    runtime::RunnerOptions ro;
    ro.timeScale = options.timeScale;
    ro.startPaused = options.startPaused;
    ro.discoveryOnly = options.mode == SessionMode::Armed;
    ro.awaitProgramStart = true;
    ro.preferredWatchdogMs = options.watchdogMs;
    // The inspection is read-only and optional: without it PhotonVision cameras use the extension's default version.
    if (const auto report = launcher::ProjectInspector(*reg.json).inspect(options.projectDir)) ro.photonVersion = report.value().photonLibVersion();
    self->runner_ = std::make_unique<runtime::SessionRunner>(*self->session_, self->inbox_, self->slot_, log, ro);
    self->runner_->setDriverStation(options.driverStation);
    self->runner_->start();

    // ---- robot launch ----
    launcher::LaunchRequest req;
    req.projectDir = options.projectDir;
    req.language = detectLanguage(options.projectDir, cfg.project.language);
    req.mode = options.launchMode;
    req.release = options.release;
    req.extensionLibrary = options.extensionLibrary;
    req.ipcEndpoint = self->endpoint_;
    req.scratchDir = options.scratchDir.empty() ? fs::temp_directory_path() / "wpiemu" : options.scratchDir;
    req.javaHome = pickJdkHome(options, cfg.project.wpilibYear);
    req.gradleOffline = options.gradleOffline;
    req.extraGradleArgs = options.extraGradleArgs;
    req.extraEnv[sdk::env_names::kLiveDiscovery] = options.preStartPolling ? "1" : "0";
    for (const auto& [k, v] : options.extraEnv) req.extraEnv[k] = v;
    fs::create_directories(req.scratchDir, ec);

    self->launcher_ = std::make_unique<launcher::RobotLauncher>(launcher::systemProcessFactory(), *reg.json, log);
    self->startedAt_ = std::chrono::steady_clock::now();
    if (Status s = self->launcher_->launch(req, *self->sink_); !s) return makeError(s.error().code, "The robot could not be launched: " + s.error().message);
    log.info("session", std::string(options.mode == SessionMode::Armed ? "launching the robot (armed)" : "starting the robot") + " (" + req.language + ", " +
                            (req.mode == launcher::LaunchMode::GradleInitScript ? "gradle simulate task" : "simulateExternal direct launch") + ")");
    self->status_.process = ProcessState::Starting;
    self->status_.link = LinkState::Connecting;
    self->status_.detail = "Building and starting the robot project...";
    return self;
}

// ---- state --------------------------------------------------------------------------------------------------------------------

std::string RobotSession::outputTail() const { return logs_->tail(); }

std::vector<SessionLogEntry> RobotSession::takeLogs(std::size_t maxEntries) { return logs_->take(maxEntries); }

void RobotSession::fail(ProcessState process, LinkState link, std::string detail) {
    status_.process = process;
    status_.link = link;
    status_.detail = std::move(detail);
    failed_ = true;
    logger_->error("session", status_.detail);
    beginTeardown(false);
}

SessionStatus RobotSession::status() {
    std::lock_guard lock(statusMutex_);
    SessionStatus& s = status_;
    s.mode = options_.mode;
    if (failed_) return s;

    const runtime::RunnerState rs = runner_->state();
    const bool linked = rs == runtime::RunnerState::Running || rs == runtime::RunnerState::Paused || rs == runtime::RunnerState::RobotStalled;
    s.programStarted = linked && runner_->programStarted();
    s.simulating = linked && !runner_->armed();
    if (linked) seenConnection_ = true;

    if (stopRequested_) {
        s.process = teardownDone_ ? ProcessState::Exited : ProcessState::Stopping;
        s.link = LinkState::Disconnected;
        s.programStarted = false;
        s.simulating = false;
        s.exitCode = 0;
        s.detail = teardownDone_ ? "Stopped" : "Stopping the robot...";
        return s;
    }

    if (const std::string refusal = runner_->refusal(); !refusal.empty()) {
        fail(ProcessState::FailedToStart, LinkState::ProtocolMismatch,
             "The HALSim extension speaks an incompatible protocol (" + refusal + "). Rebuild or reinstall the emulator so the app and halsim_wpiemu match.");
        return s;
    }

    const launcher::LaunchStatus launch = launcher_->status();
    const bool processEnded = launch.phase == launcher::LaunchPhase::Failed || launch.phase == launcher::LaunchPhase::Exited ||
                              launch.phase == launcher::LaunchPhase::Stopped;
    if (processEnded) {
        const int code = launch.exit ? launch.exit->exitCode : -1;
        s.exitCode = code;
        s.link = LinkState::Disconnected;
        s.programStarted = false;
        s.simulating = false;
        if (seenConnection_) {
            if (launch.phase == launcher::LaunchPhase::Failed) {
                fail(ProcessState::Crashed, LinkState::Disconnected,
                     "The robot process crashed or failed (" + launch.detail + ")." + outputTail());
                s.exitCode = code;
            } else {
                s.process = ProcessState::Exited;
                s.detail = "The robot program ended";
                failed_ = true;   // terminal
                beginTeardown(false);
            }
        } else {
            fail(ProcessState::FailedToStart, LinkState::Disconnected,
                 "The robot project ended before the HALSim extension connected (" + launch.detail + ")." + outputTail());
            s.exitCode = code;
        }
        return s;
    }

    if (linked) {
        s.process = ProcessState::Running;
        s.link = rs == runtime::RunnerState::RobotStalled ? LinkState::Stalled : LinkState::Connected;
        s.detail = rs == runtime::RunnerState::RobotStalled ? "The robot stopped answering (stalled)"
                                                            : (s.programStarted ? "Connected" : "Connected, the robot program is starting...");
        return s;
    }

    if (seenConnection_) {
        s.process = ProcessState::Running;
        s.link = LinkState::Disconnected;
        s.detail = "The robot disconnected from the emulator";
        return s;
    }

    const auto waited = std::chrono::steady_clock::now() - startedAt_;
    if (waited > options_.handshakeTimeout) {
        fail(ProcessState::FailedToStart, LinkState::HandshakeTimeout,
             "The robot project is running but the HALSim extension did not connect within " +
                 std::to_string(std::chrono::duration_cast<std::chrono::seconds>(options_.handshakeTimeout).count()) +
                 " s. Check the console for errors (extension not loaded, antivirus, or a blocked named pipe).");
        return s;
    }
    s.process = ProcessState::Starting;
    s.link = LinkState::Connecting;
    s.detail = launch.phase == launcher::LaunchPhase::Building ? "Building the robot project..." : "Building and starting the robot project...";
    return s;
}

// ---- stop / teardown ------------------------------------------------------------------------------------------------------------

void RobotSession::requestStop() {
    if (stopRequested_.exchange(true)) return;
    beginTeardown(true);
}

void RobotSession::beginTeardown(bool protocolStop) {
    if (teardownStarted_.exchange(true)) return;
    if (protocolStop && runner_) {
        proto::SimControl stop;
        stop.command = proto::SimCommand::Stop;
        runner_->control(stop);
    }
    teardownThread_ = std::thread([this] { teardown(); });
}

void RobotSession::teardown() {
    // 1. Give the robot a moment to leave by itself after the protocol Stop (the extension resumes HAL time and says goodbye).
    const auto deadline = std::chrono::steady_clock::now() + options_.stopGrace;
    while (runner_ && std::chrono::steady_clock::now() < deadline && runner_->state() != runtime::RunnerState::Stopped)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // 2. The robot keeps running in real time after the extension left: end the whole process tree.
    if (launcher_) launcher_->stop(std::chrono::milliseconds(200));
    // Stop accepting and join IPC producers before disposing of their consumer / pending owners.
    if (acceptor_) acceptor_->stop();
    if (runner_) runner_->stop();
    inbox_.close();
    teardownDone_ = true;
}

void RobotSession::waitUntilFinished() {
    if (teardownThread_.joinable()) teardownThread_.join();
}

RobotSession::~RobotSession() {
    stopRequested_ = true;
    beginTeardown(runner_ != nullptr);
    waitUntilFinished();
    // Members then die in reverse order: launcher (kills any leftover tree), runner, acceptor, inbox, session.
    launcher_.reset();
    runner_.reset();
    acceptor_.reset();
}

}  // namespace wpiemu::app::host
