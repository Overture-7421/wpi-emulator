#include "wpiemu/core/launcher/robot_launcher.h"

#include <algorithm>

#include "wpiemu/core/launcher/init_script.h"

namespace wpiemu::core::launcher {

using namespace wpiemu::sdk;
namespace fs = std::filesystem;

namespace {

bool isJava(const LaunchRequest& r) { return r.language == "java"; }

Status validateCommon(const LaunchRequest& r) {
    if (r.language != "java" && r.language != "cpp")
        return makeError(ErrorCode::InvalidArgument, "unknown robot language '" + r.language + "' (java|cpp)");
    if (r.projectDir.empty()) return makeError(ErrorCode::InvalidArgument, "no project directory");
    if (r.extensionLibrary.empty()) return makeError(ErrorCode::InvalidArgument, "no HALSim extension library given");
    if (r.ipcEndpoint.empty()) return makeError(ErrorCode::InvalidArgument, "no IPC endpoint given");
    return okStatus();
}

std::string join(const std::vector<std::string>& parts, char sep) {
    std::string out;
    for (const std::string& p : parts) {
        if (!out.empty()) out.push_back(sep);
        out += p;
    }
    return out;
}

const char* libraryPathVariable(bool windows, bool macos) { return windows ? "PATH" : macos ? "DYLD_LIBRARY_PATH" : "LD_LIBRARY_PATH"; }

const config::EnvLookup& effectiveHostEnv(const LaunchRequest& r) {
    static const config::EnvLookup system = config::systemEnv();
    return r.hostEnv ? r.hostEnv : system;
}

std::vector<std::string> gradleCommonArgs(const LaunchRequest& r) {
    std::vector<std::string> args{"--no-daemon", "--console=plain"};  // no daemon: the process tree stays ours (ADR 0001)
    if (r.gradleOffline) args.emplace_back("--offline");
    return args;
}

// gradlew.bat does `set CLASSPATH=%APP_HOME%\...` without quotes, so cmd.exe metacharacters in the project path break it
// whatever we do on the command line (quoting only protects the arguments we pass).
bool breaksGradlewBat(const fs::path& projectDir) { return projectDir.string().find_first_of("&|<>^") != std::string::npos; }

// Sets program + leading arguments of a Gradle wrapper invocation: gradlew(.bat), or on Windows for project paths that
// gradlew.bat cannot handle the equivalent `java -cp gradle-wrapper.jar org.gradle.wrapper.GradleWrapperMain`.
Status setWrapperInvocation(ProcessSpec& spec, const LaunchRequest& r, bool windows) {
    std::error_code ec;
    const fs::path gradlew = gradleWrapperPath(r.projectDir, windows);
    if (!fs::exists(gradlew, ec)) return makeError(ErrorCode::NotFound, "Gradle wrapper not found: " + gradlew.string());
    if (!windows || !breaksGradlewBat(r.projectDir)) {
        spec.program = gradlew.string();
        return okStatus();
    }
    const fs::path jar = r.projectDir / "gradle" / "wrapper" / "gradle-wrapper.jar";
    if (!fs::exists(jar, ec)) return makeError(ErrorCode::NotFound, "Gradle wrapper jar not found: " + jar.string());
    fs::path javaHome = r.javaHome;
    if (javaHome.empty())
        if (const auto home = effectiveHostEnv(r)("JAVA_HOME")) javaHome = *home;
    spec.program = javaHome.empty() ? "java" : (javaHome / "bin" / "java.exe").string();
    spec.args = {"-Xmx64m", "-Xms64m", "-Dorg.gradle.appname=gradlew", "-classpath", jar.string(), "org.gradle.wrapper.GradleWrapperMain"};
    return okStatus();
}

}  // namespace

void sanitizePackagedEnvironment(ProcessSpec& spec, const config::EnvLookup& hostEnv, bool windows) {
    if (!hostEnv) return;
    const auto packaged = hostEnv("WPIEMU_PACKAGED");
    if (!packaged || packaged->empty()) return;
    for (const char* var : {"QT_PLUGIN_PATH", "QML2_IMPORT_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"}) spec.unsetEnv.emplace_back(var);
    std::string appDir = hostEnv("APPDIR").value_or("");
    while (appDir.size() > 1 && (appDir.back() == '/' || appDir.back() == '\\')) appDir.pop_back();
    if (appDir.empty()) return;
    const char sep = windows ? ';' : ':';
    const auto underAppDir = [&](const std::string& entry) {
        return entry.compare(0, appDir.size(), appDir) == 0 && (entry.size() == appDir.size() || entry[appDir.size()] == '/' || entry[appDir.size()] == '\\');
    };
    for (const char* var : {"LD_LIBRARY_PATH", "PATH", "XDG_DATA_DIRS"}) {
        std::string value;
        if (const auto it = spec.env.find(var); it != spec.env.end()) value = it->second;
        else if (const auto host = hostEnv(var)) value = *host;
        else continue;
        std::vector<std::string> kept;
        bool removed = false;
        for (std::size_t start = 0; start <= value.size();) {
            std::size_t end = value.find(sep, start);
            if (end == std::string::npos) end = value.size();
            std::string entry = value.substr(start, end - start);
            start = end + 1;
            if (entry.empty()) continue;
            if (underAppDir(entry)) removed = true;
            else kept.push_back(std::move(entry));
        }
        if (!removed) continue;
        if (kept.empty() && std::string_view(var) != "PATH") {
            spec.env.erase(var);
            spec.unsetEnv.emplace_back(var);
        } else {
            spec.env[var] = join(kept, sep);
        }
    }
}

std::string halExtensionsValue(const LaunchRequest& r, bool /*windows*/) {
    return r.extensionLibrary.string();
}

std::map<std::string, std::string> emulatorEnvironment(const LaunchRequest& r, bool /*windows*/) {
    std::map<std::string, std::string> env;
    env[env_var::kExtension] = r.extensionLibrary.string();
    env[env_var::kEndpoint] = r.ipcEndpoint;
    env[env_var::kEndpointAlias] = r.ipcEndpoint;
    env[env_var::kClock] = r.realtimeClock ? "realtime" : "master";
    const std::string name = r.projectDir.filename().string();
    if (!name.empty()) env[env_var::kProgramName] = name;
    if (!r.javaHome.empty()) env["JAVA_HOME"] = r.javaHome.string();
    for (const auto& [k, v] : r.extraEnv) env[k] = v;
    return env;
}

fs::path gradleWrapperPath(const fs::path& projectDir, bool windows) { return projectDir / (windows ? "gradlew.bat" : "gradlew"); }

std::string gradleTaskName(const LaunchRequest& r) { return isJava(r) ? "simulateJava" : "simulateNative"; }

std::string externalTaskName(const LaunchRequest& r) {
    return std::string("simulateExternal") + (isJava(r) ? "Java" : "Native") + (r.release ? "Release" : "Debug");
}

fs::path externalDescriptorPath(const LaunchRequest& r) {
    return r.projectDir / "build" / "sim" / (std::string(r.release ? "release" : "debug") + "_" + (isJava(r) ? "java" : "native") + ".json");
}

Result<ProcessSpec> planGradleLaunch(const LaunchRequest& r, bool windows) {
    if (Status s = validateCommon(r); !s) return s.error();
    if (r.scratchDir.empty()) return makeError(ErrorCode::InvalidArgument, "no scratch directory for the init script");
    ProcessSpec spec;
    if (Status s = setWrapperInvocation(spec, r, windows); !s) return s.error();
    std::error_code ec;
    if (!fs::exists(r.extensionLibrary, ec)) return makeError(ErrorCode::NotFound, "HALSim extension library not found: " + r.extensionLibrary.string());

    const fs::path script = r.scratchDir / "wpiemu-init.gradle";
    if (Status s = config::atomicWriteFile(script, generateInitScript()); !s)
        return makeError(s.error().code, "cannot write the init script: " + s.error().message);

    const std::vector<std::string> common = gradleCommonArgs(r);
    spec.args.insert(spec.args.end(), common.begin(), common.end());
    spec.args.insert(spec.args.end(), {"-I", script.string()});
    spec.args.insert(spec.args.end(), r.extraGradleArgs.begin(), r.extraGradleArgs.end());
    spec.args.push_back(gradleTaskName(r));
    spec.workingDir = r.projectDir;
    spec.env = emulatorEnvironment(r, windows);
    spec.label = "gradlew " + gradleTaskName(r);
    sanitizePackagedEnvironment(spec, effectiveHostEnv(r), windows);
    return spec;
}

Result<ProcessSpec> planExternalDescribe(const LaunchRequest& r, bool windows) {
    if (Status s = validateCommon(r); !s) return s.error();
    ProcessSpec spec;
    if (Status s = setWrapperInvocation(spec, r, windows); !s) return s.error();
    const std::vector<std::string> common = gradleCommonArgs(r);
    spec.args.insert(spec.args.end(), common.begin(), common.end());
    spec.args.insert(spec.args.end(), r.extraGradleArgs.begin(), r.extraGradleArgs.end());
    spec.args.push_back(externalTaskName(r));
    spec.workingDir = r.projectDir;
    if (!r.javaHome.empty()) spec.env["JAVA_HOME"] = r.javaHome.string();
    spec.label = "gradlew " + externalTaskName(r);
    sanitizePackagedEnvironment(spec, effectiveHostEnv(r), windows);
    return spec;
}

namespace {

std::optional<fs::path> pickJar(const fs::path& libsDir) {
    std::error_code ec;
    std::optional<fs::path> best;
    std::uintmax_t bestSize = 0;
    for (fs::directory_iterator it(libsDir, ec), end; !ec && it != end; it.increment(ec)) {
        const fs::path& p = it->path();
        if (p.extension() != ".jar" || !it->is_regular_file(ec)) continue;
        const std::string name = p.filename().string();
        if (name.find("-sources") != std::string::npos || name.find("-javadoc") != std::string::npos || name.find("-plain") != std::string::npos) continue;
        const std::uintmax_t size = it->file_size(ec);
        if (!best || size > bestSize || (size == bestSize && p < *best)) {
            best = p;
            bestSize = size;
        }
    }
    return best;
}

std::string stringField(const Value& v, std::string_view key) {
    const Value* f = v.find(key);
    return f != nullptr && f->isString() ? f->asString() : std::string();
}

}  // namespace

Result<ProcessSpec> planExternalLaunch(const LaunchRequest& r, std::string_view descriptorJson, const IValueTextCodec& json,
                                       const config::EnvLookup& hostEnv, bool windows, bool macos) {
    if (Status s = validateCommon(r); !s) return s.error();
    const Result<Value> parsed = json.parse(descriptorJson);
    if (!parsed) return makeError(ErrorCode::ParseError, "simulateExternal descriptor: " + parsed.error().message);
    const Value* selected = &parsed.value();
    if (selected->isArray()) {
        // Real GradleRIO output: a list with one entry per simulation target; prefer the entry of the requested language.
        selected = nullptr;
        const std::string wanted = isJava(r) ? "java" : "native";
        for (const Value& entry : parsed.value().array()) {
            if (!entry.isObject()) continue;
            if (selected == nullptr) selected = &entry;
            if (stringField(entry, "type") == wanted) {
                selected = &entry;
                break;
            }
        }
        if (selected == nullptr) return makeError(ErrorCode::ParseError, "simulateExternal descriptor lists no simulation target");
    }
    const Value& d = *selected;
    if (!d.isObject()) return makeError(ErrorCode::ParseError, "simulateExternal descriptor is not a JSON object");

    ProcessSpec spec;
    spec.workingDir = r.projectDir;
    if (const Value* env = d.find("environment"); env != nullptr && env->isObject())
        for (std::size_t i = 0; i < env->object().size(); ++i)
            if (env->object().valueAt(i).isString()) spec.env[env->object().keyAt(i)] = env->object().valueAt(i).asString();

    std::vector<std::string> libDirs;
    if (isJava(r)) {
        const std::string libraryDir = stringField(d, "libraryDir");
        if (libraryDir.empty()) return makeError(ErrorCode::InvalidArgument, "descriptor has no libraryDir");
        const auto jar = pickJar(r.projectDir / "build" / "libs");
        if (!jar) return makeError(ErrorCode::NotFound, "no robot jar in " + (r.projectDir / "build" / "libs").string() + " (did the build run?)");
        fs::path javaHome = r.javaHome;
        if (javaHome.empty() && hostEnv)
            if (const auto home = hostEnv("JAVA_HOME")) javaHome = *home;
        spec.program = javaHome.empty() ? "java" : (javaHome / "bin" / (windows ? "java.exe" : "java")).string();
        if (macos) spec.args.emplace_back("-XstartOnFirstThread");
        spec.args.push_back("-Djava.library.path=" + libraryDir);
        spec.args.insert(spec.args.end(), {"-jar", jar->string()});
        libDirs.push_back(libraryDir);
        spec.label = "java -jar " + jar->filename().string();
    } else {
        const std::string launchfile = stringField(d, "launchfile");
        if (launchfile.empty()) return makeError(ErrorCode::InvalidArgument, "descriptor has no launchfile");
        fs::path exe = fs::path(launchfile);
        if (exe.is_relative()) exe = r.projectDir / exe;
        spec.program = exe.string();
        spec.label = exe.filename().string();
        if (const Value* paths = d.find("libpaths"); paths != nullptr && paths->isArray())
            for (const Value& p : paths->array())
                if (p.isString()) libDirs.push_back(p.asString());
    }

    // contract variables win over whatever the descriptor carries (in particular its HALSIM_EXTENSIONS for the stock Sim GUI)
    for (const auto& [k, v] : emulatorEnvironment(r, windows)) spec.env[k] = v;
    spec.env[env_var::kHalExtensions] = halExtensionsValue(r, windows);

    if (!libDirs.empty()) {
        const char* var = libraryPathVariable(windows, macos);
        std::vector<std::string> parts = libDirs;
        std::string existing;
        if (const auto it = spec.env.find(var); it != spec.env.end()) existing = it->second;
        else if (hostEnv)
            if (const auto host = hostEnv(var)) existing = *host;
        if (!existing.empty()) parts.push_back(existing);
        spec.env[var] = join(parts, windows ? ';' : ':');
    }
    sanitizePackagedEnvironment(spec, hostEnv ? hostEnv : effectiveHostEnv(r), windows);
    for (const auto& [k, v] : r.extraEnv) spec.env[k] = v;
    return spec;
}

// ---- RobotLauncher ----------------------------------------------------------------------------------------------------

RobotLauncher::RobotLauncher(ProcessFactory factory, const IValueTextCodec& json, ILogger& logger)
    : factory_(std::move(factory)), json_(json), logger_(logger) {}

RobotLauncher::~RobotLauncher() { stop(std::chrono::milliseconds(0)); }

void RobotLauncher::setOnChange(std::function<void(const LaunchStatus&)> callback) {
    std::lock_guard lock(mutex_);
    onChange_ = std::move(callback);
}

LaunchStatus RobotLauncher::status() const {
    std::lock_guard lock(mutex_);
    return status_;
}

bool RobotLauncher::waitUntilFinished(std::chrono::milliseconds timeout) const {
    std::unique_lock lock(mutex_);
    return idle_.wait_for(lock, timeout, [this] { return !active_; });
}

void RobotLauncher::publish(LaunchPhase phase, std::string detail, std::optional<ExitInfo> exit, std::int64_t pid) {
    LaunchStatus snapshot;
    std::function<void(const LaunchStatus&)> callback;
    {
        std::lock_guard lock(mutex_);
        status_.phase = phase;
        status_.detail = std::move(detail);
        status_.exit = exit;
        status_.pid = pid;
        snapshot = status_;
        callback = onChange_;
    }
    if (callback) callback(snapshot);
}

Status RobotLauncher::launch(const LaunchRequest& request, ILogSink& output) {
    {
        std::lock_guard lock(mutex_);
        if (active_) return makeError(ErrorCode::InvalidState, "a robot launch is already active");
    }
    if (worker_.joinable()) worker_.join();  // a finished previous run

    Result<ProcessSpec> first = request.mode == LaunchMode::GradleInitScript ? planGradleLaunch(request) : planExternalDescribe(request);
    if (!first) return first.error();

    {
        std::lock_guard lock(mutex_);
        stopRequested_ = false;
        stopDeadline_ = {};
        active_ = true;
        status_ = LaunchStatus{};
    }
    worker_ = std::thread([this, request, spec = std::move(first).value(), out = &output]() mutable { work(std::move(request), std::move(spec), out); });
    return okStatus();
}

Status RobotLauncher::runStep(const ProcessSpec& spec, ILogSink& output, LaunchPhase phase, ExitInfo& exit) {
    std::unique_ptr<IRobotProcess> process = factory_();
    if (!process) return makeError(ErrorCode::Internal, "process factory returned null");
    if (stopRequested_) {
        return makeError(ErrorCode::Cancelled, "stopped before the process started");
    }
    logger_.info("launcher", "starting " + spec.label);
    if (Status s = process->start(spec, output); !s) return s;
    publish(phase, spec.label, std::nullopt, process->pid());
    // Only this worker touches the process. Cancellation remains latched across native startup,
    // so stop() cannot consume its one kill attempt before a PID / job has been published.
    bool stopSent = false;
    bool killSent = false;
    for (;;) {
        if (stopRequested_ && !stopSent) {
            process->requestStop();
            stopSent = true;
        }
        bool killDue = false;
        if (stopRequested_ && !killSent) {
            std::lock_guard lock(mutex_);
            killDue = std::chrono::steady_clock::now() >= stopDeadline_;
        }
        if (killDue) {
            process->kill();
            killSent = true;
        }
        if (process->waitForExit(std::chrono::milliseconds(100))) break;
    }
    exit = process->exitInfo().value_or(ExitInfo{});
    process.reset();  // joins the readers: the sink is not used after this point
    return okStatus();
}

void RobotLauncher::work(LaunchRequest request, ProcessSpec first, ILogSink* output) {
    auto finish = [&](LaunchPhase phase, std::string detail, std::optional<ExitInfo> exit) {
        publish(phase, std::move(detail), exit, 0);
        {
            std::lock_guard lock(mutex_);
            active_ = false;
        }
        idle_.notify_all();
    };
    auto outcome = [&](const Status& started, const ExitInfo& exit, const std::string& what) {
        if (stopRequested_) return finish(LaunchPhase::Stopped, what + " stopped", exit);
        if (!started) return finish(LaunchPhase::Failed, what + ": " + started.error().message, std::nullopt);
        if (exit.exitCode == 0) return finish(LaunchPhase::Exited, what + " finished", exit);
        finish(LaunchPhase::Failed, what + " exited with code " + std::to_string(exit.exitCode), exit);
    };

    ExitInfo exit;
    if (request.mode == LaunchMode::GradleInitScript) {
        const Status s = runStep(first, *output, LaunchPhase::Running, exit);
        return outcome(s, exit, first.label);
    }

    const Status describe = runStep(first, *output, LaunchPhase::Building, exit);
    if (stopRequested_ || !describe || exit.exitCode != 0) return outcome(describe, exit, first.label);

    const fs::path descriptor = externalDescriptorPath(request);
    const Result<std::string> text = config::readTextFile(descriptor);
    if (!text) return finish(LaunchPhase::Failed, "Gradle did not produce " + descriptor.string(), std::nullopt);
#if defined(__APPLE__)
    constexpr bool kMacOs = true;
#else
    constexpr bool kMacOs = false;
#endif
    constexpr bool kWindows = fs::path::preferred_separator == '\\';
    const Result<ProcessSpec> direct = planExternalLaunch(request, text.value(), json_, config::systemEnv(), kWindows, kMacOs);
    if (!direct) return finish(LaunchPhase::Failed, "cannot plan the direct launch: " + direct.error().message, std::nullopt);
    const Status run = runStep(direct.value(), *output, LaunchPhase::Running, exit);
    outcome(run, exit, direct.value().label);
}

void RobotLauncher::requestStop(std::chrono::milliseconds grace) {
    const auto deadline = std::chrono::steady_clock::now() + grace;
    std::lock_guard lock(mutex_);
    if (!stopRequested_ || deadline < stopDeadline_) stopDeadline_ = deadline;
    stopRequested_ = true;
}

void RobotLauncher::stop(std::chrono::milliseconds grace) {
    requestStop(grace);
    if (worker_.joinable()) worker_.join();
}

}  // namespace wpiemu::core::launcher
