#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/core/config/config_store.h"
#include "wpiemu/core/launcher/process.h"
#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/core/logger.h"

namespace wpiemu::core::launcher {

// Environment variables of the emulator <-> extension contract (names owned by halsim_ext/ipc; duplicated here because
// core may not include them).
namespace env_var {
inline constexpr const char* kExtension = "WPIEMU_EXT";             // consumed by the generated init script
inline constexpr const char* kEndpoint = "WPIEMU_IPC_ENDPOINT";      // authoritative (docs/reference/extension-ipc.md)
inline constexpr const char* kEndpointAlias = "WPIEMU_ENDPOINT";
inline constexpr const char* kClock = "WPIEMU_CLOCK";                // "master" (lock-step) | "realtime"
inline constexpr const char* kProgramName = "WPIEMU_PROGRAM_NAME";
inline constexpr const char* kHalExtensions = "HALSIM_EXTENSIONS";
}  // namespace env_var

enum class LaunchMode {
    GradleInitScript,  // gradlew --no-daemon -I <generated init script> simulateJava|simulateNative
    ExternalDirect,    // gradlew simulateExternal{Java,Native}{Release,Debug} (build + describe), then launch the JVM / exe ourselves
};

struct LaunchRequest {
    std::filesystem::path projectDir;
    std::string language = "java";                // "java" | "cpp"
    LaunchMode mode = LaunchMode::GradleInitScript;
    bool release = true;                          // ExternalDirect: Release vs Debug variant
    std::filesystem::path extensionLibrary;       // absolute path of halsim_wpiemu.{dll,so,dylib}
    std::string ipcEndpoint;                       // from ITransportServer::endpoint()
    bool realtimeClock = false;                   // WPIEMU_CLOCK=realtime instead of master
    std::filesystem::path scratchDir;             // where the init script is written (app data); needed for GradleInitScript
    std::filesystem::path javaHome;               // optional JDK (the WPILib one); sets JAVA_HOME and is used for the direct launch
    bool gradleOffline = false;                   // --offline
    std::vector<std::string> extraGradleArgs;
    std::map<std::string, std::string> extraEnv;  // wins over everything above
    // Where the host environment is read from for the packaged-app sanitising below and for the library path; empty =
    // the real process environment (tests inject one).
    config::EnvLookup hostEnv;
};

// ---- packaged-app environment ---------------------------------------------------------------------------------------------

// A packaged AppImage (WPIEMU_PACKAGED set) exports its own Qt/linuxdeploy environment; the robot's gradle / JDK / native
// code must not inherit it. Adds to `spec`: unsetEnv QT_PLUGIN_PATH, QML2_IMPORT_PATH, QT_QPA_PLATFORM_PLUGIN_PATH, and
// rewrites LD_LIBRARY_PATH, PATH, XDG_DATA_DIRS (effective value = spec.env, else hostEnv) without the entries that lie
// under $APPDIR (an emptied LD_LIBRARY_PATH / XDG_DATA_DIRS is unset). No-op when WPIEMU_PACKAGED is unset/empty.
void sanitizePackagedEnvironment(ProcessSpec& spec, const config::EnvLookup& hostEnv, bool windows);

// ---- planning (pure apart from the init script file; unit-tested) -----------------------------------------------------

// HALSIM_EXTENSIONS value: our library (the emulator is the only Driver Station).
std::string halExtensionsValue(const LaunchRequest& request, bool windows);
// WPIEMU_* contract variables + JAVA_HOME + extraEnv (not HALSIM_EXTENSIONS: the init script sets that inside the task).
std::map<std::string, std::string> emulatorEnvironment(const LaunchRequest& request, bool windows);

std::filesystem::path gradleWrapperPath(const std::filesystem::path& projectDir, bool windows);
std::string gradleTaskName(const LaunchRequest& request);          // simulateJava | simulateNative
std::string externalTaskName(const LaunchRequest& request);        // simulateExternalJavaRelease ...
std::filesystem::path externalDescriptorPath(const LaunchRequest& request);  // <project>/build/sim/release_java.json ...

// gradlew --no-daemon --console=plain -I <script> simulateJava|simulateNative. Writes <scratchDir>/wpiemu-init.gradle.
// Errors: NotFound (no wrapper), InvalidArgument (no extension library / endpoint / scratch dir / unknown language).
sdk::Result<ProcessSpec> planGradleLaunch(const LaunchRequest& request, bool windows = (std::filesystem::path::preferred_separator == '\\'));
// gradlew simulateExternal<Java|Native><Release|Debug> (build + describe; nothing is run).
sdk::Result<ProcessSpec> planExternalDescribe(const LaunchRequest& request, bool windows = (std::filesystem::path::preferred_separator == '\\'));
// The direct launch from the JSON Gradle wrote (Java: {libraryDir, mainClassName, environment, extensions[]} + the jar in
// build/libs; native: {launchfile, libpaths, environment, extensions[]}). `macos` adds -XstartOnFirstThread for the JVM.
// `hostEnv` is only used to find JAVA_HOME when the request has none.
sdk::Result<ProcessSpec> planExternalLaunch(const LaunchRequest& request, std::string_view descriptorJson, const sdk::IValueTextCodec& json,
                                            const config::EnvLookup& hostEnv, bool windows, bool macos);

// ---- execution ------------------------------------------------------------------------------------------------------------

enum class LaunchPhase {
    Idle,
    Building,   // ExternalDirect: the describe step runs
    Running,    // the robot process (or the Gradle run) is up
    Exited,     // finished with exit code 0
    Failed,     // a step failed (see detail / exit code), or the plan could not be completed
    Stopped,    // ended by stop()
};

struct LaunchStatus {
    LaunchPhase phase = LaunchPhase::Idle;
    std::string detail;
    std::optional<ExitInfo> exit;
    std::int64_t pid = 0;  // of the current child
};

// Launches the robot project without editing it and supervises it. launch() returns as soon as the plan is made and the
// worker is running; progress is reported through status() / the change callback (worker thread).
// The destructor stops everything (kill tree) and joins: no orphaned robot or Gradle.
class RobotLauncher {
public:
    RobotLauncher(ProcessFactory factory, const sdk::IValueTextCodec& json, sdk::ILogger& logger);
    ~RobotLauncher();
    RobotLauncher(const RobotLauncher&) = delete;
    RobotLauncher& operator=(const RobotLauncher&) = delete;

    void setOnChange(std::function<void(const LaunchStatus&)> callback);
    sdk::Status launch(const LaunchRequest& request, ILogSink& output);
    LaunchStatus status() const;
    bool waitUntilFinished(std::chrono::milliseconds timeout) const;  // true when no worker is active
    // Nonblocking cancellation, including during native startup. Repeated requests keep the earliest kill deadline.
    void requestStop(std::chrono::milliseconds grace = std::chrono::milliseconds(3000));
    // Politely asks the process to stop, waits `grace`, then kills the whole tree; joins the worker.
    void stop(std::chrono::milliseconds grace = std::chrono::milliseconds(3000));

private:
    void work(LaunchRequest request, ProcessSpec first, ILogSink* output);
    // Runs one process to completion; an error means it could not be started.
    sdk::Status runStep(const ProcessSpec& spec, ILogSink& output, LaunchPhase phase, ExitInfo& exit);
    void publish(LaunchPhase phase, std::string detail, std::optional<ExitInfo> exit = std::nullopt, std::int64_t pid = 0);

    ProcessFactory factory_;
    const sdk::IValueTextCodec& json_;
    sdk::ILogger& logger_;

    mutable std::mutex mutex_;
    mutable std::condition_variable idle_;
    LaunchStatus status_;
    std::function<void(const LaunchStatus&)> onChange_;
    bool active_ = false;
    std::atomic<bool> stopRequested_{false};
    std::chrono::steady_clock::time_point stopDeadline_{};
    std::thread worker_;
};

}  // namespace wpiemu::core::launcher
