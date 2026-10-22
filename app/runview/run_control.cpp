#include "run_control.h"

#include "ui/qt_text.h"

namespace wpiemu::app::runview {
namespace {
constexpr ui::Tr tr{"RunStatus"};

// Words a failure the launcher recognised (HostStatus::problem) in plain language and offers the fix that exists for it.
// Unknown problems keep the raw detail.
void applyProblem(StatusPresentation& p, const HostStatus& st) {
    const QString& id = st.problem;
    if (id.isEmpty()) return;
    const auto field = [&](const char* key) { return st.problemFields.value(QString::fromLatin1(key)); };
    const auto fixSetup = [&] {
        p.action = ProblemAction::FixSetup;
        p.actionText = tr("Fix in setup");
    };
    if (id == QLatin1String("stale_vc_runtime")) {
        p.title = tr("This Java is too old for WPILib");
        p.detail = tr("The Java that started the robot loads Visual C++ runtime %1, but WPILib needs %2 or newer. Installing a newer "
                      "Visual Studio does not change that: the runtime comes from the Java's own folder (%3). "
                      "Let WPI Emulator set up its own Java, or use the Java that comes with WPILib.")
                       .arg(field("found"), field("expected"), field("runtimePath"));
        fixSetup();
    } else if (id == QLatin1String("java_unsupported_by_gradle")) {
        p.title = tr("This Java does not work with the project's Gradle");
        p.detail = tr("Gradle could not run on the selected Java, usually because it is too new. Pick Java 17 (WPILib's) in setup.");
        fixSetup();
    } else if (id == QLatin1String("java_home_invalid") || id == QLatin1String("no_usable_java")) {
        p.title = id == QLatin1String("no_usable_java") ? tr("Java is needed to run the robot") : tr("The Java folder is not valid");
        if (id == QLatin1String("java_home_invalid")) p.detail = tr("JAVA_HOME or the project's Gradle settings point to a folder with no Java in it.");
        fixSetup();
    } else if (id == QLatin1String("java_setting_unusable")) {
        p.title = tr("The Java folder chosen in Settings cannot run this project");
        p.action = ProblemAction::OpenSettings;
        p.actionText = tr("Open Settings");
    } else if (id == QLatin1String("download_failed")) {
        p.title = tr("Gradle could not download what it needs");
        p.detail = tr("The first run downloads WPILib and the project's libraries (about 1 GB). Check the internet connection, a proxy or "
                      "a firewall, then press Restart robot.");
    } else if (id == QLatin1String("gradle_wrapper_broken")) {
        p.title = tr("The project's Gradle wrapper is damaged");
        p.detail = tr("gradle/wrapper/gradle-wrapper.jar is missing or broken. Get a fresh copy of the project, or open it once in VS Code.");
    } else if (id == QLatin1String("no_native_toolchain")) {
        p.title = tr("C++ build tools are missing");
        p.detail = tr("This C++ project needs a C++ compiler. Setup can show you how to install one.");
        fixSetup();
    }
}
}  // namespace

RunControlState deriveControlState(const HostStatus& status, bool paused) {
    RunControlState s;
    s.paused = paused;
    s.canRestart = status.process != sdk::ProcessState::Stopping;   // every state but a stop in progress
    s.simulating = status.live() && status.simulating;
    return s;
}

StatusPresentation presentStatus(const HostStatus& st, bool snapshotConnected) {
    StatusPresentation p;
    p.detail = st.detail;
    switch (st.process) {
        case sdk::ProcessState::Idle:
            p.tone = Tone::Neutral;
            p.chipText = tr("Robot not running");
            p.title = tr("The robot program is not running");
            if (p.detail.isEmpty()) p.detail = tr("The robot launches automatically when you open the Run tab. Press Restart to launch it now.");
            p.showOverlay = true;
            p.offerRestart = true;
            return p;
        case sdk::ProcessState::Starting:
            p.tone = Tone::Info;
            p.chipText = tr("Starting robot…");
            p.title = tr("Starting robot program…");
            if (p.detail.isEmpty()) p.detail = tr("Building and launching your project. This can take a while the first time.");
            p.showOverlay = true;
            return p;
        case sdk::ProcessState::Stopping:
            p.tone = Tone::Info;
            p.chipText = tr("Stopping…");
            p.title = tr("Stopping robot program…");
            p.showOverlay = true;
            return p;
        case sdk::ProcessState::Exited:
            p.tone = Tone::Neutral;
            p.chipText = tr("Robot exited (code %1)").arg(st.exitCode);
            p.title = tr("Robot program exited");
            if (p.detail.isEmpty()) p.detail = tr("Exit code %1. Restart the robot to run it again.").arg(st.exitCode);
            p.showOverlay = true;
            p.offerRestart = true;
            p.offerShowConsole = true;
            return p;
        case sdk::ProcessState::Crashed:
            p.tone = Tone::Error;
            p.chipText = tr("Robot crashed (code %1)").arg(st.exitCode);
            p.title = tr("The robot program crashed");
            if (p.detail.isEmpty()) p.detail = tr("The process ended unexpectedly with exit code %1. Check the console for the stack trace.").arg(st.exitCode);
            p.showOverlay = true;
            p.offerRestart = true;
            p.offerShowConsole = true;
            applyProblem(p, st);
            return p;
        case sdk::ProcessState::FailedToStart:
            p.tone = Tone::Error;
            p.chipText = tr("Failed to start");
            p.title = tr("The robot could not be started");
            if (p.detail.isEmpty()) p.detail = tr("Check the toolchain settings and the build output in the console.");
            p.showOverlay = true;
            p.offerRestart = true;
            p.offerShowConsole = true;
            applyProblem(p, st);
            return p;
        case sdk::ProcessState::Running:
            break;
    }

    switch (st.link) {
        case sdk::LinkState::Connected:
            if (!st.programStarted) {
                p.tone = Tone::Info;
                p.chipText = tr("Starting robot program…");
                p.title = tr("Starting robot program…");
                if (p.detail.isEmpty()) p.detail = tr("Waiting for the robot to finish initializing and announce its devices.");
                p.showOverlay = true;
                p.offerShowConsole = true;
                return p;
            }
            if (!st.simulating) {
                p.tone = Tone::Success;
                p.chipText = tr("Robot ready");
                return p;
            }
            p.tone = Tone::Success;
            p.chipText = snapshotConnected ? tr("Simulating") : tr("Simulating · waiting for data");
            return p;
        case sdk::LinkState::Connecting:
        case sdk::LinkState::Disconnected:
            p.tone = Tone::Info;
            p.chipText = tr("Waiting for robot…");
            p.title = tr("Waiting for the robot extension…");
            if (p.detail.isEmpty()) p.detail = tr("The robot process is running; waiting for the simulator extension to connect.");
            p.showOverlay = true;
            p.offerShowConsole = true;
            return p;
        case sdk::LinkState::HandshakeTimeout:
            p.tone = Tone::Error;
            p.chipText = tr("Handshake timed out");
            p.offerRestart = true;
            p.title = tr("The simulator extension did not connect");
            if (p.detail.isEmpty())
                p.detail = tr("The robot started but never said hello. Make sure simulation is enabled in the project "
                              "and that the extension library could be loaded (see the console).");
            p.showOverlay = true;
            p.offerShowConsole = true;
            return p;
        case sdk::LinkState::ProtocolMismatch:
            p.tone = Tone::Error;
            p.chipText = tr("Protocol mismatch");
            p.offerRestart = true;
            p.title = tr("Incompatible simulator extension");
            if (p.detail.isEmpty()) p.detail = tr("The robot's extension speaks a protocol version this app does not support. Rebuild or update it.");
            p.showOverlay = true;
            p.offerShowConsole = true;
            return p;
        case sdk::LinkState::Stalled:
            p.tone = Tone::Warning;
            p.chipText = tr("Robot not responding");
            p.offerRestart = true;
            p.title = tr("The robot stopped responding");
            if (p.detail.isEmpty()) p.detail = tr("The lock-step watchdog expired. The robot may be blocked in your code; Restart to recover.");
            p.showOverlay = false;
            p.offerShowConsole = true;
            return p;
    }
    return p;
}

}  // namespace wpiemu::app::runview
