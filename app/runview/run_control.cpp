#include "run_control.h"

#include "ui/qt_text.h"

namespace wpiemu::app::runview {
namespace {
constexpr ui::Tr tr{"RunStatus"};
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
            return p;
        case sdk::ProcessState::FailedToStart:
            p.tone = Tone::Error;
            p.chipText = tr("Failed to start");
            p.title = tr("The robot could not be started");
            if (p.detail.isEmpty()) p.detail = tr("Check the toolchain settings and the build output in the console.");
            p.showOverlay = true;
                        p.offerRestart = true;
            p.offerShowConsole = true;
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
