#include "run_control.h"

#include <QCoreApplication>

namespace wpiemu::app::runview {
namespace {
QString tr(const char* s) { return QCoreApplication::translate("RunStatus", s); }
}  // namespace

RunControlState deriveControlState(const RunStatus& status, bool paused) {
    RunControlState s;
    s.paused = paused;
    const bool idleLike = status.process == RobotProcessState::Idle || status.process == RobotProcessState::Exited ||
                          status.process == RobotProcessState::Crashed || status.process == RobotProcessState::FailedToStart;
    const bool active = status.process == RobotProcessState::Starting || status.process == RobotProcessState::Running;
    const bool live = status.process == RobotProcessState::Running && status.link == LinkState::Connected;
    s.canRestart = active || idleLike;
    s.simulating = live && status.simulating;
    return s;
}

StatusPresentation presentStatus(const RunStatus& st, bool snapshotConnected) {
    StatusPresentation p;
    p.detail = st.detail;
    switch (st.process) {
        case RobotProcessState::Idle:
            p.tone = Tone::Neutral;
            p.chipText = tr("Robot not running");
            p.title = tr("The robot program is not running");
            if (p.detail.isEmpty()) p.detail = tr("The robot launches automatically when you open the Run tab. Press Restart to launch it now.");
            p.showOverlay = true;
            p.offerRestart = true;
            return p;
        case RobotProcessState::Starting:
            p.tone = Tone::Info;
            p.chipText = tr("Starting robot…");
            p.title = tr("Starting robot program…");
            if (p.detail.isEmpty()) p.detail = tr("Building and launching your project. This can take a while the first time.");
            p.showOverlay = true;
            return p;
        case RobotProcessState::Stopping:
            p.tone = Tone::Info;
            p.chipText = tr("Stopping…");
            p.title = tr("Stopping robot program…");
            p.showOverlay = true;
            return p;
        case RobotProcessState::Exited:
            p.tone = Tone::Neutral;
            p.chipText = tr("Robot exited (code %1)").arg(st.exitCode);
            p.title = tr("Robot program exited");
            if (p.detail.isEmpty()) p.detail = tr("Exit code %1. Restart the robot to run it again.").arg(st.exitCode);
            p.showOverlay = true;
            p.offerRestart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::Crashed:
            p.tone = Tone::Error;
            p.chipText = tr("Robot crashed (code %1)").arg(st.exitCode);
            p.title = tr("The robot program crashed");
            if (p.detail.isEmpty()) p.detail = tr("The process ended unexpectedly with exit code %1. Check the console for the stack trace.").arg(st.exitCode);
            p.showOverlay = true;
                        p.offerRestart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::FailedToStart:
            p.tone = Tone::Error;
            p.chipText = tr("Failed to start");
            p.title = tr("The robot could not be started");
            if (p.detail.isEmpty()) p.detail = tr("Check the toolchain settings and the build output in the console.");
            p.showOverlay = true;
                        p.offerRestart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::Running:
            break;
    }

    switch (st.link) {
        case LinkState::Connected:
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
        case LinkState::Connecting:
        case LinkState::Disconnected:
            p.tone = Tone::Info;
            p.chipText = tr("Waiting for robot…");
            p.title = tr("Waiting for the robot extension…");
            if (p.detail.isEmpty()) p.detail = tr("The robot process is running; waiting for the simulator extension to connect.");
            p.showOverlay = true;
            p.offerShowConsole = true;
            return p;
        case LinkState::HandshakeTimeout:
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
        case LinkState::ProtocolMismatch:
            p.tone = Tone::Error;
            p.chipText = tr("Protocol mismatch");
            p.offerRestart = true;
            p.title = tr("Incompatible simulator extension");
            if (p.detail.isEmpty()) p.detail = tr("The robot's extension speaks a protocol version this app does not support. Rebuild or update it.");
            p.showOverlay = true;
            p.offerShowConsole = true;
            return p;
        case LinkState::Stalled:
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
