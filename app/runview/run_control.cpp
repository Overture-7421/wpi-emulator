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
    s.canStart = idleLike;
    s.canStop = active;
    s.canRestart = active || idleLike;
    // Reset restarts the simulation (physics/game state) of a live session; it needs a connected robot.
    s.canReset = status.process == RobotProcessState::Running && status.link == LinkState::Connected;
    s.canPause = s.canReset;
    s.canStep = s.canReset && paused;
    return s;
}

StatusPresentation presentStatus(const RunStatus& st, bool snapshotConnected) {
    StatusPresentation p;
    p.detail = st.detail;
    switch (st.process) {
        case RobotProcessState::Idle:
            p.tone = Tone::Neutral;
            p.chipText = tr("Robot stopped");
            p.title = tr("Ready to run your robot code");
            if (p.detail.isEmpty()) p.detail = tr("Press Start to launch the robot program in simulation.");
            p.showOverlay = true;
            p.offerStart = true;
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
            if (p.detail.isEmpty()) p.detail = tr("Exit code %1. Press Start to run it again.").arg(st.exitCode);
            p.showOverlay = true;
            p.offerStart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::Crashed:
            p.tone = Tone::Error;
            p.chipText = tr("Robot crashed (code %1)").arg(st.exitCode);
            p.title = tr("The robot program crashed");
            if (p.detail.isEmpty()) p.detail = tr("The process ended unexpectedly with exit code %1. Check the console for the stack trace.").arg(st.exitCode);
            p.showOverlay = true;
            p.offerStart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::FailedToStart:
            p.tone = Tone::Error;
            p.chipText = tr("Failed to start");
            p.title = tr("The robot could not be started");
            if (p.detail.isEmpty()) p.detail = tr("Check the toolchain settings and the build output in the console.");
            p.showOverlay = true;
            p.offerStart = true;
            p.offerShowConsole = true;
            return p;
        case RobotProcessState::Running:
            break;
    }

    switch (st.link) {
        case LinkState::Connected:
            p.tone = Tone::Success;
            p.chipText = snapshotConnected ? tr("Connected") : tr("Connected · waiting for data");
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
            if (p.detail.isEmpty()) p.detail = tr("The lock-step watchdog expired. The robot may be blocked in your code; Stop and Restart to recover.");
            p.showOverlay = false;
            p.offerShowConsole = true;
            return p;
    }
    return p;
}

QString realDsText(RealDsState state) {
    switch (state) {
        case RealDsState::NotConfigured: return tr("Real DS: off");
        case RealDsState::Disconnected: return tr("Real DS: waiting");
        case RealDsState::Connected: return tr("Real DS: connected");
    }
    return {};
}

Tone realDsTone(RealDsState state) {
    switch (state) {
        case RealDsState::NotConfigured: return Tone::Neutral;
        case RealDsState::Disconnected: return Tone::Warning;
        case RealDsState::Connected: return Tone::Success;
    }
    return Tone::Neutral;
}

}  // namespace wpiemu::app::runview
