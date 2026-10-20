#pragma once

// Narrow, Qt-based seam between the run view and whatever drives the simulation. The shell provides
// the real adapter (process launcher + sim thread + IPC); FakeRunBackend (fake_run_backend.h) is a
// complete in-process implementation for tests, demos and screenshots.
//
// Threading: every method is called on the UI thread. Signals are delivered on the UI thread (a real
// backend that emits from other threads must use queued connections; RunLogEntry is registered as a
// metatype by registerRunViewMetaTypes()).

#include <QMetaType>
#include <QObject>
#include <QString>

#include <memory>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_value.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/vision/apriltag.h"

namespace wpiemu::app::runview {

// State of the robot program process launched by the emulator.
enum class RobotProcessState {
    Idle,           // never started / cleanly reset
    Starting,       // gradle / robot executable is launching
    Running,        // process alive
    Stopping,       // stop requested, waiting for exit
    Exited,         // ended by itself or by Stop, exit code 0
    Crashed,        // ended unexpectedly (non-zero exit / signal)
    FailedToStart,  // could not be launched (toolchain, build error)
};

// State of the IPC session with the HALSim extension inside the robot process.
enum class LinkState {
    Disconnected,
    Connecting,        // process is up, waiting for Hello / handshake
    Connected,
    HandshakeTimeout,  // the extension never said Hello
    ProtocolMismatch,  // Hello refused (version)
    Stalled,           // lock-step watchdog expired (robot not answering)
};

enum class LogStream { Stdout, Stderr, Emulator };

struct RunStatus {
    RobotProcessState process = RobotProcessState::Idle;
    LinkState link = LinkState::Disconnected;
    int exitCode = 0;
    QString detail;  // human readable cause (error text, version mismatch details), may be empty
    // The robot program finished starting: devices are announced and the simulation may begin.
    bool programStarted = false;
    // Lock-step simulation running. False while the robot merely waits ("armed") and detects devices.
    bool simulating = false;

    bool operator==(const RunStatus&) const = default;
};

struct RunLogEntry {
    qint64 timestampMs = 0;  // msecs since epoch
    sdk::LogLevel level = sdk::LogLevel::Info;
    LogStream stream = LogStream::Stdout;
    QString source;          // "robot", "extension", "gradle", ...
    QString text;
};

// What the scene needs to draw and what the game plugin wants shown. All members are shared, immutable
// copies made by the backend (the IField / game module live on the sim thread).
struct RunSceneInfo {
    std::shared_ptr<const sdk::RobotConfig> config;       // chassis/mechanism primitives, wheels
    std::shared_ptr<const sdk::FieldDescription> field;   // may be null (generic floor)
    std::vector<sdk::HudWidgetDescriptor> hud;
    std::vector<sdk::EditorPanelDescriptor> panels;       // viewport overlays (zones) are read from these
    std::vector<sdk::ZoneKindInfo> zoneKinds;             // kinds of RobotConfig::zones the game module declares
    std::vector<sdk::GameValueInfo> gameValues;          // values the game module publishes (labels/units for sensor sources)
    sdk::AprilTagLayout tags;                             // the game module's aprilTagLayout(); empty = no tags drawn
    QString gameName;                                     // "REBUILT (2026)"
};

class IRunBackend : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

    virtual RunStatus status() const = 0;
    virtual RunSceneInfo sceneInfo() const = 0;
    // Latest published snapshot; may be null before the first tick. Cheap: call at UI frame rate.
    virtual std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const = 0;

public slots:
    // The robot program is launched once and stays alive (detecting devices) between simulations:
    // launchRobot() is idempotent, startSimulation() begins lock-step at t = 0 (launching first when needed),
    // stopSimulation() returns to idle with the robot still running, restartRobot() relaunches the program.
    virtual void launchRobot() = 0;
    virtual void startSimulation() = 0;
    virtual void stopSimulation() = 0;
    virtual void restartRobot() = 0;
    virtual void resetSimulation() = 0;
    virtual void setPaused(bool paused) = 0;
    virtual void stepTicks(int ticks) = 0;
    virtual void setTimeScale(double scale) = 0;
    virtual void setDriverStation(const wpiemu::sdk::proto::DsState& state) = 0;

signals:
    void statusChanged();
    void sceneInfoChanged();
    void logEntry(const wpiemu::app::runview::RunLogEntry& entry);
};

// Best-effort level for a raw robot output line (WPILib prints "Warning at ...", "ERROR  1 ...").
sdk::LogLevel guessLogLevel(const QString& line, LogStream stream);

void registerRunViewMetaTypes();

}  // namespace wpiemu::app::runview

Q_DECLARE_METATYPE(wpiemu::app::runview::RunLogEntry)
Q_DECLARE_METATYPE(wpiemu::sdk::proto::DsState)
