#pragma once

#include <QMetaType>
#include <QObject>
#include <QString>

#include <memory>
#include <optional>
#include <cstdint>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app {

enum class HostProcessState { Idle, Starting, Running, Stopping, Exited, Crashed, FailedToStart };
enum class HostLinkState { Disconnected, Connecting, Connected, HandshakeTimeout, ProtocolMismatch, Stalled };
enum class HostLogStream { Stdout, Stderr, Emulator };

// Run intent belongs to the project, not the robot-code instance: config relaunches retain it.
class SimulationStartIntent {
public:
    void projectReplaced() { ++projectGeneration_; cancel(); }
    void request() { requestedGeneration_ = projectGeneration_; }
    void cancel() { requestedGeneration_.reset(); }
    bool pending() const { return requestedGeneration_ == projectGeneration_; }
    bool takeWhenReady(bool ready) {
        if (!ready || !pending()) return false;
        cancel();
        return true;
    }

private:
    std::uint64_t projectGeneration_ = 0;
    std::optional<std::uint64_t> requestedGeneration_;
};

struct HostStatus {
    HostProcessState process = HostProcessState::Idle;
    HostLinkState link = HostLinkState::Disconnected;
    int exitCode = 0;
    QString detail;
    // The robot program finished its constructor: devices are announced and the simulation may start.
    bool programStarted = false;
    // The lock-step simulation is running (as opposed to the robot merely being alive and detecting devices).
    bool simulating = false;
    // Changes for every robot-code launch, not for a physics reset or same-project config edit.
    std::uint64_t runGeneration = 0;

    bool active() const {
        return process == HostProcessState::Starting || process == HostProcessState::Running ||
               process == HostProcessState::Stopping;
    }
    // Robot alive and connected, simulation idle: the state in which devices are detected.
    bool armed() const { return process == HostProcessState::Running && link != HostLinkState::Disconnected && !simulating; }
    bool operator==(const HostStatus&) const = default;
};

struct HostLogLine {
    qint64 timestampMs = 0;
    sdk::LogLevel level = sdk::LogLevel::Info;
    HostLogStream stream = HostLogStream::Stdout;
    QString source;
    QString text;
};

// What a viewport needs to draw the loaded project.
struct HostScene {
    std::shared_ptr<const sdk::RobotConfig> config;
    std::shared_ptr<const sdk::FieldDescription> field;   // null = generic floor
    std::vector<sdk::HudWidgetDescriptor> hud;
    std::vector<sdk::EditorPanelDescriptor> panels;
    std::vector<sdk::ZoneKindInfo> zoneKinds;             // the game module's contributeZoneKinds()
    std::vector<sdk::GameValueInfo> gameValues;           // the game module's contributeGameValues() (labels/units for sensor sources)
    sdk::AprilTagLayout tags;                             // the game module's aprilTagLayout()
    QString gameName;
};

// Everything that actually runs the simulation, as the shell sees it. Real implementation: launches
// the robot with the HALSim extension and drives the sim thread (core). DemoRuntimeHost: scripted
// stand-in used by `--demo` and tests. The editor and run view adapters share ONE host, so a device
// discovered in the editor is the same device the run view sees.
//
// Threading: UI thread only; implementations marshal their own worker threads.
class IRuntimeHost : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

    virtual HostStatus status() const = 0;
    virtual HostScene scene() const = 0;
    virtual std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const = 0;
    // Id of the selected RobotConfig::startPositions entry; empty = the robot setup's default start. The host keeps it
    // across robot-code restarts and clears it on a project switch or when the entry disappears.
    virtual QString startPosition() const = 0;

    // The shell calls this whenever the open project or its (saved/edited) model changes.
    virtual void setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config,
                            const sdk::IGameModule* game) = 0;

public slots:
    // launch(): starts the robot code once (idempotent while one is alive). The single instance detects devices
    // while idle ("armed") and is the one the simulation runs on.
    virtual void launch() = 0;
    // startSimulation() begins lock-step from t = 0 on the live robot (launching it first when needed);
    // stopSimulation() returns to idle and the robot stays alive.
    virtual void startSimulation() = 0;
    virtual void stopSimulation() = 0;
    // stop(): ends the robot process. restart(): relaunches it (robot init runs again).
    virtual void stop() = 0;
    virtual void restart() = 0;
    virtual void resetSimulation() = 0;
    // Chooses where resets place the robot. Does not reset by itself; call resetSimulation() after it.
    virtual void setStartPosition(const QString& id) = 0;
    virtual void setPaused(bool paused) = 0;
    virtual void stepTicks(int ticks) = 0;
    virtual void setTimeScale(double scale) = 0;
    virtual void setDriverStation(const wpiemu::sdk::proto::DsState& state) = 0;

signals:
    void statusChanged();
    void sceneChanged();
    void startPositionChanged();
    void snapshotPublished();
    void logLine(const wpiemu::app::HostLogLine& line);
};

}  // namespace wpiemu::app

Q_DECLARE_METATYPE(wpiemu::app::HostLogLine)
