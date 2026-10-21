#pragma once

// The seam between the UI (run view, editor adapter, shell) and whatever runs the simulation.
// Links only the SDK and Qt, so the run view can consume it without core: the real host and the scripted
// stand-ins live elsewhere and implement these interfaces.

#include <QDateTime>
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
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/telemetry/session_status.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app {

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
    sdk::ProcessState process = sdk::ProcessState::Idle;
    sdk::LinkState link = sdk::LinkState::Disconnected;
    int exitCode = 0;
    QString detail;
    // The robot program finished its constructor: devices are announced and the simulation may start.
    bool programStarted = false;
    // The lock-step simulation is running (as opposed to the robot merely being alive and detecting devices).
    bool simulating = false;
    // Changes for every robot-code launch, not for a physics reset or same-project config edit.
    std::uint64_t runGeneration = 0;

    bool active() const { return sdk::isActive(process); }
    // Robot alive and connected.
    bool live() const { return sdk::isLive(process, link); }
    // Robot alive and connected, simulation idle: the state in which devices are detected.
    bool armed() const { return sdk::isArmed(process, link, simulating); }
    bool operator==(const HostStatus&) const = default;
};

struct HostLogLine {
    qint64 timestampMs = 0;
    sdk::LogLevel level = sdk::LogLevel::Info;
    sdk::LogStream stream = sdk::LogStream::Stdout;
    QString source;
    QString text;
};

// The one place the session's std::string vocabulary becomes Qt presentation data.
inline HostStatus toHostStatus(const sdk::SessionStatus& s) {
    HostStatus out;
    out.process = s.process;
    out.link = s.link;
    out.exitCode = s.exitCode;
    out.detail = QString::fromStdString(s.detail);
    out.programStarted = s.programStarted;
    out.simulating = s.simulating;
    return out;
}

inline HostLogLine toHostLogLine(const sdk::SessionLogEntry& e) {
    HostLogLine out;
    out.timestampMs = e.timestampMs;
    out.level = e.level;
    out.stream = e.stream;
    out.source = QString::fromStdString(e.source);
    out.text = QString::fromStdString(e.text);
    return out;
}

// A line the emulator itself writes to the console (as opposed to the robot's output).
inline HostLogLine emulatorLogLine(sdk::LogLevel level, const QString& source, const QString& text) {
    HostLogLine out;
    out.timestampMs = QDateTime::currentMSecsSinceEpoch();
    out.level = level;
    out.stream = sdk::LogStream::Emulator;
    out.source = source;
    out.text = text;
    return out;
}

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
    QString projectPath;                                  // empty = no project; a different path means a project switch
    // Where the emulated Driver Station sits for this project (the wizard's choice); the run view applies it once per
    // project switch and the user can move it from there.
    sdk::proto::AllianceStation defaultStation = sdk::proto::AllianceStation::Red1;
};

// What the shell tells a host about the open project.
struct HostProject {
    QString path;
    std::shared_ptr<const sdk::RobotConfig> config;
    const sdk::IGameModule* game = nullptr;
    sdk::proto::AllianceStation defaultStation = sdk::proto::AllianceStation::Red1;
};

// Populates a HostScene from the open project and its selected game module.
inline HostScene makeHostScene(const HostProject& project) {
    HostScene scene;
    scene.config = project.config;
    scene.projectPath = project.path;
    scene.defaultStation = project.defaultStation;
    if (const sdk::IGameModule* game = project.game) {
        scene.hud = game->contributeHud();
        scene.panels = game->contributeEditorPanels();
        scene.zoneKinds = game->contributeZoneKinds();
        scene.gameValues = game->contributeGameValues();
        scene.tags = game->aprilTagLayout();
        scene.gameName = QString::fromStdString(game->metadata().displayName);
    }
    return scene;
}

// Everything that actually runs the simulation, as the UI sees it. Real implementation: launches the robot with the
// HALSim extension and drives the sim thread (core). DemoRuntimeHost: scripted stand-in used by `--demo` and tests.
// The editor and the run view share ONE host, so a device discovered in the editor is the same device the run view
// sees. Pages consume this interface only; selecting the project is IProjectBinding.
//
// Threading: UI thread only; implementations marshal their own worker threads.
class IRuntimeHost : public QObject {
    Q_OBJECT
public:
    // Registers the metatypes of the queued signals, so every implementation is covered.
    explicit IRuntimeHost(QObject* parent = nullptr);

    virtual HostStatus status() const = 0;
    virtual HostScene scene() const = 0;
    // Latest published snapshot; may be null before the first tick. Cheap: call at UI frame rate.
    virtual std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const = 0;
    // Id of the selected RobotConfig::startPositions entry; empty = the robot setup's default start. The host keeps it
    // across robot-code restarts and clears it on a project switch or when the entry disappears.
    virtual QString startPosition() const = 0;

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

// Selecting the open project is the shell's job, not the pages': AppServices is the only caller.
class IProjectBinding {
public:
    virtual ~IProjectBinding() = default;
    // Called whenever the open project or its (saved/edited) model changes.
    virtual void setProject(const HostProject& project) = 0;
};

// What the composition root owns: the interface the pages see plus the project binding.
class IProjectRuntimeHost : public IRuntimeHost, public IProjectBinding {
public:
    using IRuntimeHost::IRuntimeHost;
};

void registerHostMetaTypes();

}  // namespace wpiemu::app

Q_DECLARE_METATYPE(wpiemu::app::HostLogLine)
Q_DECLARE_METATYPE(wpiemu::sdk::proto::DsState)
