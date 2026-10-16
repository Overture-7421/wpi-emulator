#pragma once

#include <QMetaType>
#include <QObject>
#include <QString>

#include <memory>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/logger.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/ui_descriptors.h"
#include "wpiemu/sdk/ipc/messages.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

namespace wpiemu::app {

// Run the robot to look at it (Run) or only to find out which devices its code declares (Discovery).
enum class RuntimeMode { Run, Discovery };

enum class HostProcessState { Idle, Starting, Running, Stopping, Exited, Crashed, FailedToStart };
enum class HostLinkState { Disconnected, Connecting, Connected, HandshakeTimeout, ProtocolMismatch, Stalled };
enum class HostLogStream { Stdout, Stderr, Emulator };

struct HostStatus {
    RuntimeMode mode = RuntimeMode::Run;
    HostProcessState process = HostProcessState::Idle;
    HostLinkState link = HostLinkState::Disconnected;
    int exitCode = 0;
    QString detail;

    bool active() const {
        return process == HostProcessState::Starting || process == HostProcessState::Running ||
               process == HostProcessState::Stopping;
    }
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

    // The shell calls this whenever the open project or its (saved/edited) model changes.
    virtual void setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config,
                            const sdk::IGameModule* game) = 0;

public slots:
    // start(): when a run in the other mode is active it is stopped first; the same mode is a no-op.
    virtual void start(wpiemu::app::RuntimeMode mode) = 0;
    virtual void stop() = 0;
    virtual void restart() = 0;
    virtual void resetSimulation() = 0;
    virtual void setPaused(bool paused) = 0;
    virtual void stepTicks(int ticks) = 0;
    virtual void setTimeScale(double scale) = 0;
    virtual void setDriverStation(const wpiemu::sdk::proto::DsState& state) = 0;

signals:
    void statusChanged();
    void sceneChanged();
    void snapshotPublished();
    void logLine(const wpiemu::app::HostLogLine& line);
};

}  // namespace wpiemu::app

Q_DECLARE_METATYPE(wpiemu::app::HostLogLine)
