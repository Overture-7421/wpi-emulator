#pragma once

#include <QTimer>

#include <filesystem>
#include <functional>
#include <memory>

#include "common/runtime_host.h"
#include "host/robot_session.h"

namespace wpiemu::app {

struct RealRuntimeOptions {
    std::filesystem::path extensionPath;   // empty = locate (WPIEMU_EXTENSION / WPIEMU_EXTENSION_DIR, install layout, build tree)
    std::filesystem::path scratchDir;      // generated init script; empty = <temp>/wpiemu
    std::filesystem::path appDir;          // empty = QCoreApplication::applicationDirPath()
    int pollIntervalMs = 30;
    std::chrono::milliseconds handshakeTimeout{std::chrono::minutes(5)};
    // Test / advanced hook: adjusts the options of every run just before it starts.
    std::function<void(host::Options&)> tweak;
};

// The real simulation host: validates the model, builds the physics world and the game plugin session, launches the
// unmodified robot project with the HALSim extension and shows what happens. All heavy lifting is in host::RobotSession
// (Qt-free); this class owns one session at a time and turns its thread-safe state into UI-thread signals by polling.
//
// Environment (advanced / diagnostics): WPIEMU_EXTENSION, WPIEMU_EXTENSION_DIR, WPIEMU_LAUNCH_MODE=gradle|direct, WPIEMU_GRADLE_OFFLINE=1,
// WPIEMU_PRESTART_POLLING=1 (poll vendor bridges while the robot constructor runs; crashes the JVM with CTRE, off by default).
class RealRuntimeHost final : public IRuntimeHost {
    Q_OBJECT
public:
    RealRuntimeHost(host::Registries registries, RealRuntimeOptions options = {}, QObject* parent = nullptr);
    ~RealRuntimeHost() override;

    HostStatus status() const override { return status_; }
    HostScene scene() const override { return scene_; }
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override;
    void setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config, const sdk::IGameModule* game) override;

    // The extension that would be launched (empty when it cannot be found).
    std::filesystem::path extensionPath() const;

public slots:
    void start(wpiemu::app::RuntimeMode mode) override;
    void stop() override;
    void restart() override;
    void resetSimulation() override;
    void setPaused(bool paused) override;
    void stepTicks(int ticks) override;
    void setTimeScale(double scale) override;
    void setDriverStation(const wpiemu::sdk::proto::DsState& state) override;

private:
    void pump();
    void setStatus(const HostStatus& status);
    void failToStart(RuntimeMode mode, const QString& detail);
    void emitLog(sdk::LogLevel level, const QString& source, const QString& text);
    void releaseSession();
    static HostStatus map(const host::SessionStatus& s);

    host::Registries registries_;
    RealRuntimeOptions options_;
    QTimer timer_;

    HostStatus status_;
    HostScene scene_;
    QString projectPath_;
    std::shared_ptr<const sdk::RobotConfig> config_;
    std::string gamePluginId_;
    std::shared_ptr<const sdk::FieldDescription> field_;
    std::string fieldPluginId_;

    std::unique_ptr<host::RobotSession> session_;
    std::shared_ptr<const sdk::ISimSnapshot> lastSnapshot_;
    const sdk::ISimSnapshot* announcedSnapshot_ = nullptr;

    sdk::proto::DsState ds_;
    bool paused_ = false;
    double timeScale_ = 1.0;
};

}  // namespace wpiemu::app
