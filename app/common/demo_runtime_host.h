#pragma once

#include <QTimer>

#include <vector>

#include "common/runtime_host.h"

namespace wpiemu::app {

// Scripted stand-in for the real runtime (`--demo`, tests, screenshots): "launches" instantly,
// announces plausible devices one after another while armed and drives the chassis around
// from the Driver Station state once the simulation starts. No process, no physics, no sockets.
class DemoRuntimeHost final : public IRuntimeHost {
    Q_OBJECT
public:
    explicit DemoRuntimeHost(QObject* parent = nullptr);

    HostStatus status() const override { return status_; }
    HostScene scene() const override { return scene_; }
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override { return slot_.latest(); }
    void setProject(const QString& projectPath, std::shared_ptr<const sdk::RobotConfig> config, const sdk::IGameModule* game) override;

    // Devices a robot with this config would typically declare.
    static std::vector<sdk::DiscoveredDevice> demoDevicesFor(const sdk::RobotConfig& config);
    // Pace of the scripted start-up and device announcements (tests use tiny values).
    void setStepIntervals(int startupMs, int announceMs) {
        startupMs_ = startupMs;
        announceMs_ = announceMs;
    }

public slots:
    void launch() override;
    void startSimulation() override;
    void stopSimulation() override;
    void stop() override;
    void restart() override;
    void resetSimulation() override;
    void setPaused(bool paused) override;
    void stepTicks(int ticks) override;
    void setTimeScale(double scale) override;
    void setDriverStation(const wpiemu::sdk::proto::DsState& state) override;
    void setRobotPose(const wpiemu::sdk::Pose3& pose) override;

private:
    void setStatus(HostProcessState process, HostLinkState link, const QString& detail = {});
    void log(sdk::LogLevel level, const QString& source, const QString& text);
    void advanceStartup();
    void announceNextDevice();
    void beginIfReady();
    void tickSimulation();
    void integrate(double dt);
    void publish();

    HostStatus status_;
    HostScene scene_;
    QString projectPath_;
    sdk::LatestSnapshotSlot slot_;
    QTimer startupTimer_;
    QTimer simTimer_;

    std::vector<sdk::DiscoveredDevice> plannedDevices_;
    std::vector<sdk::DiscoveredDevice> announced_;
    int startupStage_ = 0;
    int startupMs_ = 350;
    int announceMs_ = 450;

    sdk::proto::DsState ds_;
    bool paused_ = false;
    SimulationStartIntent beginIntent_;
    double timeScale_ = 1.0;
    std::uint64_t tick_ = 0;
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, omega_ = 0.0;
    double wheelSpin_ = 0.0;
};

}  // namespace wpiemu::app
