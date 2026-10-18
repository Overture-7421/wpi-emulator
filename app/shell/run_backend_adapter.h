#pragma once

#include "common/app_services.h"
#include "runview/run_backend.h"

namespace wpiemu::app {

// Connects the run view to the shared runtime host (process + sim thread), the project session
// (scene content) and the per-project driver-station preferences.
class RunBackendAdapter final : public runview::IRunBackend {
    Q_OBJECT
public:
    explicit RunBackendAdapter(AppServices& services, QObject* parent = nullptr);

    runview::RunStatus status() const override;
    runview::RunSceneInfo sceneInfo() const override;
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override { return services_.runtime().latestSnapshot(); }

public slots:
    void launchRobot() override;
    void startSimulation() override;
    void stopSimulation() override;
    void restartRobot() override;
    void resetSimulation() override;
    void setPaused(bool paused) override;
    void stepTicks(int ticks) override;
    void setTimeScale(double scale) override;
    void setDriverStation(const wpiemu::sdk::proto::DsState& state) override;
    void setRobotPose(const wpiemu::sdk::Pose3& pose) override;

private:
    void onLog(const HostLogLine& line);

    AppServices& services_;
    bool dsSent_ = false;
};

}  // namespace wpiemu::app
