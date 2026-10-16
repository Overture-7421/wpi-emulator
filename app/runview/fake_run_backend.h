#pragma once

// Complete in-process IRunBackend: a synthetic holonomic robot with mechanism links, instanced "fuel",
// HUD values, log output and an injectable robot-process lifecycle. Used by tests, the demo
// executable and screenshots; the shell supplies the real adapter.

#include <QTimer>

#include <functional>
#include <random>

#include "run_backend.h"

namespace wpiemu::app::runview {

class FakeRunBackend : public IRunBackend {
    Q_OBJECT
public:
    struct Options {
        bool autoRun = true;          // drive the simulation from an internal QTimer
        int startupDelayMs = 350;     // Starting -> Running, and Running -> Connected; 0 = synchronous
        int ballCount = 200;
        bool withGameContent = true;  // HUD descriptors + zone overlays + fuel
    };

    explicit FakeRunBackend(QObject* parent = nullptr) : FakeRunBackend(Options{}, parent) {}
    explicit FakeRunBackend(Options options, QObject* parent = nullptr);

    RunStatus status() const override { return status_; }
    RunSceneInfo sceneInfo() const override { return scene_; }
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override { return slot_.latest(); }
    RealDsState realDsState() const override { return realDs_; }

    // ---- test / demo hooks ----
    void tickOnce();                              // advance one 5 ms sim tick and publish a snapshot
    void tick(int count) { for (int i = 0; i < count; ++i) tickOnce(); }
    const sdk::proto::DsState& lastDriverStation() const { return ds_; }
    double timeScale() const { return timeScale_; }
    bool paused() const { return paused_; }
    int startCount() const { return startCount_; }
    int stopCount() const { return stopCount_; }
    int resetCount() const { return resetCount_; }
    int stepRequests() const { return stepRequests_; }
    void emitLog(sdk::LogLevel level, LogStream stream, const QString& source, const QString& text);
    void setRealDsState(RealDsState state);

    void simulateCrash(int exitCode = 1, const QString& detail = {});
    void simulateHandshakeTimeout();
    void simulateProtocolMismatch(const QString& detail = {});
    void simulateStall();
    void simulateFailedToStart(const QString& detail);
    void setLinkSettled();                        // jump straight to Running + Connected

    static sdk::RobotConfig makeDemoConfig();
    static sdk::FieldDescription makeDemoField();

public slots:
    void startRobot() override;
    void stopRobot() override;
    void restartRobot() override;
    void resetSimulation() override;
    void setPaused(bool paused) override;
    void stepTicks(int ticks) override;
    void setTimeScale(double scale) override;
    void setDriverStation(const wpiemu::sdk::proto::DsState& state) override;

private:
    struct Ball {
        sdk::Vec3 pos;
        sdk::Vec3 vel;
        bool flying = false;
    };

    void setStatus(RobotProcessState process, LinkState link, int exitCode = 0, const QString& detail = {});
    void beginTransition(int delayMs, std::function<void()> fn);
    void onTimer();
    void stepWorld();
    void resetWorld();
    void publish();
    void scatterBalls();

    Options options_;
    QTimer timer_;
    std::mt19937 rng_{1};
    RunStatus status_;
    RunSceneInfo scene_;
    sdk::LatestSnapshotSlot slot_;
    RealDsState realDs_ = RealDsState::NotConfigured;
    sdk::proto::DsState ds_;

    bool paused_ = false;
    int pendingSteps_ = 0;
    double timeScale_ = 1.0;
    double timeAccum_ = 0.0;
    int transitionGeneration_ = 0;

    // world
    std::uint64_t tick_ = 0;
    sdk::SimTimeUs simUs_ = 0;
    sdk::Vec3 pos_{2.0f, 4.1f, 0.2f};
    float yaw_ = 0.f;
    sdk::Vec3 vel_;      // robot frame
    float omega_ = 0.f;
    float steer_[4] = {0, 0, 0, 0};
    float spin_[4] = {0, 0, 0, 0};
    float armAngle_ = 0.f, turretYaw_ = 0.f, flywheel_ = 0.f, carriage_ = 0.f;
    std::vector<Ball> balls_;
    int fuelHeld_ = 12;
    int score_ = 0;
    double shotTimer_ = 0.0;
    double logTimer_ = 0.0;
    int startCount_ = 0, stopCount_ = 0, resetCount_ = 0, stepRequests_ = 0;
};

}  // namespace wpiemu::app::runview
