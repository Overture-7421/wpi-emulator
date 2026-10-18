#pragma once

// Complete in-process IRunBackend: a synthetic holonomic robot with mechanism links, instanced "fuel",
// HUD values, log output and an injectable robot-process lifecycle. Used by tests, the demo
// executable and screenshots; the shell supplies the real adapter.

#include <QTimer>

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "run_backend.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::app::runview {

class FakeRunBackend : public IRunBackend {
    Q_OBJECT
public:
    struct Options {
        bool autoRun = true;          // drive the simulation from an internal QTimer
        int startupDelayMs = 350;     // Starting -> Running, and Running -> Connected; 0 = synchronous
        int ballCount = 200;
        bool withGameContent = true;  // HUD descriptors + zone kinds/states + fuel
        bool withVision = true;       // two cameras (PhotonVision on the chassis, Limelight on the turret) and an AprilTag layout
    };

    // How a synthetic camera misbehaves (simulateCameraFault).
    enum class CameraFault {
        None,
        Disabled,              // switched off in the project: status "disabled"
        NotConnected,          // the robot process never publishes it
        NotUsedByRobot,        // published, but the robot code does not use it (CameraRobotState::present = false)
        PhotonUuidUnsupported, // status "photon_uuid_unsupported": refused, nothing published
        ForeignPublisher,      // status "foreign_publisher": the robot simulates this camera itself
    };

    explicit FakeRunBackend(QObject* parent = nullptr) : FakeRunBackend(Options{}, parent) {}
    explicit FakeRunBackend(Options options, QObject* parent = nullptr);

    RunStatus status() const override { return status_; }
    RunSceneInfo sceneInfo() const override { return scene_; }
    std::shared_ptr<const sdk::ISimSnapshot> latestSnapshot() const override { return slot_.latest(); }

    // ---- test / demo hooks ----
    void tickOnce();                              // advance one 5 ms sim tick and publish a snapshot
    void tick(int count) { for (int i = 0; i < count; ++i) tickOnce(); }
    const sdk::proto::DsState& lastDriverStation() const { return ds_; }
    double timeScale() const { return timeScale_; }
    bool paused() const { return paused_; }
    int startCount() const { return startCount_; }
    int resetCount() const { return resetCount_; }
    // Number of setRobotPose() calls and the last pose handed in.
    int poseCount() const { return poseCount_; }
    const sdk::Pose3& lastRobotPose() const { return lastPose_; }
    void emitLog(sdk::LogLevel level, LogStream stream, const QString& source, const QString& text);

    void simulateCrash(int exitCode = 1, const QString& detail = {});
    void simulateHandshakeTimeout();
    void simulateProtocolMismatch(const QString& detail = {});
    void simulateStall();
    void simulateFailedToStart(const QString& detail);
    void setLinkSettled();                        // jump straight to Running + Connected
    // Vision failure injection. Faults stay until changed and survive a reset.
    void simulateCameraFault(const QString& cameraId, CameraFault fault);
    void simulateUnconfiguredCamera(const QString& ntName);   // the robot code uses a camera the project does not list ("" = none)
    void simulateLayoutMissing(bool missing = true);          // no AprilTag layout: no tags and no detections

    static sdk::RobotConfig makeDemoConfig();
    static std::vector<sdk::ZoneKindInfo> makeZoneKinds();   // demo_game.pickup (Volume), demo_game.shooter (Muzzle)
    static sdk::FieldDescription makeDemoField();
    static std::vector<sdk::FiducialSample> makeDemoTags();  // 18 tags on the walls and around the two hubs

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
    struct Ball {
        sdk::Vec3 pos;
        sdk::Vec3 vel;
        bool flying = false;
    };
    // Per configured camera (RobotConfig::vision.cameras, same order).
    struct CameraRuntime {
        std::int64_t accUs = 0;        // time since the last capture
        std::uint64_t captured = 0;
        std::vector<int> visible;      // tags of the latest capture
        CameraFault fault = CameraFault::None;
    };

    void setStatus(RobotProcessState process, LinkState link, int exitCode = 0, const QString& detail = {});
    void refreshFlags();
    void beginIfReady();
    void beginTransition(int delayMs, std::function<void()> fn);
    void onTimer();
    void stepWorld();
    void stepCameras();
    void resetWorld();
    void publish();
    void publishVision(sdk::SimSnapshotData& data) const;
    void scatterBalls();
    sdk::LinkPoseSource linkSource() const;
    CameraRuntime* findCamera(const QString& cameraId);

    Options options_;
    QTimer timer_;
    std::mt19937 rng_{1};
    RunStatus status_;
    RunSceneInfo scene_;
    sdk::LatestSnapshotSlot slot_;
    sdk::proto::DsState ds_;

    bool paused_ = false;
    bool programStarted_ = false;
    bool simulating_ = false;
    bool beginWhenReady_ = false;
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
    std::vector<sdk::FiducialSample> tags_;
    std::vector<CameraRuntime> cameras_;
    bool layoutMissing_ = false;
    std::string unconfiguredCamera_;
    int fuelHeld_ = 12;
    int score_ = 0;
    double shotTimer_ = 0.0;
    double logTimer_ = 0.0;
    int startCount_ = 0, resetCount_ = 0;
    int poseCount_ = 0;
    sdk::Pose3 lastPose_;
};

}  // namespace wpiemu::app::runview
