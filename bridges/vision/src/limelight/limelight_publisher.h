#pragma once

// The Limelight camera of the vision bridge (internal to src/limelight; reach it through makeLimelightVendor()).
//
// One instance per enabled Limelight camera, living on the extension's clock thread. It publishes the device's
// NetworkTables table (limelight_keys.h) through INtApi and reads what the robot writes to it:
//   start()    creates every topic with its default value and subscribes to the robot's requests, once ntcore is
//              bound and the robot constructor ran (so what it wrote there, the camera mount and the IMU mode, is
//              already readable). Refuses a table whose per-frame topics a robot-side simulation already publishes.
//   poll()     drains the requests every tick and keeps the echoes (getpipe, camerapose_robotspace) current.
//   publish()  one processed frame: ONE clock reading stamps every value of the frame (never earlier than the
//              camera's previous stamp); the values come from computeResult().
//
// What the requests do (robotState() also reports the pipeline, LED, IMU and driver modes, the throttle, the orientation
// and the mount):
//   pipeline                   getpipe follows it (every pipeline is treated as an AprilTag pipeline)
//   camMode                    1 = driver camera: frames publish an empty result, like a device that does not process
//   throttle_set               N > 0: one frame is processed after N skipped ones; skipped frames publish nothing
//   priorityid                 the primary target is that tag only (tx, ty, ta, tid, tv)
//   fiducial_id_filters_set    only these tags localize (botpose*); rawfiducials stays unfiltered
//   camerapose_robotspace_set  the mount the device believes in (MegaTag1/2 place the robot with it), echoed to
//                              camerapose_robotspace
//   robot_orientation_set      MegaTag2's orientation, interpolated to the capture time of each frame
//   ledMode, imumode_set       recorded only: LEDs do not matter for AprilTags and the internal IMU of newer devices is
//                              not modelled (MegaTag2 always uses the orientation the robot wrote)
// The other keys the robot can write (crop, stream, snapshot, ...) only identify a camera the robot uses (discovery).

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/bridges/vision/camera_publisher.h"
#include "wpiemu/bridges/vision/limelight_math.h"
#include "wpiemu/bridges/vision/limelight_orientation.h"
#include "wpiemu/bridges/vision/limelight_result.h"

namespace wpiemu::bridges::vision::limelight {

class LimelightPublisher final : public ICameraPublisher {
public:
    LimelightPublisher(INtApi& nt, sdk::ILogger& log, const sdk::CameraSetup& setup);

    const sdk::CameraSetup& setup() const override { return setup_; }
    void start() override;
    void poll() override;
    void publish(const sdk::VisionFrame& frame) override;
    sdk::CameraRobotState robotState() const override;

private:
    // Re-check the table for robot-created topics at this many polls while none was seen (cheap, but not every tick).
    static constexpr int kPresenceRecheckPolls = 40;

    struct Topics {
        NtPublisher tv, tx, ty, txnc, tync, ta, tl, cl, tid, hb, getPipe, getPipeType;
        NtPublisher botpose, botposeWpiBlue, botposeWpiRed, botposeOrb, botposeOrbWpiBlue, botposeOrbWpiRed;
        NtPublisher rawFiducials, cameraPoseRobotSpace, hw;
    };
    struct Requests {
        NtSubscriber pipeline, ledMode, camMode, imuMode, throttle, priorityId, orientation, cameraPose, idFilters;
    };

    NtPublisher publishTopic(std::string_view key, NtType type);
    void createTopics();
    void subscribeRequests();
    void drainRequests();
    void checkPresence();
    void refuse(std::string_view status, const std::string& reason);

    void onCameraPose(const std::vector<double>& values);
    void onOrientation(const NtValue& value);
    void echoPipeline();
    void echoCameraPose();
    bool admitFrame();
    std::int64_t nextStampUs();
    void writeResult(std::int64_t timeUs, const LimelightResult& result, std::int64_t heartbeat);

    INtApi& nt_;
    sdk::ILogger& log_;
    sdk::CameraSetup setup_;
    std::string table_;

    bool started_ = false;
    bool inert_ = false;
    std::string_view status_ = camera_status::kOk;
    std::int64_t lastStampUs_ = 0;
    std::int64_t heartbeat_ = 0;
    std::uint64_t framesPublished_ = 0;
    int skippedFrames_ = 0;
    int pollsSincePresenceCheck_ = 0;
    bool present_ = false;

    Topics topics_;
    Requests requests_;

    // What the robot asked for (nullopt / default until it writes).
    std::optional<std::int32_t> pipeline_;
    std::optional<std::int32_t> ledMode_;
    std::optional<std::int32_t> imuMode_;
    std::optional<bool> driverMode_;
    std::optional<double> throttle_;
    int priorityId_ = kPriorityIdNone;
    std::vector<int> idFilter_;
    std::optional<RobotSpacePose> cameraPoseOverride_;  // as written, Limelight robot space
    Pose3d assumedRobotToCamera_;
    RobotSpacePose cameraPoseEcho_{};                   // the value of camerapose_robotspace
    OrientationHistory orientation_;

    std::vector<NtValue> scratchValues_;
    std::vector<NtTopicInfo> scratchTopics_;
};

}  // namespace wpiemu::bridges::vision::limelight
