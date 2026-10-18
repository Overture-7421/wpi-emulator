#include "limelight_publisher.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/bridges/vision/limelight_keys.h"

namespace wpiemu::bridges::vision::limelight {

namespace {

constexpr const char* kCategory = "bridge.vision.limelight";

// The number a robot wrote to a scalar key: NT delivers booleans, integers and doubles as one number. Non-finite values
// are refused so that a NaN can never reach a pose.
std::optional<double> scalar(const NtValue& value) {
    const bool numeric = value.type == NtType::Boolean || value.type == NtType::Integer || value.type == NtType::Double;
    if (!numeric || !std::isfinite(value.number)) return std::nullopt;
    return value.number;
}

std::int32_t toInt(double value) {
    return static_cast<std::int32_t>(std::lround(std::clamp(value, -2147483648.0, 2147483647.0)));
}

bool allFinite(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

}  // namespace

LimelightPublisher::LimelightPublisher(INtApi& nt, sdk::ILogger& log, const sdk::CameraSetup& setup)
    : nt_(nt),
      log_(log),
      setup_(setup),
      table_(tableFor(setup.ntName)),
      assumedRobotToCamera_(toPose3d(setup.defaultAssumedRobotToCamera)),
      cameraPoseEcho_(toLimelightRobotSpace(assumedRobotToCamera_)) {}

NtPublisher LimelightPublisher::publishTopic(std::string_view key, NtType type) {
    NtPublishOptions options;
    options.keepDuplicates = true;
    return nt_.publish(topicOf(table_, key), type, options);
}

void LimelightPublisher::createTopics() {
    topics_.tv = publishTopic(kTv, NtType::Double);
    topics_.tx = publishTopic(kTx, NtType::Double);
    topics_.ty = publishTopic(kTy, NtType::Double);
    topics_.txnc = publishTopic(kTxnc, NtType::Double);
    topics_.tync = publishTopic(kTync, NtType::Double);
    topics_.ta = publishTopic(kTa, NtType::Double);
    topics_.tl = publishTopic(kTl, NtType::Double);
    topics_.cl = publishTopic(kCl, NtType::Double);
    topics_.tid = publishTopic(kTid, NtType::Double);
    topics_.hb = publishTopic(kHb, NtType::Double);
    topics_.getPipe = publishTopic(kGetPipe, NtType::Double);
    topics_.getPipeType = publishTopic(kGetPipeType, NtType::String);
    topics_.botpose = publishTopic(kBotpose, NtType::DoubleArray);
    topics_.botposeWpiBlue = publishTopic(kBotposeWpiBlue, NtType::DoubleArray);
    topics_.botposeWpiRed = publishTopic(kBotposeWpiRed, NtType::DoubleArray);
    topics_.botposeOrb = publishTopic(kBotposeOrb, NtType::DoubleArray);
    topics_.botposeOrbWpiBlue = publishTopic(kBotposeOrbWpiBlue, NtType::DoubleArray);
    topics_.botposeOrbWpiRed = publishTopic(kBotposeOrbWpiRed, NtType::DoubleArray);
    topics_.rawFiducials = publishTopic(kRawFiducials, NtType::DoubleArray);
    topics_.cameraPoseRobotSpace = publishTopic(kCameraPoseRobotSpace, NtType::DoubleArray);
    topics_.hw = publishTopic(kHw, NtType::DoubleArray);
}

void LimelightPublisher::subscribeRequests() {
    const auto subscribe = [this](std::string_view key) {
        return nt_.subscribe(topicOf(table_, key), kDefaultNtQueueDepth);
    };
    requests_.pipeline = subscribe(kPipeline);
    requests_.ledMode = subscribe(kLedMode);
    requests_.camMode = subscribe(kCamMode);
    requests_.imuMode = subscribe(kImuModeSet);
    requests_.throttle = subscribe(kThrottleSet);
    requests_.priorityId = subscribe(kPriorityId);
    requests_.orientation = subscribe(kRobotOrientationSet);
    requests_.cameraPose = subscribe(kCameraPoseRobotSpaceSet);
    requests_.idFilters = subscribe(kFiducialIdFiltersSet);
}

void LimelightPublisher::refuse(std::string_view status, const std::string& reason) {
    inert_ = true;
    status_ = status;
    log_.warn(kCategory, "limelight '" + table_ + "': " + reason);
}

void LimelightPublisher::start() {
    if (started_) return;
    if (!nt_.available()) {
        inert_ = true;
        status_ = camera_status::kNtcoreUnavailable;
        return;
    }
    started_ = true;
    inert_ = false;
    status_ = camera_status::kOk;

    // The robot constructor already wrote the mount and the IMU mode; a new subscriber receives the latest value.
    subscribeRequests();
    drainRequests();
    checkPresence();

    for (const std::string_view key : kForeignProbeKeys) {
        const std::string topic = topicOf(table_, key);
        if (nt_.hasForeignPublisher(topic)) {
            refuse(camera_status::kForeignPublisher,
                   "a publisher of " + topic + " already exists (a robot-side simulation of the camera?); "
                   "leaving the camera alone");
            return;
        }
    }

    createTopics();
    const std::int64_t timeUs = nextStampUs();
    ResultSettings settings;
    settings.assumedRobotToCamera = assumedRobotToCamera_;
    writeResult(timeUs, computeResult(sdk::VisionFrame{}, settings), 0);

    std::array<double, kHwSize> hardware{};
    hardware[kHwCpuTempC] = kHwNominalCpuTempC;
    hardware[kHwCpuUsagePct] = kHwNominalCpuUsagePct;
    hardware[kHwRamUsagePct] = kHwNominalRamUsagePct;
    hardware[kHwFps] = setup_.frameRateHz;
    nt_.setDoubleArray(topics_.hw, timeUs, hardware);

    log_.info(kCategory, "limelight '" + table_ + "' publishes AprilTag results (" +
                             (cameraPoseOverride_ ? "mount written by the robot" : "default mount") + ")");
}

void LimelightPublisher::poll() {
    if (!started_) {
        if (nt_.available()) start();
        return;
    }
    drainRequests();
    if (!present_ && ++pollsSincePresenceCheck_ >= kPresenceRecheckPolls) {
        pollsSincePresenceCheck_ = 0;
        checkPresence();
    }
}

void LimelightPublisher::checkPresence() {
    if (present_) return;
    scratchTopics_.clear();
    nt_.topics(topicOf(table_, {}), scratchTopics_);
    for (const NtTopicInfo& info : scratchTopics_) {
        std::string_view table;
        std::string_view key;
        if (splitTopic(info.name, table, key) && isRobotWrittenKey(key)) {
            present_ = true;
            return;
        }
    }
}

void LimelightPublisher::drainRequests() {
    if (!started_) return;
    const auto drain = [this](NtSubscriber subscriber, auto&& apply) {
        scratchValues_.clear();
        nt_.read(subscriber, scratchValues_);
        for (const NtValue& value : scratchValues_) {
            present_ = true;
            apply(value);
        }
    };

    drain(requests_.pipeline, [this](const NtValue& value) {
        if (const auto n = scalar(value)) {
            pipeline_ = toInt(*n);
            echoPipeline();
        }
    });
    drain(requests_.ledMode, [this](const NtValue& value) {
        if (const auto n = scalar(value)) ledMode_ = toInt(*n);
    });
    drain(requests_.camMode, [this](const NtValue& value) {
        if (const auto n = scalar(value)) driverMode_ = toInt(*n) == kCamModeDriver;
    });
    drain(requests_.imuMode, [this](const NtValue& value) {
        if (const auto n = scalar(value)) imuMode_ = toInt(*n);
    });
    drain(requests_.throttle, [this](const NtValue& value) {
        if (const auto n = scalar(value)) throttle_ = std::max(0.0, *n);
    });
    drain(requests_.priorityId, [this](const NtValue& value) {
        if (const auto n = scalar(value)) priorityId_ = *n >= 0.0 ? toInt(*n) : kPriorityIdNone;
    });
    drain(requests_.orientation, [this](const NtValue& value) { onOrientation(value); });
    drain(requests_.cameraPose, [this](const NtValue& value) {
        if (value.type == NtType::DoubleArray) onCameraPose(value.numbers);
    });
    drain(requests_.idFilters, [this](const NtValue& value) {
        if (value.type != NtType::DoubleArray || !allFinite(value.numbers)) return;
        idFilter_.clear();
        for (const double id : value.numbers) idFilter_.push_back(toInt(id));
    });
}

void LimelightPublisher::onCameraPose(const std::vector<double>& values) {
    if (values.size() < kPose6Size || !allFinite(values)) return;
    RobotSpacePose pose{};
    std::copy_n(values.begin(), kPose6Size, pose.begin());
    cameraPoseOverride_ = pose;
    cameraPoseEcho_ = pose;
    assumedRobotToCamera_ = fromLimelightRobotSpace(pose);
    echoCameraPose();
}

void LimelightPublisher::onOrientation(const NtValue& value) {
    if (value.type != NtType::DoubleArray || value.numbers.empty() || !allFinite(value.numbers)) return;
    std::array<double, kOrientationSize> a{};
    std::copy_n(value.numbers.begin(), std::min(value.numbers.size(), a.size()), a.begin());
    RobotOrientation orientation;
    orientation.yawDeg = a[kOrientationYawDeg];
    orientation.yawRateDegPerS = a[kOrientationYawRateDegPerS];
    orientation.pitchDeg = a[kOrientationPitchDeg];
    orientation.pitchRateDegPerS = a[kOrientationPitchRateDegPerS];
    orientation.rollDeg = a[kOrientationRollDeg];
    orientation.rollRateDegPerS = a[kOrientationRollRateDegPerS];
    orientation_.add(value.timeUs, orientation);
}

void LimelightPublisher::echoPipeline() {
    if (!topics_.getPipe) return;
    nt_.setDouble(topics_.getPipe, nextStampUs(), static_cast<double>(pipeline_.value_or(kDefaultPipeline)));
}

void LimelightPublisher::echoCameraPose() {
    if (!topics_.cameraPoseRobotSpace) return;
    nt_.setDoubleArray(topics_.cameraPoseRobotSpace, nextStampUs(), cameraPoseEcho_);
}

// One frame is processed after `throttle_set` skipped ones; 0 (or unset) processes every frame.
bool LimelightPublisher::admitFrame() {
    const int skip = throttle_ ? static_cast<int>(std::min(*throttle_, 1.0e6)) : 0;
    if (skip <= 0 || skippedFrames_ >= skip) {
        skippedFrames_ = 0;
        return true;
    }
    ++skippedFrames_;
    return false;
}

// ntcore drops a value stamped earlier than the topic's last one, so a camera never goes back in time, whatever the
// clock does.
std::int64_t LimelightPublisher::nextStampUs() {
    lastStampUs_ = std::max(lastStampUs_, nt_.nowUs());
    return lastStampUs_;
}

void LimelightPublisher::publish(const sdk::VisionFrame& frame) {
    if (!started_ || inert_) return;
    drainRequests();
    if (!admitFrame()) return;

    const std::int64_t timeUs = nextStampUs();  // the ONE clock reading of this frame
    ResultSettings settings;
    settings.assumedRobotToCamera = assumedRobotToCamera_;
    settings.orientation = orientation_.at(timeUs - frame.latencyUs);
    settings.idFilter = idFilter_;
    settings.priorityId = priorityId_;
    settings.driverMode = driverMode_.value_or(false);

    heartbeat_ = (heartbeat_ + 1) % kHeartbeatWrap;
    writeResult(timeUs, computeResult(frame, settings), heartbeat_);
    ++framesPublished_;
}

void LimelightPublisher::writeResult(std::int64_t timeUs, const LimelightResult& result, std::int64_t heartbeat) {
    nt_.setDouble(topics_.tv, timeUs, result.hasTarget ? kTvValid : kTvNone);
    nt_.setDouble(topics_.tx, timeUs, result.txDeg);
    nt_.setDouble(topics_.ty, timeUs, result.tyDeg);
    nt_.setDouble(topics_.txnc, timeUs, result.txDeg);
    nt_.setDouble(topics_.tync, timeUs, result.tyDeg);
    nt_.setDouble(topics_.ta, timeUs, result.taPct);
    nt_.setDouble(topics_.tl, timeUs, result.tlMs);
    nt_.setDouble(topics_.cl, timeUs, result.clMs);
    nt_.setDouble(topics_.tid, timeUs, result.tid);
    nt_.setDouble(topics_.getPipe, timeUs, static_cast<double>(pipeline_.value_or(kDefaultPipeline)));
    nt_.setString(topics_.getPipeType, timeUs, kPipelineTypeFiducial);
    nt_.setDoubleArray(topics_.botpose, timeUs, result.botpose);
    nt_.setDoubleArray(topics_.botposeWpiBlue, timeUs, result.botposeWpiBlue);
    nt_.setDoubleArray(topics_.botposeWpiRed, timeUs, result.botposeWpiRed);
    nt_.setDoubleArray(topics_.botposeOrb, timeUs, result.botposeOrb);
    nt_.setDoubleArray(topics_.botposeOrbWpiBlue, timeUs, result.botposeOrbWpiBlue);
    nt_.setDoubleArray(topics_.botposeOrbWpiRed, timeUs, result.botposeOrbWpiRed);
    nt_.setDoubleArray(topics_.rawFiducials, timeUs, result.rawFiducials);
    nt_.setDoubleArray(topics_.cameraPoseRobotSpace, timeUs, cameraPoseEcho_);
    nt_.setDouble(topics_.hb, timeUs, static_cast<double>(heartbeat));  // last, like the device
}

sdk::CameraRobotState LimelightPublisher::robotState() const {
    sdk::CameraRobotState state;
    state.ntName = table_;
    state.vendor = sdk::CameraVendor::Limelight;
    state.cameraId = setup_.cameraId;
    state.present = present_;
    state.pipelineIndex = pipeline_;
    state.ledMode = ledMode_;
    state.imuMode = imuMode_;
    state.driverMode = driverMode_;
    state.throttle = throttle_;
    if (const RobotOrientation* orientation = orientation_.latest()) {
        std::array<double, kOrientationSize> a{};
        a[kOrientationYawDeg] = orientation->yawDeg;
        a[kOrientationYawRateDegPerS] = orientation->yawRateDegPerS;
        a[kOrientationPitchDeg] = orientation->pitchDeg;
        a[kOrientationPitchRateDegPerS] = orientation->pitchRateDegPerS;
        a[kOrientationRollDeg] = orientation->rollDeg;
        a[kOrientationRollRateDegPerS] = orientation->rollRateDegPerS;
        state.robotOrientationDeg = a;
    }
    state.cameraPoseOverride = cameraPoseOverride_;
    state.publishing = started_ && !inert_;
    state.status = std::string(status_);
    state.framesPublished = framesPublished_;
    return state;
}

}  // namespace wpiemu::bridges::vision::limelight
