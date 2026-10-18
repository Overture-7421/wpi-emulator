#include "photon_publisher.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "wpiemu/bridges/vision/photon_mapper.h"

// Topics, types and behaviour follow what a PhotonVision v2026.3.4 coprocessor publishes (NTTopicSet.java and
// NTDataPublisher.java, read for the contract only) as the nt spike verified them against the real PhotonLib in
// Java and C++ (files/spike-nt-findings.md sections 2.3, 2.4, 2.6 and 3).

namespace wpiemu::bridges::vision {

namespace {

constexpr std::string_view kLog = "bridge.vision.photon";

constexpr std::string_view kVersionTopic = "/photonvision/version";
constexpr std::string_view kLedModeStateTopic = "/photonvision/ledModeState";
constexpr std::string_view kLedModeRequestTopic = "/photonvision/ledModeRequest";

constexpr std::int64_t kLedModeDefault = -1;  // PhotonLib VisionLEDMode.kDefault
constexpr std::int64_t kNoFpsLimit = -1;      // "unlimited"
constexpr double kResultPeriodS = 0.01;       // the periodic option PhotonVision gives the result stream

// The robot's requests arrive as whatever type it published (an int, a boolean, a double): all carry the number.
bool numeric(const NtValue& value) {
    return value.type == NtType::Boolean || value.type == NtType::Integer || value.type == NtType::Double;
}

std::int32_t toInt32(double value) {
    if (!(value == value)) return 0;
    return static_cast<std::int32_t>(std::clamp(value, -2147483648.0, 2147483647.0));
}

}  // namespace

// ---- PhotonSharedState ----------------------------------------------------------------------------------------

void PhotonSharedState::adopt(INtApi& nt) {
    if (nt_ == &nt) return;
    nt_ = &nt;
    version_ = {};
    ledModeState_ = {};
    ledModeRequest_ = {};
    ledMode_.reset();
    memory_.clear();
}

void PhotonSharedState::subscribe(INtApi& nt) {
    adopt(nt);
    if (!ledModeRequest_) ledModeRequest_ = nt.subscribe(kLedModeRequestTopic, kDefaultNtQueueDepth);
}

void PhotonSharedState::publish(INtApi& nt, std::string_view versionText) {
    adopt(nt);
    if (version_ && ledModeState_) return;
    if (!version_) version_ = nt.publish(kVersionTopic, NtType::String, {});
    if (!ledModeState_) ledModeState_ = nt.publish(kLedModeStateTopic, NtType::Integer, {});
    const std::int64_t now = nt.nowUs();
    nt.setString(version_, now, versionText);
    nt.setInteger(ledModeState_, now, ledMode_.value_or(static_cast<std::int32_t>(kLedModeDefault)));
}

void PhotonSharedState::poll(INtApi& nt) {
    adopt(nt);
    if (!ledModeRequest_) return;
    inbox_.clear();
    nt.read(ledModeRequest_, inbox_);
    std::optional<std::int32_t> latest;
    for (const NtValue& value : inbox_) {
        if (numeric(value)) latest = toInt32(value.number);
    }
    if (!latest) return;
    ledMode_ = latest;
    if (ledModeState_) nt.setInteger(ledModeState_, nt.nowUs(), *latest);
}

PhotonSharedState::CameraMemory PhotonSharedState::recall(const std::string& table) const {
    const auto it = memory_.find(table);
    return it != memory_.end() ? it->second : CameraMemory{};
}

// ---- PhotonPublisher ------------------------------------------------------------------------------------------

PhotonPublisher::PhotonPublisher(INtApi& nt, sdk::ILogger& log, const sdk::CameraSetup& setup,
                                 std::shared_ptr<const PhotonCodec> codec, std::string unsupportedReason,
                                 std::shared_ptr<PhotonSharedState> shared)
    : nt_(nt),
      log_(log),
      setup_(setup),
      codec_(std::move(codec)),
      unsupportedReason_(std::move(unsupportedReason)),
      shared_(std::move(shared)) {
    if (setup_.ntName.empty()) setup_.ntName = setup_.cameraId;
    table_ = "/photonvision/" + setup_.ntName + "/";
    requestTopics_[0] = table_ + "pipelineIndexRequest";
    requestTopics_[1] = table_ + "driverModeRequest";
    requestTopics_[2] = table_ + "fpsLimitRequest";
}

void PhotonPublisher::refuse(std::string_view status, const std::string& why) {
    inert_ = true;
    status_ = std::string(status);
    log_.warn(kLog, "PhotonVision camera '" + setup_.ntName + "' is not published: " + why);
}

void PhotonPublisher::start() {
    if (started_) return;
    if (!nt_.available()) {
        status_ = std::string(camera_status::kNtcoreUnavailable);
        return;
    }
    started_ = true;
    status_.clear();

    // What the robot writes is read whatever happens next: an inert camera still reports its requests.
    pipelineRequest_ = nt_.subscribe(requestTopics_[0], kDefaultNtQueueDepth);
    driverModeRequest_ = nt_.subscribe(requestTopics_[1], kDefaultNtQueueDepth);
    fpsLimitRequest_ = nt_.subscribe(requestTopics_[2], kDefaultNtQueueDepth);
    shared_->subscribe(nt_);

    if (!codec_) {
        refuse(camera_status::kPhotonUuidUnsupported, unsupportedReason_);
        return;
    }
    // Asked BEFORE anything of ours exists on the topic: a robot-side PhotonCameraSim publishes it from its
    // constructor.
    const std::string resultTopic = table_ + "rawBytes";
    if (nt_.hasForeignPublisher(resultTopic)) {
        refuse(camera_status::kForeignPublisher,
               resultTopic + " already has a publisher (a robot-side PhotonCameraSim?); the emulator leaves it alone");
        return;
    }
    if (!createTopics()) {
        refuse(camera_status::kNtcoreUnavailable, "ntcore did not accept the publisher of " + resultTopic);
        return;
    }
    log_.info(kLog, "PhotonVision camera '" + setup_.ntName + "' publishes " + table_ + " (message definition " +
                        codec_->uuid() + ")");
}

bool PhotonPublisher::createTopics() {
    // The type string AND the message_uuid property carry the UUID: a wrong type string delivers nothing, a wrong
    // property makes PhotonLib throw at its first version check. The property goes in the very publish call, so
    // there is no moment at which the topic exists without it.
    const std::string typeString = codec_->resultTypeString();
    const std::string properties = "{\"message_uuid\":\"" + codec_->uuid() + "\"}";
    NtPublishOptions result;
    result.typeString = typeString;
    result.propertiesJson = properties;
    result.sendAll = true;
    result.keepDuplicates = true;
    result.periodicS = kResultPeriodS;
    topics_.result = nt_.publish(table_ + "rawBytes", NtType::Raw, result);
    if (!topics_.result) return false;

    const auto plain = [&](std::string_view leaf, NtType type) {
        return nt_.publish(table_ + std::string(leaf), type, NtPublishOptions{});
    };
    topics_.heartbeat = plain("heartbeat", NtType::Integer);
    topics_.latencyMillis = plain("latencyMillis", NtType::Double);
    topics_.fps = plain("fps", NtType::Double);
    topics_.hasTarget = plain("hasTarget", NtType::Boolean);
    topics_.targetPitch = plain("targetPitch", NtType::Double);
    topics_.targetYaw = plain("targetYaw", NtType::Double);
    topics_.targetArea = plain("targetArea", NtType::Double);
    topics_.targetSkew = plain("targetSkew", NtType::Double);
    topics_.targetPixelsX = plain("targetPixelsX", NtType::Double);
    topics_.targetPixelsY = plain("targetPixelsY", NtType::Double);
    NtPublishOptions pose;
    pose.typeString = "struct:Transform3d";
    topics_.targetPose = nt_.publish(table_ + "targetPose", NtType::Raw, pose);
    topics_.pipelineIndexState = plain("pipelineIndexState", NtType::Integer);
    topics_.driverMode = plain("driverMode", NtType::Boolean);
    topics_.fpsLimit = plain("fpsLimit", NtType::Integer);
    topics_.cameraIntrinsics = plain("cameraIntrinsics", NtType::DoubleArray);
    topics_.cameraDistortion = plain("cameraDistortion", NtType::DoubleArray);

    shared_->publish(nt_, codec_->version());
    // A camera rebuilt for the same table continues where its predecessor stopped: the heartbeat only ever grows,
    // timestamps never go back.
    const PhotonSharedState::CameraMemory memory = shared_->recall(table_);
    heartbeat_ = memory.heartbeat;
    lastPublishUs_ = memory.lastPublishUs;
    publishStaticValues(std::max(nt_.nowUs(), lastPublishUs_));
    return true;
}

// What a robot can read before the first frame: the camera is there, idle, with its calibration.
void PhotonPublisher::publishStaticValues(std::int64_t timeUs) {
    lastPublishUs_ = timeUs;
    nt_.setInteger(topics_.heartbeat, timeUs, heartbeat_);
    nt_.setInteger(topics_.pipelineIndexState, timeUs, 0);
    nt_.setBoolean(topics_.driverMode, timeUs, false);
    nt_.setInteger(topics_.fpsLimit, timeUs, kNoFpsLimit);
    nt_.setDouble(topics_.latencyMillis, timeUs, 0.0);
    nt_.setDouble(topics_.fps, timeUs, 0.0);
    nt_.setBoolean(topics_.hasTarget, timeUs, false);
    for (const NtPublisher& scalar : {topics_.targetPitch, topics_.targetYaw, topics_.targetArea, topics_.targetSkew,
                                      topics_.targetPixelsX, topics_.targetPixelsY}) {
        nt_.setDouble(scalar, timeUs, 0.0);
    }
    if (codec_->encode("Transform3d", photonTransform3d(sdk::Pose3{}), poseBytes_).ok()) {
        nt_.setRaw(topics_.targetPose, timeUs, poseBytes_);
    }
    // Row-major camera matrix, then k1 k2 p1 p2 k3.
    const std::array<double, 9> intrinsics = {setup_.fxPx, 0.0, setup_.cxPx, 0.0, setup_.fyPx,
                                              setup_.cyPx, 0.0, 0.0,         1.0};
    nt_.setDoubleArray(topics_.cameraIntrinsics, timeUs, intrinsics);
    nt_.setDoubleArray(topics_.cameraDistortion, timeUs, setup_.distortion);
}

void PhotonPublisher::pollRequests() {
    inbox_.clear();
    nt_.read(pipelineRequest_, inbox_);
    std::optional<std::int32_t> pipeline;
    for (const NtValue& value : inbox_) {
        if (numeric(value) && value.number >= 0.0)
            pipeline = toInt32(value.number);  // PhotonVision ignores a negative index
    }
    if (pipeline) {
        pipelineIndex_ = pipeline;
        if (!inert_) nt_.setInteger(topics_.pipelineIndexState, nt_.nowUs(), *pipeline);
    }

    inbox_.clear();
    nt_.read(driverModeRequest_, inbox_);
    std::optional<bool> driver;
    for (const NtValue& value : inbox_) {
        if (numeric(value)) driver = value.number != 0.0;
    }
    if (driver) {
        driverMode_ = driver;
        if (!inert_) nt_.setBoolean(topics_.driverMode, nt_.nowUs(), *driver);
    }

    inbox_.clear();
    nt_.read(fpsLimitRequest_, inbox_);
    std::optional<std::int32_t> fpsLimit;
    for (const NtValue& value : inbox_) {
        if (numeric(value)) fpsLimit = toInt32(value.number);
    }
    if (fpsLimit) {
        fpsLimit_ = static_cast<double>(*fpsLimit);
        if (!inert_) nt_.setInteger(topics_.fpsLimit, nt_.nowUs(), *fpsLimit);
    }
}

void PhotonPublisher::poll() {
    if (!started_) return;
    // The robot code uses this camera once its PhotonCamera created the request topics (nothing of ours is a request).
    if (!present_) {
        for (const std::string& topic : requestTopics_) present_ = present_ || nt_.hasForeignPublisher(topic);
    }
    pollRequests();
    shared_->poll(nt_);
}

void PhotonPublisher::publish(const sdk::VisionFrame& frame) {
    if (!publishing() || frame.cameraId != setup_.cameraId || !nt_.available()) return;

    // ONE clock reading stamps every value of the frame, never older than the previous frame (ntcore silently
    // drops a value that is older than its topic's last one).
    const std::int64_t publishUs = std::max(nt_.nowUs(), lastPublishUs_);
    const std::int64_t latencyUs = std::max<std::int64_t>(frame.latencyUs, 0);
    // A PhotonVision camera in driver mode runs no pipeline: it still reports a result, with nothing in it.
    const bool detecting = !driverMode_.value_or(false);
    const PhotonResultTiming timing{publishUs - latencyUs, publishUs, kPhotonTimeSinceLastPongUs};
    const PhotonValue result = mapPhotonResult(frame, timing, detecting);
    if (const sdk::Status encoded = codec_->encode(kPhotonResultMessage, result, packet_); !encoded.ok()) {
        if (!encodeFailureLogged_) {
            encodeFailureLogged_ = true;
            log_.warn(kLog, "PhotonVision camera '" + setup_.ntName + "': a frame cannot be encoded and is skipped (" +
                                encoded.error().message + ")");
        }
        return;
    }
    lastPublishUs_ = publishUs;

    const std::vector<PhotonValue>& targets = result.find("targets")->children();
    const PhotonValue* best = targets.empty() ? nullptr : &targets.front();
    const auto scalar = [&](std::string_view name) {
        const PhotonValue* value = best != nullptr ? best->find(name) : nullptr;
        return value != nullptr ? value->realValue() : 0.0;
    };

    nt_.setRaw(topics_.result, publishUs, packet_);
    nt_.setDouble(topics_.latencyMillis, publishUs, static_cast<double>(latencyUs) / 1000.0);
    nt_.setDouble(topics_.fps, publishUs, setup_.frameRateHz);
    nt_.setBoolean(topics_.hasTarget, publishUs, best != nullptr);
    nt_.setDouble(topics_.targetPitch, publishUs, scalar("pitch"));
    nt_.setDouble(topics_.targetYaw, publishUs, scalar("yaw"));
    nt_.setDouble(topics_.targetArea, publishUs, scalar("area"));
    nt_.setDouble(topics_.targetSkew, publishUs, scalar("skew"));
    const sdk::PixelPoint center = best != nullptr ? frame.observations.front().centerPx : sdk::PixelPoint{};
    nt_.setDouble(topics_.targetPixelsX, publishUs, center.x);
    nt_.setDouble(topics_.targetPixelsY, publishUs, center.y);
    const PhotonValue pose = best != nullptr ? *best->find("bestCameraToTarget") : photonTransform3d(sdk::Pose3{});
    if (codec_->encode("Transform3d", pose, poseBytes_).ok()) nt_.setRaw(topics_.targetPose, publishUs, poseBytes_);

    // The heartbeat is the pipeline's sequence number, written last (PhotonVision NTDataPublisher). max() keeps it
    // changing with every frame even if a caller repeats a sequence: PhotonLib's isConnected() needs a change at
    // least every 0.5 s.
    heartbeat_ = std::max(static_cast<std::int64_t>(frame.sequence), heartbeat_ + 1);
    nt_.setInteger(topics_.heartbeat, publishUs, heartbeat_);
    shared_->remember(table_, {heartbeat_, lastPublishUs_});
    ++framesPublished_;
}

sdk::CameraRobotState PhotonPublisher::robotState() const {
    sdk::CameraRobotState state;
    state.ntName = setup_.ntName;
    state.vendor = sdk::CameraVendor::PhotonVision;
    state.cameraId = setup_.cameraId;
    state.present = present_;
    state.pipelineIndex = pipelineIndex_;
    state.driverMode = driverMode_;
    state.fpsLimit = fpsLimit_;
    state.ledMode = shared_->ledMode();
    state.publishing = publishing();
    state.status = status_;
    state.framesPublished = framesPublished_;
    return state;
}

}  // namespace wpiemu::bridges::vision
