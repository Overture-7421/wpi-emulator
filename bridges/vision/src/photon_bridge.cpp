#include "photon_bridge.h"

#include <algorithm>
#include <cmath>

#include "vision_math.h"
#include "wpiemu/bridges/vision/photon_encoding.h"

namespace wpiemu::bridges::vision {

namespace {

constexpr std::int64_t kDistortionCoefficients = 8;  // OPENCV8 form, all zero: the simulation has no lens distortion

PhotonTransform3d toTransform(const sdk::Pose3& p) {
    return {p.position.x, p.position.y,   p.position.z,   p.rotation.w,
            p.rotation.x, p.rotation.y,   p.rotation.z};
}

std::vector<PhotonCorner> toCorners(const std::array<double, 8>& px) {
    return {{px[0], px[1]}, {px[2], px[3]}, {px[4], px[5]}, {px[6], px[7]}};
}

}  // namespace

PhotonBridge::PhotonBridge(std::shared_ptr<INtBackend> backend, VisionBridgeOptions options)
    : NtVisionBridgeBase(std::move(backend), "bridge.vision.photonvision", options) {}

void PhotonBridge::configure(const sdk::VisionSetup& setup) {
    version_ = setup.photonVersion.empty() ? std::string(photonDefaultVersion()) : setup.photonVersion;
    // An unverified version still publishes, with this build's default schema: PhotonLib's own message-definition check
    // is the final guard, and refusing would turn a harmless version bump into a missing camera.
    if (!isVerifiedPhotonVersion(version_) && warnedVersion_ != version_) {
        warnedVersion_ = version_;
        bool anyPhoton = false;
        for (const sdk::CameraSetup& c : setup.cameras) anyPhoton |= c.config.kind == sdk::CameraKind::PhotonVision;
        if (anyPhoton) {
            sink_.warn("PhotonLib " + version_ + " has not been verified with the emulator; publishing the " +
                       std::string(photonSchemaName(photonDefaultSchema())) +
                       " PhotonPipelineResult definition of this WPILib season (best effort). Verified versions are listed in "
                       "the season packs.");
        }
    }

    rootInitialized_ = false;
    configureCameras(setup, sdk::CameraKind::PhotonVision,
                     [](const sdk::CameraConfig& c) { return "/photonvision/" + c.name; });
}

void PhotonBridge::onInitializeCamera(Cam& cam, std::int64_t ntNow) {
    if (!rootInitialized_) {
        rootInitialized_ = true;
        sink_.putString("/photonvision/version", version_, ntNow);
        sink_.putInt("/photonvision/ledModeState", ledMode_, ntNow);
    }
    const sdk::CameraConfig& c = cam.config;
    const double fx = 0.5 * c.widthPx / std::tan(0.5 * c.hFovDeg / sdk::kRadToDeg);
    const double fy = 0.5 * c.heightPx / std::tan(0.5 * c.vFovDeg / sdk::kRadToDeg);
    const double cx = 0.5 * c.widthPx;
    const double cy = 0.5 * c.heightPx;
    sink_.putDoubles(cam.root + "/cameraIntrinsics", {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}, ntNow);
    sink_.putDoubles(cam.root + "/cameraDistortion", NtDoubles(kDistortionCoefficients, 0.0), ntNow);
    sink_.putInt(cam.root + "/pipelineIndexState", cam.pipelineIndex, ntNow);
    sink_.putBool(cam.root + "/driverMode", cam.driverMode, ntNow);
    if (photonSchemaForVersion(version_) == PhotonSchema::Nanoseconds)
        sink_.putBool(cam.root + "/enabled", cam.enabled, ntNow);
    sink_.putInt(cam.root + "/fpsLimit", cam.fpsLimit, ntNow);
    sink_.putDouble(cam.root + "/fps", c.fps, ntNow);
    cam.restartHeartbeat();
    sink_.putInt(cam.root + "/heartbeat", cam.heartbeat, ntNow);
    // The first (empty) result creates the rawBytes topic with its type string, which PhotonLib's version check reads
    // together with the message_uuid property (PacketPublisher sets it as a JSON string).
    publishFrame(cam, nullptr, ntNow);
    const std::string uuidJson = "\"" + std::string(photonPipelineResultUuid(photonSchemaForVersion(version_))) + "\"";
    const sdk::Status property = sink_.backend().setTopicProperty(cam.root + "/rawBytes", "message_uuid", uuidJson);
    if (!property.ok()) sink_.error("could not set message_uuid on " + cam.root + "/rawBytes: " + property.error().message);
}

void PhotonBridge::onPollCamera(Cam& cam, sdk::SimTimeUs simTimeUs, std::int64_t ntNow) {
    if (const auto v = ntAsInt(sink_.read(cam.root + "/pipelineIndexRequest")); v && *v != cam.pipelineIndex) {
        cam.pipelineIndex = *v;
        sink_.putInt(cam.root + "/pipelineIndexState", cam.pipelineIndex, ntNow);
    }
    if (const auto v = ntAsBool(sink_.read(cam.root + "/driverModeRequest")); v && *v != cam.driverMode) {
        cam.driverMode = *v;
        sink_.putBool(cam.root + "/driverMode", cam.driverMode, ntNow);
    }
    if (photonSchemaForVersion(version_) == PhotonSchema::Nanoseconds) {
        if (const auto v = ntAsBool(sink_.read(cam.root + "/enabledRequest")); v && *v != cam.enabled) {
            cam.enabled = *v;
            sink_.putBool(cam.root + "/enabled", cam.enabled, ntNow);
        }
    }
    if (const auto v = ntAsInt(sink_.read(cam.root + "/fpsLimitRequest")); v && *v != cam.fpsLimit) {
        cam.fpsLimit = *v;
        sink_.putInt(cam.root + "/fpsLimit", cam.fpsLimit, ntNow);
    }
    if (cam.beat(simTimeUs, options_.heartbeatPeriodUs)) sink_.putInt(cam.root + "/heartbeat", cam.heartbeat, ntNow);
}

void PhotonBridge::onPollBridge(std::int64_t ntNow) {
    if (const auto v = ntAsInt(sink_.read("/photonvision/ledModeRequest")); v && *v != ledMode_) {
        ledMode_ = *v;
        sink_.putInt("/photonvision/ledModeState", ledMode_, ntNow);
    }
}

void PhotonBridge::onFrame(Cam& cam, const sdk::CameraFrame& frame, std::int64_t ntNow) {
    publishFrame(cam, &frame, ntNow);
}

void PhotonBridge::publishFrame(Cam& cam, const sdk::CameraFrame* frame, std::int64_t ntNow) {
    PhotonResult result;
    double latencyMs = 0.0;
    const bool detect = frame != nullptr && !cam.driverMode && cam.enabled;

    if (frame != nullptr) {
        cam.sequence = static_cast<std::int64_t>(frame->sequence);
        latencyMs = frame->latencyMs();
        result.captureTimestampMicros = ntNow - frame->latencyUs();
    } else {
        ++cam.sequence;
        result.captureTimestampMicros = ntNow;
    }
    result.sequenceId = cam.sequence;
    result.publishTimestampMicros = ntNow;

    if (detect) {
        std::vector<const sdk::TagDetection*> tags;
        tags.reserve(frame->tags.size());
        for (const sdk::TagDetection& t : frame->tags) tags.push_back(&t);
        // Largest target first, like PhotonVision's default target sort.
        std::stable_sort(tags.begin(), tags.end(), [](const sdk::TagDetection* a, const sdk::TagDetection* b) {
            return a->areaPercent > b->areaPercent;
        });
        for (const sdk::TagDetection* t : tags) {
            PhotonTarget target;
            target.yawDeg = t->yawDeg;
            target.pitchDeg = t->pitchDeg;
            target.areaPercent = t->areaPercent;
            target.fiducialId = t->id;
            target.bestCameraToTarget = toTransform(t->cameraToTag);
            target.altCameraToTarget = target.bestCameraToTarget;
            target.poseAmbiguity = t->ambiguity;
            target.minAreaRectCorners = toCorners(t->cornersPx);
            target.detectedCorners = target.minAreaRectCorners;
            result.targets.push_back(std::move(target));
        }
        if (tags.size() >= 2 && frame->hasRobotPose) {
            PhotonMultiTag multi;
            multi.best = toTransform(frame->robotPoseField * frame->robotToCamera);  // field -> camera
            multi.alt = multi.best;
            for (const sdk::TagDetection* t : tags) multi.fiducialIdsUsed.push_back(static_cast<std::int16_t>(t->id));
            std::sort(multi.fiducialIdsUsed.begin(), multi.fiducialIdsUsed.end());
            result.multiTag = std::move(multi);
        }
    }

    const auto schema = photonSchemaForVersion(version_);
    auto encoded = encodePhotonPipelineResult(result, schema);
    if (!encoded.ok()) {
        sink_.error(encoded.error().message);
        return;
    }
    sink_.put(cam.root + "/rawBytes", ntRaw(std::move(encoded).value()), ntNow,
              photonPipelineResultTypeString(schema));

    const bool has = !result.targets.empty();
    const PhotonTarget* best = has ? &result.targets.front() : nullptr;
    sink_.putBool(cam.root + "/hasTarget", has, ntNow);
    sink_.putDouble(cam.root + "/latencyMillis", latencyMs, ntNow);
    sink_.putDouble(cam.root + "/targetYaw", has ? best->yawDeg : 0.0, ntNow);
    sink_.putDouble(cam.root + "/targetPitch", has ? best->pitchDeg : 0.0, ntNow);
    sink_.putDouble(cam.root + "/targetArea", has ? best->areaPercent : 0.0, ntNow);
    sink_.putDouble(cam.root + "/targetSkew", 0.0, ntNow);
    double pixelsX = 0.0, pixelsY = 0.0;
    if (has) {
        for (const PhotonCorner& c : best->detectedCorners) {
            pixelsX += 0.25 * c.x;
            pixelsY += 0.25 * c.y;
        }
    }
    sink_.putDouble(cam.root + "/targetPixelsX", pixelsX, ntNow);
    sink_.putDouble(cam.root + "/targetPixelsY", pixelsY, ntNow);
}

}  // namespace wpiemu::bridges::vision
