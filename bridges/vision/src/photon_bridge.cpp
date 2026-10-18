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
    : sink_(std::move(backend), "bridge.vision.photonvision"), options_(options) {}

sdk::Status PhotonBridge::start(sdk::ILogger& logger) {
    sink_.setLogger(&logger);
    started_ = true;
    return sdk::okStatus();
}

void PhotonBridge::configure(const sdk::VisionSetup& setup) {
    version_ = setup.photonVersion.empty() ? std::string(kDefaultPhotonVersion) : setup.photonVersion;
    enabled_ = isVerifiedPhotonVersion(version_);
    if (!enabled_ && warnedVersion_ != version_) {
        warnedVersion_ = version_;
        bool anyPhoton = false;
        for (const sdk::CameraSetup& c : setup.cameras) anyPhoton |= c.config.kind == sdk::CameraKind::PhotonVision;
        if (anyPhoton) {
            sink_.error("PhotonLib " + version_ +
                        " has an unverified PhotonPipelineResult definition; PhotonVision cameras are not "
                        "published (verified: v2025.3.2, v2026.1.1 - v2026.3.4, v2026.7)");
        }
    }

    cams_.clear();
    cams_.resize(setup.cameras.size());
    rootInitialized_ = false;
    for (std::size_t i = 0; i < setup.cameras.size(); ++i) {
        const sdk::CameraConfig& c = setup.cameras[i].config;
        if (c.kind != sdk::CameraKind::PhotonVision) continue;
        Cam cam;
        cam.config = c;
        cam.root = "/photonvision/" + c.name;
        cams_[i] = std::move(cam);
    }
}

PhotonBridge::Cam* PhotonBridge::camera(std::uint32_t index) {
    if (!enabled_ || index >= cams_.size() || !cams_[index]) return nullptr;
    return &*cams_[index];
}

// ntcore may not be ready until the robot constructor ran; callers retry on the next tick.
bool PhotonBridge::ensureInitialized(Cam& cam, std::int64_t ntNow) {
    if (cam.initialized) return true;
    if (!sink_.available()) return false;
    cam.initialized = true;

    if (!rootInitialized_) {
        rootInitialized_ = true;
        sink_.putString("/photonvision/version", version_, ntNow);
        sink_.putInt("/photonvision/ledModeState", ledMode_, ntNow);
    }
    const sdk::CameraConfig& c = cam.config;
    const double fx = 0.5 * c.widthPx / std::tan(0.5 * c.hFovDeg / detail::kRadToDeg);
    const double fy = 0.5 * c.heightPx / std::tan(0.5 * c.vFovDeg / detail::kRadToDeg);
    const double cx = 0.5 * c.widthPx;
    const double cy = 0.5 * c.heightPx;
    sink_.putDoubles(cam.root + "/cameraIntrinsics", {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}, ntNow);
    sink_.putDoubles(cam.root + "/cameraDistortion", NtDoubles(kDistortionCoefficients, 0.0), ntNow);
    sink_.putInt(cam.root + "/pipelineIndexState", cam.pipelineIndex, ntNow);
    sink_.putBool(cam.root + "/driverMode", cam.driverMode, ntNow);
    sink_.putInt(cam.root + "/fpsLimit", cam.fpsLimit, ntNow);
    sink_.putDouble(cam.root + "/fps", c.fps, ntNow);
    cam.heartbeat = 0;
    cam.nextHeartbeatUs = 0;
    sink_.putInt(cam.root + "/heartbeat", cam.heartbeat, ntNow);
    // The first (empty) result creates the rawBytes topic with its type string, which PhotonLib's version check reads.
    publishFrame(cam, nullptr, ntNow);
    return true;
}

void PhotonBridge::poll(sdk::SimTimeUs simTimeUs) {
    if (!enabled_ || cams_.empty() || !sink_.available()) return;
    const std::int64_t ntNow = sink_.backend().nowUs();
    bool any = false;
    for (std::optional<Cam>& slot : cams_) {
        if (!slot) continue;
        Cam& cam = *slot;
        if (!ensureInitialized(cam, ntNow)) continue;
        any = true;

        if (const auto v = ntAsInt(sink_.read(cam.root + "/pipelineIndexRequest")); v && *v != cam.pipelineIndex) {
            cam.pipelineIndex = *v;
            sink_.putInt(cam.root + "/pipelineIndexState", cam.pipelineIndex, ntNow);
        }
        if (const auto v = ntAsBool(sink_.read(cam.root + "/driverModeRequest")); v && *v != cam.driverMode) {
            cam.driverMode = *v;
            sink_.putBool(cam.root + "/driverMode", cam.driverMode, ntNow);
        }
        if (const auto v = ntAsInt(sink_.read(cam.root + "/fpsLimitRequest")); v && *v != cam.fpsLimit) {
            cam.fpsLimit = *v;
            sink_.putInt(cam.root + "/fpsLimit", cam.fpsLimit, ntNow);
        }
        if (simTimeUs >= cam.nextHeartbeatUs) {
            ++cam.heartbeat;
            cam.nextHeartbeatUs = simTimeUs + options_.heartbeatPeriodUs;
            sink_.putInt(cam.root + "/heartbeat", cam.heartbeat, ntNow);
        }
    }
    if (any) {
        if (const auto v = ntAsInt(sink_.read("/photonvision/ledModeRequest")); v && *v != ledMode_) {
            ledMode_ = *v;
            sink_.putInt("/photonvision/ledModeState", ledMode_, ntNow);
        }
    }
}

void PhotonBridge::apply(std::span<const sdk::CameraFrame> frames, sdk::SimTimeUs) {
    if (!enabled_ || frames.empty() || !sink_.available()) return;
    const std::int64_t ntNow = sink_.backend().nowUs();
    for (const sdk::CameraFrame& frame : frames) {
        Cam* cam = camera(frame.cameraIndex);
        if (cam == nullptr || !ensureInitialized(*cam, ntNow)) continue;
        publishFrame(*cam, &frame, ntNow);
    }
}

void PhotonBridge::reset() {
    for (std::optional<Cam>& slot : cams_) {
        if (!slot) continue;
        Cam& cam = *slot;
        const bool wasInitialized = cam.initialized;
        cam.initialized = false;
        cam.nextHeartbeatUs = 0;
        // Republish an empty result now when ntcore is up, otherwise on the first available tick.
        if (wasInitialized && enabled_ && sink_.available()) ensureInitialized(cam, sink_.backend().nowUs());
    }
}

void PhotonBridge::stop() noexcept {
    if (started_ && sink_.available()) {
        for (const std::optional<Cam>& slot : cams_) {
            if (slot) sink_.backend().unpublish(slot->root + "/");
        }
    }
    cams_.clear();
    started_ = false;
}

void PhotonBridge::publishFrame(Cam& cam, const sdk::CameraFrame* frame, std::int64_t ntNow) {
    PhotonResult result;
    double latencyMs = 0.0;
    const bool detect = frame != nullptr && !cam.driverMode;

    if (frame != nullptr) {
        cam.sequence = static_cast<std::int64_t>(frame->sequence);
        const sdk::SimTimeUs latencyUs = std::max<sdk::SimTimeUs>(0, frame->publishTimeUs - frame->captureTimeUs);
        latencyMs = latencyUs / 1000.0;
        result.captureTimestampMicros = ntNow - latencyUs;
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

    sink_.put(cam.root + "/rawBytes", ntRaw(encodePhotonPipelineResult(result)), ntNow,
              photonPipelineResultTypeString());

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
