#include "wpiemu/core/vision/vision_system.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "wpiemu/core/vision/pose_solver.h"

namespace wpiemu::core::vision {

using namespace wpiemu::sdk;

namespace {

// One captured frame per period; the accumulator counts micro-Hz * microseconds so that any
// frame rate with up to six decimals keeps the long-run rate exact in integer arithmetic.
constexpr std::int64_t kScheduleUnitsPerFrame = 1'000'000'000'000LL;
constexpr double kMaxFrameRateHz = 240.0;
constexpr std::int64_t kMaxLatencyUs = 60'000'000;
constexpr int kMaxImageSidePx = 16384;
// Stands in for an infinite reprojection error so that frames stay finite.
constexpr double kMaxReprojErrPx = 1e9;

// Box-Muller: two uniform draws give two independent N(0, 1) samples.
std::pair<double, double> gaussianPair(Rng& rng) {
    const double u1 = rng.nextDouble();
    const double u2 = rng.nextDouble();
    const double radius = std::sqrt(-2.0 * std::log(1.0 - u1));  // u1 in [0,1): the log is finite
    const double angle = kTwoPi * u2;
    return {radius * std::cos(angle), radius * std::sin(angle)};
}

bool finiteNonNegative(double v) { return std::isfinite(v) && v >= 0.0; }

// What makes a camera impossible to simulate, "" when fine.
std::string cameraProblem(const CameraConfig& c) {
    if (c.id.empty()) return "the camera id is empty";
    const CameraIntrinsics& in = c.intrinsics;
    if (in.widthPx < 1 || in.heightPx < 1 || in.widthPx > kMaxImageSidePx ||
        in.heightPx > kMaxImageSidePx)
        return "the image size is outside 1.." + std::to_string(kMaxImageSidePx) + " px";
    // The field of view only matters while the focal length is derived from it.
    if (!(in.fxPx > 0.0) &&
        !(std::isfinite(in.hfovRad) && in.hfovRad > 0.0 && in.hfovRad < kPi))
        return "the horizontal field of view must be in (0, pi) rad";
    const double fx = in.fx(), fy = in.fy(), cx = in.cx(), cy = in.cy();
    if (!(std::isfinite(fx) && fx > 0.0 && std::isfinite(fy) && fy > 0.0 && std::isfinite(cx) &&
          std::isfinite(cy)))
        return "the intrinsics are invalid";
    const bool rateOk = std::isfinite(c.frameRateHz) && c.frameRateHz > 0.0 &&
                        c.frameRateHz <= kMaxFrameRateHz;
    if (!rateOk || std::llround(c.frameRateHz * 1e6) < 1)
        return "the frame rate must be in (0, 240] Hz";
    if (c.latencyUs < 0 || c.latencyUs > kMaxLatencyUs || c.captureLatencyUs < 0)
        return "the latency must be in [0, 60 s]";
    if (!finiteNonNegative(c.tagSizeM) || !finiteNonNegative(c.minTagPixels) ||
        !(std::isfinite(c.maxViewAngleRad) && c.maxViewAngleRad > 0.0 && c.maxViewAngleRad <= kPi))
        return "the tag parameters are invalid";
    const CameraRealism& r = c.realism;
    if (!finiteNonNegative(r.cornerNoisePx) || r.latencyJitterUs < 0 ||
        r.latencyJitterUs > kMaxLatencyUs ||
        !(std::isfinite(r.dropoutProbability) && r.dropoutProbability >= 0.0 &&
          r.dropoutProbability <= 1.0))
        return "the realism parameters are invalid";
    return {};
}

}  // namespace

VisionSystem::~VisionSystem() = default;

Result<std::unique_ptr<VisionSystem>> VisionSystem::create(
    const VisionConfig& config, FiducialLayout layout, SimTimeUs stepUs,
    const IMechanismDirectory* startMechanisms) {
    if (stepUs <= 0)
        return makeError(ErrorCode::InvalidArgument,
                         "vision: the simulation step must be positive");

    std::unique_ptr<VisionSystem> system(new VisionSystem());
    system->config_ = config;
    system->layout_ = std::move(layout);
    system->stepUs_ = stepUs;

    std::size_t enabled = 0;
    for (std::size_t i = 0; i < config.cameras.size(); ++i) {
        const CameraConfig& c = config.cameras[i];
        if (const std::string problem = cameraProblem(c); !problem.empty())
            return makeError(ErrorCode::InvalidArgument,
                             "vision camera '" + c.id + "': " + problem);
        for (std::size_t j = 0; j < i; ++j)
            if (config.cameras[j].id == c.id)
                return makeError(ErrorCode::InvalidArgument,
                                 "vision: duplicate camera id '" + c.id + "'");
        if (c.enabled) ++enabled;
    }
    if (enabled > kMaxEnabledCameras)
        return makeError(ErrorCode::InvalidArgument,
                         "vision: at most " + std::to_string(kMaxEnabledCameras) +
                             " enabled cameras are supported");

    system->tags_.reserve(system->layout_.tags.size());
    for (const FiducialTag& tag : system->layout_.tags) {
        TagRecord record;
        record.tag = tag;
        record.pose = fromSdk(tag.pose);
        record.sizeM = tag.sizeM > 0.0 ? tag.sizeM : system->layout_.tagSizeM;
        system->tags_.push_back(std::move(record));
    }

    system->cameras_.reserve(config.cameras.size());
    for (const CameraConfig& c : config.cameras) {
        Camera cam;
        cam.config = c;
        cam.model = CameraModel(c.intrinsics);
        cam.limits = {c.minTagPixels, c.maxViewAngleRad, c.occlusion};
        cam.rateMicroHz = std::llround(c.frameRateHz * 1e6);
        cam.startRobotToCamera = toSdk(system->placeCamera(c, startMechanisms).chassisToCamera);
        system->cameras_.push_back(std::move(cam));
    }
    return Result<std::unique_ptr<VisionSystem>>(std::move(system));
}

std::size_t VisionSystem::enabledCameraCount() const {
    return static_cast<std::size_t>(std::count_if(
        cameras_.begin(), cameras_.end(), [](const Camera& c) { return c.config.enabled; }));
}

VisionSystem::Placement VisionSystem::placeCamera(const CameraConfig& config,
                                                  const IMechanismDirectory* mechanisms) const {
    Placement placement;
    Pose3d link;  // the chassis frame
    if (!config.attachment.onChassis()) {
        std::optional<Pose3> resolved;
        if (mechanisms != nullptr)
            resolved = mechanisms->linkPoseInChassis(config.attachment.parentMechanismId,
                                                     config.attachment.parentLink);
        if (resolved)
            link = fromSdk(*resolved);
        else
            placement.attachmentResolved = false;
    }
    placement.chassisToCamera = link * fromSdk(config.local);
    return placement;
}

void VisionSystem::step(const VisionWorldView& view, Rng& rng, SimTimeUs simTimeAfterStepUs) {
    std::optional<Pose3d> chassis;  // read from the world only when a capture is due
    for (Camera& cam : cameras_) {
        if (!cam.config.enabled) continue;
        cam.accumulator += stepUs_ * cam.rateMicroHz;
        if (cam.accumulator < kScheduleUnitsPerFrame) continue;
        // One capture per tick at most: a rate above the tick rate is capped, not caught up later.
        cam.accumulator =
            std::min(cam.accumulator - kScheduleUnitsPerFrame, kScheduleUnitsPerFrame);
        if (!chassis) chassis = fromSdk(view.world.pose(view.chassisBody));
        const Placement placement = placeCamera(cam.config, view.mechanisms);
        capture(cam, *chassis * placement.chassisToCamera, placement.chassisToCamera, view, rng,
                simTimeAfterStepUs);
    }
}

void VisionSystem::capture(Camera& cam, const Pose3d& fieldToCamera, const Pose3d& chassisToCamera,
                           const VisionWorldView& view, Rng& rng, SimTimeUs captureTimeUs) {
    const CameraConfig& cfg = cam.config;
    VisionFrame frame;
    frame.cameraId = cfg.id;
    frame.vendor = cfg.vendor;
    frame.ntName = cfg.networkName();
    frame.sequence = ++cam.sequence;
    frame.captureTimeUs = captureTimeUs;
    frame.robotToCamera = toSdk(chassisToCamera);

    std::int64_t latencyUs = cfg.latencyUs;
    if (cfg.realism.latencyJitterUs > 0) {
        const auto jitter = static_cast<double>(cfg.realism.latencyJitterUs);
        latencyUs += std::llround(rng.uniform(-jitter, jitter));
    }
    latencyUs = std::max<std::int64_t>(latencyUs, 0);
    SimTimeUs publishAtUs = captureTimeUs + (latencyUs + stepUs_ - 1) / stepUs_ * stepUs_;
    if (cam.lastPublishAtUs && publishAtUs < *cam.lastPublishAtUs)
        publishAtUs = *cam.lastPublishAtUs;  // never overtake the previous frame
    cam.lastPublishAtUs = publishAtUs;
    frame.latencyUs = publishAtUs - captureTimeUs;
    frame.captureLatencyUs = std::min(cfg.captureLatencyUs, frame.latencyUs);

    const bool dropped = cfg.realism.dropoutProbability > 0.0 &&
                         rng.nextDouble() < cfg.realism.dropoutProbability;
    if (!dropped) detect(cam, fieldToCamera, view.world, rng, frame);

    cam.visibleTagIds.clear();
    for (const FiducialObservation& o : frame.observations) cam.visibleTagIds.push_back(o.id);
    cam.queue.push_back({publishAtUs, std::move(frame)});
}

void VisionSystem::detect(const Camera& cam, const Pose3d& fieldToCamera,
                          const IPhysicsWorld& world, Rng& rng, VisionFrame& frame) const {
    const CameraConfig& cfg = cam.config;
    struct Detection {
        const TagRecord* tag = nullptr;
        double sizeM = 0.0;
        std::array<PixelPoint, 4> corners{};
        SquarePnpResult pnp;
        FiducialObservation observation;
    };

    std::vector<Detection> found;
    for (const TagRecord& tag : tags_) {
        Detection d;
        d.tag = &tag;
        d.sizeM = cfg.tagSizeM > 0.0 ? cfg.tagSizeM : tag.sizeM;
        const TagVisibility visibility = evaluateTagVisibility(cam.model, cam.limits, fieldToCamera,
                                                               tag.pose, d.sizeM, &world);
        if (!visibility.visible()) continue;
        d.corners = visibility.cornersPx;
        found.push_back(std::move(d));
    }
    if (found.empty()) return;

    const double sigma = cfg.realism.cornerNoisePx;
    if (sigma > 0.0) {
        for (Detection& d : found) {
            for (PixelPoint& corner : d.corners) {
                const auto [noiseX, noiseY] = gaussianPair(rng);
                corner.x += sigma * noiseX;
                corner.y += sigma * noiseY;
            }
        }
    }

    const double imageArea = static_cast<double>(cam.model.widthPx()) * cam.model.heightPx();
    std::vector<Detection> detected;
    detected.reserve(found.size());
    for (Detection& d : found) {
        d.pnp = solveSquarePnp(cam.model, d.sizeM, d.corners);
        if (!d.pnp.valid) continue;
        FiducialObservation& o = d.observation;
        o.id = d.tag->tag.id;
        o.cornersPx = d.corners;
        o.centerPx = quadCenterPx(d.corners);
        o.yawRad = cam.model.yawRad(o.centerPx);
        o.pitchRad = cam.model.pitchRad(o.centerPx);
        // In-plane rotation of the tag in the image, counter-clockwise positive (the bottom edge).
        o.skewRad = std::atan2(-(d.corners[1].y - d.corners[0].y), d.corners[1].x - d.corners[0].x);
        o.areaPct = 100.0 * polygonAreaPx(d.corners) / imageArea;
        o.distanceM = norm(d.pnp.best.cameraToTag.t);
        o.fieldToTag = d.tag->tag.pose;
        o.cameraToTagBest = toSdk(d.pnp.best.cameraToTag);
        o.cameraToTagAlt = toSdk(d.pnp.alt.cameraToTag);
        o.ambiguity = d.pnp.ambiguity;
        o.reprojErrBestPx = std::min(d.pnp.best.reprojErrPx, kMaxReprojErrPx);
        o.reprojErrAltPx = std::min(d.pnp.alt.reprojErrPx, kMaxReprojErrPx);
        detected.push_back(std::move(d));
    }

    std::stable_sort(detected.begin(), detected.end(), [](const Detection& a, const Detection& b) {
        if (a.observation.areaPct != b.observation.areaPct)
            return a.observation.areaPct > b.observation.areaPct;
        return a.observation.id < b.observation.id;
    });
    if (detected.size() > kMaxObservationsPerFrame)
        detected.erase(detected.begin() + static_cast<std::ptrdiff_t>(kMaxObservationsPerFrame),
                       detected.end());
    for (const Detection& d : detected) frame.observations.push_back(d.observation);

    if (cfg.multiTag && detected.size() >= 2) {
        std::vector<TagCornerObservation> corners;
        std::vector<SquarePnpResult> seeds;
        for (const Detection& d : detected) {
            corners.push_back({d.tag->pose, d.sizeM, d.corners});
            seeds.push_back(d.pnp);
        }
        const MultiTagPnpResult solution = solveMultiTagPnp(cam.model, corners, seeds);
        if (solution.valid) {
            frame.multiTag.valid = true;
            frame.multiTag.fieldToCamera = toSdk(solution.fieldToCamera);
            frame.multiTag.reprojErrPx = solution.reprojErrPx;
            for (const Detection& d : detected) frame.multiTag.tagIds.push_back(d.observation.id);
        }
    }
}

void VisionSystem::collectDue(SimTimeUs sensorFrameSimTimeUs, std::vector<VisionFrame>& out) {
    const SimTimeUs publishTimeUs = sensorFrameSimTimeUs - stepUs_;
    while (out.size() < kMaxFramesPerSensorFrame) {
        Camera* next = nullptr;
        for (Camera& cam : cameras_) {
            if (cam.queue.empty() || cam.queue.front().publishAtUs > publishTimeUs) continue;
            if (next == nullptr ||
                cam.queue.front().frame.captureTimeUs < next->queue.front().frame.captureTimeUs)
                next = &cam;
        }
        if (next == nullptr) break;
        PendingFrame& pending = next->queue.front();
        // Equal to the planned latency, unless the per-message cap held the frame back.
        pending.frame.latencyUs = publishTimeUs - pending.frame.captureTimeUs;
        next->lastLatencyMs = static_cast<double>(pending.frame.latencyUs) / 1000.0;
        ++next->published;
        out.push_back(std::move(pending.frame));
        next->queue.pop_front();
    }
}

void VisionSystem::reset() {
    for (Camera& cam : cameras_) {
        cam.queue.clear();
        cam.accumulator = 0;
        cam.lastPublishAtUs.reset();
        cam.visibleTagIds.clear();
        cam.lastLatencyMs = 0.0;
    }
}

VisionSetup VisionSystem::setup(std::string_view defaultPhotonUuid) const {
    const std::string uuid = !config_.photonMessageUuid.empty() ? config_.photonMessageUuid
                                                                : std::string(defaultPhotonUuid);
    VisionSetup setup;
    for (const Camera& cam : cameras_) {
        if (!cam.config.enabled) continue;
        CameraSetup c;
        c.cameraId = cam.config.id;
        c.vendor = cam.config.vendor;
        c.ntName = cam.config.networkName();
        c.widthPx = cam.model.widthPx();
        c.heightPx = cam.model.heightPx();
        c.fxPx = cam.model.fxPx();
        c.fyPx = cam.model.fyPx();
        c.cxPx = cam.model.cxPx();
        c.cyPx = cam.model.cyPx();
        c.distortion = cam.config.intrinsics.distortion;
        c.frameRateHz = cam.config.frameRateHz;
        c.latencyUs = cam.config.latencyUs;
        c.defaultAssumedRobotToCamera =
            cam.config.assumedPose ? *cam.config.assumedPose : cam.startRobotToCamera;
        c.photonMessageUuid = uuid;
        setup.cameras.push_back(std::move(c));
    }
    return setup;
}

void VisionSystem::contribute(SnapshotBuilder& out, const VisionWorldView& view,
                              const RobotCameraStates& robotStates) const {
    const Pose3d chassis = fromSdk(view.world.pose(view.chassisBody));
    for (const Camera& cam : cameras_) {
        const Placement placement = placeCamera(cam.config, view.mechanisms);
        const CameraRobotState* robot = robotStates.forCamera(cam.config.id);
        CameraSample s;
        s.id = cam.config.id;
        s.label = cam.config.label;
        s.vendor = cam.config.vendor;
        s.enabled = cam.config.enabled;
        s.pose = toSdk(chassis * placement.chassisToCamera);
        s.hfovRad = cam.model.hfovRad();
        s.vfovRad = cam.model.vfovRad();
        s.widthPx = cam.model.widthPx();
        s.heightPx = cam.model.heightPx();
        s.visibleTagIds = cam.visibleTagIds;
        s.framesCaptured = cam.sequence;
        s.framesPublished = cam.published;
        s.lastLatencyMs = cam.lastLatencyMs;
        s.robotConnected = robot != nullptr && robot->publishing;
        if (!cam.config.enabled)
            s.status = "disabled";
        else if (robot != nullptr && !robot->status.empty())
            s.status = robot->status;
        else if (!placement.attachmentResolved)
            s.status = "attachment_unresolved";
        out.addCamera(std::move(s));
    }
    for (const TagRecord& tag : tags_)
        out.addFiducial(FiducialSample{tag.tag.id, tag.tag.pose, static_cast<float>(tag.sizeM)});
}

}  // namespace wpiemu::core::vision
