#include "wpiemu/core/runtime/vision_system.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::runtime {

using namespace wpiemu::sdk;

namespace {

constexpr double kMinDepthM = 1e-3;  // corners closer than this to the image plane are clamped instead of projected through it

double gaussian(Rng& rng) {
    const double u1 = 1.0 - rng.nextDouble();  // (0, 1]
    const double u2 = rng.nextDouble();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

SimTimeUs periodUs(double fps) { return std::max<SimTimeUs>(1, frequencyToUs(fps)); }

double distance(const Vec3& a, const Vec3& b) { return static_cast<double>(length(a - b)); }

}  // namespace

VisionSystem::VisionSystem(const RobotConfig& config, AprilTagLayout layout)
    : layout_(std::move(layout)),
      robotOriginInChassis_(-config.chassis.comOffsetM.x, -config.chassis.comOffsetM.y, 0.f) {
    cameras_.reserve(config.cameras.size());
    for (const CameraConfig& cfg : config.cameras) {
        CameraRuntime cam;
        cam.config = cfg;
        cam.periodUs = periodUs(cfg.fps);
        cam.latencyUs = std::max<SimTimeUs>(0, msToUs(cfg.latencyMs));
        cameras_.push_back(std::move(cam));
    }
}

VisionSetup VisionSystem::setup() const {
    VisionSetup s;
    s.layoutName = layout_.name;
    s.fieldLengthM = static_cast<double>(layout_.fieldLengthM);
    s.fieldWidthM = static_cast<double>(layout_.fieldWidthM);
    for (const CameraRuntime& cam : cameras_) s.cameras.push_back(CameraSetup{cam.config});
    return s;
}

void VisionSystem::reset() {
    pending_.clear();
    for (CameraRuntime& cam : cameras_) {
        cam.nextCaptureUs = 0;
        cam.sequence = 0;
        cam.captured = false;
        cam.lastPose = Pose3{};
        cam.lastTagIds.clear();
    }
}

void VisionSystem::step(SimTimeUs simTimeUs, const Pose3& chassisPose, IMechanismDirectory* mechanisms, Rng& rng) {
    // The robot frame is the WPILib one: the floor point below the hull centre with the chassis yaw only, so pitch/roll of the
    // chassis never shifts the reported robot position.
    const Quat yawOnly = Quat::fromYaw(chassisPose.rotation.yaw());
    const Vec3 hullCentre = chassisPose.position + yawOnly.rotate(robotOriginInChassis_);
    const Pose3 robotFrame{Vec3{hullCentre.x, hullCentre.y, 0.f}, yawOnly};
    for (std::size_t i = 0; i < cameras_.size(); ++i) {
        CameraRuntime& cam = cameras_[i];
        if (simTimeUs < cam.nextCaptureUs) continue;
        const SimTimeUs behind = simTimeUs - cam.nextCaptureUs;
        cam.nextCaptureUs += (behind / cam.periodUs + 1) * cam.periodUs;

        CameraFrame frame = capture(i, simTimeUs, chassisPose, robotFrame, mechanisms, rng);
        const auto at = std::upper_bound(pending_.begin(), pending_.end(), frame.publishTimeUs,
                                         [](SimTimeUs t, const CameraFrame& f) { return t < f.publishTimeUs; });
        pending_.insert(at, std::move(frame));
    }
}

CameraFrame VisionSystem::capture(std::size_t index, SimTimeUs simTimeUs, const Pose3& chassisPose, const Pose3& robotFrame,
                                  IMechanismDirectory* mechanisms, Rng& rng) {
    CameraRuntime& cam = cameras_[index];
    const CameraConfig& cfg = cam.config;

    Pose3 attachment;  // the chassis frame unless attached to a mechanism link that can be resolved
    if (!cfg.attachment.onChassis() && mechanisms != nullptr)
        if (const auto p = mechanisms->linkPoseInChassis(cfg.attachment.parentMechanismId, cfg.attachment.parentLink))
            attachment = *p;
    const Pose3 cameraPose = chassisPose * attachment * cfg.local;
    const Pose3 fieldToCamera = cameraPose.inverse();

    CameraFrame frame;
    frame.cameraIndex = static_cast<std::uint32_t>(index);
    frame.sequence = cam.sequence++;
    frame.captureTimeUs = simTimeUs;
    frame.publishTimeUs = simTimeUs + cam.latencyUs;
    frame.robotToCamera = robotFrame.inverse() * cameraPose;

    const double tanH = std::tan(degToRad(cfg.hFovDeg) * 0.5);
    const double tanV = std::tan(degToRad(cfg.vFovDeg) * 0.5);
    const double maxIncidenceRad = degToRad(cfg.maxIncidenceDeg);
    const double halfW = cfg.widthPx * 0.5;
    const double halfH = cfg.heightPx * 0.5;
    const double fx = halfW / tanH;
    const double fy = halfH / tanV;
    const auto project = [&](const Vec3& p, double& u, double& v) {
        const double depth = std::max(static_cast<double>(p.x), kMinDepthM);
        u = halfW - fx * static_cast<double>(p.y) / depth;
        v = halfH - fy * static_cast<double>(p.z) / depth;
    };

    std::vector<Vec3> fieldPositions;
    for (const AprilTag& tag : layout_.tags) {
        const Pose3 cameraToTag = fieldToCamera * tag.pose;
        const Vec3 p = cameraToTag.position;
        if (!(p.x > 0.f)) continue;
        const double dist = length(p);
        if (dist > cfg.maxRangeM) continue;
        const double right = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));  // + = left
        const double up = std::atan2(static_cast<double>(p.z), static_cast<double>(p.x));
        if (std::abs(std::tan(right)) > tanH || std::abs(std::tan(up)) > tanV) continue;

        const Vec3 normal = tag.pose.transformVector(Vec3{1.f, 0.f, 0.f});
        const Vec3 toCamera = cameraPose.position - tag.pose.position;
        if (!(dot(normal, toCamera) > 0.f)) continue;
        const double cosIncidence = std::clamp(static_cast<double>(dot(normal, toCamera)) / std::max(dist, 1e-9), -1.0, 1.0);
        if (std::acos(cosIncidence) > maxIncidenceRad) continue;

        TagDetection det;
        det.id = tag.id;
        det.cameraToTag = cameraToTag;
        det.yawDeg = -radToDeg(right);
        det.pitchDeg = radToDeg(up);
        det.distanceM = dist;
        const float h = static_cast<float>(tag.sizeM * 0.5);
        // bottom-left, bottom-right, top-right, top-left as seen from the front of the tag
        const Vec3 corners[4] = {{0.f, -h, -h}, {0.f, h, -h}, {0.f, h, h}, {0.f, -h, h}};
        double area2 = 0.0;
        double prevU = 0.0, prevV = 0.0, firstU = 0.0, firstV = 0.0;
        for (int c = 0; c < 4; ++c) {
            double u = 0.0, v = 0.0;
            project(cameraToTag.transformPoint(corners[c]), u, v);
            det.cornersPx[2 * c] = u;
            det.cornersPx[2 * c + 1] = v;
            if (c == 0) {
                firstU = u;
                firstV = v;
            } else {
                area2 += prevU * v - u * prevV;
            }
            prevU = u;
            prevV = v;
        }
        area2 += prevU * firstV - firstU * prevV;
        det.areaPercent = 100.0 * std::abs(area2) * 0.5 / (static_cast<double>(cfg.widthPx) * cfg.heightPx);
        frame.tags.push_back(det);
        fieldPositions.push_back(tag.pose.position);
    }

    cam.captured = true;
    cam.lastPose = cameraPose;
    cam.lastTagIds.clear();
    for (const TagDetection& t : frame.tags) cam.lastTagIds.push_back(t.id);

    if (!frame.tags.empty()) {
        frame.hasRobotPose = true;
        Pose3 estimate = robotFrame;
        if (cfg.positionNoiseM > 0.0) {
            estimate.position.x += static_cast<float>(cfg.positionNoiseM * gaussian(rng));
            estimate.position.y += static_cast<float>(cfg.positionNoiseM * gaussian(rng));
        }
        if (cfg.yawNoiseDeg > 0.0)
            estimate.rotation = Quat::fromYaw(static_cast<float>(degToRad(cfg.yawNoiseDeg) * gaussian(rng))) * estimate.rotation;
        frame.robotPoseField = estimate;

        double distSum = 0.0, areaSum = 0.0;
        for (const TagDetection& t : frame.tags) {
            distSum += t.distanceM;
            areaSum += t.areaPercent;
        }
        frame.avgTagDistM = distSum / static_cast<double>(frame.tags.size());
        frame.avgTagAreaPercent = areaSum / static_cast<double>(frame.tags.size());
        for (std::size_t a = 0; a < fieldPositions.size(); ++a)
            for (std::size_t b = a + 1; b < fieldPositions.size(); ++b)
                frame.tagSpanM = std::max(frame.tagSpanM, distance(fieldPositions[a], fieldPositions[b]));
    }
    return frame;
}

std::vector<CameraFrame> VisionSystem::collectDue(SimTimeUs simTimeUs) {
    std::vector<CameraFrame> due;
    while (!pending_.empty() && pending_.front().publishTimeUs <= simTimeUs) {
        CameraFrame& f = pending_.front();
        if (f.cameraIndex < cameras_.size()) due.push_back(std::move(f));
        pending_.pop_front();
    }
    return due;
}

void VisionSystem::publish(SnapshotBuilder& builder) const {
    for (const CameraRuntime& cam : cameras_) {
        if (!cam.captured) continue;
        CameraState state;
        state.id = cam.config.id;
        state.worldPose = cam.lastPose;
        state.tagsVisible = !cam.lastTagIds.empty();
        state.visibleTagIds = cam.lastTagIds;
        builder.addCamera(std::move(state));
    }
}

}  // namespace wpiemu::core::runtime
