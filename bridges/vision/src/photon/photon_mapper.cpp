#include "wpiemu/bridges/vision/photon_mapper.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace wpiemu::bridges::vision {

namespace {

constexpr double kDegreesPerRadian = 180.0 / std::numbers::pi;

// PhotonVision v2026.3.4 TrackedTarget: AprilTag detections keep the default skew of 0 and class / confidence of -1.
constexpr double kAprilTagSkewDeg = 0.0;
constexpr std::int64_t kNoObjectClass = -1;
constexpr double kNoObjectConfidence = -1.0;

PhotonValue corner(const sdk::PixelPoint& point) {
    PhotonValue value = PhotonValue::structure();
    value.set("x", PhotonValue::real(point.x)).set("y", PhotonValue::real(point.y));
    return value;
}

PhotonValue cornerList(const std::array<sdk::PixelPoint, 4>& corners) {
    PhotonValue list = PhotonValue::list();
    for (const sdk::PixelPoint& point : corners) list.push(corner(point));
    return list;
}

PhotonValue mapTarget(const sdk::FiducialObservation& observation) {
    // PhotonVision corrects the pitch for the target's yaw (photon-core TargetCalculations.calculateYawPitch, v2026.3.4):
    // pitch = atan((cy - v) / (fy / cos(yaw))), the true elevation of the target above the optical axis plane. The engine's
    // pitch is atan((cy - v) / fy) (spec B, shared with Limelight's ty), which is larger for any off-axis target; the two
    // agree on the vertical centre line.
    const double pitchRad = std::atan(std::tan(observation.pitchRad) * std::cos(observation.yawRad));
    PhotonValue target = PhotonValue::structure();
    target.set("yaw", PhotonValue::real(observation.yawRad * kDegreesPerRadian))
        .set("pitch", PhotonValue::real(pitchRad * kDegreesPerRadian))
        .set("area", PhotonValue::real(observation.areaPct))
        .set("skew", PhotonValue::real(kAprilTagSkewDeg))
        .set("fiducialId", PhotonValue::integer(observation.id))
        .set("objDetectId", PhotonValue::integer(kNoObjectClass))
        .set("objDetectConf", PhotonValue::real(kNoObjectConfidence))
        .set("bestCameraToTarget", photonTransform3d(observation.cameraToTagBest))
        .set("altCameraToTarget", photonTransform3d(observation.cameraToTagAlt))
        .set("poseAmbiguity", PhotonValue::real(observation.ambiguity))
        .set("minAreaRectCorners", cornerList(observation.cornersPx))
        .set("detectedCorners", cornerList(observation.cornersPx));
    return target;
}

PhotonValue mapMultiTag(const sdk::MultiTagSolution& solution) {
    const PhotonValue pose = photonTransform3d(solution.fieldToCamera);
    PhotonValue estimated = PhotonValue::structure();
    estimated.set("best", pose)
        .set("alt", pose)
        .set("bestReprojErr", PhotonValue::real(solution.reprojErrPx))
        .set("altReprojErr", PhotonValue::real(solution.reprojErrPx))
        .set("ambiguity", PhotonValue::real(0.0));

    PhotonValue ids = PhotonValue::list();
    const std::size_t count = std::min(solution.tagIds.size(), kPhotonMaxListLength);
    for (std::size_t i = 0; i < count; ++i) ids.push(PhotonValue::integer(solution.tagIds[i]));

    PhotonValue multi = PhotonValue::structure();
    multi.set("estimatedPose", std::move(estimated)).set("fiducialIDsUsed", std::move(ids));
    return multi;
}

}  // namespace

PhotonValue photonTransform3d(double x, double y, double z, double qw, double qx, double qy, double qz) {
    PhotonValue translation = PhotonValue::structure();
    translation.set("x", PhotonValue::real(x)).set("y", PhotonValue::real(y)).set("z", PhotonValue::real(z));
    PhotonValue quaternion = PhotonValue::structure();
    quaternion.set("w", PhotonValue::real(qw))
        .set("x", PhotonValue::real(qx))
        .set("y", PhotonValue::real(qy))
        .set("z", PhotonValue::real(qz));
    PhotonValue rotation = PhotonValue::structure();
    rotation.set("q", std::move(quaternion));
    PhotonValue transform = PhotonValue::structure();
    transform.set("translation", std::move(translation)).set("rotation", std::move(rotation));
    return transform;
}

PhotonValue photonTransform3d(const sdk::Pose3& pose) {
    double w = pose.rotation.w;
    double x = pose.rotation.x;
    double y = pose.rotation.y;
    double z = pose.rotation.z;
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 0.0) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    } else {
        w = 1.0;
    }
    return photonTransform3d(pose.position.x, pose.position.y, pose.position.z, w, x, y, z);
}

PhotonValue mapPhotonResult(const sdk::VisionFrame& frame, const PhotonResultTiming& timing, bool includeTargets) {
    PhotonValue metadata = PhotonValue::structure();
    metadata.set("sequenceID", PhotonValue::integer(static_cast<std::int64_t>(frame.sequence)))
        .set("captureTimestampMicros", PhotonValue::integer(timing.captureUs))
        .set("publishTimestampMicros", PhotonValue::integer(timing.publishUs))
        .set("timeSinceLastPong", PhotonValue::integer(timing.timeSinceLastPongUs));

    PhotonValue targets = PhotonValue::list();
    if (includeTargets) {
        const std::size_t count = std::min(frame.observations.size(), kPhotonMaxListLength);
        for (std::size_t i = 0; i < count; ++i) targets.push(mapTarget(frame.observations[i]));
    }

    PhotonValue result = PhotonValue::structure();
    result.set("metadata", std::move(metadata))
        .set("targets", std::move(targets))
        .set("multitagResult",
             includeTargets && frame.multiTag.valid ? mapMultiTag(frame.multiTag) : PhotonValue::absent());
    return result;
}

sdk::Status verifyPhotonMapping(const PhotonCodec& codec) {
    sdk::VisionFrame probe;
    probe.sequence = 1;
    probe.latencyUs = 25000;
    for (int id = 1; id <= 2; ++id) {
        sdk::FiducialObservation observation;
        observation.id = id;
        probe.observations.push_back(observation);
    }
    probe.multiTag.valid = true;
    probe.multiTag.tagIds = {1, 2};

    std::vector<std::uint8_t> bytes;
    const sdk::Status status = codec.encode(kPhotonResultMessage, mapPhotonResult(probe, PhotonResultTiming{}), bytes);
    if (!status.ok()) {
        return sdk::makeError(sdk::ErrorCode::Unsupported,
                              "the PhotonVision message definition " + codec.uuid() +
                                  " needs data the bridge cannot provide: " + status.error().message);
    }
    return sdk::okStatus();
}

}  // namespace wpiemu::bridges::vision
