#pragma once

// Vendor-neutral VisionFrame -> PhotonVision result message (as a tree of named values for photon_codec.h).
//
// This file is where the PhotonVision conventions meet the neutral frame; the schema texts of the profile decide
// what is written, this decides what the values are. Facts (PhotonVision v2026.3.4, NTDataPublisher /
// TrackedTarget; PhotonLib PhotonPipelineResult; verified against the real PhotonPoseEstimator by the nt spike):
//   * yaw / pitch are degrees, yaw positive for a target to the RIGHT of the optical axis, pitch positive UP;
//     area is the percentage of the image covered by the tag quad;
//   * best / alt are the tag pose in the camera frame (X forward, Y left, Z up); ambiguity is the ratio of their
//     reprojection errors; `corners` are BL, BR, TR, TL (TargetModel.kAprilTag36h11);
//   * for an AprilTag PhotonVision writes skew 0 (TrackedTarget never sets it for a detection), objDetectId -1 and
//     objDetectConf -1: the engine's skew is not published here (a robot must see what real hardware reports);
//   * minAreaRectCorners is the quad itself (PhotonVision reports the OpenCV minimum-area rectangle of the same
//     four corners, in OpenCV's corner order; robots do not read it for AprilTags);
//   * the multi-tag result's `best` is the transform FIELD -> CAMERA, with no alternate solution: alt = best,
//     altReprojErr = bestReprojErr and ambiguity 0 (PhotonLib's own convention); fiducialIDsUsed are int16;
//   * lists carry at most 127 items: the 127 largest targets and the first 127 multi-tag ids are kept.
// A pose is written as a unit quaternion: the engine's float quaternions are normalised in double first, like the
// doubles a real coprocessor sends.

#include <cstdint>

#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::bridges::vision {

// PhotonPipelineMetadata::timeSinceLastPong of a coprocessor that is in sync with its time server: 10 ms (PhotonLib
// warns above 5 s).
inline constexpr std::int64_t kPhotonTimeSinceLastPongUs = 10000;

// The clock fields of PhotonPipelineMetadata, in the clock of the robot process (NT time == HAL FPGA time).
struct PhotonResultTiming {
    std::int64_t captureUs = 0;  // when the image was captured: publishUs - the frame's latency
    std::int64_t publishUs = 0;  // the single NT_Now() reading of the frame
    std::int64_t timeSinceLastPongUs = kPhotonTimeSinceLastPongUs;
};

// The PhotonPipelineResult value tree of one frame. includeTargets = false makes an empty result (no targets, no
// multi-tag), which is what a PhotonVision camera in driver mode publishes.
PhotonValue mapPhotonResult(const sdk::VisionFrame& frame, const PhotonResultTiming& timing,
                            bool includeTargets = true);

// A Transform3d value ({translation: {x, y, z}, rotation: {q: {w, x, y, z}}}). The Pose3 form normalises the
// quaternion (a zero quaternion becomes the identity); the numbers form writes exactly what it is given.
PhotonValue photonTransform3d(const sdk::Pose3& pose);
PhotonValue photonTransform3d(double x, double y, double z, double qw, double qx, double qy, double qz);

// Proves that this mapper can fill every field of the codec's profile: maps a probe frame (two tags and a
// multi-tag solution) and encodes it. Unsupported, naming the missing field, when the profile has a field that
// photon_mapper.cpp does not provide: the camera is then refused instead of publishing a guess.
sdk::Status verifyPhotonMapping(const PhotonCodec& codec);

}  // namespace wpiemu::bridges::vision
