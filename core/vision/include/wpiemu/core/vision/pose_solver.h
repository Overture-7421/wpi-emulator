#pragma once

// Pose solving from detected tag corners (ADR 0017). Everything runs in double on the corners as
// given: exact corners (ideal camera) give exact poses, noisy corners (realistic camera) give the
// poses a real detector would estimate from them.
//
//  * solveSquarePnp: one tag, IPPE-square (Collins & Bartoli, "Infinitesimal Plane-Based Pose
//    Estimation", the algorithm behind OpenCV SOLVEPNP_IPPE_SQUARE): a closed-form homography
//    decomposition that yields the TWO poses a planar square can explain. Each is polished by a few
//    Levenberg-Marquardt iterations on its reprojection error (a real detector refines its poses
//    too; it also makes an exact fit read as exact when the tag is seen head-on) and returned with
//    that error. Which one is best, and how ambiguous the choice is, follows from the errors.
//  * solveMultiTagPnp: all tags together, Levenberg-Marquardt over every corner, giving the camera
//    pose in the field frame. Seeds come from the single-tag solutions.
//
// Poses are in the neutral frames of the contract: camera frame X forward / Y left / Z up, tag
// frame per tag_geometry.h. "cameraToTag" is the tag frame expressed in the camera frame.

#include <array>
#include <span>

#include "wpiemu/core/vision/camera_model.h"
#include "wpiemu/core/vision/tag_geometry.h"
#include "wpiemu/core/vision/vision_math.h"

namespace wpiemu::core::vision {

struct SquarePose {
    Pose3d cameraToTag;
    // RMS pixel error per coordinate, sqrt(sum(du^2 + dv^2) / (2 N)) (what OpenCV reports);
    // infinity when a corner falls behind the camera.
    double reprojErrPx = 0.0;
};

struct SquarePnpResult {
    bool valid = false;
    SquarePose best;  // the lower reprojection error
    SquarePose alt;   // the other solution (equal to best when the tag is seen head-on)
    // best / alt reprojection error clamped to [0,1]; 0 when the best error is below 1e-9 px
    // (an exact fit is unambiguous).
    double ambiguity = 0.0;
};

// `cornersPx` in the contract order BL, BR, TR, TL. valid == false for degenerate input (corners
// collinear, behind the camera, non-finite).
SquarePnpResult solveSquarePnp(const CameraModel& camera, double tagSizeM,
                               std::span<const sdk::PixelPoint, 4> cornersPx);

// RMS reprojection error (per coordinate, see SquarePose) of a tag pose against detected corners.
double squareReprojectionErrorPx(const CameraModel& camera, const Pose3d& cameraToTag,
                                 double tagSizeM, std::span<const sdk::PixelPoint, 4> cornersPx);

struct TagCornerObservation {
    Pose3d fieldToTag;
    double sizeM = 0.0;
    std::array<sdk::PixelPoint, 4> cornersPx{};  // BL, BR, TR, TL
};

struct MultiTagPnpResult {
    bool valid = false;
    Pose3d fieldToCamera;     // the camera frame in the field frame
    double reprojErrPx = 0.0;  // RMS per coordinate over all corners of all tags
    int iterations = 0;
};

// Needs at least two tags. `seeds` (optional, one per tag or empty) are the single-tag solutions
// used to start the optimisation; they are computed when absent.
MultiTagPnpResult solveMultiTagPnp(const CameraModel& camera,
                                   std::span<const TagCornerObservation> tags,
                                   std::span<const SquarePnpResult> seeds = {});

}  // namespace wpiemu::core::vision
