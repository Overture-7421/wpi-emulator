#pragma once

// Pinhole camera model of the vision engine (ADR 0017, contract in sdk/vision/camera_config.h).
//
//  * Camera frame (vendor neutral): +X forward (optical axis), +Y left, +Z up.
//  * Image: continuous pixels, origin at the top-left corner, x right, y down, spanning
//    [0,w] x [0,h]. A camera-frame point (x, y, z) with x > 0 projects to
//        u = cx - fx * (y / x),    v = cy - fy * (z / x).
//  * Lens distortion is NOT applied in v1 (CameraIntrinsics::distortion is only published).
//  * Yaw is positive for a target to the RIGHT of the optical axis, pitch positive for UP:
//        yaw = atan((u - cx) / fx),   pitch = -atan((v - cy) / fy).

#include <optional>
#include <span>

#include "wpiemu/core/vision/vision_math.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::core::vision {

class CameraModel {
public:
    // A point closer than this to the image plane (camera-frame X, metres) is not projected.
    static constexpr double kMinDepthM = 1e-3;

    CameraModel() = default;
    // Derives fx, fy, cx, cy from hfovRad unless the explicit values are set.
    explicit CameraModel(const sdk::CameraIntrinsics& intrinsics);

    int widthPx() const { return width_; }
    int heightPx() const { return height_; }
    double fxPx() const { return fx_; }
    double fyPx() const { return fy_; }
    double cxPx() const { return cx_; }
    double cyPx() const { return cy_; }
    double hfovRad() const;
    double vfovRad() const;

    // nullopt when the point is not in front of the camera (x < kMinDepthM).
    std::optional<sdk::PixelPoint> project(const Vec3d& pointInCamera) const;
    // The camera-frame point on the pixel's ray whose X (depth along the optical axis) is `depthM`.
    Vec3d unproject(const sdk::PixelPoint& px, double depthM) const;
    // Unit direction (camera frame) of the pixel's ray.
    Vec3d rayDirection(const sdk::PixelPoint& px) const;

    // The pixel lies inside [0,w] x [0,h] (edges included).
    bool inImage(const sdk::PixelPoint& px) const;
    double yawRad(const sdk::PixelPoint& px) const;
    double pitchRad(const sdk::PixelPoint& px) const;

private:
    int width_ = 0;
    int height_ = 0;
    double fx_ = 1.0;
    double fy_ = 1.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
};

// Projects every point; returns false (and leaves `out` partly written) as soon as one point is not
// in front of the camera. `out` must have at least as many elements as `pointsInCamera`.
bool projectPolygon(const CameraModel& camera, std::span<const Vec3d> pointsInCamera,
                    std::span<sdk::PixelPoint> out);

// Absolute area of a simple polygon (shoelace), in px^2.
double polygonAreaPx(std::span<const sdk::PixelPoint> polygon);

// Length of the shortest edge of a closed polygon, in px (0 for fewer than two points).
double shortestEdgePx(std::span<const sdk::PixelPoint> polygon);

// Intersection of the diagonals 0-2 and 1-3 of a quad: the image of a planar quad's centre. Falls
// back to the corner average when the diagonals are parallel.
sdk::PixelPoint quadCenterPx(std::span<const sdk::PixelPoint, 4> corners);

}  // namespace wpiemu::core::vision
