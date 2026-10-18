#include "wpiemu/core/vision/camera_model.h"

#include <cmath>

namespace wpiemu::core::vision {

CameraModel::CameraModel(const sdk::CameraIntrinsics& intrinsics)
    : width_(intrinsics.widthPx),
      height_(intrinsics.heightPx),
      fx_(intrinsics.fx()),
      fy_(intrinsics.fy()),
      cx_(intrinsics.cx()),
      cy_(intrinsics.cy()) {}

double CameraModel::hfovRad() const { return 2.0 * std::atan(0.5 * width_ / fx_); }
double CameraModel::vfovRad() const { return 2.0 * std::atan(0.5 * height_ / fy_); }

std::optional<sdk::PixelPoint> CameraModel::project(const Vec3d& p) const {
    if (!(p.x >= kMinDepthM)) return std::nullopt;
    return sdk::PixelPoint{cx_ - fx_ * (p.y / p.x), cy_ - fy_ * (p.z / p.x)};
}

Vec3d CameraModel::unproject(const sdk::PixelPoint& px, double depthM) const {
    return {depthM, -(px.x - cx_) / fx_ * depthM, -(px.y - cy_) / fy_ * depthM};
}

Vec3d CameraModel::rayDirection(const sdk::PixelPoint& px) const {
    return normalized(unproject(px, 1.0));
}

bool CameraModel::inImage(const sdk::PixelPoint& px) const {
    return px.x >= 0.0 && px.x <= width_ && px.y >= 0.0 && px.y <= height_;
}

double CameraModel::yawRad(const sdk::PixelPoint& px) const {
    return std::atan((px.x - cx_) / fx_);
}
double CameraModel::pitchRad(const sdk::PixelPoint& px) const {
    return -std::atan((px.y - cy_) / fy_);
}

bool projectPolygon(const CameraModel& camera, std::span<const Vec3d> pointsInCamera,
                    std::span<sdk::PixelPoint> out) {
    for (std::size_t i = 0; i < pointsInCamera.size(); ++i) {
        const auto px = camera.project(pointsInCamera[i]);
        if (!px) return false;
        out[i] = *px;
    }
    return true;
}

double polygonAreaPx(std::span<const sdk::PixelPoint> polygon) {
    double twice = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const sdk::PixelPoint& a = polygon[i];
        const sdk::PixelPoint& b = polygon[(i + 1) % polygon.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return 0.5 * std::abs(twice);
}

double shortestEdgePx(std::span<const sdk::PixelPoint> polygon) {
    if (polygon.size() < 2) return 0.0;
    double shortest = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const sdk::PixelPoint& a = polygon[i];
        const sdk::PixelPoint& b = polygon[(i + 1) % polygon.size()];
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        if (i == 0 || len < shortest) shortest = len;
    }
    return shortest;
}

sdk::PixelPoint quadCenterPx(std::span<const sdk::PixelPoint, 4> c) {
    const double d1x = c[2].x - c[0].x, d1y = c[2].y - c[0].y;
    const double d2x = c[3].x - c[1].x, d2y = c[3].y - c[1].y;
    const double denom = d1x * d2y - d1y * d2x;
    const double scale = std::hypot(d1x, d1y) * std::hypot(d2x, d2y);
    if (!(std::abs(denom) > 1e-12 * scale)) {
        return {0.25 * (c[0].x + c[1].x + c[2].x + c[3].x),
                0.25 * (c[0].y + c[1].y + c[2].y + c[3].y)};
    }
    const double s = ((c[1].x - c[0].x) * d2y - (c[1].y - c[0].y) * d2x) / denom;
    return {c[0].x + s * d1x, c[0].y + s * d1y};
}

}  // namespace wpiemu::core::vision
