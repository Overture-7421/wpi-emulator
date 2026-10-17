#pragma once

// Gizmo/picking math for the editor viewport. The orbit camera itself is shared with the run view
// (common/orbit_camera.h) and re-exported here for the editor namespace.

#include <cmath>
#include <optional>

#include "common/orbit_camera.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::app::editor {

using app::CameraBasis;
using app::OrbitCamera;
using app::Ray;

// ---- gizmo math ---------------------------------------------------------------------------------
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4723)  // false positive when the parallel case is constant-folded in a caller; the divisor is guarded
#endif
// Parameter along the axis line (point + t * axis) closest to the ray. nullopt when parallel.
inline std::optional<float> closestParamOnAxis(const Ray& ray, const sdk::Vec3& axisPoint, const sdk::Vec3& axisDir) {
    const sdk::Vec3 w = axisPoint - ray.origin;
    const float a = sdk::dot(axisDir, axisDir);
    const float b = sdk::dot(axisDir, ray.direction);
    const float c = sdk::dot(ray.direction, ray.direction);
    const float d = sdk::dot(axisDir, w);
    const float e = sdk::dot(ray.direction, w);
    const float denom = a * c - b * b;
    if (std::fabs(denom) < 1e-6f) return std::nullopt;
    return (b * e - c * d) / denom;
}

inline std::optional<sdk::Vec3> intersectPlane(const Ray& ray, const sdk::Vec3& planePoint, const sdk::Vec3& normal) {
    const float denom = sdk::dot(normal, ray.direction);
    if (std::fabs(denom) < 1e-5f) return std::nullopt;
    const float t = sdk::dot(normal, planePoint - ray.origin) / denom;
    if (t < 0.f) return std::nullopt;
    return ray.origin + ray.direction * t;
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif

inline float snapValue(float value, float step) { return step > 0.f ? std::round(value / step) * step : value; }

}  // namespace wpiemu::app::editor
