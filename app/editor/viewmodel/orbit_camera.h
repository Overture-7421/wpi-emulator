#pragma once

// Orbit camera + ray helpers in the SDK's Z-up world. Pure math (no Qt): the viewport controller
// drives it from mouse input and tests drive it directly.

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::editor {

struct Ray {
    sdk::Vec3 origin;
    sdk::Vec3 direction;   // unit
};

struct CameraBasis {
    sdk::Vec3 right;
    sdk::Vec3 up;
    sdk::Vec3 forward;     // viewing direction
};

class OrbitCamera {
public:
    sdk::Vec3 target{0.f, 0.f, 0.25f};
    float yawRad = degToRad(-50.f);       // azimuth of the camera around the target (0 = on +X looking back)
    float pitchRad = degToRad(26.f);      // elevation above the floor
    float distance = 2.8f;
    float fovDeg = 45.f;

    static float degToRad(float d) { return d * static_cast<float>(sdk::kPi) / 180.f; }

    sdk::Vec3 position() const {
        const float cp = std::cos(pitchRad);
        return target + sdk::Vec3{cp * std::cos(yawRad), cp * std::sin(yawRad), std::sin(pitchRad)} * distance;
    }

    CameraBasis basis() const {
        const sdk::Vec3 forward = sdk::normalized(target - position());
        sdk::Vec3 right = sdk::cross(forward, sdk::Vec3{0.f, 0.f, 1.f});
        if (sdk::length(right) < 1e-4f) right = sdk::Vec3{0.f, 1.f, 0.f};
        right = sdk::normalized(right);
        const sdk::Vec3 up = sdk::cross(right, forward);
        return {right, up, forward};
    }

    // Camera orientation in the Z-up world: local -Z looks along `forward`, +Y is up.
    sdk::Quat rotation() const {
        const CameraBasis b = basis();
        const sdk::Vec3 back = -b.forward;
        // Rotation matrix with columns (right, up, back) -> quaternion.
        const float m00 = b.right.x, m10 = b.right.y, m20 = b.right.z;
        const float m01 = b.up.x, m11 = b.up.y, m21 = b.up.z;
        const float m02 = back.x, m12 = back.y, m22 = back.z;
        const float trace = m00 + m11 + m22;
        sdk::Quat q;
        if (trace > 0.f) {
            const float s = std::sqrt(trace + 1.f) * 2.f;
            q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
        } else if (m00 > m11 && m00 > m22) {
            const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
            q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
        } else if (m11 > m22) {
            const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
            q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
        } else {
            const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
            q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
        }
        return q.normalized();
    }

    void orbit(float dxPx, float dyPx) {
        yawRad -= degToRad(dxPx * 0.45f);
        pitchRad = std::clamp(pitchRad + degToRad(dyPx * 0.45f), degToRad(-89.f), degToRad(89.f));
    }

    void pan(float dxPx, float dyPx, float viewHeightPx) {
        const CameraBasis b = basis();
        const float unitsPerPx = 2.f * distance * std::tan(degToRad(fovDeg) * 0.5f) / std::max(viewHeightPx, 1.f);
        target += (b.right * -dxPx + b.up * dyPx) * unitsPerPx;
    }

    void zoom(float steps) { distance = std::clamp(distance * std::pow(0.88f, steps), 0.25f, 60.f); }

    void frame(const sdk::Vec3& center, float radius) {
        target = center;
        distance = std::clamp(radius / std::tan(degToRad(fovDeg) * 0.5f) * 1.15f, 0.6f, 60.f);
    }

    // Standard views. "top" keeps +X pointing up on screen.
    void preset(const char* name) {
        const std::string n = name;
        if (n == "front") { yawRad = 0.f; pitchRad = 0.f; }
        else if (n == "back") { yawRad = degToRad(180.f); pitchRad = 0.f; }
        else if (n == "side") { yawRad = degToRad(-90.f); pitchRad = 0.f; }
        else if (n == "top") { yawRad = degToRad(180.f); pitchRad = degToRad(89.f); }
        else { yawRad = degToRad(-50.f); pitchRad = degToRad(26.f); }
    }

    // Ray through a pixel of a `width` x `height` viewport (vertical field of view).
    Ray rayAt(float x, float y, float width, float height) const {
        const CameraBasis b = basis();
        const float aspect = width / std::max(height, 1.f);
        const float ndcX = (2.f * x / std::max(width, 1.f)) - 1.f;
        const float ndcY = 1.f - (2.f * y / std::max(height, 1.f));
        const float t = std::tan(degToRad(fovDeg) * 0.5f);
        const sdk::Vec3 dir = sdk::normalized(b.forward + b.right * (ndcX * t * aspect) + b.up * (ndcY * t));
        return {position(), dir};
    }

    // Screen position (pixels, y down) of a world point; nullopt behind the camera.
    std::optional<std::pair<float, float>> project(const sdk::Vec3& p, float width, float height) const {
        const CameraBasis b = basis();
        const sdk::Vec3 d = p - position();
        const float depth = sdk::dot(d, b.forward);
        if (depth <= 1e-3f) return std::nullopt;
        const float t = std::tan(degToRad(fovDeg) * 0.5f);
        const float aspect = width / std::max(height, 1.f);
        const float ndcX = sdk::dot(d, b.right) / (depth * t * aspect);
        const float ndcY = sdk::dot(d, b.up) / (depth * t);
        return std::make_pair((ndcX + 1.f) * 0.5f * width, (1.f - ndcY) * 0.5f * height);
    }
};

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
