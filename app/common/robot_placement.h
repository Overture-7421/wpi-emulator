#pragma once

// The math behind placing the robot with the mouse: a camera ray meets a horizontal plane, a planar pose follows the cursor,
// snapping and turning are plain functions. Shared by the editor viewport (the start pose marker) and the run view (dragging the
// robot live); the view models only turn pointer events into rays (OrbitCamera::rayAt) and apply the poses they get back.
// No Qt. World = the SDK's Z-up frame; a pose is planar (x, y, yaw), its height stays with whoever owns the body.

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "common/orbit_camera.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app::placement {

using sdk::StartPose;
using sdk::Vec3;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4723)  // false positive when the parallel case is constant-folded in a caller; every divisor is guarded
#endif

// Where `ray` meets the horizontal plane z = zM. nullopt when the ray is (nearly) parallel to the plane or the plane is behind
// the camera.
inline std::optional<Vec3> intersectHorizontalPlane(const Ray& ray, float zM) {
    if (!(std::fabs(ray.direction.z) > 1e-5f)) return std::nullopt;
    const float t = (zM - ray.origin.z) / ray.direction.z;
    if (!std::isfinite(t) || t < 0.f) return std::nullopt;
    return ray.origin + ray.direction * t;
}

// An axis-aligned box in the frame of a body (min / max corner), e.g. the extent of the robot's parts in the chassis frame.
struct LocalBox {
    Vec3 min;
    Vec3 max;
    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    LocalBox grown(float marginM) const { return {min - Vec3{marginM, marginM, marginM}, max + Vec3{marginM, marginM, marginM}}; }
    // An empty box that grows with include().
    static LocalBox empty() {
        const float inf = std::numeric_limits<float>::infinity();
        return {Vec3{inf, inf, inf}, Vec3{-inf, -inf, -inf}};
    }
    void include(const Vec3& p) {
        min = {std::min(min.x, p.x), std::min(min.y, p.y), std::min(min.z, p.z)};
        max = {std::max(max.x, p.x), std::max(max.y, p.y), std::max(max.z, p.z)};
    }
};

// Distance along `ray` to where it enters `box` carried by `pose` (0 when the ray starts inside); nullopt when it misses.
inline std::optional<float> rayHitDistance(const Ray& ray, const sdk::Pose3& pose, const LocalBox& box) {
    if (!box.valid()) return std::nullopt;
    const sdk::Pose3 inverse = pose.inverse();
    const Vec3 o = inverse.transformPoint(ray.origin);
    const Vec3 d = inverse.transformVector(ray.direction);
    const float origin[3] = {o.x, o.y, o.z};
    const float dir[3] = {d.x, d.y, d.z};
    const float lo[3] = {box.min.x, box.min.y, box.min.z};
    const float hi[3] = {box.max.x, box.max.y, box.max.z};
    float tNear = 0.f;
    float tFar = std::numeric_limits<float>::infinity();
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dir[i]) < 1e-9f) {
            if (origin[i] < lo[i] || origin[i] > hi[i]) return std::nullopt;   // parallel to this slab and outside it
            continue;
        }
        float t0 = (lo[i] - origin[i]) / dir[i];
        float t1 = (hi[i] - origin[i]) / dir[i];
        if (t0 > t1) std::swap(t0, t1);
        tNear = std::max(tNear, t0);
        tFar = std::min(tFar, t1);
        if (tNear > tFar) return std::nullopt;
    }
    return tNear;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

// Rounding rules while dragging; a step of 0 leaves that part free.
struct Snap {
    float moveStepM = 0.f;    // x and y become multiples of this
    float yawStepRad = 0.f;   // the heading becomes a multiple of this

    bool active() const { return moveStepM > 0.f || yawStepRad > 0.f; }
};

inline float snapTo(float value, float step) { return step > 0.f ? std::round(value / step) * step : value; }

// The angle in (-pi, pi].
inline float wrapAngle(float rad) { return static_cast<float>(std::remainder(static_cast<double>(rad), sdk::kTwoPi)); }

// One drag of a planar pose over a horizontal plane. Moving keeps the grabbed spot under the cursor (the plane passes through
// the grabbed point, so grabbing the top of a tall robot tracks as well as grabbing its base); rotating makes the pose face the
// cursor. Every result is derived from where the drag began, so snapping never eats small movements.
class FloorDrag {
public:
    enum class Mode { Move, Rotate };

    // `grab` is the 3D point under the cursor when the drag began; its height is the height of the drag plane.
    static FloorDrag move(const StartPose& pose, const Vec3& grab) {
        FloorDrag d;
        d.mode_ = Mode::Move;
        d.initial_ = d.current_ = pose;
        d.grab_ = d.cursor_ = grab;
        return d;
    }
    // The pose turns about its own origin to face the cursor on the plane at `planeZM`.
    static FloorDrag rotate(const StartPose& pose, float planeZM) {
        FloorDrag d;
        d.mode_ = Mode::Rotate;
        d.initial_ = d.current_ = pose;
        d.grab_ = d.cursor_ = Vec3{pose.xM, pose.yM, planeZM};
        return d;
    }

    Mode mode() const { return mode_; }
    float planeZM() const { return grab_.z; }
    // The pose handed out last (the starting pose before the first update).
    const StartPose& pose() const { return current_; }

    // The pose for a cursor ray. nullopt when the ray misses the plane (or, turning, points at the pivot): keep the last pose.
    std::optional<StartPose> update(const Ray& ray, const Snap& snap = {}) {
        const std::optional<Vec3> hit = intersectHorizontalPlane(ray, grab_.z);
        if (!hit) return std::nullopt;
        StartPose next = initial_;
        if (mode_ == Mode::Move) {
            next.xM = snapTo(initial_.xM + hit->x - grab_.x, snap.moveStepM);
            next.yM = snapTo(initial_.yM + hit->y - grab_.y, snap.moveStepM);
        } else {
            const float dx = hit->x - initial_.xM;
            const float dy = hit->y - initial_.yM;
            if (std::hypot(dx, dy) < 1e-3f) return std::nullopt;
            next.yawRad = wrapAngle(snapTo(std::atan2(dy, dx), snap.yawStepRad));
        }
        cursor_ = *hit;
        current_ = next;
        return next;
    }

    // Turns a moving pose by `deltaYawRad` about the spot under the cursor (it stays under it), e.g. from the mouse wheel.
    StartPose rotateBy(float deltaYawRad, const Snap& snap = {}) {
        const float c = std::cos(deltaYawRad);
        const float s = std::sin(deltaYawRad);
        const float dx = current_.xM - cursor_.x;
        const float dy = current_.yM - cursor_.y;
        StartPose next = current_;
        next.xM = cursor_.x + c * dx - s * dy;
        next.yM = cursor_.y + s * dx + c * dy;
        next.yawRad = wrapAngle(snapTo(wrapAngle(current_.yawRad + deltaYawRad), snap.yawStepRad));
        initial_ = current_ = next;
        grab_ = cursor_;
        return next;
    }

private:
    Mode mode_ = Mode::Move;
    StartPose initial_;   // where the drag began (re-based by rotateBy)
    StartPose current_;
    Vec3 grab_;           // the grabbed point; its height is the plane's
    Vec3 cursor_;         // the last cursor point on the plane
};

}  // namespace wpiemu::app::placement
