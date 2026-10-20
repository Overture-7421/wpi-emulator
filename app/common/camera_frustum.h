#pragma once

// Camera frustum geometry shared by the robot editor and the run view so both draw a simulated camera's
// field of view identically. Pure math (no Qt). Everything is in the CAMERA frame (+X = optical axis,
// +Y left, +Z up); transform the points with the camera's world pose.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#include "wpiemu/sdk/config/camera.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/units.h"

namespace wpiemu::app {

inline constexpr float kFrustumMaxDisplayLengthM = 2.0f;   // long ranges are not drawn to scale
inline constexpr float kFrustumMinDisplayLengthM = 0.2f;
inline constexpr float kFrustumMaxHalfExtentM = 2.0f;      // the far rectangle never extends farther than this from the axis
inline constexpr float kFrustumDegToRad = sdk::kPiF / 180.f;

// Indices into FrustumGeometry::pts.
enum FrustumPoint : std::uint8_t { kFrustumApex = 0, kFrustumTopLeft, kFrustumTopRight, kFrustumBottomRight, kFrustumBottomLeft };

// The 8 edges of the pyramid (4 apex rays + the far rectangle).
inline constexpr std::array<std::pair<std::uint8_t, std::uint8_t>, 8> kFrustumEdges{{
    {0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {2, 3}, {3, 4}, {4, 1}}};

// Triangles (counter-clockwise seen from outside) of the 4 sides and the far cap.
inline constexpr std::array<std::array<std::uint8_t, 3>, 6> kFrustumTriangles{{
    {0, 2, 1}, {0, 3, 2}, {0, 4, 3}, {0, 1, 4}, {1, 2, 3}, {1, 3, 4}}};

struct FrustumGeometry {
    // pts[0] = apex (camera origin); pts[1..4] = far rectangle corners in the order top-left, top-right,
    // bottom-right, bottom-left as seen looking along +X (left = +Y, up = +Z).
    std::array<sdk::Vec3, 5> pts;
    float lengthM = 0.f;   // distance of the far rectangle along +X
};

// Length used to draw a camera's frustum: its max range, clamped to a readable size and short enough that a very
// wide field of view does not produce a huge far rectangle.
inline float frustumDisplayLengthM(const sdk::CameraConfig& camera) {
    const float range = static_cast<float>(camera.maxRangeM);
    float length = !(range > 0.f) ? kFrustumMinDisplayLengthM : std::clamp(range, kFrustumMinDisplayLengthM, kFrustumMaxDisplayLengthM);
    const float widest = std::clamp(static_cast<float>(std::max(camera.hFovDeg, camera.vFovDeg)), 1.f, 179.f);
    length = std::min(length, kFrustumMaxHalfExtentM / std::tan(widest * 0.5f * kFrustumDegToRad));
    return std::max(length, 0.01f);
}

// Pyramid with its apex at the camera origin and the far rectangle `lengthM` along +X, spanning the
// horizontal / vertical field of view (clamped to a sane 1..179 degrees).
inline FrustumGeometry cameraFrustum(const sdk::CameraConfig& camera, float lengthM) {
    const float h = std::clamp(static_cast<float>(camera.hFovDeg), 1.f, 179.f) * kFrustumDegToRad;
    const float v = std::clamp(static_cast<float>(camera.vFovDeg), 1.f, 179.f) * kFrustumDegToRad;
    const float halfY = lengthM * std::tan(h * 0.5f);
    const float halfZ = lengthM * std::tan(v * 0.5f);
    FrustumGeometry g;
    g.lengthM = lengthM;
    g.pts[kFrustumApex] = {0.f, 0.f, 0.f};
    g.pts[kFrustumTopLeft] = {lengthM, halfY, halfZ};
    g.pts[kFrustumTopRight] = {lengthM, -halfY, halfZ};
    g.pts[kFrustumBottomRight] = {lengthM, -halfY, -halfZ};
    g.pts[kFrustumBottomLeft] = {lengthM, halfY, -halfZ};
    return g;
}

inline FrustumGeometry cameraFrustum(const sdk::CameraConfig& camera) {
    return cameraFrustum(camera, frustumDisplayLengthM(camera));
}

}  // namespace wpiemu::app
