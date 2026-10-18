#pragma once

// Test support for the vision bridge: builds ideal VisionFrames from geometry only (robot pose, camera mount, tag
// poses), exactly as core/vision does in ideal mode, so publisher tests do not depend on the engine.
//
// Conventions (session spec section B): camera frame X forward / Y left / Z up; image origin top-left, pixel
// u = cx - fx * y / x and v = cy - fy * z / x; tag frame +X out of the face, +Y the viewer's right, +Z up; corners
// listed BL, BR, TR, TL; yaw positive to the right of the optical axis, pitch positive up. A WPILib camera pitched
// UP by 20 degrees has pitch -20 degrees in Quat::fromEuler(roll, pitch, yaw).
//
// The caller picks `tagIds` that are visible: nothing here checks the frustum or occlusion.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/vision/fiducial_layout.h"
#include "wpiemu/sdk/vision/vision_types.h"

namespace wpiemu::tests::vision {

inline constexpr double kPi = 3.14159265358979323846;

struct FrameSpec {
    std::string cameraId = "cam";
    sdk::CameraVendor vendor = sdk::CameraVendor::PhotonVision;
    std::string ntName = "cam";
    sdk::Pose3 fieldToRobot;
    sdk::Pose3 robotToCamera;  // the PHYSICAL mount
    std::vector<int> tagIds;   // tags to include, in any order (the frame sorts by area)
    int widthPx = 1280;
    int heightPx = 800;
    double hfovRad = kPi / 2.0;
    std::uint64_t sequence = 1;
    std::int64_t captureTimeUs = 0;
    std::int64_t latencyUs = 25000;
    std::int64_t captureLatencyUs = 10000;
    bool multiTag = true;  // fill the multi-tag solution when two or more tags are included
};

// Four tags on a wall at x = 8 m facing -X (towards a robot near the origin), a fifth on the left wall.
inline sdk::FiducialLayout sampleLayout() {
    sdk::FiducialLayout layout;
    layout.id = "test.layout";
    layout.displayName = "Test layout";
    layout.tagSizeM = 0.1651;
    layout.fieldLengthM = 16.5f;
    layout.fieldWidthM = 8.2f;
    const sdk::Quat towardMinusX = sdk::Quat::fromYaw(static_cast<float>(kPi));
    layout.tags.push_back({1, sdk::Pose3({8.0f, 3.6f, 0.9f}, towardMinusX), 0.0});
    layout.tags.push_back({2, sdk::Pose3({8.0f, 4.4f, 0.9f}, towardMinusX), 0.0});
    layout.tags.push_back({3, sdk::Pose3({8.0f, 4.0f, 1.6f}, towardMinusX), 0.0});
    layout.tags.push_back({4, sdk::Pose3({7.0f, 4.0f, 0.5f}, towardMinusX), 0.0});  // closer: the largest in view
    layout.tags.push_back({5, sdk::Pose3({5.0f, 8.0f, 0.9f}, sdk::Quat::fromYaw(static_cast<float>(-kPi / 2.0))), 0.0});
    return layout;
}

// The PhotonVision sample robot's mount (samples/vision_robot/README.md): 0.30 m forward, 0.50 m up, pitched up 20 deg.
inline sdk::Pose3 sampleFrontMount() {
    return sdk::Pose3({0.30f, 0.0f, 0.50f}, sdk::Quat::fromEuler(0.f, static_cast<float>(-20.0 * kPi / 180.0), 0.f));
}

// Robot at (2, 4) on the field facing +X.
inline sdk::Pose3 sampleRobotPose() { return sdk::Pose3({2.0f, 4.0f, 0.0f}, sdk::Quat::identity()); }

inline sdk::VisionFrame makeFrame(const FrameSpec& spec, const sdk::FiducialLayout& layout) {
    const double fx = 0.5 * spec.widthPx / std::tan(0.5 * spec.hfovRad);
    const double fy = fx;
    const double cx = 0.5 * spec.widthPx;
    const double cy = 0.5 * spec.heightPx;
    const sdk::Pose3 fieldToCamera = spec.fieldToRobot * spec.robotToCamera;
    const sdk::Pose3 cameraToField = fieldToCamera.inverse();
    const auto project = [&](const sdk::Vec3& p) {
        return sdk::PixelPoint{cx - fx * p.y / p.x, cy - fy * p.z / p.x};
    };

    sdk::VisionFrame frame;
    frame.cameraId = spec.cameraId;
    frame.vendor = spec.vendor;
    frame.ntName = spec.ntName;
    frame.sequence = spec.sequence;
    frame.captureTimeUs = spec.captureTimeUs;
    frame.latencyUs = spec.latencyUs;
    frame.captureLatencyUs = spec.captureLatencyUs;
    frame.robotToCamera = spec.robotToCamera;

    for (const int id : spec.tagIds) {
        const sdk::FiducialTag* tag = layout.find(id);
        if (tag == nullptr) continue;
        const float half = static_cast<float>(0.5 * (tag->sizeM > 0.0 ? tag->sizeM : layout.tagSizeM));
        const sdk::Pose3 cameraToTag = cameraToField * tag->pose;
        const sdk::Vec3 cornersInTag[4] = {{0.f, -half, -half}, {0.f, half, -half}, {0.f, half, half}, {0.f, -half, half}};

        sdk::FiducialObservation o;
        o.id = id;
        o.fieldToTag = tag->pose;
        o.cameraToTagBest = cameraToTag;
        o.cameraToTagAlt = cameraToTag;  // no second solution: alt = best, ambiguity 0
        for (int i = 0; i < 4; ++i) o.cornersPx[static_cast<std::size_t>(i)] = project(cameraToTag.transformPoint(cornersInTag[i]));
        o.centerPx = project(cameraToTag.position);
        o.yawRad = std::atan((o.centerPx.x - cx) / fx);
        o.pitchRad = std::atan((cy - o.centerPx.y) / fy);
        double twiceArea = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
            const sdk::PixelPoint& a = o.cornersPx[i];
            const sdk::PixelPoint& b = o.cornersPx[(i + 1) % 4];
            twiceArea += a.x * b.y - b.x * a.y;
        }
        o.areaPct = 100.0 * 0.5 * std::fabs(twiceArea) / (static_cast<double>(spec.widthPx) * spec.heightPx);
        o.distanceM = sdk::length(cameraToTag.position);
        frame.observations.push_back(o);
    }
    std::stable_sort(frame.observations.begin(), frame.observations.end(),
                     [](const sdk::FiducialObservation& a, const sdk::FiducialObservation& b) { return a.areaPct > b.areaPct; });

    if (spec.multiTag && frame.observations.size() >= 2) {
        frame.multiTag.valid = true;
        frame.multiTag.fieldToCamera = fieldToCamera;
        for (const sdk::FiducialObservation& o : frame.observations) frame.multiTag.tagIds.push_back(o.id);
    }
    return frame;
}

}  // namespace wpiemu::tests::vision
