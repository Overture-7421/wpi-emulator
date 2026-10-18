#pragma once

// Synthetic vision for the stand-ins that run without the vision engine: FakeRunBackend (tests, the
// run view demo) and DemoRuntimeHost (`--demo`). Geometry only: where a configured camera is in the
// world and which AprilTags it can see (field of view, facing, apparent size; no occlusion).
// Header-only, Qt-free.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/mechanisms/attachment.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/vision/camera_config.h"

namespace wpiemu::app::demo_vision {

// Pose of the camera frame in the chassis frame: the attachment link's pose times
// CameraConfig::local. A camera on the chassis has no link; an unresolvable attachment falls back
// to the rest pose.
inline sdk::Pose3 cameraInChassis(const sdk::RobotConfig& config, const sdk::CameraConfig& camera,
                                  const sdk::LinkPoseSource& linkPoseOf) {
    if (camera.attachment.onChassis()) return camera.local;
    const auto link = sdk::resolveLinkPoseInChassis(config, camera.attachment.parentMechanismId,
                                                    camera.attachment.parentLink, linkPoseOf);
    return (link ? *link : sdk::Pose3{}) * camera.local;
}

// Can `camera`, at `cameraInWorld` (+X = optical axis), see `tag`: its centre is inside the image,
// it faces the camera within CameraConfig::maxViewAngleRad and its foreshortened edge is at least
// CameraConfig::minTagPixels long.
inline bool seesTag(const sdk::CameraConfig& camera, const sdk::Pose3& cameraInWorld,
                    const sdk::FiducialSample& tag) {
    const sdk::CameraIntrinsics& k = camera.intrinsics;
    const sdk::Vec3 p = cameraInWorld.inverse().transformPoint(tag.pose.position);
    if (p.x <= 0.05f) return false;
    const double u = k.cx() - k.fx() * static_cast<double>(p.y / p.x);
    const double v = k.cy() - k.fy() * static_cast<double>(p.z / p.x);
    if (u < 0.0 || u > k.widthPx || v < 0.0 || v > k.heightPx) return false;

    const sdk::Vec3 normal = tag.pose.rotation.rotate({1.f, 0.f, 0.f});
    const double facing = static_cast<double>(
        sdk::dot(normal, sdk::normalized(cameraInWorld.position - tag.pose.position)));
    if (facing <= std::cos(camera.maxViewAngleRad))
        return false;  // seen from behind or at a grazing angle
    const double edgeM = camera.tagSizeM > 0.0 ? camera.tagSizeM : static_cast<double>(tag.sizeM);
    return k.fx() * edgeM * facing / static_cast<double>(p.x) >= camera.minTagPixels;
}

inline std::vector<int> visibleTagIds(
    const sdk::CameraConfig& camera, const sdk::Pose3& cameraInWorld,
    std::span<const sdk::FiducialSample> tags,
    const std::function<bool(const sdk::Vec3& from, const sdk::Vec3& to)>& blocked = {}) {
    std::vector<int> out;
    for (const sdk::FiducialSample& tag : tags)
        if (seesTag(camera, cameraInWorld, tag) &&
            !(camera.occlusion && blocked && blocked(cameraInWorld.position, tag.pose.position)))
            out.push_back(tag.id);
    return out;
}

// The identity and optics half of a CameraSample; the caller fills in what the camera did.
inline sdk::CameraSample makeSample(const sdk::CameraConfig& camera,
                                    const sdk::Pose3& cameraInWorld) {
    const sdk::CameraIntrinsics& k = camera.intrinsics;
    sdk::CameraSample s;
    s.id = camera.id;
    s.label = camera.label;
    s.vendor = camera.vendor;
    s.enabled = camera.enabled;
    s.pose = cameraInWorld;
    s.hfovRad = 2.0 * std::atan(0.5 * k.widthPx / k.fx());
    s.vfovRad = k.vfovRad();
    s.widthPx = k.widthPx;
    s.heightPx = k.heightPx;
    return s;
}

// Captures a camera at `frameRateHz` has made by sim time `simTimeUs`, and how many of them have
// reached the robot after `latencyUs`. Deterministic: a pure function of the sim clock.
inline std::uint64_t framesCaptured(double frameRateHz, std::int64_t simTimeUs) {
    return simTimeUs <= 0 ? 0
                          : static_cast<std::uint64_t>(
                                std::floor(static_cast<double>(simTimeUs) * frameRateHz / 1e6));
}
inline std::uint64_t framesPublished(double frameRateHz, std::int64_t latencyUs,
                                     std::int64_t simTimeUs) {
    return framesCaptured(frameRateHz, simTimeUs - latencyUs);
}

// The camera a stand-in adds to a project that configures none (`--demo`): a PhotonVision camera on
// the front of the chassis, looking a little upwards so that it picks up tags on walls.
inline sdk::CameraConfig defaultFrontCamera(const sdk::RobotConfig& config) {
    sdk::CameraConfig camera;
    camera.id = "demo_front";
    camera.label = "Front camera";
    camera.vendor = sdk::CameraVendor::PhotonVision;
    camera.ntName = "photon-front";
    const sdk::ChassisSpec& chassis = config.chassis;
    const float topM = chassis.groundClearanceM + chassis.sizeM.z - chassis.comHeightM;
    camera.local = sdk::Pose3({chassis.sizeM.x * 0.5f - 0.05f, 0.f, topM + 0.12f},
                              sdk::Quat::fromAxisAngle({0.f, 1.f, 0.f}, -0.2f));
    camera.intrinsics.hfovRad = 1.4;
    return camera;
}

}  // namespace wpiemu::app::demo_vision
