#pragma once

// Shared fixtures for the vision bridge tests.

#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/bridges/vision/testing/fake_nt_backend.h"
#include "wpiemu/bridges/vision/vision_bridges.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/vision/vision_bridge.h"

namespace vision_test {

namespace sdk = wpiemu::sdk;
namespace vis = wpiemu::bridges::vision;

using sdk::kPi;

inline sdk::CameraConfig makeCamera(sdk::CameraKind kind, std::string name) {
    sdk::CameraConfig c;
    c.id = "cam_" + name;
    c.name = std::move(name);
    c.kind = kind;
    c.hFovDeg = 90.0;
    c.vFovDeg = 60.0;
    c.widthPx = 1280;
    c.heightPx = 800;
    c.fps = 30.0;
    c.latencyMs = 40.0;
    return c;
}

// The field size a test layout would carry (the 2026 field); VisionSetup::fieldLengthM/WidthM come from the plugin's layout.
inline constexpr double kTestFieldLengthM = 16.541;
inline constexpr double kTestFieldWidthM = 8.069;

inline sdk::VisionSetup makeSetup(std::vector<sdk::CameraConfig> cameras, std::string photonVersion = {}) {
    sdk::VisionSetup s;
    for (auto& c : cameras) s.cameras.push_back(sdk::CameraSetup{std::move(c)});
    s.photonVersion = std::move(photonVersion);
    s.fieldLengthM = kTestFieldLengthM;
    s.fieldWidthM = kTestFieldWidthM;
    return s;
}

inline sdk::TagDetection makeTag(int id, double areaPercent, double yawDeg, double pitchDeg, sdk::Vec3 inCamera) {
    sdk::TagDetection t;
    t.id = id;
    t.areaPercent = areaPercent;
    t.yawDeg = yawDeg;
    t.pitchDeg = pitchDeg;
    // The tag faces the camera: +X of the tag points back at the camera.
    t.cameraToTag = sdk::Pose3(inCamera, sdk::Quat::fromYaw(static_cast<float>(kPi)));
    t.distanceM = sdk::length(inCamera);
    t.cornersPx = {600, 400, 680, 400, 680, 480, 600, 480};
    return t;
}

// Camera 0 at (0.3, 0, 0.5) in the robot frame, robot at (2, 3, 0) with yaw 90 degrees in the field frame,
// captured at 1.00 s and published at 1.04 s (40 ms latency).
inline sdk::CameraFrame makeFrame(std::uint32_t cameraIndex, std::vector<sdk::TagDetection> tags) {
    sdk::CameraFrame f;
    f.cameraIndex = cameraIndex;
    f.sequence = 7;
    f.captureTimeUs = 1'000'000;
    f.publishTimeUs = 1'040'000;
    f.robotToCamera = sdk::Pose3({0.3f, 0.f, 0.5f}, sdk::Quat::identity());
    f.tags = std::move(tags);
    if (!f.tags.empty()) {
        f.hasRobotPose = true;
        f.robotPoseField = sdk::Pose3({2.f, 3.f, 0.f}, sdk::Quat::fromYaw(static_cast<float>(kPi / 2)));
        f.avgTagDistM = 2.5;
        f.avgTagAreaPercent = 1.5;
        f.tagSpanM = f.tags.size() > 1 ? 1.25 : 0.0;
    }
    return f;
}

// Camera 0's frame with tags 3 and 7 in view (makeFrame's pose and timing).
inline std::vector<sdk::CameraFrame> twoTagFrame() {
    return {makeFrame(0, {makeTag(3, 1.0, -10.0, 1.0, {3.f, -0.5f, 0.1f}),
                          makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})};
}

struct Rig {
    std::shared_ptr<vis::testing::FakeNtBackend> nt = std::make_shared<vis::testing::FakeNtBackend>();
    sdk::VisionBridgeRegistry registry;
    sdk::testing::CapturingLogger logger;
    std::unique_ptr<sdk::IVisionBridge> limelight;
    std::unique_ptr<sdk::IVisionBridge> photon;

    explicit Rig(vis::VisionBridgeOptions options = {}) {
        REQUIRE(vis::registerVisionBridges(registry, nt, options).ok());
        auto limelightBridge = registry.create("vision.limelight");
        auto photonBridge = registry.create("vision.photonvision");
        REQUIRE(limelightBridge.ok());
        REQUIRE(photonBridge.ok());
        limelight = std::move(limelightBridge).value();
        photon = std::move(photonBridge).value();
        REQUIRE(limelight->start(logger).ok());
        REQUIRE(photon->start(logger).ok());
        nt->nowTimeUs = 5'000'000;
    }

    void configure(const sdk::VisionSetup& setup) {
        limelight->configure(setup);
        photon->configure(setup);
    }
    void apply(const std::vector<sdk::CameraFrame>& frames, sdk::SimTimeUs simTimeUs) {
        limelight->apply(frames, simTimeUs);
        photon->apply(frames, simTimeUs);
    }
    void poll(sdk::SimTimeUs simTimeUs) {
        limelight->poll(simTimeUs);
        photon->poll(simTimeUs);
    }
};

// Little-endian reads used to check the PhotonLib packet without going through the encoder.
inline std::int64_t readI64(const std::vector<std::uint8_t>& b, std::size_t offset) {
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | b.at(offset + static_cast<std::size_t>(i));
    return static_cast<std::int64_t>(v);
}
inline double readF64(const std::vector<std::uint8_t>& b, std::size_t offset) {
    const std::int64_t bits = readI64(b, offset);
    double d;
    std::memcpy(&d, &bits, sizeof d);
    return d;
}

}  // namespace vision_test
