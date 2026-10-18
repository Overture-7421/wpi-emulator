#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/game/game_module.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/telemetry/snapshot.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/vision/camera_config.h"
#include "wpiemu/sdk/vision/fiducial_layout.h"
#include "wpiemu/sdk/vision/photon_profiles.h"
#include "wpiemu/sdk/vision/vision_types.h"

using namespace wpiemu::sdk;

namespace {

struct Pixel {
    double u = 0.0;
    double v = 0.0;
};

// The projection convention of the vision contract: a camera-frame point (X forward, Y left, Z up)
// with x > 0.
Pixel project(const CameraIntrinsics& in, const Vec3& p) {
    return {in.cx() - in.fx() * (p.y / p.x), in.cy() - in.fy() * (p.z / p.x)};
}

// A minimal bridge host / bridge / game module / snapshot: everything vision-related stays at its
// default.
class MinimalHost final : public IBridgeHost {
public:
    DeviceId announce(const DeviceDescriptor&, std::vector<SignalSpec>) override {
        return DeviceId{1};
    }
    void retire(DeviceId) override {}
    ILogger& logger() override { return log; }
    bool enableOptionalHooks() const override { return false; }
    testing::CapturingLogger log;
};

class MinimalBridge final : public IDeviceBridge {
public:
    std::string_view name() const override { return "test.minimal"; }
    Status start(IBridgeHost&) override { return okStatus(); }
    void poll() override {}
    void collectOutputs(std::vector<SignalSample>&) override {}
    void applyInputs(std::span<const SignalSample>) override {}
    void applyLoopback(DeviceId) override {}
    bool owns(DeviceId) const override { return false; }
    void stop() noexcept override {}
};

class MinimalGameModule final : public IGameModule {
public:
    GameMetadata metadata() const override {
        GameMetadata m;
        m.id = "game_minimal";
        m.year = 2999;
        return m;
    }
    Status registerMechanisms(MechanismFactory&) override { return okStatus(); }
    Result<std::unique_ptr<IField>> createField(IPhysicsWorld&) override {
        return std::unique_ptr<IField>(std::make_unique<EmptyField>());
    }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        return std::unique_ptr<IGameState>(std::make_unique<NullGameState>("game_minimal"));
    }
    PluginConfigSchema contributeConfigSchema() const override { return {}; }
};

class MinimalSnapshot final : public ISimSnapshot {
public:
    std::uint64_t tick() const override { return 0; }
    SimTimeUs simTimeUs() const override { return 0; }
    bool paused() const override { return false; }
    bool robotConnected() const override { return false; }
    const BodySample& chassis() const override { return chassisSample; }
    std::span<const BodySample> bodies() const override { return {}; }
    std::span<const WheelSample> wheels() const override { return {}; }
    std::span<const EntityGroupSample> entityGroups() const override { return {}; }
    std::span<const MechanismState> mechanisms() const override { return {}; }
    std::span<const HudValue> hud() const override { return {}; }
    std::span<const DiscoveredDevice> devices() const override { return {}; }
    std::span<const HubWarning> warnings() const override { return {}; }
    double batteryVoltageV() const override { return 12.0; }
    const proto::DsState& driverStation() const override { return ds; }
    const PhysicsStats& physicsStats() const override { return stats; }
    BodySample chassisSample;
    proto::DsState ds;
    PhysicsStats stats;
};

}  // namespace

TEST_CASE("vision model: camera, intrinsics and realism defaults") {
    const CameraConfig c;
    CHECK(c.id.empty());
    CHECK(c.vendor == CameraVendor::PhotonVision);
    CHECK(c.enabled);
    CHECK(c.attachment.onChassis());
    CHECK(c.local == Pose3{});
    CHECK(c.intrinsics.widthPx == 1280);
    CHECK(c.intrinsics.heightPx == 800);
    CHECK(c.intrinsics.hfovRad == 1.5708);
    CHECK(c.intrinsics.fxPx == 0.0);
    CHECK(c.intrinsics.fyPx == 0.0);
    CHECK(c.intrinsics.cxPx == 0.0);
    CHECK(c.intrinsics.cyPx == 0.0);
    CHECK(c.intrinsics.distortion == std::array<double, 5>{});
    CHECK(c.frameRateHz == 30.0);
    CHECK(c.latencyUs == 25000);
    CHECK(c.captureLatencyUs == 10000);
    CHECK(c.tagSizeM == 0.1651);
    CHECK(c.minTagPixels == 12.0);
    CHECK(c.maxViewAngleRad == 1.309);
    CHECK(c.occlusion);
    CHECK(c.multiTag);
    CHECK(c.realism.ideal());
    CHECK_FALSE(c.assumedPose.has_value());

    const VisionConfig v;
    CHECK(v.empty());
    CHECK(v.layoutId.empty());
    CHECK(v.layoutFile.empty());
    CHECK(v.photonMessageUuid.empty());
}

TEST_CASE("vision model: intrinsics derive the pinhole from the horizontal FOV") {
    CameraIntrinsics in;
    in.widthPx = 640;
    in.heightPx = 480;
    in.hfovRad = degToRad(60.0);
    const double fx = 320.0 / std::tan(degToRad(30.0));
    CHECK(in.fx() == doctest::Approx(fx));
    CHECK(in.fy() == doctest::Approx(fx));  // square pixels
    CHECK(in.cx() == 320.0);
    CHECK(in.cy() == 240.0);
    CHECK(in.vfovRad() == doctest::Approx(2.0 * std::atan(240.0 / fx)));

    in.hfovRad = kPi / 2.0;  // 90 degrees: the half width is the focal length
    CHECK(in.fx() == doctest::Approx(320.0));
    CHECK(in.vfovRad() == doctest::Approx(2.0 * std::atan(0.75)));
}

TEST_CASE("vision model: explicit intrinsics override the derived values") {
    CameraIntrinsics in;
    in.widthPx = 640;
    in.heightPx = 480;
    in.hfovRad = -1.0;  // ignored while fxPx is set
    in.fxPx = 700.0;
    in.fyPx = 710.0;
    in.cxPx = 330.5;
    in.cyPx = 241.25;
    CHECK(in.fx() == 700.0);
    CHECK(in.fy() == 710.0);
    CHECK(in.cx() == 330.5);
    CHECK(in.cy() == 241.25);
    CHECK(in.vfovRad() == doctest::Approx(2.0 * std::atan(240.0 / 710.0)));

    in.fyPx = 0.0;  // 0 => fy follows fx
    CHECK(in.fy() == 700.0);
    in.cxPx = 0.0;  // 0 => the image centre
    in.cyPx = 0.0;
    CHECK(in.cx() == 320.0);
    CHECK(in.cy() == 240.0);
}

TEST_CASE(
    "vision model: the projection puts the camera's left on the left of the image and up above its "
    "centre") {
    const CameraIntrinsics in;  // 1280x800, 90 degrees
    const Pixel centre = project(in, {3.f, 0.f, 0.f});
    CHECK(centre.u == doctest::Approx(640.0));
    CHECK(centre.v == doctest::Approx(400.0));

    const Pixel left = project(in, {2.f, 1.f, 0.f});
    CHECK(left.u < in.cx());
    CHECK(left.u == doctest::Approx(in.cx() - in.fx() * 0.5));
    const Pixel up = project(in, {2.f, 0.f, 1.f});
    CHECK(up.v < in.cy());
    CHECK(up.v == doctest::Approx(in.cy() - in.fy() * 0.5));

    // the FOV edges land on the image border (the point is stored in float: sub-pixel tolerance)
    const double halfH = std::tan(in.hfovRad / 2.0);
    const double halfV = std::tan(in.vfovRad() / 2.0);
    const auto onBorder = [](double got, double expected) {
        return std::abs(got - expected) < 1e-3;
    };
    CHECK(onBorder(project(in, {1.f, static_cast<float>(halfH), 0.f}).u, 0.0));
    CHECK(onBorder(project(in, {1.f, static_cast<float>(-halfH), 0.f}).u, 1280.0));
    CHECK(onBorder(project(in, {1.f, 0.f, static_cast<float>(halfV)}).v, 0.0));
    CHECK(onBorder(project(in, {1.f, 0.f, static_cast<float>(-halfV)}).v, 800.0));

    // vendor angle signs: yaw > 0 is a target to the RIGHT of the optical axis, pitch > 0 is UP
    const Pixel right = project(in, {2.f, -1.f, 0.f});
    CHECK(std::atan((right.u - in.cx()) / in.fx()) > 0.0);
    CHECK(-std::atan((up.v - in.cy()) / in.fy()) > 0.0);
}

TEST_CASE("vision model: realism is ideal only while every parameter is zero") {
    CameraRealism r;
    CHECK(r.ideal());
    r.cornerNoisePx = 0.5;
    CHECK_FALSE(r.ideal());
    r = {};
    r.latencyJitterUs = 1;
    CHECK_FALSE(r.ideal());
    r = {};
    r.dropoutProbability = 0.01;
    CHECK_FALSE(r.ideal());
}

TEST_CASE("vision model: the network name falls back to the Limelight default or the id") {
    CameraConfig pv;
    pv.id = "front";
    CHECK(pv.networkName() == "front");
    pv.ntName = "OV9281_1";
    CHECK(pv.networkName() == "OV9281_1");

    CameraConfig ll;
    ll.id = "ll_left";
    ll.vendor = CameraVendor::Limelight;
    CHECK(ll.networkName() == "limelight");
    ll.ntName = "limelight-left";
    CHECK(ll.networkName() == "limelight-left");
}

TEST_CASE("vision model: cameras are found by id and equality sees every field") {
    RobotConfig config;
    CHECK(config.vision.empty());
    CHECK(config.findCamera("front") == nullptr);

    CameraConfig a;
    a.id = "front";
    CameraConfig b = a;
    b.id = "rear";
    config.vision.cameras = {a, b};
    CHECK_FALSE(config.vision.empty());
    REQUIRE(config.findCamera("rear") != nullptr);
    CHECK(config.findCamera("rear")->id == "rear");
    CHECK(config.vision.findCamera("front") == &config.vision.cameras[0]);
    CHECK(config.findCamera("side") == nullptr);

    CHECK(a == CameraConfig(a));
    const auto differs = [&](auto change) {
        CameraConfig c = a;
        change(c);
        return !(c == a);
    };
    CHECK(differs([](CameraConfig& c) { c.assumedPose = Pose3{}; }));
    CHECK(differs([](CameraConfig& c) { c.intrinsics.distortion[4] = 0.01; }));
    CHECK(differs([](CameraConfig& c) { c.realism.cornerNoisePx = 0.1; }));
    CHECK(differs([](CameraConfig& c) { c.attachment.parentLink = "tip"; }));
    CHECK(differs([](CameraConfig& c) { c.local.position.x = 0.1f; }));
    CHECK(differs([](CameraConfig& c) { c.vendor = CameraVendor::Limelight; }));
    CHECK(differs([](CameraConfig& c) { c.captureLatencyUs = 1; }));

    VisionConfig v1;
    v1.cameras = {a};
    VisionConfig v2 = v1;
    CHECK(v1 == v2);
    v2.photonMessageUuid = "x";
    CHECK_FALSE(v1 == v2);
}

TEST_CASE("vision model: layouts find tags by id and the vision value types default to nothing") {
    FiducialLayout layout;
    layout.tags = {{1, Pose3{}, 0.0}, {7, Pose3{Vec3{1.f, 2.f, 3.f}, Quat{}}, 0.2}};
    REQUIRE(layout.find(7) != nullptr);
    CHECK(layout.find(7)->pose.position == Vec3{1.f, 2.f, 3.f});
    CHECK(layout.find(7)->sizeM == 0.2);
    CHECK(layout.find(2) == nullptr);
    CHECK(layout.tagSizeM == 0.1651);

    const VisionSetup setup;
    CHECK(setup.empty());
    VisionSetup withCamera;
    withCamera.cameras.emplace_back();
    CHECK_FALSE(withCamera.empty());
    CHECK(withCamera.cameras[0].frameRateHz == 30.0);

    const VisionFrame frame;
    CHECK(frame.observations.empty());
    CHECK_FALSE(frame.multiTag.valid);
    CHECK(frame.sequence == 0);
    const CameraRobotState state;
    CHECK(state.present);
    CHECK_FALSE(state.publishing);
    CHECK_FALSE(state.pipelineIndex.has_value());
    CHECK_FALSE(state.robotOrientationDeg.has_value());
    CHECK(state.status.empty());
    CHECK(state == CameraRobotState{});
}

TEST_CASE(
    "vision model: photonMessageUuidForVersion maps verified PhotonLib releases to the profile "
    "UUID") {
    const std::string profile = kPhotonMessageUuidV2025_3;
    CHECK(profile == "4b2ff16a964b5e2bf04be0c1454d91c4");
    for (const char* version :
         {"v2026.3.4", "2026.3.4", "V2026.3.4", "v2025.3.2", "v2025.3.1", "v2026.1.1", "v2026.3.0"})
        CHECK_MESSAGE(photonMessageUuidForVersion(version) == profile, version);
    for (const char* version :
         {"v2025.2.9", "2025.3.0", "v2026.3.5", "v2026.4.0", "v2027.1.0", "v2024.3.1"})
        CHECK_MESSAGE(photonMessageUuidForVersion(version).empty(), version);
}

TEST_CASE(
    "vision model: photonMessageUuidForVersion rejects anything that is not a plain three-part "
    "version") {
    for (const char* version :
         {"", "v", "garbage", "2026", "2026.3", "v2026.3.4.1", "v2026.3.4-beta-1", "2026.3.x",
          "v2026..4", " 2026.3.4", "2026.3.4 ", "-2026.3.4", "2026.3.99999999999999999999"})
        CHECK_MESSAGE(photonMessageUuidForVersion(version).empty(), version);
}

TEST_CASE(
    "vision defaults: bridge host, bridge and game module have no vision unless they opt in") {
    MinimalHost host;
    MinimalHost other;
    CHECK(host.visionSetup().empty());
    CHECK(&host.visionSetup() == &other.visionSetup());  // one shared empty setup

    MinimalBridge bridge;
    VisionFrame frame;
    frame.cameraId = "front";
    bridge.applyVision(std::span<const VisionFrame>(&frame, 1));  // ignored
    std::vector<CameraRobotState> out(1);
    out[0].cameraId = "kept";
    bridge.collectVision(out, true);
    bridge.collectVision(out, false);
    REQUIRE(out.size() == 1);
    CHECK(out[0].cameraId == "kept");

    const MinimalGameModule module;
    CHECK(module.contributeFiducialLayouts().empty());
}

TEST_CASE(
    "vision snapshot: the vision accessors default to empty and the builder carries what it is "
    "given") {
    MinimalSnapshot minimal;
    const ISimSnapshot& view = minimal;
    CHECK(view.cameras().empty());
    CHECK(view.fiducials().empty());
    CHECK(view.detectedCameras().empty());
    CHECK(SnapshotBuilder{}.build()->cameras().empty());

    SnapshotBuilder builder;
    CameraSample camera;
    camera.id = "front";
    camera.label = "Front";
    camera.vendor = CameraVendor::Limelight;
    camera.pose = Pose3{Vec3{1.f, 2.f, 0.5f}, Quat::fromYaw(0.3f)};
    camera.hfovRad = 1.2;
    camera.vfovRad = 0.8;
    camera.widthPx = 640;
    camera.heightPx = 480;
    camera.visibleTagIds = {3, 4};
    camera.framesCaptured = 10;
    camera.framesPublished = 9;
    camera.lastLatencyMs = 25.0;
    camera.robotConnected = true;
    camera.status = "foreign_publisher";
    builder.addCamera(camera);
    builder.addFiducial({7, Pose3{Vec3{4.f, 5.f, 1.f}, Quat{}}, 0.1651f});
    CameraRobotState detected;
    detected.ntName = "limelight-extra";
    detected.cameraId = "";
    builder.addDetectedCamera(detected);
    const auto snapshot = builder.build();

    REQUIRE(snapshot->cameras().size() == 1);
    CHECK(snapshot->cameras()[0].id == "front");
    CHECK(snapshot->cameras()[0].vendor == CameraVendor::Limelight);
    CHECK(snapshot->cameras()[0].pose.position == Vec3{1.f, 2.f, 0.5f});
    CHECK(snapshot->cameras()[0].visibleTagIds == std::vector<int>{3, 4});
    CHECK(snapshot->cameras()[0].framesPublished == 9);
    CHECK(snapshot->cameras()[0].lastLatencyMs == 25.0);
    CHECK(snapshot->cameras()[0].robotConnected);
    CHECK(snapshot->cameras()[0].status == "foreign_publisher");
    CHECK(snapshot->cameras()[0].enabled);
    REQUIRE(snapshot->fiducials().size() == 1);
    CHECK(snapshot->fiducials()[0].id == 7);
    CHECK(snapshot->fiducials()[0].sizeM == 0.1651f);
    REQUIRE(snapshot->detectedCameras().size() == 1);
    CHECK(snapshot->detectedCameras()[0].ntName == "limelight-extra");
}
