#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "common/demo_vision.h"
#include "fake_run_backend.h"
#include "wpiemu/sdk/config/validation_codes.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;
namespace code = wpiemu::sdk::validation_code;

namespace {

FakeRunBackend::Options options() {
    FakeRunBackend::Options o;
    o.autoRun = false;
    o.startupDelayMs = 0;
    return o;
}

bool near(const sdk::Vec3& a, const sdk::Vec3& b, float eps = 1e-3f) {
    return sdk::length(a - b) < eps;
}

bool contains(const std::vector<int>& ids, int id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}

const sdk::HubWarning* findWarning(const sdk::ISimSnapshot& snap, const char* warningCode) {
    for (const sdk::HubWarning& w : snap.warnings())
        if (w.code == warningCode) return &w;
    return nullptr;
}

const sdk::CameraRobotState* findDetected(const sdk::ISimSnapshot& snap, const char* cameraId) {
    for (const sdk::CameraRobotState& s : snap.detectedCameras())
        if (s.cameraId == cameraId) return &s;
    return nullptr;
}

}  // namespace

TEST_CASE(
    "fake backend vision: the cameras follow the robot and the turret and their counters advance") {
    FakeRunBackend backend(options());
    backend.startSimulation();
    backend.tick(400);  // two seconds of simulated time
    auto snap = backend.latestSnapshot();
    REQUIRE(snap != nullptr);
    REQUIRE(snap->cameras().size() == 2);
    const sdk::CameraSample& front = snap->cameras()[0];
    const sdk::CameraSample& turret = snap->cameras()[1];
    CHECK(front.id == "front");
    CHECK(front.vendor == sdk::CameraVendor::PhotonVision);
    CHECK(turret.id == "turret_cam");
    CHECK(turret.vendor == sdk::CameraVendor::Limelight);
    CHECK(front.hfovRad == doctest::Approx(1.4));
    CHECK(front.vfovRad == doctest::Approx(0.97).epsilon(0.01));
    CHECK(front.widthPx == 1280);

    // On the chassis: the pose is the chassis pose times the mount.
    CHECK(near(front.pose.position, snap->chassis().pose.transformPoint({0.33f, 0.f, 0.16f})));

    // 30 and 90 captures a second, 25 and 20 ms behind; the frames still in flight have not been
    // published yet.
    CHECK(front.framesCaptured == 60);
    CHECK(turret.framesCaptured == 180);
    CHECK(front.framesPublished == 59);
    CHECK(turret.framesPublished == 178);
    CHECK(front.lastLatencyMs == doctest::Approx(25.0));
    CHECK(turret.lastLatencyMs == doctest::Approx(20.0));
    CHECK(front.robotConnected);
    CHECK(snap->fiducials().size() == 18);

    // The Limelight sits on the turret link, so it turns with the turret.
    sdk::proto::DsState ds;
    ds.enabled = true;
    backend.setDriverStation(ds);
    backend.tick(200);
    snap = backend.latestSnapshot();
    const sdk::Quat turretYaw = snap->mechanisms()[1].links[0].pose.rotation;
    CHECK(std::fabs(turretYaw.yaw()) > 0.1f);
    const sdk::Vec3 mount = sdk::Vec3{-0.1f, 0.f, 0.f} + turretYaw.rotate({0.25f, 0.f, 0.2f});
    CHECK(near(snap->cameras()[1].pose.position, snap->chassis().pose.transformPoint(mount)));
}

TEST_CASE("fake backend vision: the tags a camera sees change as the robot moves") {
    FakeRunBackend backend(options());
    backend.startSimulation();
    backend.setRobotPose(
        sdk::Pose3({14.f, 4.115f, 0.2f}, sdk::Quat::fromYaw(0.f)));  // facing the red wall
    backend.tick(30);
    std::vector<int> ids = backend.latestSnapshot()->cameras()[0].visibleTagIds;
    CHECK(contains(ids, 5));        // the centre tag of the red wall, straight ahead
    CHECK_FALSE(contains(ids, 9));  // the blue hub is behind the robot

    backend.setRobotPose(
        sdk::Pose3({2.f, 4.115f, 0.2f}, sdk::Quat::fromYaw(0.f)));  // facing the blue hub
    backend.tick(30);
    ids = backend.latestSnapshot()->cameras()[0].visibleTagIds;
    CHECK(contains(ids, 9));  // the hub tag that faces the robot
    CHECK_FALSE(contains(ids, 5));
    CHECK_FALSE(contains(ids, 7));  // the one on the far side of the hub faces away
}

TEST_CASE(
    "fake backend vision: injected camera faults reach the snapshot, the detected states and the "
    "warnings") {
    FakeRunBackend backend(options());
    backend.startSimulation();
    backend.tick(200);
    {
        const auto snap = backend.latestSnapshot();
        CHECK(snap->warnings().empty());
        REQUIRE(snap->detectedCameras().size() == 2);
        const sdk::CameraRobotState* front = findDetected(*snap, "front");
        REQUIRE(front != nullptr);
        CHECK(front->ntName == "photon-front");
        CHECK(front->present);
        CHECK(front->publishing);
        CHECK(front->pipelineIndex == 0);
        CHECK(front->driverMode == false);
    }

    // The robot code holds driver mode (button 3) and picks pipeline 1 (button 4).
    sdk::proto::DsState ds;
    ds.joysticks[0].buttons = 4u | 8u;
    backend.setDriverStation(ds);
    backend.tick(2);
    CHECK(findDetected(*backend.latestSnapshot(), "front")->driverMode == true);
    CHECK(findDetected(*backend.latestSnapshot(), "front")->pipelineIndex == 1);

    using Fault = FakeRunBackend::CameraFault;
    backend.simulateCameraFault(QStringLiteral("front"), Fault::PhotonUuidUnsupported);
    {
        const auto snap = backend.latestSnapshot();
        CHECK(snap->cameras()[0].status == "photon_uuid_unsupported");
        CHECK_FALSE(snap->cameras()[0].robotConnected);
        CHECK(snap->cameras()[1].robotConnected);  // the other camera is untouched
        const sdk::HubWarning* w = findWarning(*snap, code::kVisionCameraStatus);
        REQUIRE(w != nullptr);
        CHECK(w->severity == sdk::Severity::Error);
        CHECK(findDetected(*snap, "front")->status == "photon_uuid_unsupported");
        CHECK_FALSE(findDetected(*snap, "front")->publishing);
    }
    backend.simulateCameraFault(QStringLiteral("front"), Fault::ForeignPublisher);
    CHECK(backend.latestSnapshot()->cameras()[0].status == "foreign_publisher");
    CHECK(findWarning(*backend.latestSnapshot(), code::kVisionCameraStatus)->severity ==
          sdk::Severity::Warning);

    backend.simulateCameraFault(QStringLiteral("front"), Fault::Disabled);
    {
        const auto snap = backend.latestSnapshot();
        CHECK_FALSE(snap->cameras()[0].enabled);
        CHECK(snap->cameras()[0].status == "disabled");
        CHECK(snap->cameras()[0].visibleTagIds.empty());
        CHECK(findDetected(*snap, "front") == nullptr);
        CHECK(findWarning(*snap, code::kVisionCameraStatus) ==
              nullptr);  // switching a camera off is not a problem
    }
    backend.simulateCameraFault(QStringLiteral("front"), Fault::NotConnected);
    CHECK(backend.latestSnapshot()->cameras()[0].status.empty());
    CHECK_FALSE(backend.latestSnapshot()->cameras()[0].robotConnected);
    backend.simulateCameraFault(QStringLiteral("front"), Fault::NotUsedByRobot);
    CHECK(backend.latestSnapshot()->cameras()[0].robotConnected);
    CHECK_FALSE(findDetected(*backend.latestSnapshot(), "front")->present);

    backend.resetSimulation();  // faults are not part of the simulation state
    backend.tick(100);          // the robot code needs a moment to create its cameras again
    CHECK_FALSE(findDetected(*backend.latestSnapshot(), "front")->present);
    backend.simulateCameraFault(QStringLiteral("front"), Fault::None);
    backend.simulateCameraFault(QStringLiteral("no such camera"), Fault::Disabled);  // ignored
    CHECK(backend.latestSnapshot()->warnings().empty());
    CHECK(findDetected(*backend.latestSnapshot(), "front")->present);

    backend.simulateUnconfiguredCamera(QStringLiteral("photon-extra"));
    {
        const auto snap = backend.latestSnapshot();
        const sdk::HubWarning* w = findWarning(*snap, code::kVisionCameraUnconfigured);
        REQUIRE(w != nullptr);
        CHECK(w->message.find("photon-extra") != std::string::npos);
        CHECK(snap->detectedCameras().size() == 3);
        CHECK(snap->detectedCameras().back().cameraId.empty());  // a detection nobody configured
    }
    backend.simulateUnconfiguredCamera({});
    CHECK(backend.latestSnapshot()->warnings().empty());

    const std::uint64_t capturedBefore = backend.latestSnapshot()->cameras()[0].framesCaptured;
    backend.simulateLayoutMissing();
    backend.tick(30);
    {
        const auto snap = backend.latestSnapshot();
        CHECK(snap->fiducials().empty());
        CHECK(snap->cameras()[0].visibleTagIds.empty());
        CHECK(snap->cameras()[0].framesCaptured >
              capturedBefore);  // it still captures, it just sees nothing
        const sdk::HubWarning* w = findWarning(*snap, code::kVisionLayoutMissing);
        REQUIRE(w != nullptr);
        CHECK(w->severity == sdk::Severity::Error);
    }
    backend.simulateLayoutMissing(false);
    CHECK_FALSE(backend.latestSnapshot()->fiducials().empty());
}

TEST_CASE(
    "stand-in vision: a tag is seen inside the field of view, facing the camera and big enough") {
    namespace dv = wpiemu::app::demo_vision;
    const sdk::CameraConfig
        camera;            // 1280 x 800, 90 degrees across, 12 px minimum, 75 degrees view angle
    const sdk::Pose3 eye;  // at the origin, looking along +X
    sdk::FiducialSample tag;
    tag.id = 1;
    tag.sizeM = 0.1651f;
    tag.pose = {{3.f, 0.f, 0.f},
                sdk::Quat::fromYaw(static_cast<float>(
                    sdk::kPi))};  // 3 m ahead, its normal pointing back at the camera
    CHECK(dv::seesTag(camera, eye, tag));

    sdk::FiducialSample moved = tag;
    moved.pose.position = {3.f, 5.f,
                           0.f};  // 59 degrees to the side, outside the 45 degree half angle
    CHECK_FALSE(dv::seesTag(camera, eye, moved));
    moved.pose.position = {-3.f, 0.f, 0.f};  // behind
    CHECK_FALSE(dv::seesTag(camera, eye, moved));
    moved = tag;
    moved.pose.rotation = sdk::Quat::fromYaw(0.f);  // facing away
    CHECK_FALSE(dv::seesTag(camera, eye, moved));
    moved.pose.rotation = sdk::Quat::fromYaw(static_cast<float>(sdk::kPi) +
                                             1.4f);  // 80 degrees off the camera: too grazing
    CHECK_FALSE(dv::seesTag(camera, eye, moved));
    moved = tag;
    moved.pose.position = {40.f, 0.f, 0.f};  // under 12 pixels wide
    CHECK_FALSE(dv::seesTag(camera, eye, moved));

    const std::vector<sdk::FiducialSample> layout{tag, moved};
    CHECK(dv::visibleTagIds(camera, eye, layout) == std::vector<int>{1});
    CHECK(dv::visibleTagIds(camera, eye, layout, [](const sdk::Vec3&, const sdk::Vec3&) {
              return true;
          }).empty());  // blocked
    sdk::CameraConfig seeThrough = camera;
    seeThrough.occlusion = false;
    CHECK(dv::visibleTagIds(seeThrough, eye, layout, [](const sdk::Vec3&, const sdk::Vec3&) {
              return true;
          }) == std::vector<int>{1});
}

TEST_CASE("fake backend vision: it can run without any camera") {
    FakeRunBackend withVision(options());
    CHECK(withVision.sceneInfo().config->vision.cameras.size() == 2);

    FakeRunBackend::Options plain = options();
    plain.withVision = false;
    FakeRunBackend backend(plain);
    CHECK(backend.sceneInfo().config->vision.empty());
    backend.startSimulation();
    backend.tick(50);
    const auto snap = backend.latestSnapshot();
    CHECK(snap->cameras().empty());
    CHECK(snap->fiducials().empty());
    CHECK(snap->detectedCameras().empty());
}
