#include <doctest/doctest.h>

#include <cmath>
#include <map>
#include <string>

#include "manual/vision_scenario.h"

using namespace wpiemu;
using namespace wpiemu::halsim::manual;

namespace {

double distance(const sdk::Vec3& a, const sdk::Vec3& b) {
    return sdk::length(a - b);
}

constexpr double kDegToRad = kPi / 180.0;

}  // namespace

TEST_CASE("vision scenario: the setup describes the cameras of the sample robots") {
    const sdk::VisionSetup setup = VisionScenario(5000).setup();
    REQUIRE(setup.cameras.size() == 2);

    const sdk::CameraSetup& photon = setup.cameras[0];
    CHECK(photon.cameraId == "photon-front");
    CHECK(photon.vendor == sdk::CameraVendor::PhotonVision);
    CHECK(photon.ntName == "photon-front");
    CHECK(photon.widthPx == 1280);
    CHECK(photon.heightPx == 800);
    CHECK(photon.frameRateHz == 30.0);

    const sdk::CameraSetup& limelight = setup.cameras[1];
    CHECK(limelight.cameraId == "limelight-rear");
    CHECK(limelight.vendor == sdk::CameraVendor::Limelight);
    CHECK(limelight.ntName == "limelight-rear");
    CHECK(limelight.widthPx == 1280);
    CHECK(limelight.heightPx == 800);
    CHECK(limelight.frameRateHz == 90.0);

    // The mounts of samples/vision_robot/README.md: PhotonVision forward and 20 deg up, Limelight
    // behind the robot, looking backwards and 25 deg up (the setup carries the pose the Limelight
    // starts with).
    const sdk::Vec3 photonAxis =
        photon.defaultAssumedRobotToCamera.transformVector({1.0f, 0.0f, 0.0f});
    CHECK(photon.defaultAssumedRobotToCamera.position == sdk::Vec3(0.30f, 0.0f, 0.50f));
    CHECK(photonAxis.x == doctest::Approx(std::cos(20.0 * kDegToRad)));
    CHECK(photonAxis.z == doctest::Approx(std::sin(20.0 * kDegToRad)));
    const sdk::Vec3 limelightAxis =
        limelight.defaultAssumedRobotToCamera.transformVector({1.0f, 0.0f, 0.0f});
    CHECK(limelight.defaultAssumedRobotToCamera.position == sdk::Vec3(-0.30f, 0.0f, 0.45f));
    CHECK(limelightAxis.x == doctest::Approx(-std::cos(25.0 * kDegToRad)));
    CHECK(limelightAxis.z == doctest::Approx(std::sin(25.0 * kDegToRad)));
}

TEST_CASE("vision scenario: each camera sees its two tags in front of it, inside the image") {
    for (const ScenarioCamera& camera : makeScenarioCameras()) {
        CAPTURE(camera.setup.cameraId);
        const sdk::VisionFrame frame = captureFrame(camera, robotPose(), 1, 0, 25000);
        REQUIRE(frame.observations.size() == 2);
        CHECK(frame.multiTag.valid);
        CHECK(frame.multiTag.tagIds.size() == 2);
        CHECK(frame.observations[0].areaPct >= frame.observations[1].areaPct);

        const sdk::Pose3 fieldToCamera = robotPose() * camera.robotToCamera;
        CHECK(frame.multiTag.fieldToCamera == fieldToCamera);
        for (const sdk::FiducialObservation& o : frame.observations) {
            CAPTURE(o.id);
            CHECK(o.cameraToTagBest.position.x > 0.0f);
            CHECK(o.distanceM == doctest::Approx(sdk::length(o.cameraToTagBest.position)));
            CHECK(o.areaPct > 0.0);
            for (const sdk::PixelPoint& corner : o.cornersPx) {
                CHECK(corner.x >= 0.0);
                CHECK(corner.x <= camera.setup.widthPx);
                CHECK(corner.y >= 0.0);
                CHECK(corner.y <= camera.setup.heightPx);
            }
            // Counter-clockwise from the bottom left as seen from the front: BL BR TR TL (image y
            // points down).
            CHECK(o.cornersPx[0].x < o.cornersPx[1].x);
            CHECK(o.cornersPx[0].y > o.cornersPx[3].y);

            // The transforms close the loop: field -> camera -> tag is the layout pose.
            const sdk::Pose3 fieldToTag = fieldToCamera * o.cameraToTagBest;
            CHECK(distance(fieldToTag.position, o.fieldToTag.position) < 1e-4);
            CHECK(distance(fieldToTag.transformVector({1.0f, 0.0f, 0.0f}),
                           o.fieldToTag.transformVector({1.0f, 0.0f, 0.0f})) < 1e-4);
        }
    }
}

TEST_CASE("vision scenario: frames follow the camera rates and the latency in ticks") {
    constexpr std::uint32_t kStepUs = 5000;
    VisionScenario scenario(kStepUs);
    std::map<std::string, int> frames;
    std::map<std::string, std::uint64_t> lastSequence;
    constexpr std::uint64_t kTicks = 400;  // 2 s of simulated time
    for (std::uint64_t tick = 1; tick <= kTicks; ++tick) {
        for (const sdk::VisionFrame& f : scenario.framesFor(tick)) {
            ++frames[f.cameraId];
            CHECK(f.sequence == ++lastSequence[f.cameraId]);
            CHECK(f.latencyUs == 25000);  // five ticks
            // latencyUs = T(N-1) - captureTimeUs for the frame that rides SensorFrame N.
            CHECK(f.captureTimeUs + f.latencyUs == static_cast<std::int64_t>(tick - 1) * kStepUs);
            CHECK(f.captureLatencyUs == 10000);
        }
    }
    // Captures up to tick 400 - 6 have been published: 1.97 s of frames.
    CHECK(std::abs(frames["photon-front"] - 59) <= 1);
    CHECK(std::abs(frames["limelight-rear"] - 177) <= 1);
}
