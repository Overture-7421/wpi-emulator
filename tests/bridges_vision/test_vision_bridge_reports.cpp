#include <doctest/doctest.h>

#include <array>
#include <functional>
#include <vector>

#include "vision_bridge_test_support.h"

// collectVision: what the bridge reports about each configured camera, in full and as deltas.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE(
    "vision bridge: configured cameras report ntcore_unavailable until it binds, then what their publisher says") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front", "photon-front"), limelightCamera("rear", "limelight-rear")}));
    rig.fake->setAvailable(false);
    rig.start();
    rig.polls(3);

    auto states = rig.collect(true);
    REQUIRE(states.size() == 2);
    for (const sdk::CameraRobotState& state : states) {
        CAPTURE(state.cameraId);
        CHECK(state.status == camera_status::kNtcoreUnavailable);
        CHECK_FALSE(state.present);  // nothing is known about the robot yet
        CHECK_FALSE(state.publishing);
        CHECK(state.framesPublished == 0);
    }
    CHECK(states[0].ntName == "photon-front");
    CHECK(states[0].vendor == sdk::CameraVendor::PhotonVision);
    CHECK(states[1].ntName == "limelight-rear");
    CHECK(states[1].vendor == sdk::CameraVendor::Limelight);

    rig.fake->setAvailable(true);
    rig.polls(1);
    states = rig.collect(false);  // the status changed: both are reported at once
    REQUIRE(states.size() == 2);
    CHECK(states[0].status.empty());
    CHECK(states[0].present);

    rig.photon->probeFor("front")->state.status = std::string(camera_status::kForeignPublisher);
    rig.photon->probeFor("front")->state.present = true;
    states = rig.collect(false);
    REQUIRE(states.size() == 1);
    CHECK(states[0].cameraId == "front");
    CHECK(states[0].status == camera_status::kForeignPublisher);
}

TEST_CASE(
    "vision bridge: a camera whose publisher does not exist yet is not reported as a ntcore problem while ntcore is "
    "up") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.host.started = false;  // ntcore is bound, the robot constructor is still running
    rig.start();
    rig.polls(3);
    const auto states = rig.collect(true);
    REQUIRE(states.size() == 1);
    CHECK(states[0].status.empty());
    CHECK_FALSE(states[0].publishing);
    CHECK(rig.photon->createCalls == 0);
}

TEST_CASE("vision bridge: collectVision reports every camera when full and only the changes otherwise") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.start();

    CHECK(rig.tick().size() == 2);  // nothing was ever reported: everything is new
    CHECK(rig.tick().empty());
    CHECK(rig.tick().empty());
    CHECK(rig.tick({}, true).size() == 2);  // a full report lists everything again ...
    CHECK(rig.tick().empty());              // ... and is the new baseline

    const auto probe = rig.photon->probeFor("front");
    probe->state.pipelineIndex = 3;
    const auto delta = rig.tick();
    REQUIRE(delta.size() == 1);  // only the camera that changed
    CHECK(delta[0].cameraId == "front");
    CHECK(delta[0].pipelineIndex == 3);
    CHECK(rig.tick().empty());
}

TEST_CASE("vision bridge: a change of any reported field is sent at once") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.start();
    rig.tick();  // the first report
    const auto probe = rig.photon->probeFor("front");

    const std::vector<std::function<void(sdk::CameraRobotState&)>> changes = {
        [](sdk::CameraRobotState& s) { s.present = false; },
        [](sdk::CameraRobotState& s) { s.pipelineIndex = 2; },
        [](sdk::CameraRobotState& s) { s.ledMode = 1; },
        [](sdk::CameraRobotState& s) { s.imuMode = 3; },
        [](sdk::CameraRobotState& s) { s.driverMode = true; },
        [](sdk::CameraRobotState& s) { s.throttle = 4.0; },
        [](sdk::CameraRobotState& s) { s.fpsLimit = 15.0; },
        [](sdk::CameraRobotState& s) { s.robotOrientationDeg = std::array<double, 6>{45.0, 0.0, 0.0, 0.0, 0.0, 0.0}; },
        [](sdk::CameraRobotState& s) { s.cameraPoseOverride = std::array<double, 6>{0.3, 0.0, 0.5, 0.0, 20.0, 0.0}; },
        [](sdk::CameraRobotState& s) { s.publishing = true; },
        [](sdk::CameraRobotState& s) { s.status = "foreign_publisher"; },
        [](sdk::CameraRobotState& s) { s.ntName = "renamed"; },
    };
    int index = 0;
    for (const auto& change : changes) {
        CAPTURE(index);
        change(probe->state);
        const auto delta = rig.tick();
        REQUIRE(delta.size() == 1);
        CHECK(delta[0] == probe->state);
        CHECK(rig.tick().empty());
        ++index;
    }
}

TEST_CASE("vision bridge: a changing frame counter is reported at most every 250 ms, any other change at once") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front");
    rig.host.adopt(setupOf({front}));
    rig.start();
    rig.fake->setNowUs(1'000'000);
    const auto full = rig.tick({}, true);  // t = 1.005 s: nothing published yet
    REQUIRE(full.size() == 1);
    const auto probe = rig.photon->probeFor("front");

    // A frame every tick for 300 ms. The first one flips `publishing`: reported at once (t = 1.010 s). From then on
    // only the counter moves, so it is refreshed every 250 ms: at t = 1.260 s, with 51 frames behind it.
    std::vector<sdk::CameraRobotState> reports;
    for (std::uint64_t i = 1; i <= 60; ++i) {
        const auto out = rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, i)});
        reports.insert(reports.end(), out.begin(), out.end());
    }
    REQUIRE(reports.size() == 2);
    CHECK(reports[0].publishing);
    CHECK(reports[0].framesPublished == 1);
    CHECK(reports[1].framesPublished == 51);
    CHECK(probe->state.framesPublished == 60);

    // Any other change goes out in the very tick it happens (t = 1.310 s, long before the next refresh), carrying the
    // current counter.
    probe->state.pipelineIndex = 1;
    const auto changed = rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, 61)});
    REQUIRE(changed.size() == 1);
    CHECK(changed[0].pipelineIndex == 1);
    CHECK(changed[0].framesPublished == 61);
    CHECK(rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, 62)}).empty());  // the counter alone: not yet

    // A full report always carries the latest counter.
    const auto latest = rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, 63)}, true);
    REQUIRE(latest.size() == 1);
    CHECK(latest[0].framesPublished == 63);
}

TEST_CASE("vision bridge: collectVision appends to the vector it is given and never clears it") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.fake->robotPublish("/photonvision/back/driverModeRequest", NtType::Boolean);
    rig.start();
    rig.polls(1);

    sdk::CameraRobotState sentinel;
    sentinel.cameraId = "sentinel";
    std::vector<sdk::CameraRobotState> out{sentinel};
    rig.bridge->collectVision(out, true);
    REQUIRE(out.size() == 4);  // sentinel, two configured cameras, one detected
    CHECK(out[0].cameraId == "sentinel");
    CHECK(out[1].cameraId == "front");
    CHECK(out[2].cameraId == "rear");
    CHECK(out[3].cameraId.empty());
    CHECK(out[3].ntName == "back");

    rig.bridge->collectVision(out, true);
    CHECK(out.size() == 7);
    rig.bridge->collectVision(out, false);  // nothing changed since: appends nothing, keeps what is there
    CHECK(out.size() == 7);
    CHECK(out[0].cameraId == "sentinel");
}

}  // namespace
