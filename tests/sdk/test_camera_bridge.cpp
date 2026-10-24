#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "wpiemu/sdk/vision/camera_bridge.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

using namespace wpiemu::sdk;

namespace {

struct TestCam : CameraBridgeState {
    int mirrored = 0;
};

VisionSetup setupOf(std::vector<CameraKind> kinds) {
    VisionSetup setup;
    for (std::size_t i = 0; i < kinds.size(); ++i) {
        CameraConfig config;
        config.kind = kinds[i];
        config.name = "cam" + std::to_string(i);
        setup.cameras.push_back(CameraSetup{config});
    }
    return setup;
}

std::string rootOf(const CameraConfig& config) { return "/" + config.name; }

}  // namespace

TEST_CASE("camera set keeps the setup's indices and serves only its kind") {
    CameraSet<TestCam> cams;
    CHECK(cams.empty());
    cams.configure(setupOf({CameraKind::PhotonVision, CameraKind::Limelight, CameraKind::PhotonVision}),
                   CameraKind::PhotonVision, rootOf);
    CHECK_FALSE(cams.empty());
    REQUIRE(cams.find(0) != nullptr);
    CHECK(cams.find(0)->root == "/cam0");
    CHECK(cams.find(0)->config.kind == CameraKind::PhotonVision);
    CHECK(cams.find(1) == nullptr);
    REQUIRE(cams.find(2) != nullptr);
    CHECK(cams.find(2)->root == "/cam2");
    CHECK(cams.find(3) == nullptr);

    int visited = 0;
    cams.forEach([&](const TestCam&) { ++visited; });
    CHECK(visited == 2);
}

TEST_CASE("camera set is empty for a setup without cameras and after clear") {
    CameraSet<TestCam> cams;
    cams.configure(VisionSetup{}, CameraKind::Limelight, rootOf);
    CHECK(cams.empty());
    cams.configure(setupOf({CameraKind::Limelight}), CameraKind::Limelight, rootOf);
    CHECK_FALSE(cams.empty());
    cams.clear();
    CHECK(cams.empty());
    CHECK(cams.find(0) == nullptr);
}

TEST_CASE("reconfiguring a camera set drops the previous state") {
    CameraSet<TestCam> cams;
    cams.configure(setupOf({CameraKind::Limelight}), CameraKind::Limelight, rootOf);
    cams.find(0)->mirrored = 5;
    cams.find(0)->initialized = true;
    cams.configure(setupOf({CameraKind::Limelight}), CameraKind::Limelight, rootOf);
    CHECK(cams.find(0)->mirrored == 0);
    CHECK_FALSE(cams.find(0)->initialized);
}

TEST_CASE("heartbeat beats on the first call and then once per period") {
    CameraBridgeState cam;
    CHECK(cam.beat(0, 100));
    CHECK(cam.heartbeat == 1);
    CHECK_FALSE(cam.beat(99, 100));
    CHECK(cam.heartbeat == 1);
    CHECK(cam.beat(100, 100));
    CHECK(cam.beat(450, 100));  // a late tick counts one beat and re-anchors the cadence on itself
    CHECK_FALSE(cam.beat(549, 100));
    CHECK(cam.beat(550, 100));
    CHECK(cam.heartbeat == 4);

    cam.restartHeartbeat();
    CHECK(cam.heartbeat == 0);
    CHECK(cam.beat(0, 100));
}

TEST_CASE("reset republishes only cameras that had their topics") {
    CameraSet<TestCam> cams;
    cams.configure(setupOf({CameraKind::Limelight, CameraKind::Limelight}), CameraKind::Limelight, rootOf);
    cams.find(0)->initialized = true;
    cams.find(0)->nextHeartbeatUs = 900;
    cams.find(1)->nextHeartbeatUs = 900;

    std::vector<std::string> republished;
    cams.reset([&](TestCam& cam) {
        CHECK_FALSE(cam.initialized);  // already cleared, so republishing can initialize it again
        republished.push_back(cam.root);
    });

    CHECK(republished == std::vector<std::string>{"/cam0"});
    CHECK_FALSE(cams.find(0)->initialized);
    CHECK(cams.find(0)->nextHeartbeatUs == 0);
    CHECK(cams.find(1)->nextHeartbeatUs == 0);
}
