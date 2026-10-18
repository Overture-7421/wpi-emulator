#include <doctest/doctest.h>

#include <memory>
#include <vector>

#include "vision_bridge_test_support.h"
#include "wpiemu/bridges/vision/vision_bridges.h"

// Registration and default wiring of the vision bridge module.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE("vision bridges: registerVisionBridges adds exactly one bridge named vision") {
    sdk::DeviceBridgeRegistry registry;
    REQUIRE(registerVisionBridges(registry).ok());
    CHECK(registry.size() == 1);
    CHECK(registry.contains("vision"));
    const sdk::BridgeInfo* info = registry.info("vision");
    REQUIRE(info != nullptr);
    CHECK(info->family == "vision");
    CHECK(info->vendor == "vision");
    CHECK_FALSE(info->description.empty());

    CHECK_FALSE(registerVisionBridges(registry).ok());  // a second registration is a duplicate, never a second bridge
    CHECK(registry.size() == 1);
}

TEST_CASE("vision bridges: the default wiring builds without ntcore and stays inert") {
    sdk::DeviceBridgeRegistry registry;
    REQUIRE(registerVisionBridges(registry).ok());
    auto created = registry.createAll();
    REQUIRE(created.ok());
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges = std::move(created).value();
    REQUIRE(bridges.size() == 1);
    sdk::IDeviceBridge& bridge = *bridges[0];
    CHECK(bridge.name() == "vision");

    FakeVisionHost host;
    host.adopt(setupOf({photonCamera("front", "photon-front")}));
    REQUIRE(bridge.start(host).ok());
    std::vector<sdk::CameraRobotState> out;
    for (int i = 0; i < 50; ++i) {
        bridge.applyVision({});
        bridge.poll();
        out.clear();
        bridge.collectVision(out, false);
    }
    bridge.collectVision(out, true);
    REQUIRE(out.size() == 1);  // the test process has no ntcore: the camera is configured and waiting
    CHECK(out[0].cameraId == "front");
    CHECK(out[0].ntName == "photon-front");
    if (out[0].status == camera_status::kNtcoreUnavailable)
        CHECK(countLogs(host.log, sdk::LogLevel::Warn, "ntcore") == 1);
    CHECK(host.announces == 0);
    bridge.stop();
}

TEST_CASE("vision bridges: the options overload injects the NetworkTables seam and the vendors") {
    auto fake = std::make_shared<testing::FakeNtApi>();
    auto photon = std::make_shared<TestVendor>(sdk::CameraVendor::PhotonVision, "/photonvision/");
    VisionBridgeOptions options;
    options.nt = fake;
    options.vendors = std::vector<std::shared_ptr<IVisionVendor>>{photon};

    sdk::DeviceBridgeRegistry registry;
    REQUIRE(registerVisionBridges(registry, options).ok());
    CHECK(registry.size() == 1);
    CHECK(registry.contains("vision"));
    auto created = registry.createAll();
    REQUIRE(created.ok());
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges = std::move(created).value();
    REQUIRE(bridges.size() == 1);

    FakeVisionHost host;
    host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    REQUIRE(bridges[0]->start(host).ok());
    bridges[0]->poll();
    CHECK(photon->createCalls == 1);           // the injected vendor made the PhotonVision camera ...
    CHECK(fake->exists("/test/front/alive"));  // ... on the injected NetworkTables
    std::vector<sdk::CameraRobotState> out;
    bridges[0]->collectVision(out, true);
    REQUIRE(out.size() == 2);
    CHECK(out[1].status == camera_status::kVendorUnavailable);  // no Limelight vendor in the injected list
    bridges[0]->stop();
}

TEST_CASE(
    "vision bridges: without injected vendors the bridge works with the shipped PhotonVision and Limelight vendors") {
    VisionBridgeOptions options;
    options.nt = std::make_shared<testing::FakeNtApi>();
    VisionBridge bridge(options);
    FakeVisionHost host;
    REQUIRE(bridge.start(host).ok());
    CHECK(bridge.stats().vendors == 2);
    bridge.stop();
}

}  // namespace
