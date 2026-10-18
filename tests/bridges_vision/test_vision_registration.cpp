#include "vision_test_rig.h"
#include "wpiemu/sdk/devices/builtin_bridges.h"

using namespace vision_test;

TEST_CASE("vision registration: both bridges are registered with the vision family") {
    sdk::VisionBridgeRegistry registry;
    REQUIRE(vis::registerVisionBridges(registry, std::make_shared<vis::testing::FakeNtBackend>()).ok());
    CHECK(registry.contains("vision.limelight"));
    CHECK(registry.contains("vision.photonvision"));
    auto limelight = registry.create("vision.limelight");
    auto photon = registry.create("vision.photonvision");
    REQUIRE(limelight.ok());
    REQUIRE(photon.ok());
    CHECK(limelight.value()->name() == "vision.limelight");
    CHECK(photon.value()->name() == "vision.photonvision");
    CHECK(vis::bridgeFamily() == "vision");

    auto all = registry.createAll();
    REQUIRE(all.ok());
    CHECK(all.value().size() == 2);
}

TEST_CASE("vision registration: a null backend and a duplicate registration are rejected") {
    sdk::VisionBridgeRegistry registry;
    CHECK_FALSE(vis::registerVisionBridges(registry, nullptr).ok());
    CHECK(registry.keys().empty());
    REQUIRE(vis::registerVisionBridges(registry, std::make_shared<vis::testing::FakeNtBackend>()).ok());
    const sdk::Status again = vis::registerVisionBridges(registry, std::make_shared<vis::testing::FakeNtBackend>());
    REQUIRE_FALSE(again.ok());
    CHECK(again.error().code == sdk::ErrorCode::AlreadyExists);
}

TEST_CASE("vision registration: without ntcore the bridges register but never publish") {
    sdk::VisionBridgeRegistry registry;
    REQUIRE(vis::registerVisionBridges(registry).ok());
    auto limelight = registry.create("vision.limelight");
    REQUIRE(limelight.ok());
    sdk::NullLogger logger;
    REQUIRE(limelight.value()->start(logger).ok());
    limelight.value()->configure(makeSetup({makeCamera(sdk::CameraKind::Limelight, "")}));
    limelight.value()->poll(0);
    limelight.value()->apply(std::vector<sdk::CameraFrame>{makeFrame(0, {})}, 0);
    limelight.value()->reset();
    limelight.value()->stop();
}
