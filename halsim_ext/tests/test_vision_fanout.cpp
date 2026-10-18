#include <doctest/doctest.h>

#include "fakes.h"
#include "wpiemu/halsim_ext/bridge_set.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {
sdk::VisionSetup twoCameras() {
    sdk::VisionSetup setup;
    sdk::CameraSetup a;
    a.config.id = "front";
    a.config.name = "front";
    sdk::CameraSetup b;
    b.config.id = "rear";
    b.config.kind = sdk::CameraKind::PhotonVision;
    b.config.name = "photon-rear";
    setup.cameras = {a, b};
    setup.photonVersion = "v2026.3.4";
    return setup;
}

sdk::CameraFrame frameFor(std::uint32_t camera, sdk::SimTimeUs captureUs, sdk::SimTimeUs publishUs) {
    sdk::CameraFrame f;
    f.cameraIndex = camera;
    f.captureTimeUs = captureUs;
    f.publishTimeUs = publishUs;
    sdk::TagDetection tag;
    tag.id = 7;
    f.tags.push_back(tag);
    return f;
}
}  // namespace

TEST_CASE("vision fan-out: a bridge set starts, configures, applies, resets and stops its vision bridges") {
    DeviceTable table;
    sdk::NullLogger logger;
    std::atomic<bool> hooks{false};
    BridgeSet set({}, table, logger);
    std::vector<std::unique_ptr<sdk::IVisionBridge>> vision;
    auto a = std::make_unique<FakeVisionBridge>("vision.a");
    auto b = std::make_unique<FakeVisionBridge>("vision.b");
    FakeVisionBridge* ra = a.get();
    FakeVisionBridge* rb = b.get();
    vision.push_back(std::move(a));
    vision.push_back(std::move(b));
    set.setVisionBridges(std::move(vision));

    // A setup that arrives before start is remembered and replayed to the started bridges.
    set.configureVision(twoCameras());
    set.start(hooks);
    CHECK(set.activeVisionCount() == 2);
    CHECK(ra->configures == 1);
    CHECK(rb->lastSetup.cameras.size() == 2);
    CHECK(rb->lastSetup.photonVersion == "v2026.3.4");

    set.configureVision(twoCameras());
    CHECK(ra->configures == 2);

    const std::vector<sdk::CameraFrame> frames{frameFor(0, 1000, 41000)};
    set.pollVision(41000);
    set.applyVision(frames, 41000);
    CHECK(ra->polls == 1);
    CHECK(ra->lastApplyUs == 41000);
    REQUIRE(rb->frameLog.size() == 1);
    CHECK(rb->frameLog[0].tags[0].id == 7);

    set.resetInputHistory();
    CHECK(ra->resets == 1);
    set.stop();
    set.stop();
    CHECK(ra->stops >= 1);
    set.applyVision(frames, 42000);  // stopped bridges are not driven
    CHECK(ra->applies == 1);
}

TEST_CASE("vision fan-out: a vision bridge that fails to start is dropped and the others continue") {
    DeviceTable table;
    sdk::NullLogger logger;
    std::atomic<bool> hooks{false};
    BridgeSet set({}, table, logger);
    std::vector<std::unique_ptr<sdk::IVisionBridge>> vision;
    auto bad = std::make_unique<FakeVisionBridge>("vision.bad");
    bad->failStart = true;
    auto good = std::make_unique<FakeVisionBridge>("vision.good");
    FakeVisionBridge* rbad = bad.get();
    FakeVisionBridge* rgood = good.get();
    vision.push_back(std::move(bad));
    vision.push_back(std::move(good));
    set.setVisionBridges(std::move(vision));
    set.start(hooks);
    CHECK(set.activeVisionCount() == 1);

    const std::vector<sdk::CameraFrame> frames{frameFor(0, 0, 40000)};
    set.applyVision(frames, 40000);
    CHECK(rbad->applies == 0);
    CHECK(rgood->applies == 1);
}

TEST_CASE("vision session: VisionSetup and SensorFrame.vision reach the vision bridges in lock-step") {
    Rig rig;
    FakeVisionBridge& cams = rig.addVisionBridge();
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    REQUIRE(rig.emulator->send(proto::Message(twoCameras())));
    rig.emulator->pendingVision = {frameFor(1, 5000, 45000)};
    const auto ack = rig.emulator->tick();
    REQUIRE(ack.has_value());
    CHECK(ack->result == proto::TickResult::Completed);
    CHECK(cams.configures >= 1);
    CHECK(cams.lastSetup.cameras.size() == 2);
    CHECK(cams.applies >= 1);
    REQUIRE(cams.frameLog.size() == 1);
    CHECK(cams.frameLog[0].cameraIndex == 1);
    CHECK(cams.frameLog[0].publishTimeUs == 45000);
    CHECK(cams.polls >= 1);
    // The bridge ran before the robot was stepped (cameras are published before HALSIM_StepTiming).
    CHECK(rig.clock.steps == 1);
}

TEST_CASE("vision session: a Reset epoch resets the vision bridges before the new epoch's frames") {
    Rig rig;
    FakeVisionBridge& cams = rig.addVisionBridge();
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->tick().has_value());
    const int before = cams.resets;

    rig.emulator->epoch += 1;  // a Reset opens a new epoch
    REQUIRE(rig.emulator->sendControl(proto::SimCommand::Reset, 1));
    REQUIRE(rig.emulator->take<proto::SimControlAck>(2000ms).has_value());
    CHECK(cams.resets > before);
}

TEST_CASE("vision session: no vision bridges registered is harmless") {
    Rig rig;
    FakeBridge& bridge = rig.addBridge();
    (void)bridge;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->send(proto::Message(twoCameras())));
    rig.emulator->pendingVision = {frameFor(0, 0, 40000)};
    const auto ack = rig.emulator->tick();
    REQUIRE(ack.has_value());
    CHECK(ack->result == proto::TickResult::Completed);
}
