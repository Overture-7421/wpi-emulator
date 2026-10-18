#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "vision_bridge_test_support.h"

// applyVision: routing by camera id and the accounting of frames nobody could publish.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE("vision bridge: frames are routed in order, without copies, to the publisher of their camera") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front", "photon-front");
    const sdk::CameraSetup rear = limelightCamera("rear", "limelight-rear");
    rig.host.adopt(setupOf({front, rear}));
    rig.start();
    rig.polls(1);

    const std::vector<sdk::VisionFrame> frames{frameFor(front, 1), frameFor(rear, 1), frameFor(front, 2)};
    REQUIRE(frames[0].observations.size() == 2);  // real frames: the tags of the sample scene
    rig.events.clear();
    rig.bridge->applyVision(frames);

    const auto probeFront = rig.photon->probeFor("front");
    const auto probeRear = rig.limelight->probeFor("rear");
    CHECK(probeFront->sequences == std::vector<std::uint64_t>{1, 2});
    CHECK(probeRear->sequences == std::vector<std::uint64_t>{1});
    CHECK(rig.events == std::vector<std::string>{"publish:1:front", "publish:1:rear", "publish:2:front"});
    REQUIRE(probeFront->frameAddresses.size() == 2);
    CHECK(probeFront->frameAddresses[0] == &frames[0]);  // the publisher saw the emulator's frame itself, not a copy
    CHECK(probeFront->frameAddresses[1] == &frames[2]);
    CHECK(probeRear->frameAddresses[0] == &frames[1]);
    CHECK(rig.bridge->stats().framesRouted == 3);
    CHECK(rig.bridge->stats().framesDropped == 0);
}

TEST_CASE("vision bridge: frames of unknown cameras are dropped and counted, with one warning per camera") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front");
    rig.host.adopt(setupOf({front}));
    rig.start();
    rig.polls(1);

    const sdk::CameraSetup ghost = photonCamera("ghost");
    const sdk::CameraSetup phantom = limelightCamera("phantom");
    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(ghost, 1), frameFor(front, 1), frameFor(ghost, 2)});
    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(ghost, 3), frameFor(phantom, 1)});

    CHECK(rig.bridge->stats().framesDropped == 4);
    CHECK(rig.bridge->stats().framesRouted == 1);
    CHECK(rig.photon->probeFor("front")->sequences == std::vector<std::uint64_t>{1});
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "'ghost'") == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "'phantom'") == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn) == 2);
}

TEST_CASE("vision bridge: frames are dropped and counted while the camera has no publisher yet") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front");
    rig.host.adopt(setupOf({front}));
    rig.fake->setAvailable(false);
    rig.start();
    rig.polls(2);

    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(front, 1), frameFor(front, 2)});
    CHECK(rig.bridge->stats().framesDropped == 2);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "dropping frames") == 0);  // its state already says why

    rig.fake->setAvailable(true);
    rig.polls(1);
    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(front, 3)});
    CHECK(rig.bridge->stats().framesDropped == 2);
    CHECK(rig.bridge->stats().framesRouted == 1);
    CHECK(rig.photon->probeFor("front")->sequences == std::vector<std::uint64_t>{3});
}

TEST_CASE("vision bridge: an empty frame list is free") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.start();
    rig.polls(2);
    const NtCalls ntBefore = rig.nt->calls;
    const VisionBridgeStats statsBefore = rig.bridge->stats();
    for (int i = 0; i < 100; ++i) rig.bridge->applyVision({});
    CHECK(rig.nt->calls.total() == ntBefore.total());
    CHECK(rig.bridge->stats().framesRouted == statsBefore.framesRouted);
    CHECK(rig.bridge->stats().framesDropped == statsBefore.framesDropped);
    CHECK(rig.photon->probeFor("front")->sequences.empty());
}

}  // namespace
