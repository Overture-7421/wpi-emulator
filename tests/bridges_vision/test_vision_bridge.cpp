#include <doctest/doctest.h>

#include <vector>

#include "vision_bridge_test_support.h"

// Lifecycle of the vision bridge core: when cameras are created, how a new HelloAck is reconciled, and what stop()
// freezes. The vendors and publishers are test doubles (the real ones are other work streams); ntcore is a FakeNtApi.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE("vision bridge: start() touches neither ntcore nor the host beyond its logger") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.start();
    CHECK(rig.nt->calls.total() == 0);
    CHECK(rig.host.announces == 0);
    CHECK(rig.photon->createCalls == 0);
    CHECK(rig.bridge->stats().vendors == 2);
}

TEST_CASE(
    "vision bridge: nothing is created before ntcore binds and the program started, then each camera starts once") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front", "photon-front"), limelightCamera("rear", "limelight-rear")}));
    rig.host.started = false;
    rig.fake->setAvailable(false);
    rig.start();

    rig.polls(3);  // neither
    rig.fake->setAvailable(true);
    rig.polls(3);  // ntcore bound, the robot constructor is still running
    rig.host.started = true;
    rig.fake->setAvailable(false);
    rig.polls(3);  // the program started, ntcore is not there
    CHECK(rig.photon->createCalls == 0);
    CHECK(rig.limelight->createCalls == 0);
    CHECK(rig.events.empty());

    rig.fake->setAvailable(true);
    rig.polls(1);  // the first poll where both hold
    const auto front = rig.photon->probeFor("front");
    const auto rear = rig.limelight->probeFor("rear");
    REQUIRE(front != nullptr);
    REQUIRE(rear != nullptr);
    CHECK(front->starts == 1);
    CHECK(rear->starts == 1);
    CHECK(front->polls == 1);  // started publishers are polled in the same poll
    CHECK(rig.bridge->stats().publishersStarted == 2);
    for (const char* id : {"front", "rear"}) {
        CAPTURE(id);
        const int created = rig.eventIndex(std::string("create:") + id);
        const int started = rig.eventIndex(std::string("start:") + id);
        const int polled = rig.eventIndex(std::string("poll:") + id);
        CHECK(created >= 0);
        CHECK(created < started);
        CHECK(started < polled);
    }
    CHECK(rig.fake->exists("/test/front/alive"));

    rig.polls(10);
    CHECK(front->starts == 1);  // exactly one start() per camera, however long it runs
    CHECK(front->polls == 11);
    CHECK(rear->starts == 1);
    CHECK(rig.photon->createCalls == 1);
    CHECK(rig.limelight->createCalls == 1);
    CHECK(rig.nt->calls.subscribe ==
          0);  // the bridge itself publishes and subscribes to nothing: that is the publishers' job
    CHECK(rig.nt->calls.set == 0);
}

TEST_CASE("vision bridge: calls before start() do nothing") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.bridge = std::make_unique<VisionBridge>(rig.options);  // never started
    const std::vector<sdk::VisionFrame> frames{frameFor(photonCamera("front"), 1)};
    const auto out = rig.tick(frames, true);
    CHECK(out.empty());
    CHECK(rig.nt->calls.total() == 0);
    CHECK(rig.photon->createCalls == 0);
    CHECK(rig.bridge->stats().framesDropped == 1);
}

TEST_CASE("vision bridge: a new HelloAck keeps unchanged cameras, recreates changed ones and silences removed ones") {
    VisionRig rig;
    const sdk::CameraSetup a = photonCamera("a", "photon-a");
    const sdk::CameraSetup b = limelightCamera("b", "limelight-b");
    const sdk::CameraSetup c = photonCamera("c", "photon-c");
    rig.host.adopt(setupOf({a, b, c}));
    rig.start();
    rig.polls(2);
    const auto probeA = rig.photon->probeFor("a");
    const auto probeB = rig.limelight->probeFor("b");
    const auto probeC = rig.photon->probeFor("c");
    REQUIRE(probeA != nullptr);
    REQUIRE(probeB != nullptr);
    REQUIRE(probeC != nullptr);
    const int pollsB = probeB->polls;
    const int pollsC = probeC->polls;
    const int publishesBefore = rig.nt->calls.publish;

    // The emulator reconnects with another setup: a unchanged, b changed (latency), c removed, d new.
    sdk::CameraSetup b2 = b;
    b2.latencyUs = 40000;
    const sdk::CameraSetup d = photonCamera("d", "photon-d");
    rig.host.adopt(setupOf({a, b2, d}));
    rig.polls(1);

    CHECK(rig.photon->probeCount("a") == 1);  // kept: same publisher, never restarted
    CHECK(probeA->starts == 1);
    CHECK(probeA->polls == 3);

    CHECK(rig.limelight->probeCount("b") == 2);  // changed: a new publisher from the new setup
    const auto newB = rig.limelight->probeFor("b");
    CHECK(newB != probeB);
    CHECK(newB->starts == 1);
    CHECK(newB->setup.latencyUs == 40000);
    CHECK(probeB->polls == pollsB);  // the old one is quiet

    CHECK(probeC->polls == pollsC);  // removed: quiet
    CHECK(rig.photon->probeCount("c") == 1);

    REQUIRE(rig.photon->probeFor("d") != nullptr);  // new
    CHECK(rig.photon->probeFor("d")->starts == 1);
    CHECK(rig.bridge->stats().publishersStarted == 5);

    // Nothing is ever unpublished: the old topics stay. The only NT calls since are the new cameras' own starts.
    CHECK(rig.fake->exists("/test/c/alive"));
    CHECK(rig.fake->exists("/test/b/alive"));
    CHECK(rig.nt->calls.publish == publishesBefore + 2);

    // Frames follow the new setup.
    const std::vector<sdk::VisionFrame> frames{frameFor(a, 1), frameFor(b2, 1), frameFor(c, 1), frameFor(d, 1)};
    rig.bridge->applyVision(frames);
    CHECK(probeA->sequences.size() == 1);
    CHECK(newB->sequences.size() == 1);
    CHECK(probeB->sequences.empty());
    CHECK(probeC->sequences.empty());
    CHECK(rig.photon->probeFor("d")->sequences.size() == 1);
    CHECK(rig.bridge->stats().framesDropped == 1);  // c is gone
}

TEST_CASE("vision bridge: a new revision with an unchanged setup keeps every publisher") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.start();
    rig.polls(2);
    const std::uint64_t started = rig.bridge->stats().publishersStarted;
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));  // reconnect, same HelloAck content
    rig.polls(2);
    CHECK(rig.bridge->stats().publishersStarted == started);
    CHECK(rig.photon->probeCount("front") == 1);
    CHECK(rig.limelight->probeCount("rear") == 1);
    CHECK(rig.photon->probeFor("front")->polls == 4);
}

TEST_CASE("vision bridge: a setup that changes while the cameras are not yet created is adopted as it stands") {
    VisionRig rig;
    rig.fake->setAvailable(false);
    rig.host.adopt(setupOf({photonCamera("old")}));
    rig.start();
    rig.polls(2);
    rig.host.adopt(setupOf({photonCamera("new")}));  // a reconnect before ntcore ever bound
    rig.polls(2);
    rig.fake->setAvailable(true);
    rig.polls(1);
    CHECK(rig.photon->probeCount("old") == 0);
    CHECK(rig.photon->probeCount("new") == 1);
    CHECK(rig.bridge->stats().publishersStarted == 1);
}

TEST_CASE("vision bridge: a camera listed twice in the setup is created once and the duplicate is warned about") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front"), photonCamera("front", "other-name")}));
    rig.start();
    rig.polls(3);
    CHECK(rig.photon->probeCount("front") == 1);
    CHECK(rig.photon->probeFor("front")->setup.ntName == "front");
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "twice") == 1);
}

TEST_CASE("vision bridge: stop() freezes everything: nothing reaches ntcore, a publisher or the log afterwards") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front", "photon-front");
    rig.host.adopt(setupOf({front, limelightCamera("rear", "limelight-rear")}));
    rig.fake->robotPublish("/photonvision/other/pipelineIndexRequest", NtType::Integer);
    rig.start();
    rig.tick();
    rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, 1)});
    const auto probeFront = rig.photon->probeFor("front");
    const auto probeRear = rig.limelight->probeFor("rear");
    REQUIRE(probeFront != nullptr);
    REQUIRE(probeRear != nullptr);
    const NtCalls ntBefore = rig.nt->calls;
    const int pollsFront = probeFront->polls;
    const int stateCalls = probeFront->robotStateCalls;
    const std::size_t published = probeFront->sequences.size();
    const std::size_t logged = rig.host.log.entries().size();
    const std::uint64_t discovered = rig.bridge->stats().discoveryRuns;
    REQUIRE(discovered > 0);

    rig.bridge->stop();
    rig.bridge->stop();  // idempotent
    const std::vector<sdk::VisionFrame> frames{frameFor(front, 2), frameFor(front, 3)};
    for (int i = 0; i < 40; ++i) {
        const auto out = rig.tick(frames, i % 7 == 0);
        CHECK(out.empty());
    }
    rig.fake->robotPublish("/photonvision/third/pipelineIndexRequest", NtType::Integer);
    rig.host.adopt(setupOf({front}));  // even a reconnect after the shutdown hook
    rig.polls(5);

    CHECK(rig.nt->calls.total() == ntBefore.total());
    CHECK(probeFront->polls == pollsFront);
    CHECK(probeFront->robotStateCalls == stateCalls);
    CHECK(probeFront->sequences.size() == published);
    CHECK(rig.host.log.entries().size() == logged);
    CHECK(rig.bridge->stats().discoveryRuns == discovered);
    CHECK(rig.bridge->stats().framesDropped == 80);

    CHECK_FALSE(rig.bridge->start(rig.host).ok());  // a stopped bridge does not come back

    rig.bridge.reset();  // destroying it touches ntcore no more than stop() did
    CHECK(rig.nt->calls.total() == ntBefore.total());
    CHECK_FALSE(probeFront->alive);
}

TEST_CASE("vision bridge: the rest of IDeviceBridge announces nothing and leaves everything alone") {
    VisionRig rig;
    const sdk::CameraSetup front = photonCamera("front");
    rig.host.adopt(setupOf({front}));
    rig.start();
    rig.polls(2);
    const NtCalls ntBefore = rig.nt->calls;
    const auto probe = rig.photon->probeFor("front");
    REQUIRE(probe != nullptr);

    sdk::IDeviceBridge& bridge = *rig.bridge;
    CHECK(bridge.name() == "vision");
    CHECK_FALSE(bridge.owns(sdk::DeviceId{1}));
    std::vector<sdk::SignalSample> samples;
    bridge.collectOutputs(samples);
    CHECK(samples.empty());
    const std::vector<sdk::SignalSample> inputs{{sdk::DeviceId{1}, 0, 1.0}};
    bridge.applyInputs(inputs);
    bridge.applyLoopback(sdk::DeviceId{1});
    bridge.resetInputHistory();  // a new epoch: publishers keep their heartbeat and sequence, nothing is forgotten
    CHECK(rig.nt->calls.total() == ntBefore.total());
    CHECK(rig.host.announces == 0);
    CHECK(rig.host.retires == 0);

    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(front, 1)});
    bridge.resetInputHistory();
    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(front, 2)});
    CHECK(probe->starts == 1);
    CHECK(probe->sequences == std::vector<std::uint64_t>{1, 2});
}

}  // namespace
