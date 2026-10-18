#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "vision_bridge_test_support.h"

// What goes wrong around the vision bridge core, and that it stays contained: ntcore that never binds, a build without
// a vendor, publishers and vendors that throw. The robot must never crash and one bad camera must not touch another.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE("vision bridge: ntcore still missing after the program started is warned about once, with the reason") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.host.started = false;
    rig.fake->setAvailable(false);
    rig.start();

    rig.polls(5);  // the robot constructor may still be running, and ntcore not yet mapped
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn) == 0);

    rig.host.started = true;
    rig.polls(30);
    REQUIRE(countLogs(rig.host.log, sdk::LogLevel::Warn) == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, rig.nt->reason) == 1);
    const auto entries = rig.host.log.entries();
    CHECK(entries.back().category == "bridge.vision");

    rig.fake->setAvailable(true);  // it binds late
    rig.polls(3);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn) == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Info, "available now") == 1);
    CHECK(rig.photon->probeCount("front") == 1);
}

TEST_CASE("vision bridge: ntcore that binds before the program started never warns") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.host.started = false;
    rig.fake->setAvailable(false);
    rig.start();
    rig.polls(3);
    rig.fake->setAvailable(true);
    rig.polls(3);
    rig.host.started = true;
    rig.polls(3);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn) == 0);
    CHECK(rig.photon->probeCount("front") == 1);
}

TEST_CASE(
    "vision bridge: a camera without a vendor in this build reports vendor_unavailable at once and is warned about "
    "once") {
    VisionRig rig;
    rig.options.vendors = std::vector<std::shared_ptr<IVisionVendor>>{rig.photon};  // no Limelight vendor
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear", "limelight-rear")}));
    rig.fake->setAvailable(false);  // independent of ntcore
    rig.start();
    rig.polls(5);

    auto states = rig.collect(true);
    const sdk::CameraRobotState* rear = findState(states, "rear");
    REQUIRE(rear != nullptr);
    CHECK(rear->status == camera_status::kVendorUnavailable);
    CHECK_FALSE(rear->publishing);
    CHECK(rear->vendor == sdk::CameraVendor::Limelight);
    CHECK(rear->ntName == "limelight-rear");
    CHECK(findState(states, "front")->status == camera_status::kNtcoreUnavailable);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "vendor_unavailable") == 1);

    rig.fake->setAvailable(true);
    rig.polls(5);
    states = rig.collect(true);
    CHECK(findState(states, "rear")->status == camera_status::kVendorUnavailable);
    CHECK(findState(states, "front")->status.empty());
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "vendor_unavailable") == 1);

    const sdk::CameraSetup rearSetup = limelightCamera("rear", "limelight-rear");
    rig.bridge->applyVision(std::vector<sdk::VisionFrame>{frameFor(rearSetup, 1)});
    CHECK(rig.bridge->stats().framesDropped == 1);
    CHECK(rig.bridge->stats().framesRouted == 0);
}

TEST_CASE("vision bridge: a vendor that makes no publisher gives vendor_unavailable and is asked once") {
    VisionRig rig;
    rig.limelight->returnNull = true;
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.start();
    rig.polls(20);

    CHECK(rig.limelight->createCalls == 1);  // not retried on every poll
    const auto states = rig.collect(true);
    CHECK(findState(states, "rear")->status == camera_status::kVendorUnavailable);
    CHECK(findState(states, "front")->status.empty());
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "vendor_unavailable") == 1);
    CHECK(rig.photon->probeFor("front")->polls == 20);  // the other camera is unaffected
}

TEST_CASE("vision bridge: an empty vendor list leaves every camera vendor_unavailable and discovery idle") {
    VisionRig rig;
    rig.options.vendors = std::vector<std::shared_ptr<IVisionVendor>>{};
    rig.host.adopt(setupOf({photonCamera("front"), limelightCamera("rear")}));
    rig.fake->robotPublish("/photonvision/other/pipelineIndexRequest", NtType::Integer);
    rig.start();
    rig.polls(5);
    const auto states = rig.collect(true);
    REQUIRE(states.size() == 2);
    CHECK(states[0].status == camera_status::kVendorUnavailable);
    CHECK(states[1].status == camera_status::kVendorUnavailable);
    CHECK(rig.bridge->stats().vendors == 0);
    CHECK(rig.bridge->stats().discoveryRuns == 0);  // nobody to ask: nothing could be discovered
    CHECK(rig.nt->calls.topics == 0);
}

// One camera of three has a publisher that throws at `site`. It alone is disabled and logged once; the others (one of
// the same vendor, one of another) keep their whole life cycle.
void checkThrowingPublisherIsIsolated(const char* site, bool nonStd) {
    CAPTURE(site);
    CAPTURE(nonStd);
    VisionRig rig;
    rig.photon->onCreate = [&](PublisherProbe& probe) {
        if (probe.setup.cameraId != "bad") return;
        probe.throwIn = site;
        probe.throwNonStd = nonStd;
    };
    const sdk::CameraSetup good = photonCamera("good");
    const sdk::CameraSetup bad = photonCamera("bad");
    const sdk::CameraSetup other = limelightCamera("other");
    rig.host.adopt(setupOf({good, bad, other}));
    rig.start();
    const auto frames = [&](std::uint64_t seq) {
        return std::vector<sdk::VisionFrame>{frameFor(good, seq), frameFor(bad, seq), frameFor(other, seq)};
    };

    rig.polls(1);         // creation, start() and the first poll(): a throw in either happens here
    rig.tick(frames(1));  // the first frames and the first report: so does a throw in publish() or robotState()
    const auto probeBad = rig.photon->probeFor("bad");
    const auto probeGood = rig.photon->probeFor("good");
    const auto probeOther = rig.limelight->probeFor("other");
    REQUIRE(probeBad != nullptr);
    CHECK(rig.bridge->stats().failures == 1);

    const int badPolls = probeBad->polls;
    const int badStateCalls = probeBad->robotStateCalls;
    const std::size_t badFrames = probeBad->sequences.size();
    for (std::uint64_t seq = 2; seq <= 11; ++seq) rig.tick(frames(seq));
    // No call reaches the failed publisher any more ...
    CHECK(probeBad->polls == badPolls);
    CHECK(probeBad->robotStateCalls == badStateCalls);
    CHECK(probeBad->sequences.size() == badFrames);
    CHECK(probeBad->starts == 1);
    // ... and the healthy cameras did not notice.
    CHECK(probeGood->starts == 1);
    CHECK(probeGood->polls == 12);
    CHECK(probeGood->sequences.size() == 11);
    CHECK(probeOther->polls == 12);
    CHECK(probeOther->sequences.size() == 11);

    const auto states = rig.collect(true);
    REQUIRE(states.size() == 3);
    CHECK(findState(states, "bad")->status == camera_status::kDisabled);
    CHECK_FALSE(findState(states, "bad")->publishing);
    CHECK(findState(states, "good")->status.empty());
    CHECK(findState(states, "good")->framesPublished == 11);
    CHECK(findState(states, "other")->status.empty());
    CHECK(rig.bridge->stats().failures == 1);  // one failure, however many ticks followed

    CHECK(countLogs(rig.host.log, sdk::LogLevel::Error) == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Error, "'bad'") == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Error, site) == 1);
}

TEST_CASE("vision bridge: a publisher that throws in start() disables only its camera") {
    checkThrowingPublisherIsIsolated("start", false);
}
TEST_CASE("vision bridge: a publisher that throws in poll() disables only its camera") {
    checkThrowingPublisherIsIsolated("poll", false);
}
TEST_CASE("vision bridge: a publisher that throws in publish() disables only its camera") {
    checkThrowingPublisherIsIsolated("publish", false);
}
TEST_CASE("vision bridge: a publisher that throws in robotState() disables only its camera") {
    checkThrowingPublisherIsIsolated("robotState", false);
}
TEST_CASE("vision bridge: a publisher that throws something that is not a std exception is contained too") {
    checkThrowingPublisherIsIsolated("poll", true);
}

TEST_CASE("vision bridge: a vendor that throws from createPublisher disables its cameras, not the bridge") {
    VisionRig rig;
    rig.limelight->throwOnCreate = true;
    const sdk::CameraSetup front = photonCamera("front");
    const sdk::CameraSetup rear = limelightCamera("rear");
    rig.host.adopt(setupOf({front, rear}));
    rig.start();
    rig.polls(1);
    rig.tick(std::vector<sdk::VisionFrame>{frameFor(front, 1), frameFor(rear, 1)});
    rig.polls(10);

    CHECK(rig.limelight->createCalls == 1);  // not retried
    const auto states = rig.collect(true);
    CHECK(findState(states, "rear")->status == camera_status::kDisabled);
    CHECK(findState(states, "front")->status.empty());
    CHECK(rig.photon->probeFor("front")->polls == 12);
    CHECK(rig.photon->probeFor("front")->sequences.size() == 1);
    CHECK(rig.bridge->stats().framesDropped == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Error, "'rear'") == 1);
}

TEST_CASE("vision bridge: a vendor whose discovery throws loses discovery alone") {
    VisionRig rig;
    rig.photon->throwOnDiscover = true;
    const sdk::CameraSetup front = photonCamera("front");
    rig.host.adopt(setupOf({front, limelightCamera("rear")}));
    rig.fake->robotPublish("/limelight-left/camerapose_robotspace_set", NtType::DoubleArray);
    rig.start();
    for (int i = 0; i < 60; ++i) rig.tick();  // 300 ms: three discovery runs

    CHECK(rig.photon->discoverCalls == 1);  // given up after the first failure
    CHECK(rig.limelight->discoverCalls >= 3);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Error, "discovery") == 1);
    const auto states = rig.collect(true);
    CHECK(findDetected(states, "limelight-left") != nullptr);
    CHECK(rig.photon->probeFor("front")->polls == 60);
}

}  // namespace
