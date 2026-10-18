#include <doctest/doctest.h>

#include <vector>

#include "vision_bridge_test_support.h"

// Idle cost: what the bridge asks of ntcore and of the log when there is nothing to do. The robot of a team that has no
// camera pays for this bridge on every tick, so it has to be next to free.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

TEST_CASE("vision bridge: with nothing configured and ntcore unavailable a poll costs one availability probe") {
    VisionRig rig;
    rig.fake->setAvailable(false);
    rig.start();
    std::vector<sdk::CameraRobotState> out;
    for (int i = 0; i < 1000; ++i) {
        rig.fake->advanceUs(kStepUs);
        rig.bridge->applyVision({});
        rig.bridge->poll();
        rig.bridge->collectVision(out, false);
    }
    CHECK(out.empty());
    CHECK(rig.nt->calls.available == 1000);
    CHECK(rig.nt->calls.unavailableReason == 1);  // read for the one warning
    CHECK(rig.nt->calls.beyondAvailability() == 0);
    CHECK(rig.photon->discoverCalls == 0);
    CHECK(rig.limelight->discoverCalls == 0);
    CHECK(rig.bridge->stats().discoveryRuns == 0);
    CHECK(rig.host.log.entries().size() == 1);  // the single "ntcore is not available" warning, not one per tick
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn) == 1);
}

TEST_CASE("vision bridge: ntcore is not probed before the robot program started, only then") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.host.started = false;  // the robot constructor is still running (WPIEMU_LIVE_DISCOVERY=1 polls this early)
    rig.start();
    rig.polls(5);
    CHECK(rig.nt->calls.available == 0);  // a Java robot maps ntcore a moment after the extension loaded
    CHECK(rig.nt->calls.beyondAvailability() == 0);
    CHECK(rig.collect(true).at(0).status.empty());  // unknown is not a problem

    rig.host.started = true;
    rig.polls(1);
    CHECK(rig.nt->calls.available == 1);
}

TEST_CASE(
    "vision bridge: with nothing configured and nothing on the robot a poll reads the time and, every 100 ms, the "
    "topics") {
    VisionRig rig;
    rig.start();
    std::vector<sdk::CameraRobotState> out;
    for (int i = 0; i < 1000; ++i) {  // five seconds
        rig.fake->advanceUs(kStepUs);
        rig.bridge->applyVision({});
        rig.bridge->poll();
        rig.bridge->collectVision(out, false);
    }
    CHECK(out.empty());
    CHECK(rig.nt->calls.available == 1000);
    CHECK(rig.nt->calls.nowUs == 1000);
    CHECK(rig.nt->calls.topics == 100);  // 50 runs, one listing per vendor
    CHECK(rig.nt->calls.publish == 0);
    CHECK(rig.nt->calls.set == 0);
    CHECK(rig.nt->calls.subscribe == 0);
    CHECK(rig.nt->calls.read == 0);
    CHECK(rig.nt->calls.hasForeignPublisher == 0);
    CHECK(rig.host.log.entries().empty());  // a quiet robot is a silent log
}

TEST_CASE(
    "vision bridge: a running camera costs the bridge no NetworkTables call of its own and no log line per tick") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front")}));
    rig.start();
    rig.polls(1);
    rig.collect(false);  // the camera's first report
    const NtCalls started = rig.nt->calls;
    const std::size_t logged = rig.host.log.entries().size();
    std::vector<sdk::CameraRobotState> out;
    for (int i = 0; i < 400; ++i) {
        rig.fake->advanceUs(kStepUs);
        rig.bridge->applyVision({});
        rig.bridge->poll();
        out.clear();
        rig.bridge->collectVision(out, false);
    }
    CHECK(rig.nt->calls.publish == started.publish);
    CHECK(rig.nt->calls.set == started.set);
    CHECK(rig.nt->calls.subscribe == started.subscribe);
    CHECK(rig.nt->calls.read == started.read);
    CHECK(rig.nt->calls.nowUs - started.nowUs == 400);   // the discovery rate limit's clock, once per poll
    CHECK(rig.nt->calls.topics - started.topics == 40);  // 20 runs over 2 s, one listing per vendor
    CHECK(rig.host.log.entries().size() == logged);
    CHECK(rig.photon->probeFor("front")->polls == 401);
}

}  // namespace
