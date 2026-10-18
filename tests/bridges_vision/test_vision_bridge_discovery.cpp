#include <doctest/doctest.h>

#include <vector>

#include "vision_bridge_test_support.h"

// Discovery: the cameras the ROBOT uses, found from the topics its code created, reported as unconfigured detections.

namespace {

using namespace wpiemu;
using namespace wpiemu::bridges::vision;
using namespace wpiemu::tests::vision_bridge;

// What a robot program leaves on NetworkTables at construction: PhotonLib request publishers, a Limelight table.
void publishRobotTopics(testing::FakeNtApi& fake) {
    fake.robotPublish("/photonvision/photon-front/pipelineIndexRequest", NtType::Integer);
    fake.robotPublish("/photonvision/photon-front/driverModeRequest", NtType::Boolean);
    fake.robotPublish("/photonvision/photon-back/pipelineIndexRequest", NtType::Integer);
    fake.robotPublish("/photonvision/ledModeRequest", NtType::Integer);  // a root topic: no camera
    fake.robotPublish("/limelight-left/camerapose_robotspace_set", NtType::DoubleArray);
    fake.robotPublish("/limelight-left/imumode_set", NtType::Double);
}

TEST_CASE("vision bridge: discovery scans the robot's topics at most every 100 ms, with no camera configured too") {
    VisionRig rig;
    rig.start();                               // an empty setup: nothing to publish, still something to discover
    for (int i = 0; i < 200; ++i) rig.tick();  // one second of 5 ms ticks

    // At 5 ms and then every 100 ms: 5, 105, ... 905.
    CHECK(rig.bridge->stats().discoveryRuns == 10);
    CHECK(rig.photon->discoverCalls == 10);
    CHECK(rig.limelight->discoverCalls == 10);
    CHECK(rig.nt->calls.topics == 20);  // one listing per vendor and run
    CHECK(rig.bridge->stats().publishersStarted == 0);
}

TEST_CASE("vision bridge: a camera the robot uses and nobody configured is detected once and reported") {
    VisionRig rig;
    rig.host.adopt(setupOf({photonCamera("front", "photon-front")}));
    publishRobotTopics(*rig.fake);
    rig.start();

    const auto first = rig.tick();
    REQUIRE(first.size() == 3);  // the configured camera, and the two the robot uses beyond it
    const sdk::CameraRobotState* back = findDetected(first, "photon-back");
    const sdk::CameraRobotState* left = findDetected(first, "limelight-left");
    REQUIRE(back != nullptr);
    REQUIRE(left != nullptr);
    CHECK(back->vendor == sdk::CameraVendor::PhotonVision);
    CHECK(back->cameraId.empty());
    CHECK(back->present);
    CHECK_FALSE(back->publishing);
    CHECK(back->status.empty());
    CHECK(left->vendor == sdk::CameraVendor::Limelight);
    CHECK(findDetected(first, "photon-front") == nullptr);  // configured: it reports itself, it is not "discovered"
    CHECK(findDetected(first, "ledModeRequest") == nullptr);
    REQUIRE(findState(first, "front") != nullptr);

    // Logged once each, however long the robot keeps using them.
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Info, "unconfigured PhotonVision camera 'photon-back'") == 1);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Info, "unconfigured Limelight camera 'limelight-left'") == 1);
    for (int i = 0; i < 100; ++i) CHECK(rig.tick().empty());
    CHECK(countLogs(rig.host.log, "unconfigured") == 2);
    CHECK(rig.bridge->stats().discoveryRuns >= 5);

    // A full report lists them again.
    const auto full = rig.collect(true);
    CHECK(full.size() == 3);
    CHECK(findDetected(full, "photon-back") != nullptr);
}

TEST_CASE("vision bridge: nothing is discovered before the program started or while ntcore is unavailable") {
    VisionRig rig;
    publishRobotTopics(*rig.fake);
    rig.host.started = false;
    rig.start();
    for (int i = 0; i < 50; ++i) rig.tick();
    CHECK(rig.nt->calls.topics == 0);
    CHECK(rig.bridge->stats().discoveryRuns == 0);
    CHECK(rig.collect(true).empty());

    rig.host.started = true;
    rig.fake->setAvailable(false);
    for (int i = 0; i < 50; ++i) rig.tick();
    CHECK(rig.nt->calls.topics == 0);
    CHECK(rig.photon->discoverCalls == 0);
    CHECK(rig.collect(true).empty());

    rig.fake->setAvailable(true);
    rig.tick();
    CHECK(rig.nt->calls.topics == 2);
    CHECK(rig.collect(true).size() == 3);  // photon-front, photon-back and limelight-left, nothing being configured
}

TEST_CASE("vision bridge: a detection the robot stops using is reported absent and comes back when it returns") {
    VisionRig rig;
    publishRobotTopics(*rig.fake);
    rig.start();
    CHECK(rig.tick().size() == 3);  // photon-front, photon-back, limelight-left (nothing is configured)

    rig.photon->ignoredNames.insert("photon-back");  // the robot code no longer has that camera
    std::vector<sdk::CameraRobotState> reports;
    for (int i = 0; i < 25; ++i) {  // past the next scan
        const auto out = rig.tick();
        reports.insert(reports.end(), out.begin(), out.end());
    }
    REQUIRE(reports.size() == 1);
    CHECK(reports[0].ntName == "photon-back");
    CHECK_FALSE(reports[0].present);
    const auto full = rig.collect(true);
    CHECK(findDetected(full, "photon-back") == nullptr);  // a full list only has what is in use
    CHECK(full.size() == 2);

    rig.photon->ignoredNames.clear();
    reports.clear();
    for (int i = 0; i < 25; ++i) {
        const auto out = rig.tick();
        reports.insert(reports.end(), out.begin(), out.end());
    }
    REQUIRE(reports.size() == 1);
    CHECK(reports[0].ntName == "photon-back");
    CHECK(reports[0].present);
    CHECK(countLogs(rig.host.log, "unconfigured PhotonVision camera 'photon-back'") == 1);  // logged once only
}

TEST_CASE("vision bridge: a detected camera that becomes configured is reported absent as a detection") {
    VisionRig rig;
    publishRobotTopics(*rig.fake);
    rig.start();
    const auto before = rig.tick();
    CHECK(findDetected(before, "photon-back") != nullptr);

    rig.host.adopt(setupOf({photonCamera("back", "photon-back")}));  // the user added it and reconnected
    std::vector<sdk::CameraRobotState> reports;
    for (int i = 0; i < 25; ++i) {
        const auto out = rig.tick();
        reports.insert(reports.end(), out.begin(), out.end());
    }
    const sdk::CameraRobotState* detection = findDetected(reports, "photon-back");
    REQUIRE(detection != nullptr);
    CHECK_FALSE(detection->present);
    const sdk::CameraRobotState* configured = findState(reports, "back");
    REQUIRE(configured != nullptr);
    CHECK(configured->ntName == "photon-back");
    const auto full = rig.collect(true);
    CHECK(findDetected(full, "photon-back") == nullptr);
    CHECK(findState(full, "back") != nullptr);
}

TEST_CASE("vision bridge: cameras are matched by vendor and name, not by name alone") {
    VisionRig rig;
    rig.fake->robotPublish("/photonvision/limelight/pipelineIndexRequest",
                           NtType::Integer);                           // a PhotonVision camera ...
    rig.fake->robotPublish("/limelight/imumode_set", NtType::Double);  // ... named like the default table
    rig.host.adopt(setupOf({limelightCamera("ll", "limelight")}));
    rig.start();
    const auto first = rig.tick();
    const sdk::CameraRobotState* detected = findDetected(first, "limelight");
    REQUIRE(detected != nullptr);
    CHECK(detected->vendor == sdk::CameraVendor::PhotonVision);  // the Limelight table is the configured camera
    CHECK(first.size() == 2);
}

TEST_CASE("vision bridge: a robot with endless cameras does not make the reports endless") {
    VisionRig rig;
    for (int i = 0; i < 80; ++i)
        rig.fake->robotPublish("/photonvision/cam" + std::to_string(i) + "/pipelineIndexRequest", NtType::Integer);
    rig.fake->robotPublish("/photonvision//pipelineIndexRequest", NtType::Integer);  // a nameless "camera" is no camera
    rig.start();
    for (int i = 0; i < 60; ++i) rig.tick();
    const auto full = rig.collect(true);
    CHECK(full.size() == 64);
    CHECK(findDetected(full, "") == nullptr);
    CHECK(countLogs(rig.host.log, sdk::LogLevel::Warn, "unconfigured cameras") == 1);  // once, not per scan
}

}  // namespace
