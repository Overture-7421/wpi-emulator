#include <doctest/doctest.h>

#include <string>
#include <vector>

#include "test_limelight_support.h"
#include "wpiemu/bridges/vision/camera_publisher.h"

namespace {

using namespace wpiemu::tests::limelight;
using wpiemu::bridges::vision::NtTopicInfo;

std::vector<NtTopicInfo> topicsOf(std::initializer_list<const char*> names) {
    std::vector<NtTopicInfo> out;
    for (const char* name : names) out.push_back({name, "double", "{}"});
    return out;
}

std::vector<std::string> discover(const std::vector<NtTopicInfo>& topics) {
    const auto vendor = wpiemu::bridges::vision::makeLimelightVendor();
    std::vector<std::string> names;
    vendor->discoverRobotCameras(topics, names);
    return names;
}

TEST_CASE("limelight vendor: identifies itself and always makes a publisher") {
    const auto vendor = wpiemu::bridges::vision::makeLimelightVendor();
    CHECK(vendor->vendor() == wpiemu::sdk::CameraVendor::Limelight);
    CHECK(vendor->topicPrefix() == "/limelight");

    FakeNtApi nt;
    RecordingLogger log;
    const auto camera = vendor->createPublisher(nt, log, rearSetup());
    REQUIRE(camera != nullptr);
    CHECK(camera->setup().cameraId == "ll-rear");
    CHECK(camera->setup().ntName == kTable);
    CHECK_FALSE(nt.exists(topic("tv")));  // nothing is created before start()
}

TEST_CASE("limelight vendor: a table is discovered from the topics the robot created, once") {
    const auto names = discover(topicsOf({"/limelight-rear/pipeline", "/limelight-rear/camerapose_robotspace_set",
                                          "/limelight-rear/robot_orientation_set", "/limelight/imumode_set"}));
    REQUIRE(names.size() == 2);
    CHECK(names[0] == "limelight-rear");  // no duplicates
    CHECK(names[1] == "limelight");
}

TEST_CASE("limelight vendor: every key the robot is known to write discovers its table") {
    for (const std::string_view key : ll::kRobotWrittenKeys) {
        const std::string name = ll::topicOf("limelight-x", key);
        const auto names = discover(topicsOf({name.c_str()}));
        CHECK_MESSAGE(names == std::vector<std::string>{"limelight-x"}, key);
    }
}

TEST_CASE("limelight vendor: the bridge's own per-frame topics never discover a camera") {
    std::vector<std::string> own;
    for (const std::string_view key : ll::kPerFrameKeys) own.push_back(ll::topicOf("limelight-left", key));
    own.push_back(ll::topicOf("limelight-left", ll::kHw));
    std::vector<NtTopicInfo> topics;
    for (const std::string& name : own) topics.push_back({name, "double", "{}"});
    CHECK(discover(topics).empty());
}

TEST_CASE("limelight vendor: only device tables with a one-level key count") {
    CHECK(discover(topicsOf({"/limelightfoo/pipeline"})).empty());          // not a device name
    CHECK(discover(topicsOf({"/limelight-/pipeline"})).empty());            // empty nickname
    CHECK(discover(topicsOf({"/photonvision/cam/pipeline"})).empty());      // another vendor
    CHECK(discover(topicsOf({"/limelight-rear/nested/pipeline"})).empty());  // not a key of the table
    CHECK(discover(topicsOf({"/limelight-rear/not_a_robot_key"})).empty());
    CHECK(discover(topicsOf({"limelight-rear/pipeline"})).empty());
    CHECK(discover({}).empty());

    // a camera whose table holds both ours and a robot's topic is discovered by the robot's
    const auto names =
        discover(topicsOf({"/limelight-rear/tv", "/limelight-rear/botpose_wpiblue", "/limelight-rear/ledMode"}));
    REQUIRE(names.size() == 1);
    CHECK(names[0] == "limelight-rear");
}

TEST_CASE("limelight vendor: discovery through NetworkTables finds the robot's cameras, not configured-only ones") {
    FakeNtApi nt;
    nt.setNowUs(1'000'000);
    RecordingLogger log;
    const auto vendor = wpiemu::bridges::vision::makeLimelightVendor();

    // two configured cameras publish; the robot only ever touches the rear one
    wpiemu::sdk::CameraSetup left = rearSetup();
    left.cameraId = "ll-left";
    left.ntName = "limelight-left";
    const auto rear = vendor->createPublisher(nt, log, rearSetup());
    const auto leftCamera = vendor->createPublisher(nt, log, left);
    rear->start();
    leftCamera->start();
    nt.robotSetDouble(topic(ll::kPipeline), 1.0, 1'100'000);
    const std::initializer_list<double> mount = {0.1, 0.0, 0.2, 0.0, 0.0, 0.0};
    nt.robotSetDoubleArray("/limelight-front/camerapose_robotspace_set", mount, 1'100'000);  // not configured

    std::vector<NtTopicInfo> topics;
    nt.topics(vendor->topicPrefix(), topics);
    std::vector<std::string> names;
    vendor->discoverRobotCameras(topics, names);
    REQUIRE(names.size() == 2);
    CHECK(names[0] == "limelight-front");  // topics are listed by name
    CHECK(names[1] == "limelight-rear");
}

}  // namespace
