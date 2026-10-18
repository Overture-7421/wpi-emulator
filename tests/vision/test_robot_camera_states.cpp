#include <doctest/doctest.h>

#include <span>
#include <string>

#include "vision_test_support.h"
#include "wpiemu/core/vision/robot_camera_states.h"

using namespace vision_test;

namespace {

sdk::CameraRobotState reported(const std::string& ntName, const std::string& cameraId = "",
                               sdk::CameraVendor vendor = sdk::CameraVendor::PhotonVision) {
    sdk::CameraRobotState s;
    s.ntName = ntName;
    s.cameraId = cameraId;
    s.vendor = vendor;
    return s;
}

void upsert(vis::RobotCameraStates& states, const sdk::CameraRobotState& s) {
    states.upsert(std::span<const sdk::CameraRobotState>(&s, 1));
}

}  // namespace

TEST_CASE("robot camera states: entries are upserted by vendor and NT name, found by camera id and cleared") {
    vis::RobotCameraStates states;
    CHECK(states.empty());

    sdk::CameraRobotState front = reported("front", "front");
    front.publishing = true;
    const sdk::CameraRobotState limelightFront = reported("front", "", sdk::CameraVendor::Limelight);
    const sdk::CameraRobotState both[] = {front, limelightFront};
    states.upsert(both);
    REQUIRE(states.states().size() == 2);  // the vendor is part of the key
    CHECK(states.states()[0] == front);

    sdk::CameraRobotState changed = front;
    changed.publishing = false;
    changed.framesPublished = 7;
    upsert(states, changed);
    REQUIRE(states.states().size() == 2);  // replaced in place, not duplicated
    CHECK(states.states()[0] == changed);
    CHECK(states.states()[1] == limelightFront);

    REQUIRE(states.forCamera("front") != nullptr);
    CHECK(*states.forCamera("front") == changed);
    CHECK(states.forCamera("rear") == nullptr);
    CHECK(states.forCamera("") == nullptr);  // a camera nobody simulates belongs to no configured one

    states.clear();
    CHECK(states.empty());
    CHECK(states.forCamera("front") == nullptr);
}

TEST_CASE("robot camera states: the number of entries is bounded and known entries still update") {
    vis::RobotCameraStates states;
    for (std::size_t i = 0; i < vis::RobotCameraStates::kMaxEntries + 10; ++i)
        upsert(states, reported("cam" + std::to_string(i)));
    CHECK(states.states().size() == vis::RobotCameraStates::kMaxEntries);

    sdk::CameraRobotState known = reported("cam0");
    known.framesPublished = 5;
    upsert(states, known);
    CHECK(states.states().size() == vis::RobotCameraStates::kMaxEntries);
    CHECK(states.states()[0].framesPublished == 5);
}

TEST_CASE("robot camera states: a camera the robot uses but the configuration does not know is a warning, even without cameras") {
    vis::RobotCameraStates states;
    upsert(states, reported("limelight-left", "", sdk::CameraVendor::Limelight));

    // no camera configured at all: this is exactly when the hint is needed
    const auto w = states.warnings(sdk::VisionConfig{});
    REQUIRE(w.size() == 1);
    CHECK(w[0].code == "vision_camera_unconfigured");
    CHECK(w[0].severity == sdk::Severity::Warning);
    CHECK_FALSE(w[0].device.valid());
    CHECK(w[0].message.find("Limelight") != std::string::npos);
    CHECK(w[0].message.find("'limelight-left'") != std::string::npos);
    CHECK(w[0].message.find("not in the vision configuration") != std::string::npos);
    CHECK(w[0].message.find("add it from the detected cameras") != std::string::npos);
}

TEST_CASE("robot camera states: the reason of an unconfigured camera tells absent, disabled and not simulated apart") {
    sdk::CameraConfig front = makeCamera("front");
    front.ntName = "photon-front";
    sdk::CameraConfig spare = makeCamera("spare");
    spare.ntName = "photon-spare";
    spare.enabled = false;
    sdk::CameraConfig left = makeCamera("left");
    left.vendor = sdk::CameraVendor::Limelight;  // the network name of a Limelight defaults to "limelight"
    const sdk::VisionConfig config = visionOf({front, spare, left});

    struct Case {
        sdk::CameraRobotState state;
        const char* reason;
    };
    const Case cases[] = {
        {reported("mystery"), "not in the vision configuration"},
        {reported("photon-spare"), "disabled in the vision configuration"},
        // configured and enabled, yet the robot process matched it to no simulated camera: the run has no
        // engine (no layout), so the hint must not send the user off to add a camera they already have
        {reported("photon-front"), "in the vision configuration but not simulated in this run"},
        {reported("limelight", "", sdk::CameraVendor::Limelight), "in the vision configuration but not simulated in this run"},
        // the vendor is part of the match
        {reported("photon-front", "", sdk::CameraVendor::Limelight), "not in the vision configuration"},
    };
    for (const Case& c : cases) {
        INFO(c.state.ntName << ": " << c.reason);
        vis::RobotCameraStates states;
        upsert(states, c.state);
        const auto w = states.warnings(config);
        REQUIRE(w.size() == 1);
        CHECK(w[0].code == "vision_camera_unconfigured");
        CHECK(w[0].message.find(c.reason) != std::string::npos);
    }

    // a camera the robot does not use needs no warning
    vis::RobotCameraStates idle;
    sdk::CameraRobotState notUsed = reported("mystery");
    notUsed.present = false;
    upsert(idle, notUsed);
    CHECK(idle.warnings(config).empty());
}

TEST_CASE("robot camera states: the status of a configured camera is a warning or an error, and recovery removes it") {
    sdk::CameraConfig front = makeCamera("front");
    front.ntName = "photon-front";
    const sdk::VisionConfig config = visionOf({front});

    vis::RobotCameraStates states;
    sdk::CameraRobotState healthy = reported("photon-front", "front");
    healthy.publishing = true;
    upsert(states, healthy);
    CHECK(states.warnings(config).empty());  // a working camera is not an issue

    struct Case {
        const char* status;
        sdk::Severity severity;
    };
    const Case cases[] = {{"ntcore_unavailable", sdk::Severity::Error},
                          {"photon_uuid_unsupported", sdk::Severity::Error},
                          {"foreign_publisher", sdk::Severity::Warning},
                          {"disabled", sdk::Severity::Warning},
                          {"something_new", sdk::Severity::Warning}};
    for (const Case& c : cases) {
        INFO(c.status);
        sdk::CameraRobotState bad = healthy;
        bad.publishing = false;
        bad.status = c.status;
        upsert(states, bad);  // replaces the entry: one warning at a time
        const auto w = states.warnings(config);
        REQUIRE(w.size() == 1);
        CHECK(w[0].code == "vision_camera_status");
        CHECK(w[0].severity == c.severity);
        CHECK_FALSE(w[0].device.valid());
        CHECK(w[0].message.find("camera 'front' (photon-front)") != std::string::npos);
    }
    upsert(states, healthy);  // the camera recovers: the warning disappears by itself
    CHECK(states.warnings(config).empty());
}
