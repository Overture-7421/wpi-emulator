#include <doctest/doctest.h>

#include <QSignalSpy>
#include <cmath>

#include "camera_hud.h"
#include "wpiemu/sdk/config/validation_codes.h"

using namespace wpiemu::app::runview;
namespace sdk = wpiemu::sdk;

namespace {

sdk::CameraSample sample(const char* id = "front") {
    sdk::CameraSample c;
    c.id = id;
    c.label = "Front camera";
    c.robotConnected = true;
    c.lastLatencyMs = 25.0;
    c.visibleTagIds = {12, 2, 9};
    return c;
}

std::uint64_t captures(std::int64_t timeUs, double hz) {
    return static_cast<std::uint64_t>(std::floor(static_cast<double>(timeUs) * hz / 1e6));
}

sdk::HubWarning warning(const char* code, sdk::Severity severity, const char* message) {
    sdk::HubWarning w;
    w.code = code;
    w.severity = severity;
    w.message = message;
    return w;
}

}  // namespace

TEST_CASE("vision hud: the capture rate is the slope of the frame counter over the sim clock") {
    RateMeter meter;
    CHECK_FALSE(meter.hz());

    // 30 Hz captured on 5 ms ticks (so the counter climbs in steps of one every 6 or 7 ticks).
    std::int64_t t = 0;
    for (; t <= 600000; t += 5000) meter.add(captures(t, 30.0), t);
    CHECK_FALSE(meter.hz());  // not a second of data yet
    for (; t <= 5000000; t += 5000) meter.add(captures(t, 30.0), t);
    REQUIRE(meter.hz());
    CHECK(*meter.hz() == doctest::Approx(30.0).epsilon(0.02));

    const double steady = *meter.hz();
    meter.add(captures(5000000, 30.0), 5000000);  // paused: the clock does not advance
    CHECK(*meter.hz() == doctest::Approx(steady));

    const std::uint64_t frozen = captures(5000000, 30.0);  // the camera stops capturing
    for (t = 5005000; t <= 9000000; t += 5000) meter.add(frozen, t);
    CHECK(*meter.hz() == doctest::Approx(0.0));

    meter.add(0, 1000);  // a reset: the clock went back
    CHECK_FALSE(meter.hz());
}

TEST_CASE("vision hud: a card names the state, the rate, the latency and the tags in view") {
    sdk::CameraSample c = sample();
    const CameraCardState ok = evaluateCameraCard(c, nullptr, 29.97);
    CHECK(ok.label == QStringLiteral("Front camera"));
    CHECK(ok.vendor == QStringLiteral("PhotonVision"));
    CHECK(ok.state == CameraState::Ok);
    CHECK(ok.stateText == QStringLiteral("Connected"));
    CHECK(ok.rateText == QStringLiteral("30 Hz"));
    CHECK(ok.latencyText == QStringLiteral("25 ms"));
    CHECK(ok.metricsText == QStringLiteral("30 Hz · 25 ms"));
    CHECK(ok.visibleCount == 3);
    CHECK(ok.tagsText == QStringLiteral("3 tags: 2, 9, 12"));  // ids sorted
    CHECK(ok.seesTags);
    CHECK(ok.requestText.isEmpty());  // the robot has not said anything about it

    c.visibleTagIds = {7};
    CHECK(evaluateCameraCard(c, nullptr, 30.0).tagsText == QStringLiteral("1 tag: 7"));
    c.visibleTagIds.clear();
    const CameraCardState none = evaluateCameraCard(c, nullptr, 4.26);
    CHECK(none.tagsText == QStringLiteral("No tags in view"));
    CHECK_FALSE(none.seesTags);
    CHECK(none.rateText == QStringLiteral("4.3 Hz"));  // one decimal below 10 Hz
    c.visibleTagIds = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    CHECK(evaluateCameraCard(c, nullptr, 30.0).tagsText ==
          QStringLiteral("10 tags: 1, 2, 3, 4, 5, 6, 7, 8, …"));
    c.visibleTagIds.clear();

    c.label.clear();
    c.vendor = sdk::CameraVendor::Limelight;
    const CameraCardState unnamed = evaluateCameraCard(c, nullptr, std::nullopt);
    CHECK(unnamed.label == QStringLiteral("front"));  // the id stands in for a missing label
    CHECK(unnamed.vendor == QStringLiteral("Limelight"));
    CHECK(unnamed.rateText == QStringLiteral("—"));  // no measurement yet

    c.robotConnected = false;
    CHECK(evaluateCameraCard(c, nullptr, 30.0).state == CameraState::Waiting);
    CHECK(evaluateCameraCard(c, nullptr, 30.0).stateText == QStringLiteral("Waiting for robot"));

    c.status = "photon_uuid_unsupported";
    const CameraCardState problem = evaluateCameraCard(c, nullptr, 30.0);
    CHECK(problem.state == CameraState::Problem);
    CHECK(problem.stateText == QStringLiteral("PhotonLib version not supported"));
    c.status = "something_new";
    CHECK(evaluateCameraCard(c, nullptr, 30.0).stateText ==
          QStringLiteral("something_new"));  // unknown statuses are shown as they are

    c.status.clear();
    c.enabled = false;
    const CameraCardState off = evaluateCameraCard(c, nullptr, 30.0);
    CHECK(off.state == CameraState::Disabled);
    CHECK(off.stateText == QStringLiteral("Disabled"));
    CHECK(off.rateText == QStringLiteral("—"));
    CHECK(off.latencyText == QStringLiteral("—"));
    CHECK(off.metricsText == QStringLiteral("Not capturing"));
}

TEST_CASE(
    "vision hud: the request line reports pipeline, driver mode and LED in each vendor's words") {
    sdk::CameraRobotState robot;
    robot.pipelineIndex = 2;
    robot.driverMode = true;
    robot.ledMode = 1;  // PhotonVision VisionLEDMode: 1 = on
    CHECK(evaluateCameraCard(sample(), &robot, 30.0).requestText ==
          QStringLiteral("Robot asked for: pipeline 2 · driver mode · LED on"));

    robot.ledMode = -1;  // kDefault says nothing
    robot.driverMode = false;
    CHECK(evaluateCameraCard(sample(), &robot, 30.0).requestText ==
          QStringLiteral("Robot asked for: pipeline 2"));
    robot.ledMode = 2;
    CHECK(evaluateCameraCard(sample(), &robot, 30.0)
              .requestText.endsWith(QStringLiteral("LED blinking")));
    robot.ledMode = 0;
    CHECK(
        evaluateCameraCard(sample(), &robot, 30.0).requestText.endsWith(QStringLiteral("LED off")));

    // Limelight numbers the LED modes differently: 0 = whatever the pipeline says, 1 off, 2 blink,
    // 3 on.
    sdk::CameraSample limelight = sample();
    limelight.vendor = sdk::CameraVendor::Limelight;
    robot.vendor = sdk::CameraVendor::Limelight;
    robot.ledMode = 0;
    CHECK(evaluateCameraCard(limelight, &robot, 30.0).requestText ==
          QStringLiteral("Robot asked for: pipeline 2"));
    robot.ledMode = 3;
    CHECK(
        evaluateCameraCard(limelight, &robot, 30.0).requestText.endsWith(QStringLiteral("LED on")));
    robot.ledMode = 1;
    CHECK(evaluateCameraCard(limelight, &robot, 30.0)
              .requestText.endsWith(QStringLiteral("LED off")));
    robot.ledMode = 9;
    CHECK(evaluateCameraCard(limelight, &robot, 30.0)
              .requestText.endsWith(QStringLiteral("LED mode 9")));

    sdk::CameraRobotState idle;  // present, nothing requested
    CHECK(evaluateCameraCard(sample(), &idle, 30.0).requestText ==
          QStringLiteral("The robot code uses this camera"));
    idle.present = false;
    CHECK(evaluateCameraCard(sample(), &idle, 30.0).requestText ==
          QStringLiteral("The robot code does not use this camera"));
}

TEST_CASE(
    "vision hud: the robot's state of a camera is matched by id, else by NetworkTables name and "
    "vendor") {
    sdk::CameraRobotState byId;
    byId.ntName = "photon-front";
    byId.cameraId = "front";
    sdk::CameraRobotState byName;  // the bridge saw the name but matched no configured camera
    byName.ntName = "limelight";
    byName.vendor = sdk::CameraVendor::Limelight;
    const std::vector<sdk::CameraRobotState> detected{byId, byName};

    sdk::CameraSample front = sample("front");
    CHECK(matchRobotState(front, "whatever", detected) == &detected[0]);

    sdk::CameraSample limelight = sample("ll");
    limelight.vendor = sdk::CameraVendor::Limelight;
    CHECK(matchRobotState(limelight, "limelight", detected) == &detected[1]);
    CHECK(matchRobotState(limelight, "", detected) == nullptr);  // no configured name to go by
    CHECK(matchRobotState(limelight, "limelight-left", detected) == nullptr);  // another table
    sdk::CameraSample photon = sample("pv");
    CHECK(matchRobotState(photon, "limelight", detected) == nullptr);  // the vendor differs
}

TEST_CASE(
    "vision hud: the model has a card per camera, follows the robot's request and emits only for "
    "changed cards") {
    auto config = std::make_shared<sdk::RobotConfig>();
    sdk::CameraConfig configured;
    configured.id = "front";
    configured.ntName = "cam-a";
    config->vision.cameras.push_back(configured);
    CameraHudModel model;
    model.setConfig(config);

    sdk::SimSnapshotData d;
    d.cameras = {sample("front")};
    sdk::CameraRobotState robot;
    robot.ntName = "cam-a";  // no id: matched through the configured name
    robot.pipelineIndex = 3;
    d.detectedCameras = {robot};
    d.simTimeUs = 0;
    const sdk::SimSnapshot first(d);
    model.update(&first);
    REQUIRE(model.rowCount() == 1);
    CHECK(model.index(0).data(CameraHudModel::RequestRole).toString() ==
          QStringLiteral("Robot asked for: pipeline 3"));
    CHECK(model.index(0).data(CameraHudModel::LabelRole).toString() ==
          QStringLiteral("Front camera"));
    CHECK(model.index(0).data(CameraHudModel::ActiveRole).toBool());
    CHECK(model.index(0).data(CameraHudModel::StateTextRole).toString() ==
          QStringLiteral("Connected"));

    QSignalSpy changed(&model, &QAbstractItemModel::dataChanged);
    QSignalSpy reset(&model, &QAbstractItemModel::modelReset);
    model.update(&first);
    CHECK(changed.count() == 0);
    d.cameras[0].visibleTagIds = {2};
    const sdk::SimSnapshot sawOne(d);
    model.update(&sawOne);
    CHECK(changed.count() == 1);
    CHECK(model.index(0).data(CameraHudModel::TagsRole).toString() == QStringLiteral("1 tag: 2"));
    CHECK(reset.count() == 0);

    // The measured rate: 30 Hz over four seconds of sim time.
    for (std::int64_t t = 5000; t <= 4000000; t += 5000) {
        d.simTimeUs = t;
        d.cameras[0].framesCaptured = captures(t, 30.0);
        const sdk::SimSnapshot snap(d);
        model.update(&snap);
    }
    CHECK(model.index(0).data(CameraHudModel::RateRole).toString() == QStringLiteral("30 Hz"));

    d.cameras.push_back(sample("rear"));  // another camera: the list changes shape
    const sdk::SimSnapshot two(d);
    model.update(&two);
    CHECK(reset.count() == 1);
    CHECK(model.rowCount() == 2);

    model.update(nullptr);  // the run is over
    CHECK(model.rowCount() == 0);
}

TEST_CASE(
    "vision hud: vision warnings get readable titles and keep their severity, other warnings are "
    "ignored") {
    namespace code = sdk::validation_code;
    sdk::SimSnapshotData d;
    d.warnings = {warning(code::kVisionLayoutMissing, sdk::Severity::Error, "No layout 'x'"),
                  warning("unmapped_loopback", sdk::Severity::Warning, "not a vision problem"),
                  warning(code::kVisionCameraUnconfigured, sdk::Severity::Warning,
                          "The robot uses 'photon-extra'"),
                  warning(code::kVisionCameraStatus, sdk::Severity::Warning, ""),
                  warning("vision_from_the_future", sdk::Severity::Info, "new")};
    const sdk::SimSnapshot snap(d);
    const std::vector<VisionNotice> notices = visionNotices(&snap);
    REQUIRE(notices.size() == 4);
    CHECK(notices[0].title == QStringLiteral("AprilTag layout not found"));
    CHECK(notices[0].severity == sdk::Severity::Error);
    CHECK(notices[0].detail == QStringLiteral("No layout 'x'"));
    CHECK(notices[1].title == QStringLiteral("Camera not configured"));
    CHECK(notices[1].severity == sdk::Severity::Warning);
    CHECK(notices[2].title == QStringLiteral("Camera problem"));
    CHECK_FALSE(notices[2].detail.isEmpty());  // a warning without text still explains itself
    CHECK(notices[3].title == QStringLiteral("Vision warning"));
    CHECK(visionNotices(nullptr).empty());
    CHECK(visionNoticeTitle("unmapped_loopback").isEmpty());
}

TEST_CASE("vision hud: the notice model raises each new notice once and clears with the warning") {
    namespace code = sdk::validation_code;
    VisionNoticeModel model;
    QSignalSpy raised(&model, &VisionNoticeModel::noticeRaised);
    const auto snapshotWith = [](std::vector<sdk::HubWarning> warnings) {
        sdk::SimSnapshotData d;
        d.warnings = std::move(warnings);
        return sdk::SimSnapshot(std::move(d));
    };

    const auto none = snapshotWith({});
    model.update(&none, 1000);
    CHECK(model.rowCount() == 0);

    const sdk::HubWarning unconfigured =
        warning(code::kVisionCameraUnconfigured, sdk::Severity::Warning, "uses 'photon-extra'");
    const auto one = snapshotWith({unconfigured});
    model.update(&one, 1100);
    REQUIRE(model.rowCount() == 1);
    CHECK(model.index(0).data(VisionNoticeModel::TitleRole).toString() ==
          QStringLiteral("Camera not configured"));
    CHECK(model.index(0).data(VisionNoticeModel::SeverityRole).toInt() ==
          static_cast<int>(sdk::Severity::Warning));
    REQUIRE(raised.count() == 1);
    CHECK(raised.at(0).at(1).toString() == QStringLiteral("uses 'photon-extra'"));

    model.update(&one, 1200);
    CHECK(raised.count() == 1);  // still there: not raised again

    const auto two = snapshotWith(
        {unconfigured, warning(code::kVisionLayoutMissing, sdk::Severity::Error, "no layout")});
    model.update(&two, 1300);
    CHECK(model.rowCount() == 2);
    CHECK(raised.count() == 2);  // only the new one
    CHECK(raised.at(1).at(0).toString() == QStringLiteral("AprilTag layout not found"));
    CHECK(raised.at(1).at(2).toInt() == static_cast<int>(sdk::Severity::Error));

    // A problem that flickers is shown every time but announced at most every kRepeatAnnounceMs.
    model.update(&none, 1400);
    CHECK(model.rowCount() == 0);
    model.update(&one, 1500);
    CHECK(model.rowCount() == 1);
    CHECK(raised.count() == 2);
    model.update(&none, 1600);
    model.update(&one, 1500 + VisionNoticeModel::kRepeatAnnounceMs);
    CHECK(raised.count() == 3);  // long enough since the last announcement

    model.update(nullptr, 1600 + VisionNoticeModel::kRepeatAnnounceMs);  // a new run
    model.update(&one, 1700 + VisionNoticeModel::kRepeatAnnounceMs);
    CHECK(raised.count() == 4);
}
