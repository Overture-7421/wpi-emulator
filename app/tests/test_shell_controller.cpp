#include <doctest/doctest.h>

#include <QSignalSpy>

#include <algorithm>
#include <cmath>
#include <vector>

#include "common/robot_basics.h"
#include "shell/run_backend_adapter.h"
#include "shell/shell_controller.h"
#include "test_support.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

namespace {
// Saves a model for the project so it opens straight into the editor.
void configure(AppServices& s, const TestProject& project) {
    REQUIRE(s.session().open(project.path()).ok());
    REQUIRE(s.session().commit(buildRobotConfig(RobotBasics{}, s.buildContext(s.session().inspection())), ProjectPrefs{}).ok());
    REQUIRE(s.session().close().ok());
}
}  // namespace

TEST_CASE("controller distinguishes opened, needs-setup and invalid projects") {
    TestEnvironment env;
    ShellController controller(*env.services);
    QSignalSpy errors(&controller, &ShellController::errorOccurred);

    TestProject fresh;
    CHECK(controller.openProject(fresh.path()) == OpenOutcome::NeedsSetup);
    CHECK(controller.hasProject());
    CHECK(!controller.canRun());

    controller.closeProject();
    TestProject configured;
    configure(*env.services, configured);
    CHECK(controller.openProject(configured.path()) == OpenOutcome::Opened);
    CHECK(controller.canRun());
    CHECK(controller.mode() == AppMode::Editor);
    CHECK(controller.windowTitle().contains(QFileInfo(configured.path()).fileName()));

    QTemporaryDir notAProject;
    CHECK(controller.openProject(notAProject.path()) == OpenOutcome::Invalid);
    CHECK(errors.count() == 1);
    CHECK(controller.projectName() == QFileInfo(configured.path()).fileName());   // still on the previous project
}

TEST_CASE("run mode needs a robot model") {
    TestEnvironment env;
    ShellController controller(*env.services);
    QSignalSpy modes(&controller, &ShellController::modeChanged);
    controller.setMode(AppMode::Run);
    CHECK(controller.mode() == AppMode::Editor);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    modes.clear();
    controller.setMode(AppMode::Run);
    CHECK(controller.mode() == AppMode::Run);
    CHECK(modes.count() == 1);
    controller.setMode(AppMode::Run);   // no change, no signal
    CHECK(modes.count() == 1);
}

TEST_CASE("the last project is restored at start-up unless disabled or unfinished") {
    TestEnvironment env;
    ShellController controller(*env.services);
    CHECK(!controller.restoreLastProject());   // nothing yet

    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    controller.closeProject();
    CHECK(!controller.hasProject());

    CHECK(controller.restoreLastProject());
    CHECK(controller.hasProject());
    CHECK(controller.canRun());

    controller.closeProject();
    AppSettingsData data = env.services->settings().data();
    data.reopenLastProject = false;
    env.services->settings().update(data);
    CHECK(!controller.restoreLastProject());
    CHECK(!controller.hasProject());

    data.reopenLastProject = true;
    env.services->settings().update(data);
    TestProject unfinished;
    REQUIRE(controller.openProject(unfinished.path()) == OpenOutcome::NeedsSetup);   // wizard abandoned
    controller.closeProject();
    CHECK(!controller.restoreLastProject());   // continue from the welcome page instead of re-running setup
    CHECK(!controller.hasProject());
}

TEST_CASE("the runtime follows the open project") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    CHECK(env.demo->scene().config == nullptr);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    REQUIRE(env.demo->scene().config != nullptr);
    CHECK(env.demo->scene().gameName.contains(QStringLiteral("2026")));
    CHECK(controller.gameLabel().contains(QStringLiteral("2026")));
    CHECK(controller.gameModuleAvailable());
    controller.closeProject();
    CHECK(env.demo->scene().config == nullptr);
}

TEST_CASE("opening a project launches the robot, which detects the demo devices while armed") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);

    CHECK(env.demo->status().active());
    CHECK_FALSE(env.demo->status().simulating);
    const size_t expected = DemoRuntimeHost::demoDevicesFor(*env.services->session().config()).size();
    REQUIRE(waitFor([&] {
        const auto snap = env.demo->latestSnapshot();
        return snap && snap->devices().size() == expected;
    }));
    CHECK(env.demo->status().link == HostLinkState::Connected);
    CHECK(env.demo->status().programStarted);
    CHECK(controller.runtimeLabel().contains(QStringLiteral("ready")));

    controller.stopRobot();
    CHECK(!env.demo->status().active());
}

TEST_CASE("starting the simulation reuses the launched robot and stopping leaves it running") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    REQUIRE(waitFor([&] { return env.demo->status().programStarted; }));

    controller.startSimulation();
    CHECK(controller.mode() == AppMode::Run);
    REQUIRE(waitFor([&] { return env.demo->status().simulating; }));
    CHECK(controller.runtimeLabel() == QStringLiteral("Simulating"));

    // enabling the robot in the demo moves the chassis
    sdk::proto::DsState ds;
    ds.enabled = true;
    env.demo->setDriverStation(ds);
    REQUIRE(waitFor([&] {
        const auto snap = env.demo->latestSnapshot();
        return snap && snap->chassis().pose.position.x != 0.f;
    }));

    controller.stopSimulation();
    CHECK_FALSE(env.demo->status().simulating);
    CHECK(env.demo->status().active());   // the robot process stays alive
    controller.stopRobot();
}

TEST_CASE("the run view teleports the robot through the shell and Reset returns to the saved start pose") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    REQUIRE(env.services->session().open(p.path()).ok());
    sdk::RobotConfig config = buildRobotConfig(RobotBasics{}, env.services->buildContext(env.services->session().inspection()));
    config.sim.startPose = sdk::StartPose{3.f, -2.f, 0.5f};
    REQUIRE(env.services->session().commit(config, ProjectPrefs{}).ok());
    REQUIRE(env.services->session().close().ok());
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);   // the pose travelled through robot.json
    REQUIRE(waitFor([&] { return env.demo->status().programStarted; }));
    controller.startSimulation();
    REQUIRE(waitFor([&] { return env.demo->status().simulating; }));

    const auto pose = [&] { return env.demo->latestSnapshot()->chassis().pose; };
    CHECK(pose().position.x == doctest::Approx(3.f));
    CHECK(pose().position.y == doctest::Approx(-2.f));
    CHECK(pose().rotation.yaw() == doctest::Approx(0.5f));

    RunBackendAdapter backend(*env.services);   // what the run view talks to
    backend.setRobotPose(sdk::Pose3({1.f, 4.f, 0.2f}, sdk::Quat::fromYaw(-1.f)));
    CHECK(pose().position.x == doctest::Approx(1.f));
    CHECK(pose().position.y == doctest::Approx(4.f));
    CHECK(pose().rotation.yaw() == doctest::Approx(-1.f));
    CHECK(env.demo->latestSnapshot()->chassis().linearVelocity == sdk::Vec3{});   // a teleport leaves the robot at rest

    backend.resetSimulation();
    CHECK(pose().position.x == doctest::Approx(3.f));
    CHECK(pose().position.y == doctest::Approx(-2.f));
    CHECK(pose().rotation.yaw() == doctest::Approx(0.5f));
    controller.stopRobot();
}

TEST_CASE("the demo runtime simulates a front camera and the game's AprilTags") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);   // the model lists no camera
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    REQUIRE(waitFor([&] { return env.demo->status().programStarted; }));
    controller.startSimulation();
    REQUIRE(waitFor([&] { return env.demo->status().simulating; }));

    // One default camera on the chassis, the 2026 layout, and the robot process "using" the camera.
    REQUIRE(waitFor([&] {
        const auto snap = env.demo->latestSnapshot();
        return snap && !snap->cameras().empty() && snap->cameras()[0].framesCaptured > 0;
    }));
    const auto snap = env.demo->latestSnapshot();
    REQUIRE(snap->cameras().size() == 1);
    const sdk::CameraSample& camera = snap->cameras()[0];
    CHECK(camera.id == "demo_front");
    CHECK(camera.robotConnected);
    CHECK(camera.status.empty());
    CHECK(camera.framesPublished <= camera.framesCaptured);
    REQUIRE(snap->detectedCameras().size() == 1);
    CHECK(snap->detectedCameras()[0].cameraId == "demo_front");
    CHECK(snap->detectedCameras()[0].ntName == "photon-front");
    REQUIRE_FALSE(snap->fiducials().empty());
    CHECK(snap->warnings().empty());

    // Put the robot a metre and a half in front of a tag and facing it: the camera sees it.
    const sdk::FiducialSample tag = snap->fiducials()[1];
    const sdk::Vec3 normal = tag.pose.rotation.rotate({1.f, 0.f, 0.f});
    const sdk::Vec3 spot = tag.pose.position + normal * 1.5f;
    env.demo->setRobotPose(sdk::Pose3({spot.x, spot.y, 0.2f}, sdk::Quat::fromYaw(std::atan2(-normal.y, -normal.x))));
    const std::vector<int> seen = env.demo->latestSnapshot()->cameras()[0].visibleTagIds;
    CHECK(std::find(seen.begin(), seen.end(), tag.id) != seen.end());
    controller.stopRobot();
}

TEST_CASE("entering the run tab starts the simulation by itself") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    CHECK_FALSE(env.demo->status().simulating);

    controller.setMode(AppMode::Run);
    REQUIRE(waitFor([&] { return env.demo->status().simulating; }));
    controller.stopRobot();
}

TEST_CASE("setup completion switches to the editor and launches the robot") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    controller.setMode(AppMode::Run);
    controller.stopRobot();
    controller.setupCompleted();
    CHECK(controller.mode() == AppMode::Editor);
    CHECK(env.demo->status().active());
    CHECK_FALSE(env.demo->status().simulating);
    controller.stopRobot();
}

TEST_CASE("opening another project cancels an unready Run request but a same-project restart preserves it") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject first;
    TestProject second;
    configure(*env.services, first);
    configure(*env.services, second);
    env.demo->setStepIntervals(100, 5);
    REQUIRE(controller.openProject(first.path()) == OpenOutcome::Opened);
    controller.startSimulation();
    CHECK_FALSE(env.demo->status().programStarted);
    REQUIRE(controller.openProject(second.path()) == OpenOutcome::Opened);
    REQUIRE(waitFor([&] { return env.demo->status().programStarted; }));
    CHECK(controller.mode() == AppMode::Editor);
    CHECK_FALSE(env.demo->status().simulating);
    env.demo->stop();
    controller.startSimulation();
    CHECK_FALSE(env.demo->status().programStarted);
    env.demo->restart();
    REQUIRE(waitFor([&] { return env.demo->status().simulating; }));
}
