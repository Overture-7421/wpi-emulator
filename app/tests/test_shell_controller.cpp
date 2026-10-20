#include <doctest/doctest.h>

#include <QSignalSpy>

#include "common/robot_basics.h"
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

TEST_CASE("the wizard's Driver Station choice and the open project reach the runtime scene") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    AppServices& s = *env.services;
    REQUIRE(s.session().open(p.path()).ok());
    ProjectPrefs prefs;
    prefs.alliance = sdk::Alliance::Blue;
    prefs.station = 3;
    REQUIRE(s.session().commit(buildRobotConfig(RobotBasics{}, s.buildContext(s.session().inspection())), prefs).ok());
    REQUIRE(s.session().close().ok());

    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    CHECK(env.demo->scene().projectPath == s.session().path());
    CHECK(env.demo->scene().defaultStation == sdk::proto::AllianceStation::Blue3);
    CHECK(env.demo->scene().config != nullptr);
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
    CHECK(env.demo->status().link == sdk::LinkState::Connected);
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
