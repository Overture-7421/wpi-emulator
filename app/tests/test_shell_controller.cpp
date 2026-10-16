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
    s.session().close();
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

TEST_CASE("starting device detection discovers the demo devices through the runtime host") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);

    controller.startDiscovery();
    CHECK(env.demo->status().mode == RuntimeMode::Discovery);
    const size_t expected = DemoRuntimeHost::demoDevicesFor(*env.services->session().config()).size();
    REQUIRE(waitFor([&] {
        const auto snap = env.demo->latestSnapshot();
        return snap && snap->devices().size() == expected;
    }));
    CHECK(env.demo->status().link == HostLinkState::Connected);
    CHECK(controller.runtimeLabel().contains(QStringLiteral("Detecting")));

    controller.stopRobot();
    CHECK(!env.demo->status().active());
}

TEST_CASE("starting the robot switches to run mode and replaces a detection run") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    controller.startDiscovery();
    controller.startRobot();
    CHECK(controller.mode() == AppMode::Run);
    CHECK(env.demo->status().mode == RuntimeMode::Run);
    REQUIRE(waitFor([&] { return env.demo->status().link == HostLinkState::Connected; }));

    // enabling the robot in the demo moves the chassis
    sdk::proto::DsState ds;
    ds.enabled = true;
    env.demo->setDriverStation(ds);
    REQUIRE(waitFor([&] {
        const auto snap = env.demo->latestSnapshot();
        return snap && snap->chassis().pose.position.x != 0.f;
    }));
    controller.stopRobot();
}

TEST_CASE("setup completion switches to the editor and can launch detection") {
    TestEnvironment env;
    ShellController controller(*env.services);
    TestProject p;
    configure(*env.services, p);
    REQUIRE(controller.openProject(p.path()) == OpenOutcome::Opened);
    controller.setMode(AppMode::Run);
    controller.setupCompleted(/*detectDevices=*/true);
    CHECK(controller.mode() == AppMode::Editor);
    CHECK(env.demo->status().active());
    CHECK(env.demo->status().mode == RuntimeMode::Discovery);
    controller.stopRobot();
}
