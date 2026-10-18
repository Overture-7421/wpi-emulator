#include <doctest/doctest.h>

#include <QFileInfo>
#include <QSignalSpy>

#include "test_support.h"
#include "wizard/wizard_view_model.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

TEST_CASE("wizard starts at the project step and gates Next on a valid project") {
    TestEnvironment env;
    WizardViewModel vm(*env.services);
    CHECK(vm.step() == WizardStep::Project);
    CHECK(!vm.canGoBack());
    CHECK(!vm.canGoNext());

    vm.setProjectPath(QStringLiteral("/no/such/folder"));
    CHECK(!vm.projectValid());
    CHECK(!vm.canGoNext());
    vm.goTo(WizardStep::Robot);   // cannot skip ahead
    CHECK(vm.step() == WizardStep::Project);

    TestProject project;
    env.addProject(project);
    vm.setProjectPath(project.path());
    CHECK(vm.projectValid());
    CHECK(vm.inspection().wpilibYear == 2026);
    CHECK(vm.basics().name == QFileInfo(project.path()).fileName());   // robot is named after the project
    CHECK(!vm.hasExistingConfig());
}

TEST_CASE("toolchain is checked in the background and the result is delivered once") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardViewModel vm(*env.services);
    QSignalSpy changed(&vm, &WizardViewModel::toolchainChanged);
    vm.setProjectPath(project.path());
    CHECK(vm.toolchainRunning());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    CHECK(!vm.toolchainRunning());
    CHECK(vm.toolchain().canRunRobot());
    CHECK(!vm.toolchain().coreOnly());
    CHECK(changed.count() >= 2);   // started + finished

    vm.next();
    CHECK(vm.step() == WizardStep::Toolchain);
    vm.recheckToolchain();
    CHECK(vm.toolchainRunning());
    CHECK(!vm.canGoNext());   // not while a check is running
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    CHECK(vm.canGoNext());
}

TEST_CASE("a missing JDK does not block setup, only discovery") {
    TestEnvironment env(/*healthyMachine=*/false);
    TestProject project;
    env.addProject(project);
    WizardViewModel vm(*env.services);
    vm.setProjectPath(project.path());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    CHECK(!vm.toolchain().canRunRobot());
    CHECK(vm.canGoNext());
    CHECK(!vm.discoveryPossible());
    vm.next();
    vm.next();
    CHECK(vm.step() == WizardStep::Robot);
    vm.next();
    CHECK(vm.step() == WizardStep::Summary);
    const WizardResult r = vm.finish();
    CHECK(r.ok);
    CHECK(!r.detectDevices);   // asked for by default, but impossible without a JDK
}

TEST_CASE("an unsupported season falls back to core-only mode") {
    TestEnvironment env;
    TestProject project(QStringLiteral("java"), 2031);
    env.addProject(project);
    WizardViewModel vm(*env.services);
    vm.setProjectPath(project.path());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    CHECK(vm.toolchain().coreOnly());
    CHECK(vm.canGoNext());
    CHECK(vm.previewConfig().plugins.empty());
}

TEST_CASE("navigation walks through all four steps and back") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardViewModel vm(*env.services);
    QSignalSpy steps(&vm, &WizardViewModel::stepChanged);
    vm.setProjectPath(project.path());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    vm.next();
    vm.next();
    vm.next();
    CHECK(vm.step() == WizardStep::Summary);
    CHECK(vm.isLastStep());
    CHECK(!vm.canGoNext());
    CHECK(steps.count() == 3);
    vm.back();
    CHECK(vm.step() == WizardStep::Robot);
    vm.goTo(WizardStep::Project);
    CHECK(vm.step() == WizardStep::Project);
    vm.goTo(WizardStep::Summary);   // everything before is complete, so jumping forward is allowed
    CHECK(vm.step() == WizardStep::Summary);
}

TEST_CASE("invalid robot values block the robot step") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    WizardViewModel vm(*env.services);
    vm.setProjectPath(project.path());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    vm.next();
    vm.next();
    RobotBasics b = vm.basics();
    b.massKg = 1.0;
    vm.setBasics(b);
    CHECK(!vm.basicsValid());
    CHECK(!vm.canGoNext());
    CHECK(!vm.basicsIssues().empty());
    b.massKg = 50.0;
    vm.setBasics(b);
    CHECK(vm.canGoNext());
}

TEST_CASE("switching the drivetrain loads its defaults but keeps the robot name and alliance") {
    TestEnvironment env;
    WizardViewModel vm(*env.services);
    RobotBasics b = vm.basics();
    b.name = QStringLiteral("Bessie");
    b.alliance = Alliance::Blue;
    b.station = 3;
    vm.setBasics(b);
    vm.setDrivetrainKind(sdk::drivetrain_kind::kTank);
    CHECK(vm.basics().drivetrain == sdk::drivetrain_kind::kTank);
    CHECK(vm.basics().wheelsPerSide == 3);
    CHECK(vm.basics().name == QStringLiteral("Bessie"));
    CHECK(vm.basics().alliance == Alliance::Blue);
    CHECK(vm.basics().station == 3);
}

TEST_CASE("finish saves the model, remembers the alliance and opens the project") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    AppServices& s = *env.services;
    WizardViewModel vm(s);
    vm.setProjectPath(project.path());
    REQUIRE(waitFor([&] { return vm.toolchainChecked(); }));
    RobotBasics b = vm.basics();
    b.name = QStringLiteral("Wizard bot");
    b.alliance = Alliance::Blue;
    b.station = 1;
    b.massKg = 50.0;
    vm.setBasics(b);
    vm.setDetectDevicesAfter(true);

    CHECK(!s.configs().exists(project.path()));   // nothing yet: cancelling would leave no trace
    const WizardResult result = vm.finish();
    REQUIRE(result.ok);
    CHECK(result.detectDevices);
    CHECK(result.outcome.exportSaved);
    CHECK(QFileInfo::exists(project.path() + "/.wpiemu/robot.json"));
    CHECK(s.session().hasConfig());
    CHECK(s.session().config()->name == "Wizard bot");
    CHECK(s.session().config()->chassis.massKg == doctest::Approx(50.0));
    CHECK(s.session().config()->plugins.count("game_2026") == 1);
    CHECK(s.session().prefs().alliance == Alliance::Blue);
    CHECK(s.session().prefs().station == 1);
    CHECK(s.recentProjects().lastProject() == project.path());
}

TEST_CASE("finish refuses an incomplete wizard and writes nothing") {
    TestEnvironment env;
    WizardViewModel vm(*env.services);
    const WizardResult noProject = vm.finish();
    CHECK(!noProject.ok);
    CHECK(!noProject.message.isEmpty());

    TestProject project;
    env.addProject(project);
    vm.setProjectPath(project.path());
    RobotBasics b = vm.basics();
    b.massKg = 0.1;
    vm.setBasics(b);
    CHECK(!vm.finish().ok);
    CHECK(!env.services->configs().exists(project.path()));
}

TEST_CASE("reopening the wizard on a configured project starts from the stored robot") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    AppServices& s = *env.services;
    {
        WizardViewModel first(s);
        first.setProjectPath(project.path());
        RobotBasics b = defaultBasicsFor(sdk::drivetrain_kind::kTank);
        b.name = QStringLiteral("Stored");
        b.wheelsPerSide = 4;
        first.setBasics(b);
        REQUIRE(first.finish().ok);
    }
    // keep a mechanism the editor added; the wizard must not drop it
    sdk::RobotConfig edited = *s.session().config();
    sdk::MechanismConfig m;
    m.id = "intake";
    m.type = "mechanism";
    edited.mechanisms.push_back(m);
    REQUIRE(s.session().commit(edited, s.session().prefs()).ok());

    WizardViewModel again(s);   // the project is current: the wizard picks it up
    CHECK(again.projectValid());
    CHECK(again.hasExistingConfig());
    CHECK(again.basics().name == QStringLiteral("Stored"));
    CHECK(again.basics().drivetrain == sdk::drivetrain_kind::kTank);
    CHECK(again.basics().wheelsPerSide == 4);

    RobotBasics b = again.basics();
    b.massKg = 48.0;
    again.setBasics(b);
    REQUIRE(again.finish().ok);
    CHECK(s.session().config()->chassis.massKg == doctest::Approx(48.0));
    CHECK(s.session().config()->mechanisms.size() == 1);
}
