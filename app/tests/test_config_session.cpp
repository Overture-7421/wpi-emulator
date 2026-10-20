#include <doctest/doctest.h>

#include <QFile>
#include <QFileInfo>

#include "common/robot_basics.h"
#include "test_support.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

namespace {
sdk::RobotConfig sampleConfig(AppServices& s, const ProjectInspection& in) {
    return buildRobotConfig(RobotBasics{}, s.buildContext(in));
}
}  // namespace

TEST_CASE("session opens a project, reports the missing setup and records it as recent") {
    TestEnvironment env;
    TestProject project;
    env.addProject(project);
    ProjectSession& session = env.services->session();

    CHECK(!session.hasProject());
    int projectSignals = 0;
    QObject::connect(&session, &ProjectSession::projectChanged, [&] { ++projectSignals; });

    REQUIRE(session.open(project.path()).ok());
    CHECK(session.hasProject());
    CHECK(session.needsSetup());
    CHECK(!session.hasConfig());
    CHECK(session.inspection().wpilibYear == 2026);
    CHECK(session.gameModule() != nullptr);
    CHECK(env.services->recentProjects().lastProject() == project.path());
    CHECK(projectSignals == 1);

    // a nested path resolves to the project root
    REQUIRE(session.close().ok());
    REQUIRE(session.open(project.path() + "/src/main/java").ok());
    CHECK(session.path() == project.path());
}

TEST_CASE("session refuses folders that are not WPILib projects") {
    TestEnvironment env;
    QTemporaryDir empty;
    const auto status = env.services->session().open(empty.path());
    CHECK(!status.ok());
    CHECK(!status.error().message.empty());
    CHECK(!env.services->session().hasProject());
    CHECK(env.services->recentProjects().paths().isEmpty());
}

TEST_CASE("commit writes the app-data copy and the project export, and reopening restores them") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());

    sdk::RobotConfig config = sampleConfig(*env.services, session.inspection());
    config.name = "Saved robot";
    const SaveOutcome outcome = session.commit(config, ProjectPrefs{sdk::Alliance::Blue, 1});
    CHECK(outcome.ok());
    CHECK(outcome.exportSaved);
    CHECK(QFileInfo::exists(outcome.appDataPath));
    CHECK(QFileInfo::exists(project.path() + "/.wpiemu/robot.json"));
    CHECK(session.hasConfig());
    CHECK(!session.needsSetup());

    // a fresh session over the same stores sees the stored model
    REQUIRE(session.close().ok());
    REQUIRE(session.open(project.path()).ok());
    REQUIRE(session.hasConfig());
    CHECK(session.config()->name == "Saved robot");
    CHECK(session.prefs().alliance == sdk::Alliance::Blue);
    CHECK(session.prefs().station == 1);
}

TEST_CASE("the project export alone is enough to reopen (shared via version control)") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    const SaveOutcome outcome = session.commit(sampleConfig(*env.services, session.inspection()), ProjectPrefs{});
    REQUIRE(outcome.ok());
    REQUIRE(session.close().ok());
    REQUIRE(QFile::remove(outcome.appDataPath));
    REQUIRE(session.open(project.path()).ok());
    CHECK(session.hasConfig());
}

TEST_CASE("an unreadable stored model asks for setup instead of crashing") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    const SaveOutcome outcome = session.commit(sampleConfig(*env.services, session.inspection()), ProjectPrefs{});
    REQUIRE(outcome.ok());
    REQUIRE(session.close().ok());
    writeFile(outcome.appDataPath, "{ this is not json");
    writeFile(outcome.exportPath, "{ nor is this");
    REQUIRE(session.open(project.path()).ok());
    CHECK(session.needsSetup());
    CHECK(!session.loadError().isEmpty());
}

TEST_CASE("read-only project folder only downgrades the save to a warning") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    // a regular file where the export folder should be makes the export impossible
    writeFile(project.path() + "/.wpiemu", "blocker");
    const SaveOutcome outcome = session.commit(sampleConfig(*env.services, session.inspection()), ProjectPrefs{});
    CHECK(outcome.ok());
    CHECK(outcome.hasWarning());
    CHECK(!outcome.exportError.isEmpty());
    CHECK(session.hasConfig());
}
