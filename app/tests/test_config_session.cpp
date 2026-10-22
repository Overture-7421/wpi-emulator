#include <doctest/doctest.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include "test_support.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

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

TEST_CASE("commit writes the project file, and reopening restores it") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());

    sdk::RobotConfig config = defaultRobotConfig(*env.services);
    config.name = "Saved robot";
    const SaveOutcome outcome = session.commit(config, ProjectPrefs{sdk::Alliance::Blue, 1});
    CHECK(outcome.ok());
    CHECK(outcome.path == env.services->configs().configPath(project.path()));
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

TEST_CASE("the project file can be shared with another checkout") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    sdk::RobotConfig config = defaultRobotConfig(*env.services);
    config.name = "Shared robot";
    const SaveOutcome outcome = session.commit(config, ProjectPrefs{});
    REQUIRE(outcome.ok());
    REQUIRE(session.close().ok());

    TestProject checkout;
    const QString shared = env.services->configs().configPath(checkout.path());
    REQUIRE(QDir().mkpath(QFileInfo(shared).absolutePath()));
    REQUIRE(QFile::copy(outcome.path, shared));
    REQUIRE(session.open(checkout.path()).ok());
    REQUIRE(session.hasConfig());
    CHECK(session.config()->name == config.name);

    env.services->bindRuntime(session.config());
    CHECK(env.demo->scene().projectPath == checkout.path());
}

TEST_CASE("an unreadable stored model asks for setup instead of crashing") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    const SaveOutcome outcome = session.commit(defaultRobotConfig(*env.services), ProjectPrefs{});
    REQUIRE(outcome.ok());
    REQUIRE(session.close().ok());
    writeFile(outcome.path, "{ this is not json");
    REQUIRE(session.open(project.path()).ok());
    CHECK(session.needsSetup());
    CHECK(!session.loadError().isEmpty());

    // finishing setup again replaces the file but keeps the unreadable one
    REQUIRE(session.commit(defaultRobotConfig(*env.services), ProjectPrefs{}).ok());
    CHECK(session.hasConfig());
    CHECK(QFileInfo::exists(outcome.path + ".unreadable.bak"));
}

TEST_CASE("a project folder that cannot be written fails the save") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    // a regular file where the .wpiemu folder should be makes the save impossible
    writeFile(project.path() + "/.wpiemu", "blocker");
    const SaveOutcome outcome = session.commit(defaultRobotConfig(*env.services), ProjectPrefs{});
    CHECK_FALSE(outcome.ok());
    CHECK(!outcome.error.isEmpty());
    CHECK_FALSE(session.hasConfig());
}

namespace {

sdk::RobotConfig namedConfig(AppServices& services, const char* name) {
    sdk::RobotConfig config = defaultRobotConfig(services);
    config.name = name;
    return config;
}

// Pumps events for a while so the watcher and its settle timer get every chance to act.
void settle() { (void)waitFor([] { return false; }, 900); }

}  // namespace

TEST_CASE("a robot setup changed on disk, as by a git pull, is loaded again") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    REQUIRE(session.commit(namedConfig(*env.services, "Before"), ProjectPrefs{}).ok());

    // written outside the session, the way a pull replaces the file
    REQUIRE(env.services->configs().save(project.path(), namedConfig(*env.services, "Pulled")).ok());
    CHECK(waitFor([&] { return session.config()->name == "Pulled"; }));
}

TEST_CASE("the app's own saves do not reload the robot setup") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    REQUIRE(session.commit(namedConfig(*env.services, "First"), ProjectPrefs{}).ok());
    int changes = 0;
    QObject::connect(&session, &ProjectSession::configChanged, [&] { ++changes; });

    REQUIRE(session.commit(namedConfig(*env.services, "Second"), ProjectPrefs{}).ok());
    settle();
    CHECK(changes == 1);   // the commit itself, no reload on top
    CHECK(session.config()->name == "Second");
}

TEST_CASE("an outside change never replaces unsaved edits without asking") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    REQUIRE(session.commit(namedConfig(*env.services, "Mine"), ProjectPrefs{}).ok());
    session.setUnsavedChangesQuery([] { return true; });
    int conflicts = 0;
    QObject::connect(&session, &ProjectSession::externalChangeConflict, [&] { ++conflicts; });

    REQUIRE(env.services->configs().save(project.path(), namedConfig(*env.services, "Pulled")).ok());
    REQUIRE(waitFor([&] { return conflicts == 1; }));
    CHECK(session.config()->name == "Mine");

    // asked once until answered, however many times the file moves meanwhile
    REQUIRE(env.services->configs().save(project.path(), namedConfig(*env.services, "Pulled again")).ok());
    settle();
    CHECK(conflicts == 1);

    // "keep my edits": the same file does not ask again, a newer change does
    session.keepLocalChanges();
    settle();
    CHECK(conflicts == 1);
    REQUIRE(env.services->configs().save(project.path(), namedConfig(*env.services, "Pulled later")).ok());
    REQUIRE(waitFor([&] { return conflicts == 2; }));

    // "load it from disk"
    REQUIRE(session.reloadFromDisk().ok());
    CHECK(session.config()->name == "Pulled later");
}

TEST_CASE("an unreadable or deleted robot setup on disk keeps the open setup and says so") {
    TestEnvironment env;
    TestProject project;
    ProjectSession& session = env.services->session();
    REQUIRE(session.open(project.path()).ok());
    const SaveOutcome outcome = session.commit(namedConfig(*env.services, "Open"), ProjectPrefs{});
    REQUIRE(outcome.ok());
    QStringList messages;
    QObject::connect(&session, &ProjectSession::externalChangeIgnored, [&](const QString& m) { messages << m; });

    writeFile(outcome.path, "<<<<<<< HEAD a merge conflict");
    REQUIRE(waitFor([&] { return messages.size() == 1; }));
    CHECK(messages.front().contains("could not be read"));
    CHECK(session.hasConfig());
    CHECK(session.config()->name == "Open");
    settle();
    CHECK(messages.size() == 1);   // said once for that content

    REQUIRE(QFile::remove(outcome.path));
    REQUIRE(waitFor([&] { return messages.size() == 2; }));
    CHECK(messages.back().contains("deleted"));
    CHECK(session.config()->name == "Open");
}
