// The real runtime host (RealRuntimeHost, the default of AppServices) driven headlessly against a synthetic project whose
// "gradlew" is a tiny script: start-up failures, stop, restart and the shell's view of them. No WPILib, JDK or display needed.
#include <doctest/doctest.h>

#include <QDir>
#include <QFile>
#include <QSignalSpy>

#include "common/real_runtime_host.h"
#include "common/robot_basics.h"
#include "gui_support.h"
#include "shell/main_window.h"
#include "shell/page_factory.h"
#include "shell/shell_controller.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;
namespace sdk = wpiemu::sdk;

namespace {

#if defined(_WIN32)
constexpr const char* kWaitScript = "@echo off\r\necho fake gradle started\r\n:loop\r\nping -n 2 127.0.0.1 >nul\r\ngoto loop\r\n";
constexpr const char* kFailScript = "@echo off\r\necho fake gradle: BUILD FAILED\r\nexit /b 3\r\n";
constexpr const char* kJavaHomeScript = "@echo off\r\necho gradle JAVA_HOME=%JAVA_HOME%\r\nexit /b 3\r\n";
constexpr const char* kWrapper = "/gradlew.bat";
#else
constexpr const char* kWaitScript = "#!/bin/sh\necho fake gradle started\nwhile true; do sleep 1; done\n";
constexpr const char* kFailScript = "#!/bin/sh\necho fake gradle: BUILD FAILED\nexit 3\n";
constexpr const char* kJavaHomeScript = "#!/bin/sh\necho \"gradle JAVA_HOME=$JAVA_HOME\"\nexit 3\n";
constexpr const char* kWrapper = "/gradlew";
#endif

// App services with the real host, one open project with a saved robot model, and a stand-in for halsim_wpiemu.
struct RealFixture {
    GuiEnvironment env{true, true};
    ShellController controller{*env.services};
    TestProject project;
    QTemporaryDir extensionDir;
    QStringList logs;
    QByteArray previousExtensionEnv = qgetenv("WPIEMU_EXTENSION");

    explicit RealFixture(const char* script) {
        writeFile(project.path() + kWrapper, script);
#if !defined(_WIN32)
        QFile::setPermissions(project.path() + kWrapper, QFile::permissions(project.path() + kWrapper) | QFile::ExeOwner);
#endif
        env.addProject(project);
        const QString extension = extensionDir.path() + "/halsim_wpiemu_fake";
        writeFile(extension, "x");
        qputenv("WPIEMU_EXTENSION", extension.toUtf8());

        REQUIRE(controller.openProject(project.path()) == OpenOutcome::NeedsSetup);
        ProjectSession& session = env.services->session();
        const sdk::RobotConfig config = buildRobotConfig(defaultBasicsFor(sdk::drivetrain_kind::kSwerve), env.services->buildContext(session.inspection()));
        REQUIRE(session.commit(config, session.prefs()).ok());
        QObject::connect(&env.services->runtime(), &IRuntimeHost::logLine, [this](const HostLogLine& l) { logs << l.text; });
    }
    ~RealFixture() {
        env.services->runtime().stop();
        waitFor([&] { return !env.services->runtime().status().active(); }, 15000);
        if (previousExtensionEnv.isEmpty()) qunsetenv("WPIEMU_EXTENSION");
        else qputenv("WPIEMU_EXTENSION", previousExtensionEnv);
    }

    IRuntimeHost& host() { return env.services->runtime(); }
    bool waitProcess(sdk::ProcessState state, int timeoutMs = 20000) {
        return waitFor([&] { return host().status().process == state; }, timeoutMs);
    }
    bool logged(const QString& fragment) const {
        for (const QString& l : logs)
            if (l.contains(fragment)) return true;
        return false;
    }
};

}  // namespace

TEST_CASE("the default runtime is the real host, not the 'no runtime' placeholder") {
    TestEnvironment env(true, true);
    CHECK(dynamic_cast<RealRuntimeHost*>(&env.services->runtime()) != nullptr);
    CHECK(env.services->runtime().status().process == sdk::ProcessState::Idle);
}

TEST_CASE("real host: pending Run intent belongs to the project across config edits and relaunches") {
    // No configured model: launch fails synchronously, without starting a robot or an IPC worker.
    RealRuntimeHost host({});
    host.setProject(QStringLiteral("A"), {}, nullptr);
    host.startSimulation();
    REQUIRE(host.simulationStartPending());
    host.restart();
    CHECK(host.simulationStartPending());
    host.setProject(QStringLiteral("A"), std::make_shared<const sdk::RobotConfig>(), nullptr);
    CHECK(host.simulationStartPending());
    host.setProject(QStringLiteral("B"), {}, nullptr);
    CHECK_FALSE(host.simulationStartPending());
    host.startSimulation();
    CHECK(host.simulationStartPending());
    host.stopSimulation();
    CHECK_FALSE(host.simulationStartPending());
}

TEST_CASE("real host: the selected start position belongs to the project and its config") {
    auto withPositions = [](std::initializer_list<const char*> ids) {
        sdk::RobotConfig c;
        for (const char* id : ids) c.startPositions.push_back({id, id, sdk::Alliance::Red, {1.f, 2.f, 0.f}});
        return std::make_shared<const sdk::RobotConfig>(std::move(c));
    };
    RealRuntimeHost host({});
    QSignalSpy changed(&host, &IRuntimeHost::startPositionChanged);
    host.setProject(QStringLiteral("A"), withPositions({"a", "b"}), nullptr);
    host.setStartPosition(QStringLiteral("a"));
    CHECK(host.startPosition() == QStringLiteral("a"));
    host.setStartPosition(QStringLiteral("missing"));
    CHECK(host.startPosition().isEmpty());
    host.setStartPosition(QStringLiteral("a"));
    host.setProject(QStringLiteral("A"), withPositions({"a"}), nullptr);   // an edit that keeps it
    CHECK(host.startPosition() == QStringLiteral("a"));
    host.setProject(QStringLiteral("A"), withPositions({"b"}), nullptr);   // deleted
    CHECK(host.startPosition().isEmpty());
    host.setStartPosition(QStringLiteral("b"));
    host.setProject(QStringLiteral("B"), withPositions({"b"}), nullptr);   // another project
    CHECK(host.startPosition().isEmpty());
    CHECK(changed.count() == 6);
}

TEST_CASE("Run intent: readiness consumes only the current project's deliberate request") {
    SimulationStartIntent intent;
    intent.projectReplaced();   // A
    intent.request();
    CHECK_FALSE(intent.takeWhenReady(false));
    CHECK(intent.pending());
    intent.projectReplaced();   // B becomes ready after A's late start
    CHECK_FALSE(intent.takeWhenReady(true));
    intent.request();
    CHECK(intent.takeWhenReady(true));
    CHECK_FALSE(intent.takeWhenReady(true));
    intent.request();           // same-project edited-config/restart does not replace the project
    CHECK_FALSE(intent.takeWhenReady(false));
    CHECK(intent.takeWhenReady(true));
}

TEST_CASE("real host: a project whose build fails is reported as FailedToStart with the console tail") {
    RealFixture f(kFailScript);
    REQUIRE(dynamic_cast<RealRuntimeHost*>(&f.host()) != nullptr);
    CHECK(f.host().scene().config != nullptr);
    CHECK(f.host().scene().gameName.contains("2026"));
    CHECK(f.host().scene().field != nullptr);   // the field the plugin builds, for the viewport

    f.controller.launchRobot();
    REQUIRE(f.waitProcess(sdk::ProcessState::FailedToStart));
    const HostStatus s = f.host().status();
    CHECK(s.exitCode == 3);
    CHECK(s.detail.contains("BUILD FAILED"));
    CHECK(!s.active());
    CHECK(f.controller.runtimeLabel() == QStringLiteral("Robot failed to start"));
    CHECK(f.logged("fake gradle: BUILD FAILED"));
    // the user can try again after fixing the project
    f.controller.rescanDevices();
    REQUIRE(f.waitProcess(sdk::ProcessState::FailedToStart));
}

TEST_CASE("real host: the JDK folder chosen in Settings is what Gradle is launched with") {
    RealFixture f(kJavaHomeScript);
    AppSettingsData chosen = f.env.services->settings().data();
    chosen.jdkHome = QStringLiteral("/chosen/jdk");
    f.env.services->settings().update(chosen);

    f.controller.launchRobot();
    REQUIRE(f.waitProcess(sdk::ProcessState::FailedToStart));
    CHECK(f.logged("gradle JAVA_HOME=/chosen/jdk"));
}

TEST_CASE("real host: Stop ends a starting robot and the process tree, restart starts a fresh run") {
    RealFixture f(kWaitScript);
    f.host().launch();
    CHECK(f.host().status().process == sdk::ProcessState::Starting);
    CHECK(f.host().status().active());
    CHECK(f.controller.runtimeLabel() == QStringLiteral("Starting robot…"));
    CHECK(f.host().latestSnapshot() != nullptr);   // an idle model snapshot is available right away

    QSignalSpy spy(&f.host(), &IRuntimeHost::statusChanged);
    REQUIRE(waitFor([&] { return f.logged("fake gradle started"); }));   // the shell has to start before Stop can end its output
    f.host().stop();
    CHECK(f.host().status().process == sdk::ProcessState::Stopping);
    REQUIRE(f.waitProcess(sdk::ProcessState::Exited));
    CHECK(spy.count() >= 1);
    CHECK(f.logged("fake gradle started"));

    f.host().launch();   // after a stop a new run starts cleanly
    CHECK(f.host().status().process == sdk::ProcessState::Starting);
    f.host().launch();   // launching twice keeps the one instance
    CHECK(f.host().status().process == sdk::ProcessState::Starting);
    f.host().restart();
    CHECK(f.host().status().process == sdk::ProcessState::Starting);
    f.host().stop();
    REQUIRE(f.waitProcess(sdk::ProcessState::Exited));
}

TEST_CASE("real host: the window shows a failed start and closing the project stops the robot") {
    RealFixture f(kWaitScript);
    MainWindow window(*f.env.services, f.controller, f.env.ui, createShellPages(*f.env.services, f.env.ui, &f.controller));
    window.show();
    f.controller.launchRobot();
    CHECK(f.host().status().active());
    f.controller.closeProject();
    REQUIRE(waitFor([&] { return !f.host().status().active(); }));
    CHECK(f.host().status().process == sdk::ProcessState::Idle);   // no project: nothing to report
    CHECK(window.currentPage() == MainWindow::PageId::Welcome);
}

// ---- opt-in: the real shell host against a real WPILib project (WPIEMU_E2E=1; see app/host/tests/e2e_test.cpp for the full run) ----
namespace {

void copyTree(const QString& from, const QString& to) {
    QDir().mkpath(to);
    const QDir src(from);
    for (const QFileInfo& e : src.entryInfoList(QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden)) {
        const QString name = e.fileName();
        if (name == ".git" || name == "build" || name == ".gradle" || name == "logs") continue;
        if (e.isDir()) copyTree(e.absoluteFilePath(), to + "/" + name);
        else QFile::copy(e.absoluteFilePath(), to + "/" + name);
    }
}

}  // namespace

TEST_CASE("real host e2e (opt-in): the shell discovers the devices of a real robot and runs it") {
    if (qEnvironmentVariableIsEmpty("WPIEMU_E2E")) {
        MESSAGE("skipped: set WPIEMU_E2E=1 to run the shell against a real WPILib project");
        return;
    }
    const QString original = !qEnvironmentVariableIsEmpty("WPIEMU_E2E_PROJECT")
                                 ? qEnvironmentVariable("WPIEMU_E2E_PROJECT")
                                 : QStringLiteral(WPIEMU_SAMPLES_DIR "/swerve_robot/java");
    if (!QFileInfo::exists(original + "/gradlew.bat") && !QFileInfo::exists(original + "/gradlew")) {
        MESSAGE("skipped: no robot project at " << original.toStdString());
        return;
    }
    qunsetenv("WPIEMU_EXTENSION");   // exercise the real lookup (install layout, then the build-tree fallback)

    GuiEnvironment env(true, true);
    ShellController controller(*env.services);
    QTemporaryDir work;
    const QString project = work.path() + "/project";
    copyTree(original, project);
    env.probe->files.insert(QDir::cleanPath(project + "/gradlew"));
    REQUIRE(controller.openProject(project) == OpenOutcome::NeedsSetup);
    ProjectSession& session = env.services->session();
    const sdk::RobotConfig config = buildRobotConfig(defaultBasicsFor(sdk::drivetrain_kind::kSwerve), env.services->buildContext(session.inspection()));
    REQUIRE(session.commit(config, session.prefs()).ok());
    IRuntimeHost& host = env.services->runtime();
    int discovered = 0;
    QObject::connect(&host, &IRuntimeHost::snapshotPublished, [&] {
        if (const auto snap = host.latestSnapshot()) discovered = static_cast<int>(snap->devices().size());
    });

    controller.launchRobot();
    REQUIRE_MESSAGE(waitFor([&] { return discovered >= 16 || !host.status().active(); }, 240000), host.status().detail.toStdString());
    CHECK(discovered >= 16);
    CHECK(host.status().process == sdk::ProcessState::Running);
    CHECK(host.status().link == sdk::LinkState::Connected);
    host.stop();
    REQUIRE(waitFor([&] { return !host.status().active(); }, 30000));
    CHECK(host.status().process == sdk::ProcessState::Exited);

    sdk::proto::DsState ds;
    ds.enabled = true;
    ds.mode = sdk::proto::RobotMode::Teleop;
    host.setDriverStation(ds);
    controller.startSimulation();
    std::shared_ptr<const sdk::ISimSnapshot> snap;
    REQUIRE_MESSAGE(waitFor(
                        [&] {
                            snap = host.latestSnapshot();
                            return (snap && snap->robotConnected() && snap->simTimeUs() > 3'000'000) || !host.status().active();
                        },
                        240000),
                    host.status().detail.toStdString());
    REQUIRE(snap != nullptr);
    CHECK(snap->robotConnected());
    CHECK(snap->simTimeUs() > 3'000'000);
    CHECK(snap->driverStation().enabled);
    CHECK(host.status().link == sdk::LinkState::Connected);
    host.stop();
    REQUIRE(waitFor([&] { return !host.status().active(); }, 30000));
    CHECK(host.status().process == sdk::ProcessState::Exited);
}
