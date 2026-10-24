#pragma once

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QSet>
#include <QTemporaryDir>

#include <chrono>
#include <functional>
#include <map>
#include <memory>

#include "common/app_services.h"
#include "common/demo_runtime_host.h"
#include "common/key_value_store.h"
#include "common/robot_basics.h"
#include "common/system_probe.h"
#include "wpiemu/sdk/testing/wait.h"

namespace wpiemu::app::testing {

// Runs the event loop until `done()` or the timeout; true when `done()` became true.
inline bool waitFor(const std::function<bool()>& done, int timeoutMs = 8000) {
    return sdk::testing::waitUntil(done, std::chrono::milliseconds(timeoutMs), std::chrono::milliseconds::zero(),
                                   [] { QCoreApplication::processEvents(QEventLoop::AllEvents, 20); });
}

inline void writeFile(const QString& path, const QByteArray& content) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (f.open(QIODevice::WriteOnly)) f.write(content);
}

// A fake machine: only what is registered exists.
class FakeProbe final : public ISystemProbe {
public:
    HostOs os = HostOs::Linux;
    QString home = QStringLiteral("/home/user");
    QSet<QString> files;
    QSet<QString> dirs;
    std::map<QString, QString> environment;
    std::map<QString, QString> onPath;
    std::map<QString, ProcessResult> processes;   // by program
    std::map<QString, QString> fileVersions;      // by cleaned path (DLL product versions)
    QString cpu = QStringLiteral("x86_64");
    mutable int runCount = 0;

    HostOs hostOs() const override { return os; }
    QString cpuArchitecture() const override { return cpu; }
    QString fileVersion(const QString& path) const override {
        if (versionHook) return versionHook(QDir::cleanPath(path));
        const auto it = fileVersions.find(QDir::cleanPath(path));
        return it == fileVersions.end() ? QString() : it->second;
    }
    std::function<QString(const QString&)> versionHook;   // overrides fileVersions (tests whose files really move around)
    QString homeDir() const override { return home; }
    QString env(const QString& name) const override {
        const auto it = environment.find(name);
        return it == environment.end() ? QString() : it->second;
    }
    bool fileExists(const QString& path) const override { return files.contains(QDir::cleanPath(path)); }
    bool dirExists(const QString& path) const override { return dirs.contains(QDir::cleanPath(path)); }
    QString findInPath(const QString& exe) const override {
        const auto it = onPath.find(exe);
        return it == onPath.end() ? QString() : it->second;
    }
    ProcessResult run(const QString& program, const QStringList&, int) const override {
        ++runCount;
        const auto it = processes.find(program);
        return it == processes.end() ? ProcessResult{} : it->second;
    }

    // A working JDK: java, its compiler and the version text `java -version` prints.
    void addJava(const QString& javaPath, const QString& versionText, bool withCompiler = true) {
        files.insert(QDir::cleanPath(javaPath));
        if (withCompiler) {
            QString javac = javaPath;
            const int at = javac.lastIndexOf(QLatin1String("java"));
            if (at >= 0) javac.replace(at, 4, QStringLiteral("javac"));
            files.insert(QDir::cleanPath(javac));
        }
        ProcessResult r;
        r.started = true;
        r.exitCode = 0;
        r.output = versionText;
        processes[javaPath] = r;
    }
};

// A throw-away WPILib project folder.
struct TestProject {
    QTemporaryDir dir;
    QString language = QStringLiteral("java");
    int year = 2026;

    explicit TestProject(const QString& lang = QStringLiteral("java"), int yr = 2026, bool gradleRio = true) : language(lang), year(yr) {
        const QString root = dir.path();
        const QString plugin =
            gradleRio ? QStringLiteral("id \"%1\" version \"%2.2.1\"")
                            .arg(year >= 2027 ? QStringLiteral("org.wpilib.GradleRIO") : QStringLiteral("edu.wpi.first.GradleRIO"))
                            .arg(yr)
                      : QStringLiteral("id \"java\"");
        writeFile(root + "/build.gradle", QStringLiteral("plugins {\n  id \"%1\"\n  %2\n}\n").arg(lang == "cpp" ? "cpp" : "java", plugin).toUtf8());
        writeFile(root + "/gradlew", "#!/bin/sh\n");
        writeFile(root + "/gradlew.bat", "@echo off\n");
        writeFile(root + "/.wpilib/wpilib_preferences.json",
                  QString::fromUtf8(R"json({"enableCppIntellisense": false, "currentLanguage": "%1", "projectYear": "%2", "teamNumber": 1234})json")
                      .arg(lang)
                      .arg(yr)
                      .toUtf8());
        writeFile(root + "/vendordeps/Phoenix6.json", R"json({"name": "CTRE-Phoenix (v6)", "version": "26.1.0", "uuid": "x"})json");
        QDir().mkpath(root + (lang == "cpp" ? "/src/main/cpp" : "/src/main/java"));
    }

    QString path() const { return QDir::cleanPath(dir.path()); }
};

// Everything a test needs: in-memory settings, a temp app-data folder, a fake machine and the
// demo runtime host, wired like the real composition root.
struct TestEnvironment {
    QTemporaryDir appData;
    FakeProbe* probe = nullptr;
    DemoRuntimeHost* demo = nullptr;
    std::unique_ptr<AppServices> services;

    explicit TestEnvironment(bool healthyMachine = true, bool realRuntime = false,
                             const std::function<void(AppServicesOptions&)>& tweak = {}) {
        AppServicesOptions options;
        options.store = std::make_unique<MemoryKeyValueStore>();
        options.appDataRoot = std::filesystem::path(appData.path().toStdU16String());
        auto fake = std::make_unique<FakeProbe>();
        probe = fake.get();
        if (healthyMachine) {
            fake->os = HostOs::Linux;
            fake->home = QStringLiteral("/home/user");
            fake->addJava(QStringLiteral("/jdk/bin/java"), QStringLiteral("openjdk version \"17.0.9\" 2023-10-17"));
            fake->environment[QStringLiteral("JAVA_HOME")] = QStringLiteral("/jdk");
            fake->onPath[QStringLiteral("g++")] = QStringLiteral("/usr/bin/g++");
            fake->dirs.insert(QStringLiteral("/home/user/wpilib/2026"));
        }
        options.probe = std::move(fake);
        if (!realRuntime) {   // otherwise AppServices builds the real host (RealRuntimeHost), like main()
            auto host = std::make_unique<DemoRuntimeHost>();
            demo = host.get();
            host->setStepIntervals(5, 5);
            options.runtime = std::move(host);
        }
        if (tweak) tweak(options);
        services = std::make_unique<AppServices>(std::move(options));
    }

    // The fake machine must know the project's wrapper script (the real probe would see the file).
    void addProject(const TestProject& project, bool withWrapper = true) {
        if (withWrapper) probe->files.insert(QDir::cleanPath(project.path() + "/gradlew"));
    }
};

// The robot model the wizard builds for an inspected project from its starting values for `drivetrainKind`.
inline sdk::RobotConfig defaultRobotConfig(const AppServices& services, const ProjectInspection& inspection,
                                           const std::string& drivetrainKind = sdk::drivetrain_kind::kSwerve) {
    return buildRobotConfig(defaultBasicsFor(drivetrainKind), services.buildContext(inspection));
}

// The same for the project that is open in `services`.
inline sdk::RobotConfig defaultRobotConfig(AppServices& services) {
    return defaultRobotConfig(services, services.session().inspection());
}

}  // namespace wpiemu::app::testing
