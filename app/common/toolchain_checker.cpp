#include "common/toolchain_checker.h"

#include <QDir>
#include <QRegularExpression>

#include "wpiemu/sdk/vision/photon_profiles.h"

namespace wpiemu::app {

namespace {

QString join(const QString& a, const QString& b) { return QDir::cleanPath(a + QLatin1Char('/') + b); }

QString display(const QString& path) { return QDir::toNativeSeparators(path); }

constexpr int kProbeTimeoutMs = 8000;
constexpr int kMinJdkMajor = 17;

}  // namespace

std::optional<int> ToolchainChecker::parseJavaMajor(const QString& text) {
    static const QRegularExpression re(QStringLiteral("version\\s+\"(\\d+)(?:\\.(\\d+))?"));
    const auto m = re.match(text);
    if (!m.hasMatch()) return std::nullopt;
    const int first = m.captured(1).toInt();
    if (first == 1 && !m.captured(2).isEmpty()) return m.captured(2).toInt();   // 1.8.0_352 -> 8
    return first;
}

QString ToolchainChecker::defaultWpilibHome(int year) const {
    const QString yearDir = year > 0 ? QString::number(year) : QString();
    const QString base = probe_.hostOs() == HostOs::Windows ? QStringLiteral("C:/Users/Public/wpilib")
                                                            : join(probe_.homeDir(), QStringLiteral("wpilib"));
    return yearDir.isEmpty() ? base : join(base, yearDir);
}

QString ToolchainChecker::findWpilibHome(const ToolchainInput& input) const {
    if (!input.wpilibHomeOverride.isEmpty())
        return probe_.dirExists(input.wpilibHomeOverride) ? QDir::cleanPath(input.wpilibHomeOverride) : QString();
    const QString byYear = defaultWpilibHome(input.wpilibYear);
    if (input.wpilibYear > 0 && probe_.dirExists(byYear)) return byYear;
    if (input.wpilibYear > 0 && probe_.hostOs() == HostOs::Windows) {
        // Per-user installs live under the home folder as well.
        const QString perUser = join(join(probe_.homeDir(), QStringLiteral("wpilib")), QString::number(input.wpilibYear));
        if (probe_.dirExists(perUser)) return perUser;
    }
    return {};
}

ToolchainCheck ToolchainChecker::checkJdk(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kJdk);
    check.title = tr("Java (JDK %1+)").arg(kMinJdkMajor);
    check.requiredToRun = true;

    const QString exe = probe_.hostOs() == HostOs::Windows ? QStringLiteral("java.exe") : QStringLiteral("java");
    QStringList candidates;
    if (!input.jdkHomeOverride.isEmpty()) candidates << join(join(input.jdkHomeOverride, QStringLiteral("bin")), exe);
    if (const QString javaHome = probe_.env(QStringLiteral("JAVA_HOME")); !javaHome.isEmpty())
        candidates << join(join(javaHome, QStringLiteral("bin")), exe);
    if (const QString wpilib = findWpilibHome(input); !wpilib.isEmpty())
        candidates << join(join(join(wpilib, QStringLiteral("jdk")), QStringLiteral("bin")), exe);
    if (const QString onPath = probe_.findInPath(exe); !onPath.isEmpty()) candidates << onPath;

    QString lastProblem;
    for (const QString& java : candidates) {
        if (!probe_.fileExists(java)) {
            lastProblem = tr("%1 does not exist.").arg(display(java));
            continue;
        }
        const ProcessResult result = probe_.run(java, {QStringLiteral("-version")}, kProbeTimeoutMs);
        const auto major = result.started ? parseJavaMajor(result.output) : std::nullopt;
        if (!major) {
            lastProblem = tr("Could not run %1.").arg(display(java));
            continue;
        }
        if (*major < kMinJdkMajor) {
            lastProblem = tr("%1 is Java %2; Gradle needs Java %3 or newer.").arg(display(java)).arg(*major).arg(kMinJdkMajor);
            continue;
        }
        check.status = CheckStatus::Ok;
        check.detail = tr("Java %1 at %2").arg(*major).arg(display(java));
        return check;
    }

    check.status = CheckStatus::Missing;
    check.detail = lastProblem.isEmpty() ? tr("No Java installation was found.") : lastProblem;
    check.fixHint = tr("Install the WPILib bundle (it includes a JDK) or a JDK %1+, then set JAVA_HOME "
                       "or point the JDK folder in Settings.").arg(kMinJdkMajor);
    return check;
}

ToolchainCheck ToolchainChecker::checkGradleWrapper(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kGradle);
    check.title = tr("Gradle wrapper");
    check.requiredToRun = true;
    const QString script = probe_.hostOs() == HostOs::Windows ? QStringLiteral("gradlew.bat") : QStringLiteral("gradlew");
    const QString path = join(input.projectPath, script);
    if (!input.projectPath.isEmpty() && probe_.fileExists(path)) {
        check.status = CheckStatus::Ok;
        check.detail = display(path);
        return check;
    }
    check.status = CheckStatus::Missing;
    check.detail = tr("%1 was not found in the project folder.").arg(script);
    check.fixHint = tr("Open the project once in VS Code with the WPILib extension, or run \"gradle wrapper\" in the project folder.");
    return check;
}

ToolchainCheck ToolchainChecker::checkWpilibInstall(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kWpilib);
    check.title = input.wpilibYear > 0 ? tr("WPILib %1 installation").arg(input.wpilibYear) : tr("WPILib installation");
    const QString home = findWpilibHome(input);
    if (!home.isEmpty()) {
        check.status = CheckStatus::Ok;
        check.detail = display(home);
        return check;
    }
    check.status = CheckStatus::Warning;
    const QString expected = input.wpilibHomeOverride.isEmpty() ? defaultWpilibHome(input.wpilibYear) : input.wpilibHomeOverride;
    check.detail = tr("Not found at %1.").arg(display(expected));
    check.fixHint = tr("Optional if a JDK is installed elsewhere. Install WPILib for this season or set its folder in Settings.");
    return check;
}

ToolchainCheck ToolchainChecker::checkCppToolchain(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kCppToolchain);
    check.title = tr("C++ toolchain");
    if (input.language != QLatin1String("cpp")) {
        check.status = CheckStatus::NotApplicable;
        check.detail = input.language == QLatin1String("java") ? tr("Not needed for a Java project.")
                                                               : tr("Not needed unless the project is C++.");
        return check;
    }
    check.requiredToRun = true;

    switch (probe_.hostOs()) {
        case HostOs::Windows: {
            const QString programFiles = probe_.env(QStringLiteral("ProgramFiles(x86)"));
            const QString vswhere = join(programFiles, QStringLiteral("Microsoft Visual Studio/Installer/vswhere.exe"));
            if (!programFiles.isEmpty() && probe_.fileExists(vswhere)) {
                const ProcessResult r = probe_.run(
                    vswhere,
                    {QStringLiteral("-latest"), QStringLiteral("-products"), QStringLiteral("*"), QStringLiteral("-requires"),
                     QStringLiteral("Microsoft.VisualStudio.Component.VC.Tools.x86.x64"), QStringLiteral("-property"),
                     QStringLiteral("installationPath")},
                    kProbeTimeoutMs);
                if (const QString found = r.output.trimmed(); r.started && !found.isEmpty()) {
                    check.status = CheckStatus::Ok;
                    check.detail = tr("Visual Studio C++ tools at %1").arg(found.section(QLatin1Char('\n'), 0, 0).trimmed());
                    return check;
                }
            }
            if (const QString cl = probe_.findInPath(QStringLiteral("cl.exe")); !cl.isEmpty()) {
                check.status = CheckStatus::Ok;
                check.detail = display(cl);
                return check;
            }
            check.status = CheckStatus::Missing;
            check.detail = tr("No MSVC compiler was found.");
            check.fixHint = tr("Install Visual Studio 2022 (or Build Tools) with the \"Desktop development with C++\" workload.");
            return check;
        }
        case HostOs::MacOs: {
            const QString clang = probe_.findInPath(QStringLiteral("clang++"));
            if (!clang.isEmpty()) {
                check.status = CheckStatus::Ok;
                check.detail = display(clang);
                return check;
            }
            check.status = CheckStatus::Missing;
            check.detail = tr("No clang++ was found.");
            check.fixHint = tr("Run \"xcode-select --install\" to install the Xcode command line tools.");
            return check;
        }
        case HostOs::Linux: break;
    }
    for (const char* name : {"g++", "clang++", "c++"}) {
        if (const QString found = probe_.findInPath(QLatin1String(name)); !found.isEmpty()) {
            check.status = CheckStatus::Ok;
            check.detail = display(found);
            return check;
        }
    }
    check.status = CheckStatus::Missing;
    check.detail = tr("No g++ or clang++ was found.");
    check.fixHint = tr("Install a C++ compiler, e.g. \"sudo apt install build-essential\".");
    return check;
}

ToolchainCheck ToolchainChecker::checkGameModule(const ToolchainInput& input, ToolchainReport& report) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kGameModule);
    check.title = tr("Game module");
    const sdk::IGameModule* module = input.wpilibYear > 0 ? plugins_.findByYear(input.wpilibYear) : nullptr;
    if (module != nullptr) {
        const sdk::GameMetadata meta = module->metadata();
        report.gamePluginId = QString::fromStdString(meta.id);
        report.gamePluginName = QString::fromStdString(meta.displayName);
        check.status = CheckStatus::Ok;
        check.detail = tr("%1 field, game pieces and scoring are available.").arg(report.gamePluginName);
        return check;
    }
    check.status = CheckStatus::Warning;
    if (input.wpilibYear > 0) {
        check.detail = tr("No game module for the %1 season yet. Running in core-only mode: the robot, drivetrain "
                          "and mechanisms simulate on an empty field.").arg(input.wpilibYear);
        check.fixHint = tr("Core-only mode is fully usable. A %1 module can be added as a plugin later.").arg(input.wpilibYear);
    } else {
        check.detail = tr("The season could not be detected, so no game module was selected (core-only mode).");
        check.fixHint = tr("Check that build.gradle declares the GradleRIO plugin version.");
    }
    return check;
}

ToolchainCheck ToolchainChecker::checkVision(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kVision);
    check.title = tr("Vision (PhotonLib)");
    if (!input.hasPhotonlib) {
        check.status = CheckStatus::NotApplicable;
        check.detail = tr("No PhotonLib vendor library: PhotonVision cameras are not used (Limelight cameras need no vendor library).");
        return check;
    }
    const QString uuid = QString::fromStdString(sdk::photonMessageUuidForVersion(input.photonlibVersion.toStdString()));
    if (!uuid.isEmpty()) {
        check.status = CheckStatus::Ok;
        check.detail = tr("PhotonLib %1: supported (message definition %2...)").arg(input.photonlibVersion, uuid.left(8));
        return check;
    }
    // PhotonLib throws when the message definition differs from its own, so the bridge refuses PhotonVision cameras instead of guessing.
    check.status = CheckStatus::Warning;
    check.detail = input.photonlibVersion.isEmpty()
                       ? tr("The PhotonLib version could not be read from its vendordep file: PhotonVision cameras are disabled for this "
                            "project to avoid crashing the robot.")
                       : tr("PhotonLib %1 has not been verified with this emulator: PhotonVision cameras are disabled for this project to "
                            "avoid crashing the robot.").arg(input.photonlibVersion);
    check.fixHint = tr("Update the emulator, or, once you have verified that this PhotonLib release still uses the PhotonVision message "
                       "definition %1, set vision.photonMessageUuid to that value in robot.json.")
                        .arg(QString::fromLatin1(sdk::kPhotonMessageUuidV2025_3));
    return check;
}

ToolchainReport ToolchainChecker::run(const ToolchainInput& input) const {
    ToolchainReport report;
    report.checks.push_back(checkJdk(input));
    report.checks.push_back(checkGradleWrapper(input));
    report.checks.push_back(checkWpilibInstall(input));
    report.checks.push_back(checkCppToolchain(input));
    report.checks.push_back(checkGameModule(input, report));
    report.checks.push_back(checkVision(input));
    return report;
}

}  // namespace wpiemu::app
