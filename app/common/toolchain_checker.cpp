#include "common/toolchain_checker.h"

#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QVersionNumber>

#include <algorithm>
#include <filesystem>

#include "wpiemu/core/launcher/jdk_policy.h"
#include "wpiemu/core/launcher/project_inspector.h"

namespace wpiemu::app {

namespace {

QString join(const QString& a, const QString& b) { return QDir::cleanPath(a + QLatin1Char('/') + b); }

QString display(const QString& path) { return QDir::toNativeSeparators(path); }

constexpr int kProbeTimeoutMs = 8000;
constexpr int kFirstSeasonYear = 2024;   // season-literal-ok: oldest WPILib season whose JDK is considered as a fallback

core::config::Platform platformOf(HostOs os) {
    switch (os) {
        case HostOs::Windows: return core::config::Platform::Windows;
        case HostOs::MacOs: return core::config::Platform::MacOS;
        case HostOs::Linux: break;
    }
    return core::config::Platform::Linux;
}

std::filesystem::path folderOf(const QString& text) {
    return text.isEmpty() ? std::filesystem::path() : std::filesystem::path(text.toStdU16String());
}

// Probe paths use forward slashes whatever machine the probe stands in for.
QString probePath(const std::filesystem::path& path, HostOs os) {
    QString text = QString::fromStdU16String(path.generic_u16string());
    if (os == HostOs::Windows) text.replace(QLatin1Char('\\'), QLatin1Char('/'));
    return text;
}

}  // namespace

std::optional<int> ToolchainChecker::parseJavaMajor(const QString& text) { return core::launcher::parseJavaMajor(text.toStdString()); }

QStringList ToolchainChecker::wpilibRoots() const {
    const HostOs os = probe_.hostOs();
    QStringList roots;
    for (const std::filesystem::path& root : core::launcher::wpilibRootCandidates(
             platformOf(os), folderOf(probe_.env(QStringLiteral("PUBLIC"))), folderOf(probe_.homeDir())))
        roots << probePath(root, os);
    return roots;
}

QString ToolchainChecker::defaultWpilibHome(int year) const {
    const QStringList roots = wpilibRoots();
    if (roots.isEmpty()) return {};
    return year > 0 ? join(roots.first(), installFolder(year)) : roots.first();
}

QString ToolchainChecker::installFolder(int year) const {
    const sdk::SeasonProfile* season = seasons_.find(year);
    return QString::fromStdString(core::launcher::wpilibInstallFolderName(year, season != nullptr ? season->installFolder : std::string_view()));
}

QString ToolchainChecker::findWpilibHome(const ToolchainInput& input) const {
    if (!input.wpilibHomeOverride.isEmpty())
        return probe_.dirExists(input.wpilibHomeOverride) ? QDir::cleanPath(input.wpilibHomeOverride) : QString();
    if (input.wpilibYear <= 0) return {};
    const QString folder = installFolder(input.wpilibYear);
    for (const QString& root : wpilibRoots()) {
        const QString home = join(root, folder);
        if (probe_.dirExists(home)) return home;
    }
    return {};
}

std::optional<QString> ToolchainChecker::judgeJdk(const QString& java, bool trusted, const ToolchainInput& input, int* majorOut) const {
    using namespace core::launcher;
    const bool windows = probe_.hostOs() == HostOs::Windows;
    if (!probe_.fileExists(java)) return tr("%1 does not exist.").arg(display(java));
    const ProcessResult result = probe_.run(java, {QStringLiteral("-version")}, kProbeTimeoutMs);
    JdkFacts facts;
    facts.hasJava = true;
    if (result.started) facts.major = core::launcher::parseJavaMajor(result.output.toStdString());
    if (majorOut != nullptr && facts.major) *majorOut = *facts.major;

    const QString clean = QDir::cleanPath(java);
    const QString bin = clean.left(clean.lastIndexOf(QLatin1Char('/')));
    facts.hasCompiler = probe_.fileExists(join(bin, windows ? QStringLiteral("javac.exe") : QStringLiteral("javac")));
    if (windows) {
        const QString version = loadedVcRuntime(probe_, bin);
        if (!version.isEmpty()) facts.vcRuntime = version.toStdString();
    }

    const int maxMajor = maxJavaForGradle(input.gradleVersion.toStdString(), input.minJavaMajor);
    JdkVerdict verdict = evaluateJdk(facts, input.minJavaMajor, maxMajor, windows);
    // The WPILib JDK of the project's season is what WPILib ships and tests; it is not second-guessed.
    if (trusted && (verdict.problem == JdkProblem::TooNewForGradle || verdict.problem == JdkProblem::StaleVcRuntime ||
                    verdict.problem == JdkProblem::NoCompiler))
        verdict.problem = JdkProblem::None;

    switch (verdict.problem) {
        case JdkProblem::None: return std::nullopt;
        case JdkProblem::NotRunnable: return tr("Could not run %1.").arg(display(java));
        case JdkProblem::TooOld:
            return tr("%1 is Java %2; this project needs Java %3 or newer.").arg(display(java)).arg(verdict.major).arg(input.minJavaMajor);
        case JdkProblem::TooNewForGradle:
            return tr("%1 is Java %2, which this project's Gradle %3 cannot run on (Java %4 is the newest it supports).")
                .arg(display(java)).arg(verdict.major)
                .arg(input.gradleVersion.isEmpty() ? QStringLiteral("8.11") : input.gradleVersion).arg(verdict.maxMajor);
        case JdkProblem::NoCompiler:
            return tr("%1 is a Java runtime without a compiler (javac); the robot needs a full JDK.").arg(display(java));
        case JdkProblem::StaleVcRuntime:
            return tr("%1 bundles Visual C++ runtime %2, but WPILib needs %3 or newer. Installing a newer Visual Studio does not change this.")
                .arg(display(java), QString::fromStdString(verdict.vcRuntime), QString::fromLatin1(kMinVcRuntime.data(), static_cast<int>(kMinVcRuntime.size())));
    }
    return std::nullopt;
}

JdkResolution ToolchainChecker::resolveJdk(const ToolchainInput& input) const {
    using core::launcher::JdkSource;
    const bool windows = probe_.hostOs() == HostOs::Windows;
    const QString exe = windows ? QStringLiteral("java.exe") : QStringLiteral("java");
    const auto inBin = [&](const QString& home) { return join(join(home, QStringLiteral("bin")), exe); };

    JdkResolution res;
    // Judges one candidate; true when it is usable (and fills `res`). Failures are remembered for the report.
    const auto consider = [&](JdkSource source, const QString& java, const QString& home, bool trusted) {
        int major = 0;
        if (const std::optional<QString> problem = judgeJdk(java, trusted, input, &major)) {
            res.skipped.push_back({display(java), *problem});
            return false;
        }
        res.found = true;
        res.source = source;
        res.java = java;
        res.home = home;
        res.major = major;
        return true;
    };

    for (const JdkSource source : core::launcher::kJdkSources) {
        switch (source) {
            case JdkSource::Setting:
                if (input.jdkHomeOverride.isEmpty()) break;
                if (!consider(source, inBin(input.jdkHomeOverride), QDir::cleanPath(input.jdkHomeOverride), false)) res.explicitFailed = true;
                return res;   // an explicit choice is never silently replaced
            case JdkSource::WpilibInstall:
                if (const QString wpilib = findWpilibHome(input); !wpilib.isEmpty()) {
                    const QString home = join(wpilib, QStringLiteral("jdk"));
                    if (probe_.fileExists(inBin(home)) && consider(source, inBin(home), home, true)) return res;
                }
                break;
            case JdkSource::WpilibOtherSeason:
                // Every season the emulator knows, newest first, then the older ones down to the first considered season.
                for (int year = seasons_.size() > 0 ? seasons_.list().back().year : kFirstSeasonYear; year >= kFirstSeasonYear; --year) {
                    if (year == input.wpilibYear) continue;
                    const QString folder = installFolder(year);
                    for (const QString& root : wpilibRoots()) {
                        const QString home = join(join(root, folder), QStringLiteral("jdk"));
                        if (probe_.fileExists(inBin(home)) && consider(source, inBin(home), home, false)) return res;
                    }
                }
                break;
            case JdkSource::JavaHome:
                if (const QString javaHome = probe_.env(QStringLiteral("JAVA_HOME")); !javaHome.isEmpty())
                    if (probe_.fileExists(inBin(javaHome)) && consider(source, inBin(javaHome), QDir::cleanPath(javaHome), false)) return res;
                break;
            case JdkSource::Path:
                if (const QString java = probe_.findInPath(exe); !java.isEmpty()) {
                    const QString clean = QDir::cleanPath(java);
                    const QString bin = clean.left(clean.lastIndexOf(QLatin1Char('/')));
                    const bool inBinDir = bin.endsWith(QLatin1String("/bin"));
                    if (consider(source, clean, inBinDir ? bin.left(bin.size() - 4) : QString(), false)) return res;
                }
                break;
            case JdkSource::Managed:
                if (!input.managedJdkHome.isEmpty() && consider(source, inBin(input.managedJdkHome), QDir::cleanPath(input.managedJdkHome), false)) return res;
                break;
        }
    }
    return res;
}

ToolchainCheck ToolchainChecker::checkJdk(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kJdk);
    check.title = tr("Java (JDK %1+)").arg(input.minJavaMajor);
    check.requiredToRun = true;
    check.status = CheckStatus::Missing;

    // Judge the JDK the launcher will use, not whichever one happens to work.
    const JdkResolution jdk = resolveJdk(input);
    QStringList skipped;
    for (const JdkSkip& s : jdk.skipped) skipped << s.reason;

    if (jdk.found) {
        check.status = CheckStatus::Ok;
        check.detail = tr("Java %1 at %2").arg(jdk.major).arg(display(jdk.java));
        if (!skipped.isEmpty()) check.detail += tr("\nPassed over: %1").arg(skipped.join(QLatin1Char(' ')));
        return check;
    }

    check.detail = skipped.isEmpty() ? tr("No Java installation was found.") : skipped.join(QLatin1Char('\n'));
    if (jdk.explicitFailed) {
        check.fixAction = FixAction::OpenSettings;
        check.fixHint = tr("Choose a different JDK folder in Settings, or clear it so the emulator can pick one.");
    } else if (input.managedJavaAvailable) {
        check.fixAction = FixAction::GetJava;
        check.fixHint = tr("Let WPI Emulator download Java %1 for you (no administrator rights needed), or install the WPILib bundle, "
                           "which includes a JDK.").arg(input.minJavaMajor);
    } else {
        check.fixHint = tr("Install the WPILib bundle (it includes a JDK) or a JDK %1+, then set JAVA_HOME "
                           "or point the JDK folder in Settings.").arg(input.minJavaMajor);
    }
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
    check.fixHint = tr("Optional: WPI Emulator works without it, and Gradle downloads the WPILib libraries on the first run (about 1 GB). "
                       "Install WPILib for this season to use its Java and skip that download.");
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
            struct VsInstance {
                QString name, path;
                QVersionNumber version;
            };
            std::vector<VsInstance> instances;
            const QString programFiles = probe_.env(QStringLiteral("ProgramFiles(x86)"));
            const QString vswhere = join(programFiles, QStringLiteral("Microsoft Visual Studio/Installer/vswhere.exe"));
            if (!programFiles.isEmpty() && probe_.fileExists(vswhere)) {
                const ProcessResult r = probe_.run(
                    vswhere,
                    {QStringLiteral("-all"), QStringLiteral("-products"), QStringLiteral("*"), QStringLiteral("-requires"),
                     QStringLiteral("Microsoft.VisualStudio.Component.VC.Tools.x86.x64"), QStringLiteral("-format"), QStringLiteral("json")},
                    kProbeTimeoutMs);
                const QJsonArray found = r.started ? QJsonDocument::fromJson(r.output.toUtf8()).array() : QJsonArray();
                for (const QJsonValue& v : found) {
                    const QJsonObject o = v.toObject();
                    instances.push_back({o.value(QStringLiteral("displayName")).toString(), o.value(QStringLiteral("installationPath")).toString(),
                                         QVersionNumber::fromString(o.value(QStringLiteral("installationVersion")).toString())});
                }
            }
            // WPILib's desktop libraries need Visual Studio 2022 17.9 or newer. Gradle 8.x only knows Visual Studio up to 2022
            // (version 17), so a machine with nothing but Visual Studio 2026 (18) may not be recognised by the project's build.
            const QVersionNumber minimum(17, 9);
            const QVersionNumber knownToGradle(17, 99);
            std::vector<const VsInstance*> usable;
            const VsInstance* tooOld = nullptr;
            const VsInstance* tooNew = nullptr;
            for (const VsInstance& vs : instances) {
                if (vs.version >= minimum && vs.version <= knownToGradle) usable.push_back(&vs);
                else if (vs.version < minimum) tooOld = &vs;
                else tooNew = &vs;
            }
            if (!usable.empty()) {
                const VsInstance* best = *std::max_element(usable.begin(), usable.end(), [](const VsInstance* a, const VsInstance* b) { return a->version < b->version; });
                check.status = CheckStatus::Ok;
                check.detail = tr("%1 (C++ tools) at %2").arg(best->name.isEmpty() ? tr("Visual Studio") : best->name, display(best->path));
                return check;
            }
            if (const QString cl = probe_.findInPath(QStringLiteral("cl.exe")); !cl.isEmpty()) {
                check.status = CheckStatus::Ok;
                check.detail = display(cl);
                return check;
            }
            check.status = CheckStatus::Missing;
            if (tooNew != nullptr) {
                check.detail = tr("%1 has the C++ tools, but WPILib's Gradle may not recognise a Visual Studio newer than 2022.").arg(tooNew->name.isEmpty() ? tr("Visual Studio") : tooNew->name);
                check.fixHint = tr("Install the Visual Studio 2022 Build Tools next to it; both can stay installed.");
            } else if (tooOld != nullptr) {
                check.detail = tr("%1 is older than Visual Studio 2022 17.9, which WPILib needs.").arg(tooOld->name.isEmpty() ? tr("Visual Studio") : tooOld->name);
                check.fixHint = tr("Update Visual Studio, or install the Visual Studio 2022 Build Tools.");
            } else {
                check.detail = tr("No MSVC compiler was found.");
                check.fixHint = tr("Install Visual Studio 2022 (or Build Tools) with the \"Desktop development with C++\" workload.");
            }
            if (input.buildToolsInstallAvailable) {
                check.fixAction = FixAction::InstallBuildTools;
                check.fixHint += tr(" WPI Emulator can start that install for you (several GB; Windows asks for administrator permission).");
            }
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

ToolchainCheck ToolchainChecker::checkWpilibVersion(const ToolchainInput& input) const {
    ToolchainCheck check;
    check.id = QString::fromLatin1(check_id::kWpilibVersion);
    check.title = tr("WPILib version");
    const sdk::SeasonProfile* season = seasons_.find(input.wpilibYear);
    if (season == nullptr || input.wpilibVersion.isEmpty()) {
        check.status = CheckStatus::NotApplicable;
        check.detail = tr("The project does not declare a WPILib version the emulator knows.");
        return check;
    }
    if (season->verifiesWpilib(input.wpilibVersion.toStdString())) {
        check.status = CheckStatus::Ok;
        check.detail = input.wpilibVersion;
        return check;
    }
    check.status = CheckStatus::Warning;
    check.detail = tr("%1 has not been verified with the emulator (verified: %2). The run continues on a best-effort basis.")
                       .arg(input.wpilibVersion, QString::fromStdString(season->wpilibVersion));
    check.fixHint = tr("If the robot misbehaves, use the verified WPILib version or report the version you use.");
    return check;
}

ToolchainReport ToolchainChecker::run(const ToolchainInput& input) const {
    ToolchainReport report;
    if (input.extensionLibrary) {
        ToolchainCheck extension;
        extension.id = QString::fromLatin1(check_id::kHalExtension);
        extension.title = input.wpilibYear > 0 ? tr("WPILib %1 HALSim extension").arg(input.wpilibYear)
                                              : tr("Default HALSim extension (project year unknown)");
        extension.requiredToRun = true;
        extension.status = input.extensionLibrary->isEmpty() ? CheckStatus::Missing : CheckStatus::Ok;
        extension.detail = input.extensionLibrary->isEmpty()
            ? tr("No matching extension is installed. Installed seasons: %1.")
                  .arg(input.installedExtensionSeasons.isEmpty() ? tr("none") : input.installedExtensionSeasons.join(QStringLiteral(", ")))
            : display(*input.extensionLibrary);
        if (extension.status == CheckStatus::Missing)
            extension.fixHint = tr("Reinstall an emulator package with this season, or explicitly set WPIEMU_EXTENSION.");
        report.checks.push_back(std::move(extension));
    }
    report.checks.push_back(checkJdk(input));
    report.checks.push_back(checkGradleWrapper(input));
    report.checks.push_back(checkWpilibInstall(input));
    report.checks.push_back(checkCppToolchain(input));
    report.checks.push_back(checkGameModule(input, report));
    if (const ToolchainCheck version = checkWpilibVersion(input); version.status == CheckStatus::Warning)
        report.checks.push_back(version);   // a version that is fine (or unknown) adds no row
    return report;
}

}  // namespace wpiemu::app
