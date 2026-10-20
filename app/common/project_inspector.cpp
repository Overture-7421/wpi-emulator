#include "common/project_inspector.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <filesystem>

#include "wpiemu/core/launcher/project_inspector.h"

namespace wpiemu::app {

namespace {

QString findBuildFile(const QDir& dir) {
    for (const char* name : {"build.gradle", "build.gradle.kts"})
        if (dir.exists(QLatin1String(name))) return dir.filePath(QLatin1String(name));
    return {};
}

QString fromCore(const std::string& text) { return QString::fromStdString(text); }

}  // namespace

QString GradleProjectInspector::resolveProjectRoot(const QString& path) {
    QFileInfo info(path);
    QDir dir = info.isDir() ? QDir(info.absoluteFilePath()) : info.absoluteDir();
    const QString start = dir.absolutePath();
    for (int depth = 0; depth < 8; ++depth) {
        if (!findBuildFile(dir).isEmpty()) return dir.absolutePath();
        if (!dir.cdUp()) break;
    }
    return start;
}

ProjectInspection GradleProjectInspector::inspect(const QString& rawPath) const {
    ProjectInspection out;
    const QFileInfo info(rawPath);
    out.path = QDir::toNativeSeparators(info.absoluteFilePath());
    out.projectName = info.fileName();
    out.exists = info.isDir();
    if (rawPath.trimmed().isEmpty()) return out;
    if (!out.exists) {
        out.notes.push_back({sdk::Severity::Error, tr("This folder does not exist.")});
        return out;
    }

    const auto inspected = core::launcher::ProjectInspector(json_).inspect(std::filesystem::path(info.absoluteFilePath().toStdU16String()));
    if (!inspected) {
        out.notes.push_back({sdk::Severity::Error, fromCore(inspected.error().message)});
        return out;
    }
    const core::launcher::ProjectReport& report = inspected.value();
    if (!report.hasBuildScript) {
        out.notes.push_back({sdk::Severity::Error,
                             tr("No build.gradle found here. Pick the folder that contains your robot project's build.gradle.")});
        return out;
    }
    if (!report.gradleRioFound) {
        out.notes.push_back({sdk::Severity::Error, tr("build.gradle does not apply the GradleRIO plugin, so this is not a WPILib robot project.")});
        return out;
    }
    out.isWpilibProject = true;
    out.gradleRioVersion = fromCore(report.gradleRioVersion);
    out.wpilibYear = report.wpilibYear;
    out.language = fromCore(report.language);
    out.teamNumber = std::max(report.teamNumber, 0);
    out.hasGradleWrapper = report.hasGradlewScript || report.hasGradlewBat;
    for (const core::launcher::VendorDep& dep : report.vendorDeps) out.vendorDeps.push_back({fromCore(dep.name), fromCore(dep.version)});

    if (out.wpilibYear == 0)
        out.notes.push_back({sdk::Severity::Warning, tr("Could not read the GradleRIO version, so the WPILib season is unknown.")});
    if (out.language.isEmpty())
        out.notes.push_back({sdk::Severity::Warning, tr("Could not tell whether this is a Java or C++ project.")});
    if (!out.hasGradleWrapper)
        out.notes.push_back({sdk::Severity::Warning, tr("No Gradle wrapper (gradlew) found; the robot cannot be launched without it.")});
    return out;
}

}  // namespace wpiemu::app
