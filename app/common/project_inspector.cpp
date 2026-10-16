#include "common/project_inspector.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

namespace wpiemu::app {

namespace {

QString readText(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return {};
    return QString::fromUtf8(f.readAll());
}

QJsonObject readJsonObject(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return {};
    return QJsonDocument::fromJson(f.readAll()).object();
}

QString findBuildFile(const QDir& dir) {
    for (const char* name : {"build.gradle", "build.gradle.kts"})
        if (dir.exists(QLatin1String(name))) return dir.filePath(QLatin1String(name));
    return {};
}

}  // namespace

int GradleProjectInspector::yearFromGradleRioVersion(const QString& version) {
    static const QRegularExpression re(QStringLiteral("^(\\d{4})\\."));
    const auto m = re.match(version.trimmed());
    return m.hasMatch() ? m.captured(1).toInt() : 0;
}

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

    const QDir dir(info.absoluteFilePath());
    const QString buildFile = findBuildFile(dir);
    if (buildFile.isEmpty()) {
        out.notes.push_back({sdk::Severity::Error,
                             tr("No build.gradle found here. Pick the folder that contains your robot project's build.gradle.")});
        return out;
    }

    const QString build = readText(buildFile);
    if (!build.contains(QLatin1String("GradleRIO"))) {
        out.notes.push_back({sdk::Severity::Error, tr("build.gradle does not apply the GradleRIO plugin, so this is not a WPILib robot project.")});
        return out;
    }
    out.isWpilibProject = true;

    static const QRegularExpression versionRe(
        QStringLiteral("edu\\.wpi\\.first\\.GradleRIO[\"']\\)?\\s*version\\s*\\(?\\s*[\"']([^\"']+)[\"']"));
    if (const auto m = versionRe.match(build); m.hasMatch()) {
        out.gradleRioVersion = m.captured(1);
        out.wpilibYear = yearFromGradleRioVersion(out.gradleRioVersion);
    }

    const QJsonObject prefs = readJsonObject(dir.filePath(QStringLiteral(".wpilib/wpilib_preferences.json")));
    out.teamNumber = prefs.value(QStringLiteral("teamNumber")).toInt(0);
    const QString prefLang = prefs.value(QStringLiteral("currentLanguage")).toString().toLower();
    if (prefLang == QLatin1String("java") || prefLang == QLatin1String("cpp")) out.language = prefLang;
    if (out.wpilibYear == 0) {
        const QJsonValue year = prefs.value(QStringLiteral("projectYear"));
        out.wpilibYear = year.isString() ? year.toString().toInt() : year.toInt(0);
    }

    if (out.language.isEmpty()) {
        const bool java = dir.exists(QStringLiteral("src/main/java"));
        const bool cpp = dir.exists(QStringLiteral("src/main/cpp"));
        if (java != cpp) out.language = java ? QStringLiteral("java") : QStringLiteral("cpp");
        else if (build.contains(QRegularExpression(QStringLiteral("id\\s*\\(?\\s*[\"']cpp[\"']")))) out.language = QStringLiteral("cpp");
        else if (build.contains(QRegularExpression(QStringLiteral("id\\s*\\(?\\s*[\"']java[\"']")))) out.language = QStringLiteral("java");
    }

    const QDir vendordeps(dir.filePath(QStringLiteral("vendordeps")));
    for (const QFileInfo& f : vendordeps.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name)) {
        const QJsonObject o = readJsonObject(f.absoluteFilePath());
        VendorDep dep;
        dep.name = o.value(QStringLiteral("name")).toString(f.completeBaseName());
        dep.version = o.value(QStringLiteral("version")).toString();
        out.vendorDeps.push_back(dep);
    }

    out.hasGradleWrapper = dir.exists(QStringLiteral("gradlew")) || dir.exists(QStringLiteral("gradlew.bat"));

    if (out.wpilibYear == 0)
        out.notes.push_back({sdk::Severity::Warning, tr("Could not read the GradleRIO version, so the WPILib season is unknown.")});
    if (out.language.isEmpty())
        out.notes.push_back({sdk::Severity::Warning, tr("Could not tell whether this is a Java or C++ project.")});
    if (!out.hasGradleWrapper)
        out.notes.push_back({sdk::Severity::Warning, tr("No Gradle wrapper (gradlew) found; the robot cannot be launched without it.")});
    return out;
}

}  // namespace wpiemu::app
