#pragma once

#include <QCoreApplication>
#include <QString>
#include <QStringList>

#include <vector>

#include "wpiemu/sdk/core/severity.h"

namespace wpiemu::app {

struct InspectionNote {
    sdk::Severity severity = sdk::Severity::Info;
    QString text;
};

struct VendorDep {
    QString name;
    QString version;
};

// What we learned by looking at a folder; everything is derived from files, nothing is executed.
struct ProjectInspection {
    QString path;                 // folder that was inspected (project root once resolved)
    QString projectName;
    bool exists = false;
    bool isWpilibProject = false; // build.gradle(.kts) applying GradleRIO
    bool hasGradleWrapper = false;
    QString language;             // "java" | "cpp" | "" (unknown)
    int wpilibYear = 0;           // 0 = unknown
    QString gradleRioVersion;
    int teamNumber = 0;           // 0 = unknown
    std::vector<VendorDep> vendorDeps;
    std::vector<InspectionNote> notes;

    // The project can be configured: a WPILib project was recognised.
    bool usable() const { return exists && isWpilibProject; }
    bool hasErrors() const {
        for (const auto& n : notes)
            if (n.severity == sdk::Severity::Error) return true;
        return false;
    }
    bool hasVendorDep(const QString& needle) const {
        for (const auto& d : vendorDeps)
            if (d.name.contains(needle, Qt::CaseInsensitive)) return true;
        return false;
    }
};

// Port: core/runtime may provide a richer implementation; the wizard only depends on this.
class IProjectInspector {
public:
    virtual ~IProjectInspector() = default;
    virtual ProjectInspection inspect(const QString& path) const = 0;
};

// Reads build.gradle(.kts), .wpilib/wpilib_preferences.json and vendordeps/*.json.
class GradleProjectInspector final : public IProjectInspector {
    Q_DECLARE_TR_FUNCTIONS(GradleProjectInspector)
public:
    ProjectInspection inspect(const QString& path) const override;

    // A dropped/typed path may be a file inside the project or a sub-folder: walk up a few levels
    // to the folder holding build.gradle. Returns the input folder when none is found.
    static QString resolveProjectRoot(const QString& path);

    // "2026.1.1-beta-2" -> 2026; 0 when malformed.
    static int yearFromGradleRioVersion(const QString& version);
};

}  // namespace wpiemu::app
