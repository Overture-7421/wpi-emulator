#include <doctest/doctest.h>

#include <QDir>
#include <QFile>

#include "common/project_inspector.h"
#include "test_support.h"

using namespace wpiemu::app;
using namespace wpiemu::app::testing;

TEST_CASE("inspector reads season, language, team and vendor libraries") {
    TestProject java(QStringLiteral("java"), 2026);
    const ProjectInspection in = GradleProjectInspector().inspect(java.path());
    CHECK(in.usable());
    CHECK(in.wpilibYear == 2026);
    CHECK(in.gradleRioVersion == QStringLiteral("2026.2.1"));
    CHECK(in.language == QStringLiteral("java"));
    CHECK(in.teamNumber == 1234);
    CHECK(in.hasGradleWrapper);
    REQUIRE(in.vendorDeps.size() == 1);
    CHECK(in.vendorDeps[0].version == QStringLiteral("26.1.0"));
    CHECK(in.hasVendorDep(QStringLiteral("phoenix")));
    CHECK(!in.hasErrors());

    TestProject cpp(QStringLiteral("cpp"), 2025);
    const ProjectInspection c = GradleProjectInspector().inspect(cpp.path());
    CHECK(c.language == QStringLiteral("cpp"));
    CHECK(c.wpilibYear == 2025);
}

TEST_CASE("inspector explains why a folder is not a WPILib project") {
    const GradleProjectInspector inspector;

    const ProjectInspection missing = inspector.inspect(QStringLiteral("/definitely/not/here"));
    CHECK(!missing.usable());
    CHECK(missing.hasErrors());

    QTemporaryDir empty;
    const ProjectInspection noBuild = inspector.inspect(empty.path());
    CHECK(!noBuild.usable());
    REQUIRE(!noBuild.notes.empty());
    CHECK(noBuild.notes.front().text.contains(QStringLiteral("build.gradle")));

    TestProject plain(QStringLiteral("java"), 2026, /*gradleRio=*/false);
    const ProjectInspection notRio = inspector.inspect(plain.path());
    CHECK(!notRio.usable());
    CHECK(notRio.notes.front().text.contains(QStringLiteral("GradleRIO")));

    CHECK(inspector.inspect(QString()).notes.empty());   // blank input is not an error yet
}

TEST_CASE("inspector warns about a missing wrapper and an unreadable season") {
    TestProject p;
    QFile::remove(p.path() + "/gradlew");
    QFile::remove(p.path() + "/gradlew.bat");
    writeFile(p.path() + "/build.gradle", "plugins { id \"edu.wpi.first.GradleRIO\" }\n");
    QFile::remove(p.path() + "/.wpilib/wpilib_preferences.json");
    const ProjectInspection in = GradleProjectInspector().inspect(p.path());
    CHECK(in.usable());
    CHECK(in.wpilibYear == 0);
    CHECK(!in.hasGradleWrapper);
    CHECK(in.notes.size() >= 2);
    CHECK(!in.hasErrors());
}

TEST_CASE("kotlin build scripts and sub-folders resolve to the project root") {
    TestProject p;
    QFile::remove(p.path() + "/build.gradle");
    writeFile(p.path() + "/build.gradle.kts", "plugins { id(\"org.wpilib.GradleRIO\") version \"2027.0.0-alpha-7\" }\n");
    const ProjectInspection in = GradleProjectInspector().inspect(p.path());
    CHECK(in.usable());
    CHECK(in.wpilibYear == 2027);

    const QString nested = p.path() + "/src/main/java/frc/robot";
    QDir().mkpath(nested);
    CHECK(QDir::cleanPath(GradleProjectInspector::resolveProjectRoot(nested)) == p.path());
    CHECK(QDir::cleanPath(GradleProjectInspector::resolveProjectRoot(p.path() + "/build.gradle.kts")) == p.path());
}
