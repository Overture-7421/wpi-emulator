#include <doctest/doctest.h>

#include <QDir>
#include <QTemporaryDir>

#include "common/app_settings.h"
#include "common/key_value_store.h"
#include "common/project_prefs.h"
#include "common/recent_projects.h"
#include "common/unit_system.h"

using namespace wpiemu::app;
namespace sdk = wpiemu::sdk;

TEST_CASE("unit formatter converts between SI and display units") {
    const UnitFormatter imperial(UnitSystem::Imperial);
    CHECK(imperial.lengthToDisplay(0.0254) == doctest::Approx(1.0));
    CHECK(imperial.lengthFromDisplay(27.0) == doctest::Approx(0.6858));
    CHECK(imperial.massToDisplay(0.45359237) == doctest::Approx(1.0));
    CHECK(imperial.formatLength(0.6858).toStdString() == "27.0 in");
    CHECK(imperial.formatMass(54.0, 0).toStdString() == "119 lb");

    const UnitFormatter metric(UnitSystem::Metric);
    CHECK(metric.lengthToDisplay(0.5) == doctest::Approx(50.0));
    CHECK(metric.lengthFromDisplay(50.0) == doctest::Approx(0.5));
    CHECK(metric.massToDisplay(10.0) == doctest::Approx(10.0));
    CHECK(metric.formatLength(0.5).toStdString() == "50.0 cm");

    CHECK(unitSystemFromKey(unitSystemKey(UnitSystem::Metric)) == UnitSystem::Metric);
    CHECK(unitSystemFromKey(QStringLiteral("nonsense"), UnitSystem::Imperial) == UnitSystem::Imperial);
}

TEST_CASE("settings persist, sanitise and notify") {
    MemoryKeyValueStore store;
    {
        AppSettings settings(store);
        CHECK(settings.data().theme == ThemeMode::System);
        CHECK(settings.data().physicsRateHz == kDefaultPhysicsRateHz);
        CHECK(settings.stepUs() == 5000);

        int notifications = 0;
        QObject::connect(&settings, &AppSettings::changed, [&](const AppSettingsData&) { ++notifications; });

        AppSettingsData next = settings.data();
        next.theme = ThemeMode::Dark;
        next.units = UnitSystem::Metric;
        next.physicsRateHz = 1234;   // not an offered rate -> falls back to the default
        next.wpilibHome = QStringLiteral("/opt/wpilib");
        settings.update(next);
        CHECK(notifications == 1);
        CHECK(settings.data().physicsRateHz == kDefaultPhysicsRateHz);

        settings.update(settings.data());   // no change: no signal
        CHECK(notifications == 1);

        next.physicsRateHz = 1000;
        settings.update(next);
        CHECK(settings.stepUs() == 1000);
    }
    AppSettings reloaded(store);
    CHECK(reloaded.data().theme == ThemeMode::Dark);
    CHECK(reloaded.data().units == UnitSystem::Metric);
    CHECK(reloaded.data().physicsRateHz == 1000);
    CHECK(reloaded.data().wpilibHome == QStringLiteral("/opt/wpilib"));
}

TEST_CASE("recent projects are most-recent-first, unique and capped") {
    MemoryKeyValueStore store;
    QTemporaryDir root;
    RecentProjects recent(store);
    int changes = 0;
    QObject::connect(&recent, &RecentProjects::changed, [&] { ++changes; });

    const auto make = [&](int i) {
        const QString p = root.filePath(QStringLiteral("p%1").arg(i));
        QDir().mkpath(p);
        return p;
    };
    for (int i = 0; i < RecentProjects::kMaxEntries + 3; ++i) recent.touch(make(i));
    CHECK(recent.paths().size() == RecentProjects::kMaxEntries);
    CHECK(recent.lastProject() == QDir::cleanPath(make(RecentProjects::kMaxEntries + 2)));

    recent.touch(make(5));   // re-touch moves to the front without duplicating
    CHECK(recent.paths().front() == QDir::cleanPath(make(5)));
    CHECK(recent.paths().count(QDir::cleanPath(make(5))) == 1);

    // persisted
    RecentProjects reloaded(store);
    CHECK(reloaded.paths() == recent.paths());

    // missing folders are pruned
    QDir(make(5)).removeRecursively();
    CHECK(recent.pruneMissing() == 1);
    CHECK(!recent.paths().contains(QDir::cleanPath(make(5))));

    recent.clear();
    CHECK(recent.paths().isEmpty());
    CHECK(changes > 0);
}

TEST_CASE("project prefs are stored per project") {
    MemoryKeyValueStore store;
    ProjectPrefsStore prefs(store);
    QTemporaryDir a, b;
    CHECK(prefs.load(a.path()) == ProjectPrefs{});
    prefs.save(a.path(), ProjectPrefs{sdk::Alliance::Blue, 3});
    CHECK(prefs.load(a.path()).alliance == sdk::Alliance::Blue);
    CHECK(prefs.load(a.path()).station == 3);
    CHECK(prefs.load(b.path()) == ProjectPrefs{});
    prefs.save(a.path(), ProjectPrefs{sdk::Alliance::Red, 99});   // out-of-range station is clamped
    CHECK(prefs.load(a.path()).station == 3);
}
