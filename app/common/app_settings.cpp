#include "common/app_settings.h"

namespace wpiemu::app {

namespace {
constexpr auto kTheme = "settings/theme";
constexpr auto kUnits = "settings/units";
constexpr auto kRate = "settings/physicsRateHz";
constexpr auto kThreads = "settings/physicsThreads";
constexpr auto kWpilib = "settings/wpilibHome";
constexpr auto kJdk = "settings/jdkHome";
constexpr auto kReopen = "settings/reopenLastProject";
constexpr auto kLanguage = "settings/language";
}  // namespace

QString themeModeKey(ThemeMode mode) {
    switch (mode) {
        case ThemeMode::Light: return QStringLiteral("light");
        case ThemeMode::Dark: return QStringLiteral("dark");
        case ThemeMode::System: break;
    }
    return QStringLiteral("system");
}

ThemeMode themeModeFromKey(const QString& key, ThemeMode fallback) {
    if (key == QLatin1String("light")) return ThemeMode::Light;
    if (key == QLatin1String("dark")) return ThemeMode::Dark;
    if (key == QLatin1String("system")) return ThemeMode::System;
    return fallback;
}

int sanitizePhysicsRateHz(int hz) {
    for (const int allowed : kPhysicsRatesHz)
        if (allowed == hz) return hz;
    return kDefaultPhysicsRateHz;
}

AppSettings::AppSettings(IKeyValueStore& store, QObject* parent) : QObject(parent), store_(store) { load(); }

int sanitizePhysicsThreads(int threads) {
    for (const int allowed : kPhysicsThreadChoices)
        if (allowed == threads) return threads;
    return kDefaultPhysicsThreads;
}

void AppSettings::load() {
    data_.theme = themeModeFromKey(store_.value(kTheme).toString());
    data_.units = unitSystemFromKey(store_.value(kUnits).toString());
    data_.physicsRateHz = sanitizePhysicsRateHz(store_.value(kRate, kDefaultPhysicsRateHz).toInt());
    data_.physicsThreads = sanitizePhysicsThreads(store_.value(kThreads, kDefaultPhysicsThreads).toInt());
    data_.wpilibHome = store_.value(kWpilib).toString();
    data_.jdkHome = store_.value(kJdk).toString();
    data_.reopenLastProject = store_.value(kReopen, true).toBool();
    data_.language = store_.value(kLanguage).toString();
}

void AppSettings::save() const {
    store_.setValue(kTheme, themeModeKey(data_.theme));
    store_.setValue(kUnits, unitSystemKey(data_.units));
    store_.setValue(kRate, data_.physicsRateHz);
    store_.setValue(kThreads, data_.physicsThreads);
    store_.setValue(kWpilib, data_.wpilibHome);
    store_.setValue(kJdk, data_.jdkHome);
    store_.setValue(kReopen, data_.reopenLastProject);
    store_.setValue(kLanguage, data_.language);
    store_.sync();
}

void AppSettings::update(const AppSettingsData& next) {
    AppSettingsData clean = next;
    clean.physicsRateHz = sanitizePhysicsRateHz(clean.physicsRateHz);
    clean.physicsThreads = sanitizePhysicsThreads(clean.physicsThreads);
    if (clean == data_) return;
    data_ = clean;
    save();
    emit changed(data_);
}

}  // namespace wpiemu::app
