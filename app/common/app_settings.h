#pragma once

#include <QObject>
#include <QString>
#include <QStringList>

#include "common/key_value_store.h"
#include "common/unit_system.h"

namespace wpiemu::app {

enum class ThemeMode { System, Light, Dark };

QString themeModeKey(ThemeMode mode);
ThemeMode themeModeFromKey(const QString& key, ThemeMode fallback = ThemeMode::System);

// Physics step rates offered in the UI; the step is 1e6 / rate microseconds (always an integer).
inline constexpr int kPhysicsRatesHz[] = {100, 200, 250, 500, 1000};
inline constexpr int kDefaultPhysicsRateHz = 200;
int sanitizePhysicsRateHz(int hz);

struct AppSettingsData {
    ThemeMode theme = ThemeMode::System;
    UnitSystem units = UnitSystem::Imperial;
    int physicsRateHz = kDefaultPhysicsRateHz;
    QString wpilibHome;   // override of the WPILib install dir (empty = auto-detect)
    QString jdkHome;      // override of the JDK used for gradle (empty = auto-detect)
    bool reopenLastProject = true;
    QString language;     // UI locale override, empty = system

    bool operator==(const AppSettingsData&) const = default;
};

// Observable application preferences (view-model for the settings dialog and everything that
// reacts to them). Persists through an injected store.
class AppSettings : public QObject {
    Q_OBJECT
public:
    explicit AppSettings(IKeyValueStore& store, QObject* parent = nullptr);

    const AppSettingsData& data() const { return data_; }
    void update(const AppSettingsData& next);

    std::int64_t stepUs() const { return 1000000 / data_.physicsRateHz; }

signals:
    void changed(const wpiemu::app::AppSettingsData& data);

private:
    void load();
    void save() const;

    IKeyValueStore& store_;
    AppSettingsData data_;
};

}  // namespace wpiemu::app
