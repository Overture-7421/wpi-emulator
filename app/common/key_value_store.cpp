#include "common/key_value_store.h"

#include <QSettings>

namespace wpiemu::app {

struct QSettingsKeyValueStore::Impl {
    QSettings settings;
    Impl() = default;
    explicit Impl(const QString& path) : settings(path, QSettings::IniFormat) {}
};

QSettingsKeyValueStore::QSettingsKeyValueStore() : impl_(std::make_unique<Impl>()) {}
QSettingsKeyValueStore::QSettingsKeyValueStore(const QString& iniPath) : impl_(std::make_unique<Impl>(iniPath)) {}
QSettingsKeyValueStore::~QSettingsKeyValueStore() = default;

QVariant QSettingsKeyValueStore::value(const QString& key, const QVariant& fallback) const {
    return impl_->settings.value(key, fallback);
}

void QSettingsKeyValueStore::setValue(const QString& key, const QVariant& value) {
    impl_->settings.setValue(key, value);
}

void QSettingsKeyValueStore::remove(const QString& key) { impl_->settings.remove(key); }

void QSettingsKeyValueStore::sync() { impl_->settings.sync(); }

}  // namespace wpiemu::app
