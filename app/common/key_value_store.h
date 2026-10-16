#pragma once

#include <QHash>
#include <QString>
#include <QVariant>

#include <memory>

namespace wpiemu::app {

// Minimal persistence port so models never touch QSettings directly (and tests stay in memory).
class IKeyValueStore {
public:
    virtual ~IKeyValueStore() = default;
    virtual QVariant value(const QString& key, const QVariant& fallback = {}) const = 0;
    virtual void setValue(const QString& key, const QVariant& value) = 0;
    virtual void remove(const QString& key) = 0;
    virtual void sync() = 0;
};

class MemoryKeyValueStore final : public IKeyValueStore {
public:
    QVariant value(const QString& key, const QVariant& fallback = {}) const override {
        return values_.value(key, fallback);
    }
    void setValue(const QString& key, const QVariant& value) override { values_.insert(key, value); }
    void remove(const QString& key) override { values_.remove(key); }
    void sync() override {}

private:
    QHash<QString, QVariant> values_;
};

// Native per-user settings (registry / plist / ini) under the application's organisation.
class QSettingsKeyValueStore final : public IKeyValueStore {
public:
    QSettingsKeyValueStore();
    ~QSettingsKeyValueStore() override;
    // Explicit ini file: used by tests and portable setups.
    explicit QSettingsKeyValueStore(const QString& iniPath);

    QVariant value(const QString& key, const QVariant& fallback = {}) const override;
    void setValue(const QString& key, const QVariant& value) override;
    void remove(const QString& key) override;
    void sync() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace wpiemu::app
