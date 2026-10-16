#pragma once

#include <QObject>
#include <QStringList>

#include "common/key_value_store.h"

namespace wpiemu::app {

// Most-recently-used project folders plus the project that is reopened on the next start.
class RecentProjects : public QObject {
    Q_OBJECT
public:
    static constexpr int kMaxEntries = 10;

    explicit RecentProjects(IKeyValueStore& store, QObject* parent = nullptr);

    QStringList paths() const { return paths_; }
    QString lastProject() const { return paths_.isEmpty() ? QString() : paths_.front(); }

    // Moves `path` to the front (normalised, de-duplicated, capped).
    void touch(const QString& path);
    void remove(const QString& path);
    void clear();
    // Drops entries whose folder no longer exists; returns how many were removed.
    int pruneMissing();

signals:
    void changed();

private:
    void save();

    IKeyValueStore& store_;
    QStringList paths_;
};

}  // namespace wpiemu::app
