#pragma once

#include <QString>

#include "common/key_value_store.h"
#include "common/robot_basics.h"

namespace wpiemu::app {

// Per-project choices that are not part of the robot model: where the robot starts / which driver
// station the emulated DS reports.
struct ProjectPrefs {
    sdk::Alliance alliance = sdk::Alliance::Red;
    int station = 2;   // 1..3

    bool operator==(const ProjectPrefs&) const = default;
};

class ProjectPrefsStore {
public:
    explicit ProjectPrefsStore(IKeyValueStore& store) : store_(store) {}

    ProjectPrefs load(const QString& projectPath) const;
    void save(const QString& projectPath, const ProjectPrefs& prefs);

private:
    static QString keyFor(const QString& projectPath, const char* field);

    IKeyValueStore& store_;
};

}  // namespace wpiemu::app
