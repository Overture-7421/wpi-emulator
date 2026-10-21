#pragma once

// The start position a host places the robot at on reset: an id into RobotConfig::startPositions, empty = the robot
// setup's default start. Every IRuntimeHost keeps it with these rules (see IRuntimeHost::startPosition()).

#include <QString>

#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app {

class StartPositionSelection {
public:
    const QString& id() const { return id_; }

    // The selected entry of `config`; null without a selection or when `config` has no such entry.
    const sdk::StartPosition* find(const sdk::RobotConfig* config) const {
        return config != nullptr && !id_.isEmpty() ? config->findStartPosition(id_.toStdString()) : nullptr;
    }

    // Selects `id` only when `config` has that entry, anything else selects nothing. Returns whether the selection changed.
    bool select(const sdk::RobotConfig* config, const QString& id) {
        const bool known = config != nullptr && config->findStartPosition(id.toStdString()) != nullptr;
        const QString next = known ? id : QString();
        if (next == id_) return false;
        id_ = next;
        return true;
    }

    // The selection belongs to its project: dropped on a project switch or when `config` lost the entry.
    // Returns whether the selection changed.
    bool reconcile(const sdk::RobotConfig* config, bool sameProject) {
        if (id_.isEmpty() || (sameProject && find(config) != nullptr)) return false;
        id_.clear();
        return true;
    }

private:
    QString id_;
};

}  // namespace wpiemu::app
