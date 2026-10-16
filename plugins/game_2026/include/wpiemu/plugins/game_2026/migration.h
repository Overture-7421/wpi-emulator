#pragma once

// Versioned migration of the plugin's persisted data (see docs/game_2026.md "Config versions").
//   v1: pre-release draft layout (never documented as a stable format).
//   v2: current layout (config.h).

#include <memory>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::plugins::game_2026 {

// Section data v1 -> v2: renames the draft keys to the documented ones.
class ConfigV1ToV2Migrator final : public sdk::IConfigMigrator {
public:
    int fromVersion() const override { return 1; }
    int toVersion() const override { return 2; }
    sdk::Status migrate(sdk::Value& data) const override;
};

std::vector<std::unique_ptr<sdk::IConfigMigrator>> makeConfigMigrators();

// Whole-config hook for persisted sections older than v2: renames the draft mechanism types and
// properties of mechanisms this plugin owns. Idempotent; touches nothing else.
sdk::Status migrateGame2026Config(sdk::RobotConfig& config, int persistedVersion);

}  // namespace wpiemu::plugins::game_2026
