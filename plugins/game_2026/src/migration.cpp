#include "wpiemu/plugins/game_2026/migration.h"

#include <string>
#include <utility>

#include "wpiemu/plugins/game_2026/fuel_ports.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"

namespace wpiemu::plugins::game_2026 {

namespace {

// Moves `from` to `to` unless `to` already exists (then the stale `from` is just dropped).
void renameKey(sdk::Value& object, const char* from, const char* to) {
    if (!object.isObject()) return;
    sdk::Value* old = object.object().find(from);
    if (old == nullptr) return;
    if (object.object().find(to) == nullptr) {
        const sdk::Value moved = *old;
        object.object().erase(from);
        object.object().set(to, moved);
    } else {
        object.object().erase(from);
    }
}

struct Rename {
    const char* from;
    const char* to;
};

constexpr Rename kSectionRenames[] = {
    {"fuelCapacity", "capacity"},       {"preload", "preloadFuel"},
    {"neutralZoneFuel", "neutralFuel"}, {"depotCount", "depotFuel"},
    {"hubShifts", "enforceHubShifts"},  {"firstInactive", "firstInactiveHub"},
};

constexpr Rename kIntakePropertyRenames[] = {
    {"zone", "pickupZone"},
    {"intakeRoller", "rollerMechanism"},
    {"intakeRate", "intakeRateHz"},
};

constexpr Rename kShooterPropertyRenames[] = {
    {"exitEfficiency", "efficiency"},         {"feedMechanismId", "feedMechanism"},
    {"flywheelMechanismId", "flywheelMechanism"}, {"hoodMechanismId", "hoodMechanism"},
    {"turretMechanismId", "turretMechanism"}, {"fireRate", "fireRateHz"},
};

}  // namespace

sdk::Status ConfigV1ToV2Migrator::migrate(sdk::Value& data) const {
    if (!data.isObject())
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, "game_2026 config section must be an object");
    for (const Rename& r : kSectionRenames) renameKey(data, r.from, r.to);
    return sdk::okStatus();
}

std::vector<std::unique_ptr<sdk::IConfigMigrator>> makeConfigMigrators() {
    std::vector<std::unique_ptr<sdk::IConfigMigrator>> m;
    m.push_back(std::make_unique<ConfigV1ToV2Migrator>());
    return m;
}

sdk::Status migrateGame2026Config(sdk::RobotConfig& config, int persistedVersion) {
    if (persistedVersion >= 2) return sdk::okStatus();
    const std::string id(pluginId());
    for (sdk::MechanismConfig& m : config.mechanisms) {
        if (!sdk::ownsMechanismType(id, m.type)) continue;  // never touch core or other plugins' mechanisms
        if (m.type == "game_2026.intake") m.type = kIntakeType;
        else if (m.type == "game_2026.shooter") m.type = kShooterType;
        if (!m.properties.isObject()) continue;
        if (m.type == kIntakeType)
            for (const Rename& r : kIntakePropertyRenames) renameKey(m.properties, r.from, r.to);
        else if (m.type == kShooterType)
            for (const Rename& r : kShooterPropertyRenames) renameKey(m.properties, r.from, r.to);
    }
    return sdk::okStatus();
}

}  // namespace wpiemu::plugins::game_2026
