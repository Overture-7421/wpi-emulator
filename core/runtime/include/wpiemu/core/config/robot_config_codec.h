#pragma once

#include <memory>
#include <vector>

#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/config/plugin_config.h"

namespace wpiemu::core::config {

// What fromValue() learned besides the config itself.
struct DecodeInfo {
    int persistedVersion = 0;  // schemaVersion found in the stored value (1 when the key is absent)
    bool migrated = false;     // the core migration chain ran
};

// RobotConfig <-> sdk::Value (the robot.json layout, docs/runtime.md "robot.json").
//
//  * fromValue: core migrations first (each IConfigMigrator upgrades the raw Value by exactly one version;
//    a newer file is refused with VersionMismatch), then a lenient decode: absent fields keep their
//    defaults, wrongly typed fields are a ParseError naming the path. Plugin sections are kept verbatim.
//  * Unknown fields (root, project, chassis, drivetrain, battery, sim, each mechanism, each wheel, each
//    plugin section) are preserved in RobotConfig::unknownFields and written back by toValue.
//  * toValue writes BOTH the legacy `parent` and the structured `attachment` of every mechanism (SDK 1.1);
//    fromValue prefers the structured form.
//  * Floats are written as the shortest decimal that round-trips (0.7 stays 0.7).
class RobotConfigCodec final : public sdk::IRobotConfigCodec {
public:
    explicit RobotConfigCodec(std::vector<std::unique_ptr<sdk::IConfigMigrator>> coreMigrators = {});

    sdk::Result<sdk::RobotConfig> fromValue(const sdk::Value& value) const override;
    sdk::Result<sdk::RobotConfig> fromValue(const sdk::Value& value, DecodeInfo* info) const;
    sdk::Value toValue(const sdk::RobotConfig& config) const override;

    // schemaVersion of a stored value; 1 when absent, 0 when `value` is not an object.
    static int storedVersion(const sdk::Value& value);

private:
    std::vector<std::unique_ptr<sdk::IConfigMigrator>> migrators_;
};

}  // namespace wpiemu::core::config
