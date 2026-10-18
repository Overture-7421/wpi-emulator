#pragma once

#include "wpiemu/sdk/config/config_io.h"
#include "wpiemu/sdk/config/plugin_config.h"

namespace wpiemu::core::config {

// RobotConfig <-> sdk::Value (the robot.json layout, docs/runtime.md "robot.json").
//
//  * fromValue: a lenient decode of the current schema: absent fields keep their defaults, unknown fields are
//    ignored, wrongly typed fields are a ParseError naming the path. A `schemaVersion` other than
//    kRobotConfigSchemaVersion is refused with VersionMismatch (no migration, ADR 0013). Plugin sections are
//    kept verbatim.
//  * toValue writes the structured `attachment` of every mechanism.
//  * Floats are written as the shortest decimal that round-trips (0.7 stays 0.7).
class RobotConfigCodec final : public sdk::IRobotConfigCodec {
public:
    sdk::Result<sdk::RobotConfig> fromValue(const sdk::Value& value) const override;
    sdk::Value toValue(const sdk::RobotConfig& config) const override;

};

}  // namespace wpiemu::core::config
