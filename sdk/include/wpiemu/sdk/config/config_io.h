#pragma once

// Interfaces for turning RobotConfig into/from storage. SDK has no JSON dependency:
//   RobotConfig <-IRobotConfigCodec-> Value <-IValueTextCodec-> text (JSON)
// Implementations live in core (codec, validator) and app/core (stores). Plugins never touch these.

#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/registry.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/core/value.h"

namespace wpiemu::sdk {

class IValueTextCodec {
    WPIEMU_INTERFACE(IValueTextCodec)
public:
    virtual Result<Value> parse(std::string_view text) const = 0;     // ParseError on malformed text
    virtual std::string dump(const Value& value, bool pretty) const = 0;
};

class IRobotConfigCodec {
    WPIEMU_INTERFACE(IRobotConfigCodec)
public:
    // Applies core schema migrations (older versions upgrade; newer => VersionMismatch). Plugin
    // sections are kept verbatim here; the coordinator then runs the selected plugin's migration
    // (IGameModule::configMigrators, then IGameModule::migrateRobotConfig). Order: core migration ->
    // plugin migration -> defaults -> validation -> construction. Writers emit BOTH
    // MechanismConfig::parent and the structured attachment; readers prefer the structured form.
    virtual Result<RobotConfig> fromValue(const Value& value) const = 0;
    virtual Value toValue(const RobotConfig& config) const = 0;
};

// Persistence targets: app-data copy and <project>/.wpiemu/robot.json (same payload, two stores).
class IRobotConfigStore {
    WPIEMU_INTERFACE(IRobotConfigStore)
public:
    virtual std::string describe() const = 0;
    virtual Result<RobotConfig> load() = 0;       // NotFound when absent
    virtual Status save(const RobotConfig& config) = 0;
};

class MechanismFactory;
class GamePluginRegistry;
class DrivetrainRegistry;

// Whole-model validation: unknown mechanism types, missing/duplicate ids, property schema
// violations, slot mappings to incompatible devices, dangling parents, plugin sections
// older/newer than the plugin supports, etc. Implemented in core.
//
// SDK 1.1 contract (every Issue below carries a stable `code`, see config/validation_codes.h):
//  * device aliases: append validateDeviceAliases(config.allMappings()) (devices/device_identity.h);
//    independent mappings of aliases of one physical device are an Error (device_alias_conflict);
//  * mechanism attachment: append validateMechanismAttachments(config, mechanisms)
//    (mechanisms/attachment.h): missing parent / missing link / cycle / reserved id are Errors.
// Validation runs after migration and defaults (see IGameModule::migrateRobotConfig for the pipeline).
class IConfigValidator {
    WPIEMU_INTERFACE(IConfigValidator)
public:
    virtual std::vector<Issue> validate(const RobotConfig& config, const MechanismFactory& mechanisms,
                                        const DrivetrainRegistry& drivetrains,
                                        const GamePluginRegistry& plugins) const = 0;
};

}  // namespace wpiemu::sdk
