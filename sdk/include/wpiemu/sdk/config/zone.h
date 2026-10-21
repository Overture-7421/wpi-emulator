#pragma once

// Zones: game-owned places on the robot (a pickup volume, a muzzle, ...) persisted in RobotConfig::zones.
//
// A zone is attached to the chassis or to a mechanism link, exactly like a mechanism mount:
//   pose_in_chassis(zone) = pose_in_chassis(attachment parent link) * zone.local
// so it moves with that mechanism's link. Core never interprets a zone: `kind` and `properties` belong to
// the game plugin that owns the kind (a plugin owns the kinds starting with its id + '.', e.g.
// "game_2026.pickup"), which publishes them as ZoneKindInfo (game/zone_kind.h). Core validates the
// structure and checks `properties` against the kind's PropertySchema.
//
// A zone is only live while its `activation` holds: a list of conditions on values published by
// mechanisms (their MechanismState::values, e.g. "flywheel velocity > 300 rad/s"), combined with AND/OR.
// Evaluate it with evaluateActivation (game/zone_activation.h). An empty condition list is always active.
//
// Stored in SI units. There is no migration (docs/CONTRIBUTING.md); see docs/reference/game-plugin.md.

#include <string>
#include <vector>

#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/mechanisms/mechanism_state.h"

namespace wpiemu::sdk {

enum class ConditionOp {    Above,   // v > threshold (|v| when `absolute`)
    Below,   // v < threshold (|v| when `absolute`)
    Within,  // |v - threshold| <= tolerance
};

// One test on a value a mechanism publishes. The value is read in SI (see readMechanismValue).
struct ZoneCondition {
    std::string mechanismId;             // MechanismConfig::id of the observed mechanism
    std::string value = value_name::kVelocity;   // published value name (NamedValue::name) of that mechanism
    ConditionOp op = ConditionOp::Above;
    double threshold = 0.0;              // SI (m, rad, m/s, rad/s, ...)
    double tolerance = 0.1;              // Within: |v - threshold| <= tolerance (SI)
    bool absolute = true;                // Above/Below compare |v| instead of v

    bool operator==(const ZoneCondition&) const = default;
};

enum class ActivationMode {
    All,  // every condition holds (AND)
    Any,  // at least one condition holds (OR)
};

struct ZoneActivation {
    ActivationMode mode = ActivationMode::All;
    std::vector<ZoneCondition> conditions;   // empty = always active

    bool operator==(const ZoneActivation&) const = default;
};

struct ZoneConfig {
    std::string id;                          // unique among zones (and distinct from mechanism ids)
    std::string kind;                        // plugin-owned, e.g. "game_2026.pickup"
    std::string label;
    MechanismAttachment attachment;          // chassis (default) or mechanism + link; the zone moves with it
    Pose3 local;                             // relative to the attachment link frame
    Vec3 sizeM{0.3f, 0.3f, 0.3f};            // full extents; Volume kinds only
    Value properties = Value::makeObject();  // validated against the kind's PropertySchema
    ZoneActivation activation;

    bool operator==(const ZoneConfig&) const = default;
};

}  // namespace wpiemu::sdk
