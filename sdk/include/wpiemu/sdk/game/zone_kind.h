#pragma once

// What a game plugin publishes about the zone kinds it owns (ZoneConfig::kind). The editor builds the
// "add zone" menu, the inspector (from `properties`) and the 3D gizmo from this; core validates
// ZoneConfig::properties against it. Returned by IGameModule::contributeZoneKinds().

#include <string>

#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/core/math.h"

namespace wpiemu::sdk {

enum class ZoneShape {
    Volume,  // a box: ZoneConfig::sizeM is meaningful and must be > 0
    Muzzle,  // a point + direction (+X of ZoneConfig::local): sizeM is ignored
};

struct ZoneKindInfo {
    std::string kind;          // full kind, prefixed with the plugin id: "game_2026.pickup"
    std::string displayName;
    std::string description;
    ZoneShape shape = ZoneShape::Volume;
    PropertySchema properties;                // kind-specific ZoneConfig::properties
    Vec3 defaultSizeM{0.3f, 0.3f, 0.3f};      // used by the editor for a new Volume zone
    Pose3 defaultLocal;                       // used by the editor for a new zone
    std::string color = "#4488ff";            // gizmo color
};

}  // namespace wpiemu::sdk
