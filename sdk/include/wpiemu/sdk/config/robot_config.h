#pragma once

// The persisted robot model (robot.json) as plain data. SDK defines the shape; core implements
// (de)serialisation (IRobotConfigCodec) and validation (IConfigValidator); the editor edits it.
// Stored in SI units.

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/mechanisms/motion_link.h"
#include "wpiemu/sdk/physics/chassis_spec.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk {

// Version of the CORE schema (chassis/drivetrain/mechanisms/mappings). Bumped by ADR when a
// non-additive change happens; additive fields keep the version and default when absent.
inline constexpr int kRobotConfigSchemaVersion = 1;

struct ProjectInfo {
    std::string path;           // robot project root (contains build.gradle)
    std::string language;       // "java" | "cpp"
    int wpilibYear = 0;         // GradleRIO year, selects the game plugin
    std::string gradleRioVersion;
};

struct BatteryConfig {
    double nominalVoltageV = 12.6;
    double internalResistanceOhm = 0.015;
    double brownoutVoltageV = 6.8;
};

struct SimSettings {
    std::int64_t stepUs = 5000;     // 200 Hz
    std::uint64_t seed = 1;
    PhysicsConfig physics;
};

struct DrivetrainConfig {
    DrivetrainSpec spec;
    std::vector<SlotMapping> mappings;   // owner == slot_owner::kDrivetrain
};

// Reserved attachment names (SDK 1.1). "chassis" is the parent id meaning the robot chassis; "root" is
// the link every mechanism and the chassis expose (its own origin/mount frame). Neither may be used as
// a mechanism instance id / non-root link name.
inline constexpr const char* kChassisParent = "chassis";
inline constexpr const char* kRootLink = "root";

// Where a mechanism's mount frame hangs: a named link of another mechanism (or the chassis).
struct MechanismAttachment {
    std::string parentMechanismId = kChassisParent;  // "chassis" or "" = the chassis
    std::string parentLink = kRootLink;              // must exist on the parent; the chassis only has "root"

    bool onChassis() const { return parentMechanismId.empty() || parentMechanismId == kChassisParent; }
    bool operator==(const MechanismAttachment&) const = default;
};

// A mechanism instance placed on the robot.
struct MechanismConfig {
    std::string id;          // unique instance id (also the SlotAddress owner), e.g. "intake"
    std::string type;        // MechanismFactory key, e.g. "roller", "elevator", "game_2026.shooter"
    std::string label;
    // LEGACY (1.0): "chassis" or another mechanism id; implies parentLink "root". Kept source-compatible;
    // `attachment` below wins when present. Writers keep both in sync (attachTo()).
    std::string parent = kChassisParent;
    Pose3 mount;             // relative to the parent LINK frame (the chassis frame for the chassis)
    std::vector<PrimitiveSpec> geometry;   // box/cylinder primitives (visual and optional collision)
    Value properties = Value::makeObject();   // validated against the template's PropertySchema
    std::vector<SlotMapping> mappings;
    // SDK 1.1: structured attachment. Absent => derived from `parent` with link "root".
    std::optional<MechanismAttachment> attachment;

    MechanismAttachment resolvedAttachment() const {
        if (attachment) return *attachment;
        MechanismAttachment a;
        a.parentMechanismId = parent.empty() ? std::string(kChassisParent) : parent;
        return a;
    }
    // Sets the structured attachment and mirrors the id into the legacy `parent`.
    void attachTo(std::string parentMechanismId, std::string parentLink = kRootLink) {
        MechanismAttachment a;
        a.parentMechanismId = parentMechanismId.empty() ? std::string(kChassisParent) : std::move(parentMechanismId);
        a.parentLink = std::move(parentLink);
        parent = a.parentMechanismId;
        attachment = std::move(a);
    }
};

// One plugin's namespaced, versioned section.
struct PluginSection {
    int schemaVersion = 1;
    Value data = Value::makeObject();
};

struct RobotConfig {
    int schemaVersion = kRobotConfigSchemaVersion;
    std::string name = "Robot";
    ProjectInfo project;
    std::string gamePluginId;    // empty = auto-select by project.wpilibYear

    ChassisSpec chassis;
    DrivetrainConfig drivetrain;
    std::vector<MechanismConfig> mechanisms;
    std::vector<SlotMapping> robotMappings;   // owner == slot_owner::kRobot (e.g. "imu")

    BatteryConfig battery;
    SimSettings sim;

    // key = plugin id (e.g. "game_2026"). Opaque to core.
    std::map<std::string, PluginSection, std::less<>> plugins;

    // SDK 1.1 (additive). Fields the codec did not recognise (written by a newer/other tool), kept so a
    // load -> save round trip does not lose them. Object keyed by owner path ("" = root, "mechanisms/<id>",
    // "drivetrain", "drivetrain/wheels/<name>", ...), each value an object of unknown key -> Value.
    // Written by IRobotConfigCodec only; editors leave it alone.
    Value unknownFields = Value::makeObject();

    std::vector<MotionLink> motionLinks;
    // Explicit independent configurations opt out of the legacy tank upgrade, too.
    bool motionLinksConfigured = false;

    const MechanismConfig* findMechanism(std::string_view id) const {
        for (const auto& m : mechanisms)
            if (m.id == id) return &m;
        return nullptr;
    }
    const PluginSection* findPlugin(std::string_view id) const {
        const auto it = plugins.find(id);
        return it == plugins.end() ? nullptr : &it->second;
    }
    PluginSection& pluginSection(std::string_view id) {
        auto it = plugins.find(id);
        if (it == plugins.end()) it = plugins.emplace(std::string(id), PluginSection{}).first;
        return it->second;
    }

    // Every slot mapping in the model, in a stable order (robot, drivetrain, mechanisms).
    std::vector<SlotMapping> allMappings() const {
        std::vector<SlotMapping> out = robotMappings;
        out.insert(out.end(), drivetrain.mappings.begin(), drivetrain.mappings.end());
        for (const auto& m : mechanisms) out.insert(out.end(), m.mappings.begin(), m.mappings.end());
        return out;
    }
};

}  // namespace wpiemu::sdk
