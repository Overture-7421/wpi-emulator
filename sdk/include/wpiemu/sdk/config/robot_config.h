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

#include "wpiemu/sdk/config/camera.h"
#include "wpiemu/sdk/config/mechanism_attachment.h"
#include "wpiemu/sdk/config/zone.h"
#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/value.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/mechanisms/motion_link.h"
#include "wpiemu/sdk/physics/chassis_spec.h"
#include "wpiemu/sdk/physics/physics_types.h"

namespace wpiemu::sdk {

// Version of the CORE schema (chassis/drivetrain/mechanisms/mappings). The SDK stays at version 1 until
// release (ADR 0013): files with any other version are rejected, there is no migration.
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

// A mechanism instance placed on the robot.
struct MechanismConfig {
    std::string id;          // unique instance id (also the SlotAddress owner), e.g. "intake"
    std::string type;        // MechanismFactory key, e.g. "roller", "elevator", "game_2026.shooter"
    std::string label;
    Pose3 mount;             // relative to the parent LINK frame (the chassis frame for the chassis)
    std::vector<PrimitiveSpec> geometry;   // box/cylinder primitives (visual and optional collision)
    Value properties = Value::makeObject();   // validated against the template's PropertySchema
    std::vector<SlotMapping> mappings;
    MechanismAttachment attachment;   // defaults to the chassis root

    void attachTo(std::string parentMechanismId, std::string parentLink = kRootLink) {
        attachment.parentMechanismId = parentMechanismId.empty() ? std::string(kChassisParent) : std::move(parentMechanismId);
        attachment.parentLink = std::move(parentLink);
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

    std::vector<MotionLink> motionLinks;

    // Game-owned zones (pickup volumes, muzzles, ...) attached to the chassis or a mechanism link (config/zone.h).
    std::vector<ZoneConfig> zones;

    // Simulated vision cameras (Limelight / PhotonVision) attached to the chassis or a mechanism link (config/camera.h).
    std::vector<CameraConfig> cameras;

    const CameraConfig* findCamera(std::string_view id) const {
        for (const auto& c : cameras)
            if (c.id == id) return &c;
        return nullptr;
    }
    const MechanismConfig* findMechanism(std::string_view id) const {
        for (const auto& m : mechanisms)
            if (m.id == id) return &m;
        return nullptr;
    }
    const ZoneConfig* findZone(std::string_view id) const {
        for (const auto& z : zones)
            if (z.id == id) return &z;
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
