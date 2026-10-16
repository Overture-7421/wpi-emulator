#pragma once

// Pure edit operations on a RobotConfig (no Qt, no undo): the vocabulary the editor's commands are
// built from. Each function either performs the edit and returns success, or leaves the config
// untouched. They are the unit-tested heart of the editor model.
//
// Display frame (shared with the viewport): world origin on the floor below the HULL CENTRE, Z up.
// The chassis frame (origin = COM, ADR 0003) is at (comOffset.xy, comHeight). Wheel `positionM.z` is a
// height above the floor, so a wheel's world z is positionM.z (see wheelWorld()).

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "editor_backend.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

namespace wpiemu::app::editor::ops {

inline constexpr const char* kChassisOwner = "chassis";   // owner string of chassis primitives

// ---- naming -----------------------------------------------------------------------------------
std::string baseIdForType(std::string_view mechanismType);                 // "game_2026.fuel_intake" -> "fuel_intake"
std::string uniqueMechanismId(const sdk::RobotConfig& c, std::string base);
std::string uniqueWheelName(const sdk::RobotConfig& c, std::string base);
std::string displayNameFor(const sdk::MechanismConfig& m, const EditorCatalog& catalog);

// ---- frames -----------------------------------------------------------------------------------
sdk::Pose3 chassisFrameWorld(const sdk::ChassisSpec& chassis);
sdk::Pose3 hullWorld(const sdk::ChassisSpec& chassis);                     // hull box centre
sdk::Pose3 wheelWorld(const sdk::ChassisSpec& chassis, const sdk::WheelSpec& wheel);   // hub, floor-referenced z
// Rest pose of `link` of a mechanism relative to its root (template LinkSpec::local); root = identity.
sdk::LinkPoseSource restLinkPoses(const sdk::RobotConfig& c, const EditorCatalog& catalog);
// Pose of a mechanism link in the CHASSIS frame; nullopt on a broken chain.
std::optional<sdk::Pose3> linkInChassis(const sdk::RobotConfig& c, const EditorCatalog& catalog,
                                        std::string_view mechanismId, std::string_view link = sdk::kRootLink);
sdk::Pose3 mechanismRootWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id);
// World pose of the frame a mechanism's `mount` is expressed in (the parent link, or the chassis frame).
sdk::Pose3 mechanismParentWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::MechanismConfig& m);

// ---- mechanisms -------------------------------------------------------------------------------
sdk::MechanismConfig makeMechanism(const sdk::RobotConfig& c, const sdk::MechanismTemplateInfo& info,
                                   const sdk::MechanismAttachment& attachment);
// Returns the new instance id. The template must exist.
std::string addMechanism(sdk::RobotConfig& c, const sdk::MechanismTemplateInfo& info, const sdk::MechanismAttachment& attachment);
std::vector<std::string> childrenOf(const sdk::RobotConfig& c, std::string_view id);
std::vector<std::string> descendantsOf(const sdk::RobotConfig& c, std::string_view id);
bool canReparent(const sdk::RobotConfig& c, std::string_view id, std::string_view newParent);
// Keeps the world placement by recomputing the mount (rest poses).
bool reparentMechanism(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id,
                       const std::string& newParent, const std::string& newLink, bool keepWorldPose = true);
// Children are re-attached to the removed mechanism's parent and keep their world placement.
bool removeMechanism(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id);
// Copies the mechanism and its sub-tree; mappings are NOT copied (a device drives one thing).
std::string duplicateMechanism(sdk::RobotConfig& c, std::string_view id);
// Updates attachments and slot owners. false when `newId` is empty/reserved/taken.
bool renameMechanism(sdk::RobotConfig& c, std::string_view id, const std::string& newId);
// Parents before children (stable). Broken chains keep their relative order at the end.
void normalizeOrder(sdk::RobotConfig& c);

// ---- device mappings --------------------------------------------------------------------------
std::vector<sdk::SlotMapping>* mappingList(sdk::RobotConfig& c, std::string_view owner);
const sdk::SlotMapping* findMapping(const sdk::RobotConfig& c, const sdk::SlotAddress& slot);
// The address to store for a device: an already-declared alias of the same physical device wins so a
// second mapping never turns into an alias conflict just because the bus label differs.
sdk::DeviceKey declaredKeyFor(const sdk::RobotConfig& c, const sdk::DeviceKey& proposed);
bool setMapping(sdk::RobotConfig& c, const sdk::SlotAddress& slot, const sdk::DeviceKey& device);
bool clearMapping(sdk::RobotConfig& c, const sdk::SlotAddress& slot);
std::vector<sdk::SlotAddress> slotsUsing(const sdk::RobotConfig& c, const sdk::PhysicalDeviceId& device);

// ---- slots ------------------------------------------------------------------------------------
std::vector<sdk::SlotSpec> robotSlotSpecs();
std::vector<sdk::SlotSpec> drivetrainSlotSpecs(const sdk::DrivetrainSpec& spec);
// Owners in tree order: robot, drivetrain, then mechanisms.
std::vector<sdk::SlotSpec> slotSpecsFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);
std::vector<std::string> slotOwners(const sdk::RobotConfig& c);
// "Intake > motor", "Wheel fl > drive"
std::string slotDisplayName(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::SlotAddress& slot);

// ---- chassis / drivetrain ---------------------------------------------------------------------
// Moves the COM inside the hull. With `compensate` every chassis-frame quantity (wheels, chassis
// primitives, chassis-attached mounts) is re-expressed so the robot looks the same.
void setCom(sdk::RobotConfig& c, float offsetX, float offsetY, float height, bool compensate = true,
            const EditorCatalog* catalog = nullptr);
void setDrivetrainKind(sdk::RobotConfig& c, const std::string& kind);
std::string addWheel(sdk::RobotConfig& c);
bool removeWheel(sdk::RobotConfig& c, const std::string& name);
bool renameWheel(sdk::RobotConfig& c, const std::string& oldName, const std::string& newName);
sdk::WheelSpec* findWheel(sdk::RobotConfig& c, std::string_view name);
const sdk::WheelSpec* findWheel(const sdk::RobotConfig& c, std::string_view name);

// ---- primitives -------------------------------------------------------------------------------
std::vector<sdk::PrimitiveSpec>* primitivesOf(sdk::RobotConfig& c, std::string_view owner);
const std::vector<sdk::PrimitiveSpec>* primitivesOf(const sdk::RobotConfig& c, std::string_view owner);
int addPrimitive(sdk::RobotConfig& c, std::string_view owner, sdk::PrimitiveKind kind);
bool removePrimitive(sdk::RobotConfig& c, std::string_view owner, int index);
int duplicatePrimitive(sdk::RobotConfig& c, std::string_view owner, int index);

// ---- plugin-defined volumes (Box3 properties) -------------------------------------------------
struct VolumeRef {
    std::string owner;        // mechanism id, or "" for the plugin config section
    std::string key;          // property key holding a Box3
    std::string label;
    std::string color;
    bool editable = true;
    sdk::Value defaultValue;  // schema default, used while the key is absent
};
// Every Box3 property exposed for the mechanism (template schema + matching panels), or for the plugin
// section when `mechanismId` is empty.
std::vector<VolumeRef> volumesFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view mechanismId);
// The object holding the volume's property (mechanism properties or the plugin section data).
sdk::Value* volumeStore(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);
const sdk::Value* volumeStore(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);

}  // namespace wpiemu::app::editor::ops
