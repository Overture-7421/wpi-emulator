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
#include <utility>
#include <vector>

#include "editor_backend.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/game/field.h"
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
// `variant` = {property key, option value} (e.g. {"motion", "arm"}): the new mechanism starts with that property set, and its
// id / label follow the option ("arm", "Arm"). Empty key = the template's defaults.
using MechanismVariant = std::pair<std::string, std::string>;
std::string addMechanism(sdk::RobotConfig& c, const sdk::MechanismTemplateInfo& info, const sdk::MechanismAttachment& attachment,
                         const MechanismVariant& variant = {});
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
// Parents before children (stable). Broken chains keep their relative order at the end.
void normalizeOrder(sdk::RobotConfig& c);

// ---- zones ------------------------------------------------------------------------------------
std::string uniqueZoneId(const sdk::RobotConfig& c, std::string base);   // distinct from zone AND mechanism ids
std::string zoneDisplayName(const sdk::ZoneConfig& z, const EditorCatalog& catalog);
// Pose of the frame a zone's `local` pose is expressed in: the parent link (chassis frame when on the chassis).
sdk::Pose3 zoneParentWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z);
sdk::Pose3 zoneWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneConfig& z);
// The kind's defaults: pose, size, label and schema default properties.
sdk::ZoneConfig makeZone(const sdk::RobotConfig& c, const sdk::ZoneKindInfo& info, const sdk::MechanismAttachment& attachment);
// Returns the new zone id. An attachment to an unknown mechanism / link falls back to the chassis.
std::string addZone(sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::ZoneKindInfo& info,
                    const sdk::MechanismAttachment& attachment);
bool removeZone(sdk::RobotConfig& c, std::string_view id);
// Copy next to the original (same attachment and activation); returns the new id, empty when unknown.
std::string duplicateZone(sdk::RobotConfig& c, std::string_view id);
// Re-attaches a zone to another parent / link; with `keepWorldPose` its local pose is recomputed so it does not move.
bool reattachZone(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                  const std::string& newLink, bool keepWorldPose = true);
sdk::ZoneConfig* findZone(sdk::RobotConfig& c, std::string_view id);
// Activation edits. Conditions keep their mechanism id even when the mechanism is later deleted (the validator warns).
// addZoneCondition appends a condition on the first mechanism (empty id when there is none) and returns its index, -1 when
// the zone is unknown.
int addZoneCondition(sdk::RobotConfig& c, std::string_view zoneId);
bool removeZoneCondition(sdk::RobotConfig& c, std::string_view zoneId, int index);
bool setZoneCondition(sdk::RobotConfig& c, std::string_view zoneId, int index, const sdk::ZoneCondition& condition);
bool setZoneActivationMode(sdk::RobotConfig& c, std::string_view zoneId, sdk::ActivationMode mode);

// ---- cameras ----------------------------------------------------------------------------------
std::string uniqueCameraId(const sdk::RobotConfig& c, std::string base);   // distinct from camera, mechanism AND zone ids
std::string cameraDisplayName(const sdk::CameraConfig& cam);
// Pose of the frame a camera's `local` pose is expressed in: the parent link (chassis frame when on the chassis).
sdk::Pose3 cameraParentWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam);
sdk::Pose3 cameraWorld(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam);   // display frame
// Robot-to-camera transform in the WPILib robot frame (origin on the floor below the hull centre, +X forward, +Y left,
// yaw CCW): the display frame IS that frame, so this is cameraWorld. For a camera on a moving mechanism it is the rest pose.
sdk::Pose3 cameraRobotPose(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam);
// A forward-facing camera of `kind`: near the front of the hull on the chassis, at the link origin on a mechanism.
// Unique id, label and robot-code name per kind; optics are CameraConfig's defaults.
sdk::CameraConfig makeCamera(const sdk::RobotConfig& c, sdk::CameraKind kind, const sdk::MechanismAttachment& attachment);
// Returns the new camera id. An attachment to an unknown mechanism / link falls back to the chassis.
std::string addCamera(sdk::RobotConfig& c, const EditorCatalog& catalog, sdk::CameraKind kind,
                      const sdk::MechanismAttachment& attachment);
bool removeCamera(sdk::RobotConfig& c, std::string_view id);
// Copy next to the original (same attachment, optics and kind, a fresh name); returns the new id, empty when unknown.
std::string duplicateCamera(sdk::RobotConfig& c, std::string_view id);
// Re-attaches a camera to another parent / link; with `keepWorldPose` its local pose is recomputed so it does not move.
bool reattachCamera(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& newParent,
                    const std::string& newLink, bool keepWorldPose = true);
sdk::CameraConfig* findCamera(sdk::RobotConfig& c, std::string_view id);
// The robot-code name no other camera of the kind uses, derived from `base` ("" is the Limelight default name).
std::string uniqueCameraName(const sdk::RobotConfig& c, sdk::CameraKind kind, std::string base, std::string_view exceptId = {});

// ---- sensors ----------------------------------------------------------------------------------
std::string uniqueSensorId(const sdk::RobotConfig& c, std::string base);   // distinct from sensor, mechanism, zone AND camera ids
std::string sensorDisplayName(const sdk::SensorConfig& s);
// A sensor of `kind` with the kind's default response and no source; returns the new id.
std::string addSensor(sdk::RobotConfig& c, sdk::SensorKind kind);
bool removeSensor(sdk::RobotConfig& c, std::string_view id);
// Copy next to the original (same source and response, mappings are NOT copied); returns the new id, empty when unknown.
std::string duplicateSensor(sdk::RobotConfig& c, std::string_view id);
sdk::SensorConfig* findSensor(sdk::RobotConfig& c, std::string_view id);

// ---- start positions --------------------------------------------------------------------------
std::string uniqueStartPositionId(const sdk::RobotConfig& c, std::string base);   // distinct among start positions only
std::string startPositionDisplayName(const sdk::StartPosition& p);
// A blue position at a neutral pose; returns the new id.
std::string addStartPosition(sdk::RobotConfig& c);
bool removeStartPosition(sdk::RobotConfig& c, std::string_view id);
// Copy next to the original; returns the new id, empty when unknown.
std::string duplicateStartPosition(sdk::RobotConfig& c, std::string_view id);
// Reuses a position for the other alliance: the pose flipped by the field's symmetry, the alliance toggled and an
// alliance word in the label swapped (else " (Red)" / " (Blue)" appended). Empty when unknown or the frame has no symmetry.
std::string copyStartPositionToOtherAlliance(sdk::RobotConfig& c, std::string_view id, const sdk::FieldFrame& frame);
sdk::StartPosition* findStartPosition(sdk::RobotConfig& c, std::string_view id);

// ---- device mappings --------------------------------------------------------------------------
std::vector<sdk::SlotMapping>* mappingList(sdk::RobotConfig& c, std::string_view owner);
const sdk::SlotMapping* findMapping(const sdk::RobotConfig& c, const sdk::SlotAddress& slot);
// The address to store for a device: an already-declared alias of the same physical device wins so a
// second mapping never turns into an alias conflict just because the bus label differs.
sdk::DeviceKey declaredKeyFor(const sdk::RobotConfig& c, const sdk::DeviceKey& proposed);
bool setMapping(sdk::RobotConfig& c, const sdk::SlotAddress& slot, const sdk::DeviceKey& device);
bool clearMapping(sdk::RobotConfig& c, const sdk::SlotAddress& slot);
// Marks a mapped motor slot as spinning against the mechanism's positive direction. false when unmapped.
bool setMappingReversed(sdk::RobotConfig& c, const sdk::SlotAddress& slot, bool reversed);
std::vector<sdk::SlotAddress> slotsUsing(const sdk::RobotConfig& c, const sdk::PhysicalDeviceId& device);

// ---- slots ------------------------------------------------------------------------------------
std::vector<sdk::SlotSpec> robotSlotSpecs();
std::vector<sdk::SlotSpec> drivetrainSlotSpecs(const sdk::DrivetrainSpec& spec);
// Owners in tree order: robot, drivetrain, then mechanisms.
std::vector<sdk::SlotSpec> slotSpecsFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);
std::vector<std::string> slotOwners(const sdk::RobotConfig& c);
// A gearbox with several motors: one row per motor, the count lives in a mechanism property (`countKey`) or in
// WheelSpec::driveMotorCount (`countKey` = "<wheel>.driveMotorCount").
struct MotorGroup {
    std::string countKey;
    int count = 1;                      // active motors
    int maxCount = 1;
    std::vector<std::string> slotNames; // active motor slots, motor 1 first
};
std::vector<MotorGroup> motorGroups(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);
// Sets the motor count (clamped) and unmaps the motors that no longer exist. false when unknown/unchanged.
bool setMotorCount(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, const std::string& countKey, int count);
// Drops mappings of motor slots beyond the current motor count (after a property / wheel edit).
void pruneInactiveMappings(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner);
// "fl.drive" / "fl.drive3" / "fl.steer" -> "fl"; empty for anything else.
std::string wheelOfSlot(std::string_view slotName);
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
