#pragma once

// Stable machine keys for Issue::code / HubWarning::code. Validators and hubs MUST use
// these exact strings so the UI and tests can react without parsing messages.

namespace wpiemu::sdk::validation_code {

// ---- device identity (devices/device_identity.h, docs/ARCHITECTURE.md) ---------------------------------------
// Two or more robot-declared addresses that differ only by bus resolve to one physical device and
// are mapped to different slots. Error: the config is rejected; at runtime the device is looped
// back (never half-driven) until the mappings are fixed.
inline constexpr const char* kDeviceAliasConflict = "device_alias_conflict";
// Several declared addresses resolve to one physical device but only one slot uses them. Warning.
inline constexpr const char* kDeviceAliased = "device_aliased";
// The declared bus differs from the bus observed at runtime (L3). Warning; observation wins.
inline constexpr const char* kDeviceBusMismatch = "device_bus_mismatch";

// ---- mechanism attachment (mechanisms/attachment.h) -----------------------------------------------
inline constexpr const char* kAttachmentParentMissing = "attachment_parent_missing";
inline constexpr const char* kAttachmentLinkMissing = "attachment_link_missing";
inline constexpr const char* kAttachmentCycle = "attachment_cycle";
inline constexpr const char* kMechanismIdReserved = "mechanism_id_reserved";
inline constexpr const char* kTemplateLinkInvalid = "template_link_invalid";

// ---- start positions (config/config_checks.h) ------------------------------------------------------------------
inline constexpr const char* kStartPositionIdInvalid = "start_position_id_invalid";        // empty
inline constexpr const char* kStartPositionIdDuplicate = "start_position_id_duplicate";
inline constexpr const char* kStartPositionInvalid = "start_position_invalid";            // non-finite pose

// ---- zones (config/zone.h, config/config_checks.h); attachment problems use kAttachment* ---------------------------
inline constexpr const char* kZoneIdInvalid = "zone_id_invalid";                // empty id
inline constexpr const char* kZoneIdDuplicate = "zone_id_duplicate";            // duplicate, or equal to a mechanism / camera id
inline constexpr const char* kZoneKindNotOwned = "zone_kind_not_owned";         // no plugin, or kind lacks the plugin id prefix (Error)
inline constexpr const char* kZoneKindUnknown = "zone_kind_unknown";           // owned prefix but not declared by the plugin (Warning)
inline constexpr const char* kZonePropertyInvalid = "zone_property_invalid";
inline constexpr const char* kZoneGeometryInvalid = "zone_geometry_invalid";    // non-finite pose, size <= 0 on a Volume
inline constexpr const char* kZoneConditionInvalid = "zone_condition_invalid";  // empty value name, non-finite threshold
inline constexpr const char* kZoneConditionMechanismUnknown = "zone_condition_mechanism_unknown";  // Warning

// ---- cameras (config/camera.h, config/config_checks.h); attachment problems use kAttachment* ---------------------
inline constexpr const char* kCameraIdInvalid = "camera_id_invalid";            // empty id
inline constexpr const char* kCameraIdDuplicate = "camera_id_duplicate";        // duplicate, or equal to a mechanism / zone id
inline constexpr const char* kCameraNameDuplicate = "camera_name_duplicate";    // two cameras of one kind share the robot-code name (Error)
inline constexpr const char* kCameraGeometryInvalid = "camera_geometry_invalid";  // non-finite mount pose
inline constexpr const char* kCameraOpticsInvalid = "camera_optics_invalid";    // fov/range/fps/latency/resolution/noise out of range

// ---- sensors (config/sensor.h, config/config_checks.h) -----------------------------------------------------------
inline constexpr const char* kSensorIdInvalid = "sensor_id_invalid";            // empty, reserved, or contains '.'
inline constexpr const char* kSensorIdDuplicate = "sensor_id_duplicate";        // duplicate, or equal to a mechanism / zone / camera id
inline constexpr const char* kSensorSourceMissing = "sensor_source_missing";    // no game value selected (Warning)
inline constexpr const char* kSensorSourceNotOwned = "sensor_source_not_owned";  // no plugin, or the value lacks the plugin id prefix (Error)
inline constexpr const char* kSensorSourceUnknown = "sensor_source_unknown";    // owned prefix but not declared by the plugin (Warning)
inline constexpr const char* kSensorPropertyInvalid = "sensor_property_invalid";  // response schema violation, or a degenerate linear range

}  // namespace wpiemu::sdk::validation_code
