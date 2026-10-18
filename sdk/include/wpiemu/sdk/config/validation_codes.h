#pragma once

// Stable machine keys for Issue::code / HubWarning::code. Validators and hubs MUST use
// these exact strings so the UI and tests can react without parsing messages.

namespace wpiemu::sdk::validation_code {

// ---- device identity (devices/device_identity.h, ADR 0006) ---------------------------------------
// Two or more robot-declared addresses that differ only by bus resolve to one physical device and
// are mapped to different slots. Error: the config is rejected; at runtime the device is looped
// back (never half-driven) until the mappings are fixed.
inline constexpr const char* kDeviceAliasConflict = "device_alias_conflict";
// Several declared addresses resolve to one physical device but only one slot uses them. Warning.
inline constexpr const char* kDeviceAliased = "device_aliased";
// The declared bus differs from the bus observed at runtime (L3). Warning; observation wins.
inline constexpr const char* kDeviceBusMismatch = "device_bus_mismatch";

// ---- simulation settings (config/robot_config.h) ----------------------------------------------------
// SimSettings::startPose has a non-finite component (Error), or lies more than 1000 m from the field origin (Warning).
inline constexpr const char* kStartPoseInvalid = "start_pose_invalid";

// ---- mechanism attachment (mechanisms/attachment.h) -----------------------------------------------
inline constexpr const char* kAttachmentParentMissing = "attachment_parent_missing";
inline constexpr const char* kAttachmentLinkMissing = "attachment_link_missing";
inline constexpr const char* kAttachmentCycle = "attachment_cycle";
inline constexpr const char* kMechanismIdReserved = "mechanism_id_reserved";
inline constexpr const char* kTemplateLinkInvalid = "template_link_invalid";

}  // namespace wpiemu::sdk::validation_code
