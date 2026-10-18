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

// ---- vision cameras (vision/camera_config.h, ADR 0017) --------------------------------------------
// Camera ids share the namespace of mechanism and zone ids. Attachment problems of a camera reuse
// the kAttachment* codes above (path "vision.cameras[i].attachment"), exactly like zones.
// Twice, or equal to a mechanism / zone id.
inline constexpr const char* kCameraIdDuplicate = "camera_id_duplicate";
// Empty, "chassis" or "root".
inline constexpr const char* kCameraIdReserved = "camera_id_reserved";
// Two enabled cameras of the same vendor with the same network name.
inline constexpr const char* kCameraNameDuplicate = "camera_name_duplicate";
inline constexpr const char* kCameraIntrinsicsInvalid = "camera_intrinsics_invalid";
// Frame rate outside (0, 240] Hz.
inline constexpr const char* kCameraFrameRateInvalid = "camera_frame_rate_invalid";
// Latency (or capture latency) negative, or above 1 s.
inline constexpr const char* kCameraLatencyInvalid = "camera_latency_invalid";
// Tag size, minimum tag pixels or maximum view angle.
inline constexpr const char* kCameraTagParamsInvalid = "camera_tag_params_invalid";
inline constexpr const char* kCameraRealismInvalid = "camera_realism_invalid";
// VisionConfig::layoutId names no layout of the selected game plugin although that plugin provides
// some (Warning; not checked when a layoutFile is set or no plugin is selected). Resolving the
// layout itself is the vision runtime's job.
inline constexpr const char* kCameraLayoutUnknown = "camera_layout_unknown";

// Runtime HubWarning codes of the vision subsystem (not raised by the config validator).
// No fiducial layout could be resolved: unknown VisionConfig::layoutId, unreadable layoutFile. Error.
inline constexpr const char* kVisionLayoutMissing = "vision_layout_missing";
// The robot process uses a camera the config does not list (CameraRobotState::cameraId empty). Warning.
inline constexpr const char* kVisionCameraUnconfigured = "vision_camera_unconfigured";
// The robot process reports a problem with a camera (CameraRobotState::status). Warning or Error.
inline constexpr const char* kVisionCameraStatus = "vision_camera_status";

}  // namespace wpiemu::sdk::validation_code
