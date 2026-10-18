#pragma once

// PropertySchema + Value adapters for the vision part of the robot model (ADR 0017): one camera (RobotConfig::vision.cameras) split
// into the cards the inspector shows, and the project-level vision settings (AprilTag layout, PhotonLib UUID). Like the robot / chassis
// adapters in spec_values.h they let the SAME generic property form and the same undo path edit a part of the model that has no
// mechanism template.
//
// The form keys are the persisted field names, except where the editor shows a friendlier unit: `latencyMs`, `captureLatencyMs`,
// `latencyJitterMs` (stored in microseconds), `dropoutPercent` (stored as a probability) and the display-only `advancedIntrinsics`
// (fx, fy, cx, cy are explicit) and `assumedOverride` (the Limelight pose override is set).

#include <string>
#include <string_view>

#include "editor_backend.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor::spec {

// ---- one camera ----
sdk::PropertySchema cameraGeneralSchema(const sdk::CameraConfig& camera);              // label, network name, enabled
sdk::PropertySchema cameraPlacementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& camera);
sdk::PropertySchema cameraOpticsSchema(const sdk::CameraConfig& camera);               // resolution, field of view / camera matrix, rate, latency
sdk::PropertySchema cameraDetectionSchema();                                            // tag size, minimum pixels, view angle, occlusion, multi-tag
sdk::PropertySchema cameraRealismSchema();                                              // corner noise, latency jitter, dropout
sdk::PropertySchema cameraAssumedSchema();                                              // Limelight only: the pose the device believes
// Every key of the schemas above for `camera`.
sdk::Value cameraValues(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& camera);
// Applies one form edit to the camera `cameraId`; false when nothing changed.
bool applyCamera(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view cameraId, const std::string& key, const sdk::Value& value);
// Where the robot finds the camera on NetworkTables: "/photonvision/<name>" for PhotonVision, "/<name>" (the table) for a Limelight.
std::string cameraTablePath(const sdk::CameraConfig& camera);

// ---- project-level settings (the Vision card) ----
sdk::PropertySchema visionSchema(const EditorCatalog& catalog);
sdk::Value visionValues(const sdk::VisionConfig& vision);
bool applyVision(sdk::RobotConfig& c, const std::string& key, const sdk::Value& value);

// The editor's own check of a PhotonVision message UUID: 32 hexadecimal digits (an MD5 digest). Empty is valid (derive it).
bool isPhotonMessageUuid(std::string_view uuid);

}  // namespace wpiemu::app::editor::spec
