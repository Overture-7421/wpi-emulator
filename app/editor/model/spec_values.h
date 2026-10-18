#pragma once

// PropertySchema + Value adapters for the parts of the robot model that have no mechanism template:
// robot, chassis, drivetrain, wheels and geometry primitives. They let the SAME generic property form
// (and the same undo path) edit core data and plugin/mechanism data.

#include <string>
#include <vector>

#include "editor_backend.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/robot_config.h"

namespace wpiemu::app::editor::spec {

// Motors offered for wheel drive/steer (names == MotorSpec::name).
std::vector<sdk::MotorSpec> motorPresets();
sdk::MotorSpec motorByName(const std::string& name, const sdk::MotorSpec& fallback);

// ---- robot ----
sdk::PropertySchema robotSchema();
sdk::Value robotValues(const sdk::RobotConfig& c);
void applyRobot(sdk::RobotConfig& c, const std::string& key, const sdk::Value& v);

// ---- start pose (x, y in the field frame, heading; SimSettings::startPose) ----
sdk::PropertySchema startPoseSchema();
// The EFFECTIVE pose: the configured one, else the catalog's default placement, so the fields show where the robot starts.
sdk::Value startPoseValues(const sdk::RobotConfig& c, const EditorCatalog& catalog);
// Sets one component; the other two keep their effective values, so editing any field writes the whole pose.
void applyStartPose(sdk::RobotConfig& c, const EditorCatalog& catalog, const std::string& key, const sdk::Value& v);

// ---- chassis (hull, mass, COM, material) ----
sdk::PropertySchema chassisSchema();
sdk::Value chassisValues(const sdk::ChassisSpec& s);
// Edits one key. COM keys re-origin the chassis frame (see ops::setCom) so nothing moves visually.
void applyChassis(sdk::RobotConfig& c, const EditorCatalog& catalog, const std::string& key, const sdk::Value& v);

// ---- drivetrain ----
sdk::PropertySchema drivetrainSchema(const EditorCatalog& catalog);
sdk::Value drivetrainValues(const sdk::DrivetrainSpec& s);
void applyDrivetrain(sdk::RobotConfig& c, const std::string& key, const sdk::Value& v);

// ---- wheel ----
sdk::PropertySchema wheelSchema(const sdk::WheelSpec& wheel);
sdk::Value wheelValues(const sdk::WheelSpec& w);
void applyWheel(sdk::RobotConfig& c, const std::string& wheelName, const std::string& key, const sdk::Value& v);

// ---- primitive (box / cylinder) ----
sdk::PropertySchema primitiveSchema();
sdk::Value primitiveValues(const sdk::PrimitiveSpec& p);
void applyPrimitive(sdk::PrimitiveSpec& p, const std::string& key, const sdk::Value& v);

}  // namespace wpiemu::app::editor::spec
