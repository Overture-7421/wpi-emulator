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

// The SDK motor table entry named `name` (what the wheel motor dropdowns store), else `fallback`.
sdk::MotorSpec motorByName(const std::string& name, const sdk::MotorSpec& fallback);

// ---- robot ----
sdk::PropertySchema robotSchema();
sdk::Value robotValues(const sdk::RobotConfig& c);
void applyRobot(sdk::RobotConfig& c, const std::string& key, const sdk::Value& v);

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
