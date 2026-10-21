#include "model/spec_values.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "model/config_ops.h"
#include "wpiemu/sdk/config/robot_spec_bindings.h"

namespace wpiemu::app::editor::spec {

using sdk::PropertySchema;
using sdk::Value;

namespace {

float numF(const Value& v, float fallback) { return static_cast<float>(v.asDouble(static_cast<double>(fallback))); }

std::vector<sdk::EnumOption> drivetrainOptions(const EditorCatalog& catalog) {
    std::vector<sdk::EnumOption> options;
    for (const auto& d : catalog.drivetrains)
        options.push_back({d.kind, d.info.displayName.empty() ? d.kind : d.info.displayName});
    return options;
}

}  // namespace

sdk::MotorSpec motorByName(const std::string& name, const sdk::MotorSpec& fallback) {
    return sdk::motorByName(name, fallback);
}

// ---- robot ------------------------------------------------------------------------------------
PropertySchema robotSchema() { return sdk::robotEditorBindings().schema(); }

Value robotValues(const sdk::RobotConfig& c) { return sdk::robotEditorBindings().encode(c); }

void applyRobot(sdk::RobotConfig& c, const std::string& key, const Value& v) {
    sdk::robotEditorBindings().apply(c, key, v);
}

// ---- chassis ----------------------------------------------------------------------------------
PropertySchema chassisSchema() { return sdk::chassisEditorBindings().schema(); }

Value chassisValues(const sdk::ChassisSpec& s) { return sdk::chassisEditorBindings().encode(s); }

void applyChassis(sdk::RobotConfig& c, const EditorCatalog& catalog, const std::string& key, const Value& v) {
    sdk::ChassisSpec& s = c.chassis;
    const std::string k = sdk::chassisEditorFieldKey(key);
    if (k == "comOffsetX")
        ops::setCom(c, numF(v, s.comOffsetM.x), s.comOffsetM.y, s.comHeightM, true, &catalog);
    else if (k == "comOffsetY")
        ops::setCom(c, s.comOffsetM.x, numF(v, s.comOffsetM.y), s.comHeightM, true, &catalog);
    else if (k == "comHeight")
        ops::setCom(c, s.comOffsetM.x, s.comOffsetM.y, numF(v, s.comHeightM), true, &catalog);
    else
        sdk::chassisEditorBindings().apply(s, k, v);
}

// ---- drivetrain -------------------------------------------------------------------------------
PropertySchema drivetrainSchema(const EditorCatalog& catalog) {
    return sdk::drivetrainEditorBindings(drivetrainOptions(catalog)).schema();
}

Value drivetrainValues(const sdk::DrivetrainSpec& s) { return sdk::drivetrainEditorBindings().encode(s); }

void applyDrivetrain(sdk::RobotConfig& c, const EditorCatalog& catalog, const std::string& key, const Value& v) {
    if (key == "kind")
        ops::setDrivetrainKind(c, catalog, v.asString(c.drivetrain.spec.kind));
    else
        sdk::drivetrainEditorBindings().apply(c.drivetrain.spec, key, v);
}

// ---- wheel ------------------------------------------------------------------------------------
PropertySchema wheelSchema(const sdk::WheelSpec& wheel) { return sdk::wheelEditorBindings(wheel).schema(); }

Value wheelValues(const sdk::WheelSpec& w) { return sdk::wheelEditorBindings(w).encode(w); }

void applyWheel(sdk::RobotConfig& c, const std::string& wheelName, const std::string& key, const Value& v) {
    const std::string k = sdk::wheelEditorFieldKey(key);
    if (k == "name") {
        ops::renameWheel(c, wheelName, v.asString(wheelName));
        return;
    }
    sdk::WheelSpec* w = ops::findWheel(c, wheelName);
    if (w == nullptr) return;
    if (k == "driveMotorCount") {
        sdk::wheelEditorBindings(*w).apply(*w, k, v, true);
        auto& maps = c.drivetrain.mappings;
        maps.erase(std::remove_if(maps.begin(), maps.end(),
                                  [&](const sdk::SlotMapping& m) {
                                      for (int i = 0; i < w->driveMotorCount; ++i)
                                          if (m.slot.slot == sdk::driveSlotName(w->name, i)) return false;
                                      return ops::wheelOfSlot(m.slot.slot) == w->name &&
                                             m.slot.slot != sdk::steerSlotName(w->name) &&
                                             m.slot.slot != sdk::steerEncoderSlotName(w->name);
                                  }),
                   maps.end());
        return;
    }
    sdk::wheelEditorBindings(*w).apply(*w, k, v);
    if (k == "kind") {
        w->steered = w->kind == sdk::WheelKind::Swerve;
        if (!w->steered) {
            auto& maps = c.drivetrain.mappings;
            maps.erase(std::remove_if(maps.begin(), maps.end(),
                                      [&](const sdk::SlotMapping& m) {
                                          return m.slot.slot == sdk::steerSlotName(w->name) ||
                                                 m.slot.slot == sdk::steerEncoderSlotName(w->name);
                                      }),
                       maps.end());
        }
    }
}

// ---- primitive --------------------------------------------------------------------------------
PropertySchema primitiveSchema() { return sdk::primitiveEditorBindings().schema(); }

Value primitiveValues(const sdk::PrimitiveSpec& p) { return sdk::primitiveEditorBindings().encode(p); }

void applyPrimitive(sdk::PrimitiveSpec& p, const std::string& key, const Value& v) {
    sdk::primitiveEditorBindings().apply(p, key, v);
}

}  // namespace wpiemu::app::editor::spec
