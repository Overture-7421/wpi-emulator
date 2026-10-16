#include "wpiemu/plugins/game_2026/fuel_intake.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "wpiemu/plugins/game_2026/mechanism_io.h"
#include "wpiemu/plugins/game_2026/property_helpers.h"
#include "wpiemu/sdk/core/sim_context.h"

namespace wpiemu::plugins::game_2026 {

namespace {

constexpr const char* kGroupZone = "Pickup zone";
constexpr const char* kGroupActivation = "Activation";
constexpr const char* kGroupLimits = "Limits";

sdk::PropertySchema intakeSchema() {
    sdk::PropertySchema s;
    auto& p = s.properties;
    sdk::Box3 zone;
    zone.pose = sdk::Pose3{{0.45f, 0.f, 0.15f}, sdk::Quat{}};
    zone.sizeM = {0.30f, 0.60f, 0.30f};
    p.push_back(props::box("pickupZone", "Pickup zone", kGroupZone, zone,
                           "Volume (mechanism frame) in which fuel centres are absorbed while the intake is active."));

    p.push_back(props::choice("activation", "Active when", kGroupActivation, "roller",
                              {{"roller", "Roller spinning"},
                               {"deployed", "Deployed"},
                               {"roller_and_deployed", "Roller spinning AND deployed"},
                               {"roller_or_deployed", "Roller spinning OR deployed"},
                               {"always", "Always"}},
                              "Condition under which the pickup zone absorbs fuel."));
    p.push_back(props::text("rollerMechanism", "Roller mechanism", kGroupActivation, "",
                            "Id of the mechanism whose speed is the roller (empty: roller condition always true)."));
    p.push_back(props::text("rollerValue", "Roller value", kGroupActivation, "velocity",
                            "Published value name of that mechanism (e.g. velocity)."));
    p.push_back(props::real("rollerThreshold", "Roller threshold", kGroupActivation, 20.0, 0.0, 1e6,
                            sdk::PropertyUnit::RadiansPerSecond, "|value| (SI) above which the roller counts as spinning."));
    p.push_back(props::text("deployMechanism", "Deploy mechanism", kGroupActivation, "",
                            "Id of the mechanism that deploys the intake (empty: deploy condition always true)."));
    p.push_back(props::text("deployValue", "Deploy value", kGroupActivation, "position",
                            "Published value name of that mechanism (e.g. position)."));
    p.push_back(props::real("deployedPosition", "Deployed position", kGroupActivation, 0.0, -1e6, 1e6,
                            sdk::PropertyUnit::None, "SI value of the deploy mechanism when the intake is out."));
    p.push_back(props::real("deployedTolerance", "Deployed tolerance", kGroupActivation, 0.1, 0.0, 1e6,
                            sdk::PropertyUnit::None, "Allowed deviation from the deployed position."));

    p.push_back(props::real("intakeRateHz", "Intake rate", kGroupLimits, 15.0, 0.0, 1000.0, sdk::PropertyUnit::Hertz,
                            "Maximum fuel absorbed per second by this intake (0 = unlimited)."));
    return s;
}

IntakeActivation parseActivation(const std::string& s) {
    if (s == "deployed") return IntakeActivation::Deployed;
    if (s == "roller_and_deployed") return IntakeActivation::RollerAndDeployed;
    if (s == "roller_or_deployed") return IntakeActivation::RollerOrDeployed;
    if (s == "always") return IntakeActivation::Always;
    return IntakeActivation::Roller;
}

}  // namespace

IntakeParams parseIntakeParams(const sdk::Value& properties) {
    const sdk::PropertySchema schema = intakeSchema();
    sdk::Value v = properties;
    sdk::applyDefaults(schema, v);
    IntakeParams p;
    if (!sdk::fromValue(v.get("pickupZone"), p.pickupZone)) sdk::fromValue(schema.find("pickupZone")->defaultValue, p.pickupZone);
    p.activation = parseActivation(v.get("activation").asString());
    p.rollerMechanism = v.get("rollerMechanism").asString();
    p.rollerValue = v.get("rollerValue").asString("velocity");
    p.rollerThreshold = std::max(0.0, v.get("rollerThreshold").asDouble(p.rollerThreshold));
    p.deployMechanism = v.get("deployMechanism").asString();
    p.deployValue = v.get("deployValue").asString("position");
    p.deployedPosition = v.get("deployedPosition").asDouble(p.deployedPosition);
    p.deployedTolerance = std::max(0.0, v.get("deployedTolerance").asDouble(p.deployedTolerance));
    p.intakeRateHz = std::max(0.0, v.get("intakeRateHz").asDouble(p.intakeRateHz));
    return p;
}

sdk::MechanismTemplateInfo fuelIntakeTemplate() {
    sdk::MechanismTemplateInfo info;
    info.type = kIntakeType;
    info.displayName = "Fuel Intake";
    info.category = "Intake";
    info.origin = "game_2026";
    info.description =
        "REBUILT fuel pickup zone. Absorbs fuel into the robot's fuel meter while the referenced roller spins "
        "and/or the referenced deploy mechanism is out. Add it next to your roller/arm mechanisms.";
    info.properties = intakeSchema();
    sdk::PrimitiveSpec frame;
    frame.name = "intake_frame";
    frame.kind = sdk::PrimitiveKind::Box;
    frame.local = sdk::Pose3{{0.45f, 0.f, 0.15f}, sdk::Quat{}};
    frame.sizeM = {0.30f, 0.60f, 0.02f};
    frame.color = "#ff9f1c";
    info.defaultGeometry.push_back(std::move(frame));
    return info;
}

FuelIntakeMechanism::FuelIntakeMechanism(const sdk::MechanismBuildContext& ctx)
    : id_(ctx.config.id), params_(parseIntakeParams(ctx.config.properties)), mountFallback_(ctx.config.mount),
      rootInChassis_(ctx.config.mount) {}

bool FuelIntakeMechanism::rollerSatisfied(sdk::SimContext& ctx) {
    if (params_.rollerMechanism.empty()) return true;
    const MechanismReading r = readMechanism(ctx.mechanisms(), params_.rollerMechanism, params_.rollerValue);
    if (!r.found) warnMissing(ctx, params_.rollerMechanism);
    return exceeds(r, params_.rollerThreshold);
}

bool FuelIntakeMechanism::deploySatisfied(sdk::SimContext& ctx) {
    if (params_.deployMechanism.empty()) return true;
    const MechanismReading r = readMechanism(ctx.mechanisms(), params_.deployMechanism, params_.deployValue);
    if (!r.found) {
        warnMissing(ctx, params_.deployMechanism);
        return false;
    }
    return r.hasValue ? std::fabs(r.value - params_.deployedPosition) <= params_.deployedTolerance : r.active;
}

void FuelIntakeMechanism::warnMissing(sdk::SimContext& ctx, const std::string& mechanism) {
    if (std::find(warned_.begin(), warned_.end(), mechanism) != warned_.end()) return;
    warned_.push_back(mechanism);
    ctx.logger().warn("plugin.game_2026", "fuel intake '" + id_ + "' references unknown mechanism '" + mechanism + "'");
}

void FuelIntakeMechanism::tick(sdk::SimContext& ctx, double dt) {
    rootInChassis_ = mountFallback_;
    if (sdk::IMechanismDirectory* dir = ctx.mechanisms())
        if (const auto p = dir->linkPoseInChassis(id_, sdk::kRootLink)) rootInChassis_ = *p;

    switch (params_.activation) {
        case IntakeActivation::Roller: active_ = rollerSatisfied(ctx); break;
        case IntakeActivation::Deployed: active_ = deploySatisfied(ctx); break;
        case IntakeActivation::RollerAndDeployed: {
            const bool roller = rollerSatisfied(ctx), deployed = deploySatisfied(ctx);
            active_ = roller && deployed;
            break;
        }
        case IntakeActivation::RollerOrDeployed: {
            const bool roller = rollerSatisfied(ctx), deployed = deploySatisfied(ctx);
            active_ = roller || deployed;
            break;
        }
        case IntakeActivation::Always: active_ = true; break;
    }

    // Token bucket: primed while idle so the first fuel is taken the moment the intake becomes active.
    if (!active_) budget_ = 1.0;
    else budget_ = std::min(2.0, budget_ + params_.intakeRateHz * dt);
}

int FuelIntakeMechanism::pickupBudget() const {
    if (!active_) return 0;
    if (params_.intakeRateHz <= 0.0) return std::numeric_limits<int>::max();
    return static_cast<int>(std::floor(budget_));
}

void FuelIntakeMechanism::consumePickups(int count) {
    if (params_.intakeRateHz > 0.0) budget_ = std::max(0.0, budget_ - static_cast<double>(count));
}

sdk::Box3 FuelIntakeMechanism::zoneInChassis() const {
    sdk::Box3 z = params_.pickupZone;
    z.pose = rootInChassis_ * z.pose;
    return z;
}

void FuelIntakeMechanism::reset() {
    active_ = false;
    budget_ = 1.0;
    rootInChassis_ = mountFallback_;
    warned_.clear();
}

sdk::MechanismState FuelIntakeMechanism::state() const {
    sdk::MechanismState s;
    s.id = id_;
    s.type = std::string(kIntakeType);
    s.active = active_;
    s.values.push_back({"active", active_ ? 1.0 : 0.0, ""});
    return s;
}

std::unique_ptr<sdk::IMechanism> createFuelIntake(const sdk::MechanismBuildContext& ctx) {
    return std::make_unique<FuelIntakeMechanism>(ctx);
}

}  // namespace wpiemu::plugins::game_2026
