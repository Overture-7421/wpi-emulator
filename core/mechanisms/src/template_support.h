#pragma once

// PRIVATE: base class and helpers shared by the generic mechanism templates.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "axis_support.h"
#include "wpiemu/core/mechanisms/mechanism_templates.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::core::mechanisms::detail {

// Collaborators a template needs at construction (shared with the registering code).
struct TemplateDeps {
    std::shared_ptr<const MotorCatalog> motors;
    std::shared_ptr<const sdk::PlantFactory> plants;
};

enum class AxisName { X, Y, Z };
const char* axisKey(AxisName a);
bool parseAxis(const std::string& text, AxisName& out);
sdk::Vec3 unitVector(AxisName a);
sdk::Pose3 rotationAbout(AxisName a, double angleRad);
sdk::Pose3 translationAlong(AxisName a, double distanceM);
double wrapAngle(double angleRad);   // to (-pi, pi]
sdk::PropertySpec axisEnumProp(const std::string& key, const std::string& label, AxisName def, const std::string& group);

// Builds one axis from the properties (motor catalog + plant factory lookups).
sdk::Result<std::unique_ptr<MechanismAxis>> buildAxis(const std::string& owner, const AxisOptions& options,
                                                      const sdk::Value& props, const TemplateDeps& deps);

// Common runtime of the templates: owns the axes, ticks them, caches the published state.
class AxisMechanismBase : public sdk::IMechanism {
public:
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(sdk::SimContext& ctx, double dtSeconds) override;
    void reset() override;
    sdk::MechanismState state() const override { return state_; }
    double batteryCurrentA() const override;
    std::vector<sdk::MotionAxisBinding> motionAxes() override;
    void finishMotionTick(sdk::SimContext& ctx, double dtSeconds) override;

protected:
    AxisMechanismBase(const sdk::MechanismConfig& config);

    MechanismAxis& addAxis(std::unique_ptr<MechanismAxis> axis);
    MechanismAxis& axis(std::size_t i) { return *axes_[i]; }
    const MechanismAxis& axis(std::size_t i) const { return *axes_[i]; }

    // Called by the most-derived constructor once it is fully built.
    void publishNow() { refresh(); }

    virtual void onTick(sdk::SimContext& /*ctx*/, double /*dtSeconds*/) {}
    // Fill `active`, `links` and `values`; id/type are set by the base.
    virtual void publish(sdk::MechanismState& out) const = 0;

private:
    void refresh();

    std::string id_;
    std::string type_;
    std::vector<std::unique_ptr<MechanismAxis>> axes_;
    sdk::MechanismState state_;
};

using TemplateBuilder =
    std::function<sdk::Result<std::unique_ptr<sdk::IMechanism>>(const sdk::MechanismConfig&, const sdk::Value& props)>;

// Registers `info` with a factory that defaults + validates the instance properties against
// info.properties, logs failures through ctx.sim.logger() and returns null for them.
sdk::Status registerValidated(sdk::MechanismFactory& factory, sdk::MechanismTemplateInfo info, TemplateBuilder builder);

// The single core template (mechanism_family.cpp).
sdk::Status registerMechanism(sdk::MechanismFactory& factory, const TemplateDeps& deps);

// NamedValue helper.
inline sdk::NamedValue namedValue(std::string name, double value, std::string unit) {
    return {std::move(name), value, std::move(unit)};
}

}  // namespace wpiemu::core::mechanisms::detail
