#include "template_support.h"

#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::mechanisms {

namespace detail {

const char* axisKey(AxisName a) {
    switch (a) {
        case AxisName::X: return "x";
        case AxisName::Y: return "y";
        case AxisName::Z: return "z";
    }
    return "y";
}

bool parseAxis(const std::string& text, AxisName& out) {
    if (text == "x") out = AxisName::X;
    else if (text == "y") out = AxisName::Y;
    else if (text == "z") out = AxisName::Z;
    else return false;
    return true;
}

sdk::Vec3 unitVector(AxisName a) {
    switch (a) {
        case AxisName::X: return {1.f, 0.f, 0.f};
        case AxisName::Y: return {0.f, 1.f, 0.f};
        case AxisName::Z: return {0.f, 0.f, 1.f};
    }
    return {0.f, 1.f, 0.f};
}

sdk::Pose3 rotationAbout(AxisName a, double angleRad) {
    return {sdk::Vec3{}, sdk::Quat::fromAxisAngle(unitVector(a), static_cast<float>(angleRad))};
}

sdk::Pose3 translationAlong(AxisName a, double distanceM) {
    return {unitVector(a) * static_cast<float>(distanceM), sdk::Quat{}};
}

sdk::PropertySpec axisEnumProp(const std::string& key, const std::string& label, AxisName def,
                               const std::string& group) {
    return sdk::props::choice(key, label, group, axisKey(def), {{"x", "X"}, {"y", "Y"}, {"z", "Z"}},
                              "Axis in the mechanism frame (+X forward, +Y left, +Z up).");
}

sdk::Result<std::unique_ptr<MechanismAxis>> buildAxis(const std::string& owner, const AxisOptions& options,
                                                      const sdk::Value& props, const TemplateDeps& deps) {
    auto settings = readAxisSettings(options, props, *deps.motors);
    if (!settings.ok()) return settings.error();
    auto plant = deps.plants->createFor(settings.value().plant);
    if (!plant.ok()) return plant.error();
    MotorDrive drive(settings.value().plant.motor, settings.value().statorCurrentLimitA, settings.value().neutral);
    return std::make_unique<MechanismAxis>(owner, settings.value(), std::move(plant).value(), std::move(drive));
}

AxisMechanismBase::AxisMechanismBase(const sdk::MechanismConfig& config) : id_(config.id), type_(config.type) {
    state_.id = id_;
    state_.type = type_;
}

MechanismAxis& AxisMechanismBase::addAxis(std::unique_ptr<MechanismAxis> axis) {
    axes_.push_back(std::move(axis));
    return *axes_.back();
}

void AxisMechanismBase::tick(sdk::SimContext& ctx, double dtSeconds) {
    for (auto& a : axes_) a->tick(ctx, dtSeconds);
    finishMotionTick(ctx, dtSeconds);
}

std::vector<sdk::MotionAxisBinding> AxisMechanismBase::motionAxes() {
    std::vector<sdk::MotionAxisBinding> out;
    for (auto& a : axes_) out.push_back(a->motionBinding());
    return out;
}

void AxisMechanismBase::finishMotionTick(sdk::SimContext& ctx, double dtSeconds) {
    onTick(ctx, dtSeconds);
    refresh();
}

void AxisMechanismBase::reset() {
    for (auto& a : axes_) a->reset();
    refresh();
}

double AxisMechanismBase::batteryCurrentA() const {
    double total = 0.0;
    for (const auto& a : axes_) total += a->batteryCurrentA();
    return total;
}

void AxisMechanismBase::refresh() {
    sdk::MechanismState s;
    s.id = id_;
    s.type = type_;
    publish(s);
    state_ = std::move(s);
}

sdk::Status registerValidated(sdk::MechanismFactory& factory, sdk::MechanismTemplateInfo info, TemplateBuilder builder) {
    const sdk::PropertySchema schema = info.properties;
    const std::string type = info.type;
    return factory.registerTemplate(
        std::move(info),
        [schema, type, builder = std::move(builder)](const sdk::MechanismBuildContext& ctx)
            -> std::unique_ptr<sdk::IMechanism> {
            sdk::Value props = ctx.config.properties;
            sdk::applyDefaults(schema, props);
            for (const sdk::Issue& issue : sdk::validateProperties(schema, props, "properties"))
                if (issue.severity == sdk::Severity::Error) {
                    ctx.sim.logger().error("mechanisms", "mechanism '" + ctx.config.id + "' (" + type + "): " +
                                                             issue.path + ": " + issue.message);
                    return nullptr;
                }
            auto built = builder(ctx.config, props);
            if (!built.ok()) {
                ctx.sim.logger().error("mechanisms", "mechanism '" + ctx.config.id + "' (" + type + "): " +
                                                         built.error().message);
                return nullptr;
            }
            return std::move(built).value();
        });
}

}  // namespace detail

sdk::Status registerGenericMechanisms(sdk::MechanismFactory& factory, MechanismTemplateServices services) {
    detail::TemplateDeps deps;
    deps.motors = services.motors ? std::move(services.motors) : std::make_shared<const MotorCatalog>(MotorCatalog::builtin());
    if (services.plants) {
        deps.plants = std::move(services.plants);
    } else {
        auto plants = std::make_shared<sdk::PlantFactory>();
        if (const sdk::Status s = registerBuiltinPlants(*plants); !s.ok()) return s;
        deps.plants = std::move(plants);
    }
    return detail::registerMechanism(factory, deps);
}

}  // namespace wpiemu::core::mechanisms
