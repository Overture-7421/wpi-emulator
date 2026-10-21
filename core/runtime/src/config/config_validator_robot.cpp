#include <cmath>
#include <string>

#include "config_validator_steps.h"
#include "wpiemu/sdk/config/config_checks.h"

namespace wpiemu::core::config::detail {

using namespace wpiemu::sdk;

void validateDrivetrain(const RobotConfig& c, const DrivetrainRegistry& drivetrains, const TireModelRegistry* tires,
                        std::vector<Issue>& out) {
    const DrivetrainSpec& spec = c.drivetrain.spec;
    if (!drivetrains.contains(spec.kind))
        add(out, Severity::Error, "drivetrain.kind", "unknown drivetrain kind '" + spec.kind + "'", issue_code::kDrivetrainKindUnknown);
    if (tires != nullptr && !tires->contains(spec.tireModel))
        add(out, Severity::Error, "drivetrain.tireModel", "unknown tire model '" + spec.tireModel + "'", issue_code::kTireModelUnknown);
    sdk::validateWheelSpecs(spec, out);
}

void validateSimSettings(const RobotConfig& c, std::vector<Issue>& out) {
    if (c.sim.stepUs <= 0 || kRobotPeriodUs % c.sim.stepUs != 0)
        add(out, Severity::Error, "sim.stepUs", "step must be positive and divide the 20 ms robot period evenly", issue_code::kSimInvalid);
    if (c.sim.physics.subSteps < 1) add(out, Severity::Error, "sim.physics.subSteps", "at least 1 sub-step", issue_code::kSimInvalid);
    if (c.sim.physics.workerCount < 1) add(out, Severity::Error, "sim.physics.workerCount", "at least 1 worker", issue_code::kSimInvalid);
    const BatteryConfig& b = c.battery;
    if (!(b.nominalVoltageV > 0.0) || !std::isfinite(b.nominalVoltageV))
        add(out, Severity::Error, "battery.nominalVoltageV", "nominal voltage must be positive", issue_code::kBatteryInvalid);
    if (!(b.internalResistanceOhm >= 0.0) || !std::isfinite(b.internalResistanceOhm))
        add(out, Severity::Error, "battery.internalResistanceOhm", "resistance must be >= 0", issue_code::kBatteryInvalid);
    if (!(b.brownoutVoltageV >= 0.0) || b.brownoutVoltageV >= b.nominalVoltageV)
        add(out, Severity::Error, "battery.brownoutVoltageV", "brownout voltage must be in [0, nominal)", issue_code::kBatteryInvalid);
}

}  // namespace wpiemu::core::config::detail
