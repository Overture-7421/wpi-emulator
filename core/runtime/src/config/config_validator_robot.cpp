#include <cmath>
#include <set>
#include <string>

#include "config_validator_steps.h"
#include "wpiemu/sdk/dynamics/motor_model.h"

namespace wpiemu::core::config::detail {

using namespace wpiemu::sdk;

namespace {

bool finitePositive(float v) { return std::isfinite(v) && v > 0.f; }
bool finiteNonNegative(float v) { return std::isfinite(v) && v >= 0.f; }

void validateTire(const TireParams& t, const std::string& base, std::vector<Issue>& out) {
    const auto positive = [&](float v, const char* field, const char* why) {
        if (!finitePositive(v)) add(out, Severity::Error, base + "." + field, why, issue_code::kTireInvalid);
    };
    positive(t.peakSlipLong, "peakSlipLong", "peak longitudinal slip must be positive and finite");
    positive(t.peakSlipLat, "peakSlipLat", "peak lateral slip must be positive and finite");
    positive(t.vRefMps, "vRefMps", "reference creep velocity must be positive and finite");
    positive(t.lateralScale, "lateralScale", "lateral scale must be positive and finite");
    if (!std::isfinite(t.slideRatio)) add(out, Severity::Error, base + ".slideRatio", "slide ratio must be finite", issue_code::kTireInvalid);
    if (!std::isfinite(t.tractionAngleRad))
        add(out, Severity::Error, base + ".tractionAngleRad", "traction angle must be finite", issue_code::kTireInvalid);
}

void validateMotor(const MotorSpec& m, const std::string& base, std::vector<Issue>& out) {
    for (const MotorSpecFault& f : motorSpecFaults(m))
        add(out, Severity::Error, base + "." + f.field, f.why, issue_code::kMotorInvalid);
}

}  // namespace

void validateChassis(const RobotConfig& c, std::vector<Issue>& out) {
    const ChassisSpec& ch = c.chassis;
    const auto bad = [&](const std::string& field, const std::string& why) {
        add(out, Severity::Error, "chassis." + field, why, issue_code::kChassisInvalid);
    };
    if (!finite(ch.sizeM) || ch.sizeM.x <= 0.f || ch.sizeM.y <= 0.f || ch.sizeM.z <= 0.f) bad("sizeM", "chassis size must be positive and finite");
    if (!finitePositive(ch.massKg)) bad("massKg", "chassis mass must be positive");
    if (!finitePositive(ch.comHeightM)) bad("comHeightM", "COM height must be positive");
    if (!finiteNonNegative(ch.groundClearanceM)) bad("groundClearanceM", "ground clearance must be >= 0");
    if (!finite(ch.comOffsetM)) bad("comOffsetM", "COM offset must be finite");
    if (!finite(ch.inertiaDiagKgM2) || ch.inertiaDiagKgM2.x < 0.f || ch.inertiaDiagKgM2.y < 0.f || ch.inertiaDiagKgM2.z < 0.f)
        bad("inertiaDiagKgM2", "inertia must be >= 0 (0 = derive from the box)");
    if (!finiteNonNegative(ch.material.friction) || !finiteNonNegative(ch.material.restitution))
        bad("material", "friction and restitution must be >= 0");
}

void validateDrivetrain(const RobotConfig& c, const DrivetrainRegistry& drivetrains, const TireModelRegistry* tires,
                        std::vector<Issue>& out) {
    const DrivetrainSpec& spec = c.drivetrain.spec;
    if (!drivetrains.contains(spec.kind))
        add(out, Severity::Error, "drivetrain.kind", "unknown drivetrain kind '" + spec.kind + "'", issue_code::kDrivetrainKindUnknown);
    if (tires != nullptr && !tires->contains(spec.tireModel))
        add(out, Severity::Error, "drivetrain.tireModel", "unknown tire model '" + spec.tireModel + "'", issue_code::kTireModelUnknown);
    if (spec.wheels.empty())
        add(out, Severity::Warning, "drivetrain.wheels", "the drivetrain has no wheels", issue_code::kDrivetrainNoWheels);

    std::set<std::string> names;
    for (std::size_t i = 0; i < spec.wheels.size(); ++i) {
        const WheelSpec& w = spec.wheels[i];
        const std::string base = "drivetrain.wheels[" + std::to_string(i) + "]";
        if (w.name.empty() || w.name.find('.') != std::string::npos)
            add(out, Severity::Error, base + ".name", "wheel name must be non-empty and must not contain '.'", issue_code::kWheelInvalid);
        else if (!names.insert(w.name).second)
            add(out, Severity::Error, base + ".name", "duplicate wheel name '" + w.name + "'", issue_code::kWheelDuplicate);
        if (!finitePositive(w.radiusM)) add(out, Severity::Error, base + ".radiusM", "wheel radius must be positive", issue_code::kWheelInvalid);
        if (!finite(w.positionM)) add(out, Severity::Error, base + ".positionM", "wheel position must be finite", issue_code::kWheelInvalid);
        if (w.driveMotorCount < 1 || w.driveMotorCount > kMaxMotorsPerAxis)
            add(out, Severity::Error, base + ".driveMotorCount",
                "drive motors per wheel must be in [1, " + std::to_string(kMaxMotorsPerAxis) + "]", issue_code::kWheelInvalid);
        if (!finitePositive(w.driveGearRatio)) add(out, Severity::Error, base + ".driveGearRatio", "gear ratio must be positive", issue_code::kWheelInvalid);
        if (!finitePositive(w.tire.mu)) add(out, Severity::Error, base + ".tire.mu", "friction coefficient must be positive", issue_code::kWheelInvalid);
        validateTire(w.tire, base + ".tire", out);
        validateMotor(w.driveMotor, base + ".driveMotor", out);
        if (isSteered(w)) {
            if (!finitePositive(w.steerGearRatio))
                add(out, Severity::Error, base + ".steerGearRatio", "steer gear ratio must be positive", issue_code::kWheelInvalid);
            validateMotor(w.steerMotor, base + ".steerMotor", out);
        }
    }
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
