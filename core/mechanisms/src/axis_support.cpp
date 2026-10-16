#include "axis_support.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "wpiemu/sdk/core/units.h"

namespace wpiemu::core::mechanisms::detail {

namespace {

sdk::Error invalidProp(const std::string& key, const std::string& why) {
    return sdk::makeError(sdk::ErrorCode::InvalidArgument, "property '" + key + "' " + why);
}

bool parseMotion(const std::string& s, Motion& out) {
    if (s == "rotary") out = Motion::Rotary;
    else if (s == "arm") out = Motion::Arm;
    else if (s == "linear") out = Motion::Linear;
    else return false;
    return true;
}

sdk::PropertyUnit positionUnit(const AxisOptions& o) {
    if (o.selectableMotion) return sdk::PropertyUnit::None;
    return o.motion == Motion::Linear ? sdk::PropertyUnit::Meters : sdk::PropertyUnit::Radians;
}

}  // namespace

const char* motionKey(Motion m) {
    switch (m) {
        case Motion::Rotary: return "rotary";
        case Motion::Arm: return "arm";
        case Motion::Linear: return "linear";
    }
    return "rotary";
}

std::string propKey(std::string_view prefix, std::string_view base) {
    std::string out(prefix);
    out.append(base);
    if (!prefix.empty()) out[prefix.size()] = static_cast<char>(std::toupper(static_cast<unsigned char>(base.front())));
    return out;
}

double getDouble(const sdk::Value& props, const std::string& key, double fallback) {
    const sdk::Value* v = props.find(key);
    return v != nullptr && v->isNumber() ? v->asDouble() : fallback;
}
bool getBool(const sdk::Value& props, const std::string& key, bool fallback) {
    const sdk::Value* v = props.find(key);
    return v != nullptr && v->isBool() ? v->asBool() : fallback;
}
std::string getString(const sdk::Value& props, const std::string& key, const std::string& fallback) {
    const sdk::Value* v = props.find(key);
    return v != nullptr && v->isString() ? v->asString() : fallback;
}

sdk::PropertySpec doubleProp(std::string key, std::string label, std::string description, sdk::PropertyUnit unit,
                             double def, std::string group, double min, double max, bool advanced) {
    sdk::PropertySpec p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.description = std::move(description);
    p.type = sdk::PropertyType::Double;
    p.unit = unit;
    p.defaultValue = def;
    p.min = min;
    p.max = max;
    p.group = std::move(group);
    p.advanced = advanced;
    return p;
}

sdk::PropertySpec boolProp(std::string key, std::string label, std::string description, bool def, std::string group,
                           bool advanced) {
    sdk::PropertySpec p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.description = std::move(description);
    p.type = sdk::PropertyType::Bool;
    p.defaultValue = def;
    p.group = std::move(group);
    p.advanced = advanced;
    return p;
}

sdk::PropertySpec enumProp(std::string key, std::string label, std::string description, std::string def,
                           std::vector<sdk::EnumOption> options, std::string group, bool advanced) {
    sdk::PropertySpec p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.description = std::move(description);
    p.type = sdk::PropertyType::Enum;
    p.defaultValue = std::move(def);
    p.options = std::move(options);
    p.group = std::move(group);
    p.advanced = advanced;
    return p;
}

AxisSlotNames axisSlotNames(const AxisOptions& o) {
    AxisSlotNames n;
    for (int i = 0; i < std::max(o.maxMotors, 1); ++i)
        n.motors.push_back(propKey(o.prefix, i == 0 ? "motor" : "motor" + std::to_string(i + 1)));
    n.encoder = propKey(o.prefix, "encoder");
    n.velocity = propKey(o.prefix, "velocitySensor");
    n.reverseLimit = propKey(o.prefix, "reverseLimit");
    n.forwardLimit = propKey(o.prefix, "forwardLimit");
    return n;
}

std::vector<sdk::SlotSpec> axisSlots(const AxisOptions& o) {
    const AxisSlotNames names = axisSlotNames(o);
    std::vector<sdk::SlotSpec> out;
    for (std::size_t i = 0; i < names.motors.size(); ++i) {
        sdk::SlotSpec s;
        s.name = names.motors[i];
        s.label = o.title + (i == 0 ? "Motor" : "Motor " + std::to_string(i + 1));
        s.type = sdk::SlotType::Motor;
        s.required = i == 0 && o.motorRequired;
        s.description = i == 0 ? "Motor controller driving this axis; the simulator writes rotor position/velocity back."
                               : "Additional motor on the same gearbox (leave unmapped to mirror the first motor).";
        out.push_back(std::move(s));
    }
    if (o.encoderSlots) {
        out.push_back({names.encoder, o.title + "Position sensor", sdk::SlotType::PositionSensor, false,
                       "Encoder / CANcoder on the mechanism output (see the sensor ratio property)."});
        out.push_back({names.velocity, o.title + "Velocity sensor", sdk::SlotType::VelocitySensor, false,
                       "Velocity-only sensor on the mechanism output."});
    }
    if (o.limitSwitchSlots) {
        out.push_back({names.reverseLimit, o.title + "Reverse limit switch", sdk::SlotType::LimitSwitch, false,
                       "Closes at the lower (minimum position) stop."});
        out.push_back({names.forwardLimit, o.title + "Forward limit switch", sdk::SlotType::LimitSwitch, false,
                       "Closes at the upper (maximum position) stop."});
    }
    return out;
}

std::vector<sdk::PropertySpec> axisProperties(const AxisOptions& o, const MotorCatalog& motors) {
    using sdk::PropertyUnit;
    std::vector<sdk::PropertySpec> out;
    const auto k = [&](std::string_view base) { return propKey(o.prefix, base); };
    const std::string gMotor = o.title + "Motor";
    const std::string gDyn = o.title + "Dynamics";
    const std::string gLim = o.title + "Limits";
    const std::string gSen = o.title + "Sensors";

    std::vector<sdk::EnumOption> motorOptions;
    for (const MotorCatalogEntry& e : motors.entries()) motorOptions.push_back({e.key, e.label});
    out.push_back(enumProp(k("motorModel"), "Motor model", "WPILib DCMotor data of one motor.", o.motorModel,
                           std::move(motorOptions), gMotor));
    sdk::PropertySpec count;
    count.key = k("motorCount");
    count.label = "Motor count";
    count.description = "Identical motors on the gearbox (map extra ones to the follower slots, or leave them mirrored).";
    count.type = sdk::PropertyType::Int;
    count.unit = PropertyUnit::Count;
    count.defaultValue = o.motorCount;
    count.min = 1;
    count.max = 8;
    count.group = gMotor;
    out.push_back(std::move(count));
    out.push_back(doubleProp(k("gearRatio"), "Gear ratio", "Motor : mechanism output (> 1 reduces speed).",
                             PropertyUnit::Ratio, o.gearRatio, gMotor, 0.01, 10000.0));
    out.push_back(doubleProp(k("efficiency"), "Drivetrain efficiency", "Fraction of motor torque delivered to the output.",
                             PropertyUnit::Ratio, o.efficiency, gMotor, 0.05, 1.0, true));
    out.push_back(boolProp(k("motorInverted"), "Motor inverted",
                           "Positive command moves the mechanism in the negative direction.", false, gMotor));
    out.push_back(enumProp(k("neutralMode"), "Neutral mode", "Behaviour at zero output: brake shorts the motor, coast frees it.",
                           o.brake ? "brake" : "coast", {{"coast", "Coast"}, {"brake", "Brake"}}, gMotor));
    out.push_back(doubleProp(k("statorCurrentLimitA"), "Stator current limit",
                             "Per-motor current limit, 0 = unlimited.", PropertyUnit::Amps, 0.0, gMotor, 0.0, 500.0));
    out.push_back(boolProp(k("reportMotorLimits"), "Report limits on motor controller",
                           "Also set the motor controller's forward/reverse limit signals at the stops.", false, gMotor, true));

    const bool rotaryJ = o.selectableMotion || o.motion != Motion::Linear;
    const bool massy = o.selectableMotion || o.motion != Motion::Rotary;
    const bool arm = o.selectableMotion || o.motion == Motion::Arm;
    const bool linear = o.selectableMotion || o.motion == Motion::Linear;
    if (o.selectableMotion)
        out.push_back(enumProp(k("motion"), "Motion type", "Rotary spinner, gravity-loaded arm, or linear slide.",
                               motionKey(o.motion),
                               {{"rotary", "Rotary"}, {"arm", "Arm / pivot"}, {"linear", "Linear / elevator"}}, gDyn));
    if (rotaryJ)
        out.push_back(doubleProp(k("momentOfInertiaKgM2"), "Moment of inertia",
                                 o.motion == Motion::Arm || o.selectableMotion
                                     ? "About the rotation axis. Arm: 0 = uniform rod about its end, 1/3 m L^2."
                                     : "About the rotation axis.",
                                 PropertyUnit::KgM2, o.inertiaKgM2, gDyn, o.motion == Motion::Rotary && !o.selectableMotion ? 1e-6 : 0.0,
                                 100.0));
    if (massy)
        out.push_back(doubleProp(k("massKg"), "Mass", "Arm mass, or carriage + payload mass for a linear axis.",
                                 PropertyUnit::Kilograms, o.massKg, gDyn, 0.001, 2000.0));
    if (arm) {
        sdk::PropertySpec len = doubleProp(k("lengthM"), "Arm length", "Pivot to tip.", PropertyUnit::Meters, o.lengthM,
                                           gDyn, 0.001, 10.0);
        sdk::PropertySpec com = doubleProp(k("comFraction"), "Centre of mass",
                                           "Position of the centre of mass along the arm, 0 = pivot, 1 = tip.",
                                           PropertyUnit::Ratio, o.comFraction, gDyn, 0.0, 1.0, true);
        if (o.selectableMotion) {
            len.visibleWhenKey = com.visibleWhenKey = k("motion");
            len.visibleWhenEquals = com.visibleWhenEquals = "arm";
        }
        out.push_back(std::move(len));
        out.push_back(std::move(com));
    }
    if (linear) {
        sdk::PropertySpec drum = doubleProp(k("drumRadiusM"), "Drum radius", "Converts shaft rotation to travel.",
                                            PropertyUnit::Meters, o.drumRadiusM, gDyn, 1e-4, 1.0);
        if (o.selectableMotion) {
            drum.visibleWhenKey = k("motion");
            drum.visibleWhenEquals = "linear";
        }
        out.push_back(std::move(drum));
    }
    if (massy)
        out.push_back(boolProp(k("simulateGravity"), "Simulate gravity",
                               "Arm: torque m g L_cm cos(angle) with 0 rad = horizontal. Linear: constant -g (vertical).",
                               o.gravity, gDyn));
    out.push_back(doubleProp(k("viscousFrictionNmS"), "Viscous friction",
                             "Output-side damping: N*m*s/rad (rotary/arm) or N*s/m (linear).", PropertyUnit::None,
                             o.viscousFrictionNmS, gDyn, 0.0, 1000.0, true));
    out.push_back(doubleProp(k("coulombFrictionNm"), "Coulomb friction",
                             "Output-side dry friction: N*m (rotary/arm) or N (linear); also holds at rest.",
                             PropertyUnit::None, 0.0, gDyn, 0.0, 1000.0, true));

    if (o.limits) {
        const PropertyUnit u = positionUnit(o);
        out.push_back(doubleProp(k("minPosition"), "Minimum position", "Hard stop; equal to the maximum = no stops.", u,
                                 o.minPosition, gLim, -1e4, 1e4));
        out.push_back(doubleProp(k("maxPosition"), "Maximum position", "Hard stop; equal to the minimum = no stops.", u,
                                 o.maxPosition, gLim, -1e4, 1e4));
        out.push_back(doubleProp(k("startPosition"), "Start position", "Position after reset (inside the stops).", u,
                                 o.startPosition, gLim, -1e4, 1e4));
        if (o.limitSwitchSlots) {
            out.push_back(doubleProp(k("limitMargin"), "Limit switch margin", "Trigger band before the stop.", u, 1e-6,
                                     gLim, 0.0, 1.0, true));
            out.push_back(boolProp(k("limitPressedValue"), "Limit reads true when pressed",
                                   "Off for normally-closed wiring (reads false when pressed).", true, gLim, true));
        }
    }

    if (o.encoderSlots) {
        out.push_back(doubleProp(k("sensorRatio"), "Sensor ratio",
                                 "Sensor rotations per mechanism-output rotation (linear: per drum rotation).",
                                 PropertyUnit::Ratio, 1.0, gSen, 0.001, 10000.0));
        out.push_back(boolProp(k("sensorInverted"), "Sensor inverted", "Negate the position/velocity sensors.", false, gSen));
        out.push_back(doubleProp(k("sensorOffsetRot"), "Sensor offset", "Added to the sensor position, in rotations.",
                                 PropertyUnit::None, 0.0, gSen, -1e6, 1e6, true));
    }
    return out;
}

sdk::Result<AxisSettings> readAxisSettings(const AxisOptions& o, const sdk::Value& props, const MotorCatalog& motors) {
    const auto k = [&](std::string_view base) { return propKey(o.prefix, base); };
    AxisSettings s;
    s.motion = o.motion;
    if (o.selectableMotion) {
        const std::string text = getString(props, k("motion"), motionKey(o.motion));
        if (!parseMotion(text, s.motion)) return invalidProp(k("motion"), "has unknown value '" + text + "'");
    }

    const std::string motorKey = getString(props, k("motorModel"), o.motorModel);
    const sdk::MotorSpec* motor = motors.spec(motorKey);
    if (motor == nullptr) return invalidProp(k("motorModel"), "names unknown motor '" + motorKey + "'");

    sdk::PlantSpec& p = s.plant;
    p.motor = *motor;
    p.motorCount = static_cast<int>(std::lround(getDouble(props, k("motorCount"), o.motorCount)));
    p.gearRatio = getDouble(props, k("gearRatio"), o.gearRatio);
    p.efficiency = getDouble(props, k("efficiency"), o.efficiency);
    p.viscousFrictionNmS = getDouble(props, k("viscousFrictionNmS"), o.viscousFrictionNmS);
    p.coulombFrictionNm = getDouble(props, k("coulombFrictionNm"), 0.0);

    const double massKg = getDouble(props, k("massKg"), o.massKg);
    switch (s.motion) {
        case Motion::Rotary:
            p.model = o.rotaryModel;
            p.kind = sdk::PlantKind::Angular;
            p.momentOfInertiaKgM2 = getDouble(props, k("momentOfInertiaKgM2"), o.inertiaKgM2);
            break;
        case Motion::Arm: {
            p.model = plant_model::kArm;
            p.kind = sdk::PlantKind::Angular;
            p.massKg = massKg;
            const double length = getDouble(props, k("lengthM"), o.lengthM);
            const double inertia = getDouble(props, k("momentOfInertiaKgM2"), o.inertiaKgM2);
            p.momentOfInertiaKgM2 = inertia > 0.0 ? inertia : massKg * length * length / 3.0;
            p.armLengthM = getDouble(props, k("comFraction"), o.comFraction) * length;
            p.simulateGravity = getBool(props, k("simulateGravity"), o.gravity);
            break;
        }
        case Motion::Linear:
            p.model = plant_model::kElevator;
            p.kind = sdk::PlantKind::Linear;
            p.massKg = massKg;
            p.drumRadiusM = getDouble(props, k("drumRadiusM"), o.drumRadiusM);
            p.simulateGravity = getBool(props, k("simulateGravity"), o.gravity);
            break;
    }

    if (o.limits) {
        p.minPosition = getDouble(props, k("minPosition"), o.minPosition);
        p.maxPosition = getDouble(props, k("maxPosition"), o.maxPosition);
        s.startPosition = getDouble(props, k("startPosition"), o.startPosition);
        s.limited = p.maxPosition > p.minPosition;
        if (p.maxPosition < p.minPosition)
            return invalidProp(k("maxPosition"), "must not be below the minimum position");
        if (s.limited && (s.startPosition < p.minPosition || s.startPosition > p.maxPosition))
            return invalidProp(k("startPosition"), "must lie within the stops");
        s.limitMargin = getDouble(props, k("limitMargin"), 1e-6);
        s.limitPressedValue = getBool(props, k("limitPressedValue"), true);
    }

    if (const sdk::Status plantStatus = validatePlantSpec(p); !plantStatus.ok())
        return sdk::makeError(sdk::ErrorCode::InvalidArgument, plantStatus.error().message);

    s.statorCurrentLimitA = getDouble(props, k("statorCurrentLimitA"), 0.0);
    if (!(s.statorCurrentLimitA >= 0.0)) return invalidProp(k("statorCurrentLimitA"), "must be >= 0");
    s.neutral = getString(props, k("neutralMode"), o.brake ? "brake" : "coast") == "brake" ? NeutralMode::Brake
                                                                                              : NeutralMode::Coast;
    s.motorInverted = getBool(props, k("motorInverted"), false);
    s.reportMotorLimits = getBool(props, k("reportMotorLimits"), false);
    s.encoder.sensorRatio = getDouble(props, k("sensorRatio"), 1.0);
    s.encoder.inverted = getBool(props, k("sensorInverted"), false);
    s.encoder.offsetRot = getDouble(props, k("sensorOffsetRot"), 0.0);
    if (!(s.encoder.sensorRatio > 0.0)) return invalidProp(k("sensorRatio"), "must be > 0");
    return s;
}

MechanismAxis::MechanismAxis(std::string owner, AxisOptions options, AxisSettings settings,
                             std::unique_ptr<sdk::IPlant> plant, MotorDrive drive)
    : owner_(std::move(owner)), options_(std::move(options)), settings_(std::move(settings)),
      slots_(axisSlotNames(options_)), plant_(std::move(plant)), drive_(std::move(drive)) {
    if (!plant_) sdk::contractViolation("MechanismAxis: null plant");
    rotor_ = {settings_.plant.gearRatio, settings_.motorInverted};
    reverseSwitch_ = {settings_.plant.minPosition, LimitSwitchModel::Side::AtOrBelow, settings_.limitMargin,
                      settings_.limitPressedValue};
    forwardSwitch_ = {settings_.plant.maxPosition, LimitSwitchModel::Side::AtOrAbove, settings_.limitMargin,
                      settings_.limitPressedValue};
    reset();
}

void MechanismAxis::reset() {
    plant_->reset({settings_.startPosition, 0.0});
    appliedVoltageV_ = motorCurrentA_ = batteryCurrentA_ = 0.0;
}

double MechanismAxis::outputRotations(double position) const {
    const double rad = position * (plant_->kind() == sdk::PlantKind::Linear ? 1.0 / settings_.plant.drumRadiusM : 1.0);
    return rad / sdk::kTwoPi;
}

void MechanismAxis::tick(sdk::SimContext& ctx, double dt) {
    sdk::IDeviceHub& hub = ctx.devices();
    const double battery = hub.batteryVoltage();
    const double sign = settings_.motorInverted ? -1.0 : 1.0;
    const double omega = plant_->motorShaftOmegaRadPerSec();
    const std::size_t motorCount = static_cast<std::size_t>(settings_.plant.motorCount);

    const auto motorVoltage = [&](const sdk::MotorOutput& raw) {
        sdk::MotorOutput out = raw;
        out.voltageV *= sign;
        out.dutyCycle *= sign;
        out.torqueCurrentA *= sign;
        return drive_.voltage(out, omega, battery);
    };

    // Unmapped follower motors mirror the primary; mapped ones contribute their own voltage.
    // Motor currents are linear in the voltage, so the plant sees the mean over all motors.
    std::vector<double> volts;
    volts.reserve(motorCount);
    volts.push_back(motorVoltage(hub.motorOutput(address(slots_.motors.front()))));
    for (std::size_t i = 1; i < slots_.motors.size() && volts.size() < motorCount; ++i) {
        const sdk::MotorOutput out = hub.motorOutput(address(slots_.motors[i]));
        if (out.valid) volts.push_back(motorVoltage(out));
    }
    while (volts.size() < motorCount) volts.push_back(volts.front());

    double sum = 0.0, current = 0.0, draw = 0.0;
    for (const double v : volts) {
        const double i = drive_.currentA(v, omega);
        sum += v;
        current += std::abs(i);
        draw += std::max(0.0, v * i);
    }
    appliedVoltageV_ = sum / static_cast<double>(motorCount);
    motorCurrentA_ = current;
    batteryCurrentA_ = battery > 0.1 ? draw / battery : 0.0;

    const double previousVelocity = plant_->state().velocity;
    plant_->step({appliedVoltageV_, externalLoad_, dt});
    writeFeedback(hub, previousVelocity, dt);
}

void MechanismAxis::writeFeedback(sdk::ISensorWriter& hub, double previousVelocity, double dt) {
    const sdk::PlantState x = plant_->state();
    const double rot = outputRotations(x.position);
    const double rps = outputRotations(x.velocity);
    const double rps2 = dt > 0.0 ? outputRotations(x.velocity - previousVelocity) / dt : 0.0;

    sdk::MotorFeedback feedback = rotor_.sample(rot, rps, rps2);
    if (settings_.reportMotorLimits && settings_.limited) {
        feedback.forwardLimit = forwardSwitch_.pressed(x.position);
        feedback.reverseLimit = reverseSwitch_.pressed(x.position);
    }
    for (const std::string& slot : slots_.motors) hub.writeMotorFeedback(address(slot), feedback);

    if (options_.encoderSlots) {
        const sdk::EncoderFeedback e = settings_.encoder.sample(rot, rps);
        hub.writeEncoder(address(slots_.encoder), e);
        hub.writeEncoder(address(slots_.velocity), e);
    }
    if (options_.limitSwitchSlots) {
        const bool idle = !settings_.limitPressedValue;
        hub.writeDigital(address(slots_.reverseLimit), settings_.limited ? reverseSwitch_.reading(x.position) : idle);
        hub.writeDigital(address(slots_.forwardLimit), settings_.limited ? forwardSwitch_.reading(x.position) : idle);
    }
}

}  // namespace wpiemu::core::mechanisms::detail
