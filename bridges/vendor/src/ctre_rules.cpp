#include "wpiemu/bridges/vendor/ctre_rules.h"

#include <optional>
#include <utility>

namespace wpiemu::bridges::vendor::ctre {

namespace {

using sdk::Capability;
using sdk::CapabilitySet;
using sdk::SignalDirection;
using sdk::SimDeviceRule;
using sdk::SimValueBinding;

SimValueBinding binding(std::string child, std::string simValue, std::string signalName, std::string unit,
                     double scale = 1.0, std::optional<SignalDirection> direction = std::nullopt,
                     std::string loopbackTo = {}) {
    SimValueBinding b;
    b.child = std::move(child);
    b.simValue = std::move(simValue);
    b.signal = std::move(signalName);
    b.unit = std::move(unit);
    b.scale = scale;
    b.direction = direction;
    b.loopbackTo = std::move(loopbackTo);
    return b;
}

SimDeviceRule base(std::string name, const char* family, const char* type, CapabilitySet caps, std::string regex,
                   int childGroup) {
    SimDeviceRule r;
    r.name = std::move(name);
    r.family = family;
    r.type = type;
    r.vendor = kVendor;
    r.bus = sdk::kBusUnknown;
    r.busIsStatic = false;  // SimDevice names carry no bus (ADR 0006)
    r.nameRegex = std::move(regex);
    r.idGroup = 1;
    r.childGroup = childGroup;
    r.capabilities = caps;
    return r;
}

// TalonFX and TalonFXS expose identical SimDevices ("Talon FX (v6)" / "Talon FXS (v6)").
SimDeviceRule talonFxLike(const char* ruleName, const char* type, const char* model) {
    const CapabilitySet caps = Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor |
                               Capability::VelocitySensor | Capability::LimitInputs | Capability::SupplyVoltage;
    const std::string deviceModel = std::string(model) + R"( \(v6\)\[(\d+)\])";
    SimDeviceRule r = base(ruleName, kFamilyPhoenix6, type, caps,
                           "(?:CANMotor|CANEncoder|CANDIO):" + deviceModel + "(?:/(Rotor Sensor|Fwd Limit|Rev Limit))?", 2);
    namespace sg = sdk::signal;
    r.bindings = {
        // root CANMotor: supplyVoltage is the only sim input; the rest is firmware output we read.
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "dutyCycle", sg::kDutyCycle, ""),
        binding("", "motorVoltage", sg::kMotorVoltage, "V", 1.0, std::nullopt, sg::kRotorVelocity),
        binding("", "torqueCurrent", sg::kTorqueCurrent, "A"),
        binding("", "supplyCurrent", sg::kSupplyCurrentOut, "A"),
        // rotor sensor: WE drive the rotor (ADR 0002); `position` is the readonly firmware-reported value.
        binding("Rotor Sensor", "rawPositionInput", sg::kRotorPosition, "rot"),
        binding("Rotor Sensor", "velocity", sg::kRotorVelocity, "rps"),
        binding("Rotor Sensor", "acceleration", sg::kRotorAcceleration, "rps2"),
        binding("Rotor Sensor", "position", vsignal::kRotorPositionReported, "rot"),
        // limit inputs are Bidir in HAL; the emulator only writes them.
        binding("Fwd Limit", "value", sg::kForwardLimit, "", 1.0, SignalDirection::Input),
        binding("Rev Limit", "value", sg::kReverseLimit, "", 1.0, SignalDirection::Input),
    };
    return r;
}

SimDeviceRule canCoderRule() {
    const CapabilitySet caps = Capability::PositionSensor | Capability::VelocitySensor | Capability::SupplyVoltage;
    SimDeviceRule r = base("ctre.cancoder", kFamilyPhoenix6, kTypeCanCoder, caps,
                           R"(CANEncoder:CANcoder \(v6\)\[(\d+)\])", 0);
    namespace sg = sdk::signal;
    r.bindings = {
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "rawPositionInput", sg::kPosition, "rot"),
        binding("", "velocity", sg::kVelocity, "rps"),
        binding("", "position", vsignal::kPositionReported, "rot"),
    };
    return r;
}

SimDeviceRule pigeon2Rule() {
    const CapabilitySet caps = Capability::Imu | Capability::SupplyVoltage;
    SimDeviceRule r = base("ctre.pigeon2", kFamilyPhoenix6, kTypePigeon2, caps, R"(CANGyro:Pigeon 2 \(v6\)\[(\d+)\])", 0);
    namespace sg = sdk::signal;
    r.bindings = {
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "rawYawInput", sg::kYaw, "rad", kDegToRad),
        binding("", "pitch", sg::kPitch, "rad", kDegToRad),
        binding("", "roll", sg::kRoll, "rad", kDegToRad),
        binding("", "angularVelX", sg::kAngularVelX, "rad/s", kDegToRad),
        binding("", "angularVelY", sg::kAngularVelY, "rad/s", kDegToRad),
        binding("", "angularVelZ", sg::kAngularVelZ, "rad/s", kDegToRad),
        binding("", "yaw", vsignal::kYawReported, "rad", kDegToRad),
    };
    return r;
}

SimDeviceRule canDleRule() {
    SimDeviceRule r = base("ctre.candle", kFamilyPhoenix6, kTypeCanDle, Capability::Led | Capability::SupplyVoltage,
                           R"(CAN:CANdle \(v6\)\[(\d+)\])", 0);
    namespace sg = sdk::signal;
    r.bindings = {
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "fiveVRail", vsignal::kFiveVRail, "V"),
        binding("", "outputCurrent", vsignal::kOutputCurrent, "A"),
        binding("", "temperature", vsignal::kTemperature, "degC"),
        binding("", "vbatModulation", vsignal::kVBatModulation, ""),
    };
    return r;
}

SimDeviceRule canRangeRule() {
    SimDeviceRule r = base("ctre.canrange", kFamilyPhoenix6, kTypeCanRange, Capability::Distance | Capability::SupplyVoltage,
                           R"(CAN:CANrange \(v6\)\[(\d+)\])", 0);
    namespace sg = sdk::signal;
    r.bindings = {
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "distance", sg::kDistance, "m"),
        // isDetected is computed by the firmware sim from `distance` and the robot's config (read-only for us).
        binding("", "isDetected", vsignal::kDetectedReported, ""),
    };
    return r;
}

SimDeviceRule canDiRule() {
    const CapabilitySet caps =
        Capability::DigitalInput | Capability::PositionSensor | Capability::VelocitySensor | Capability::SupplyVoltage;
    SimDeviceRule r = base("ctre.candi", kFamilyPhoenix6, kTypeCanDi, caps,
                           R"((?:CAN|CANDutyCycle|CANEncoder|CANDIO):CANdi \(v6\)\[(\d+)\](?:/(PWM1|PWM2|Quadrature|S1|S2))?)", 2);
    namespace sg = sdk::signal;
    r.bindings = {
        binding("", "supplyVoltage", sg::kSupplyVoltage, "V"),
        binding("", "requestedOutputCurrent", vsignal::kRequestedOutputCurrent, "A"),
        binding("", "outputCurrent", vsignal::kOutputCurrent, "A"),
        binding("PWM1", "position", vsignal::kPwm1Position, "rot"),
        binding("PWM1", "velocity", vsignal::kPwm1Velocity, "rps"),
        binding("PWM1", "connected", vsignal::kPwm1Connected, ""),
        binding("PWM1", "riseToRise", vsignal::kPwm1RiseToRise, "s"),
        binding("PWM1", "riseToFall", vsignal::kPwm1RiseToFall, "s"),
        binding("PWM2", "position", vsignal::kPwm2Position, "rot"),
        binding("PWM2", "velocity", vsignal::kPwm2Velocity, "rps"),
        binding("PWM2", "connected", vsignal::kPwm2Connected, ""),
        binding("PWM2", "riseToRise", vsignal::kPwm2RiseToRise, "s"),
        binding("PWM2", "riseToFall", vsignal::kPwm2RiseToFall, "s"),
        binding("Quadrature", "rawPositionInput", sg::kPosition, "rot"),
        binding("Quadrature", "velocity", sg::kVelocity, "rps"),
        binding("Quadrature", "position", vsignal::kPositionReported, "rot"),
        // S1/S2 are labelled kOutput by the wrapper but are the sim's inputs (closed = true): override.
        binding("S1", "value", sg::kDigitalIn, "", 1.0, SignalDirection::Input),
        binding("S2", "value", vsignal::kDigitalIn2, "", 1.0, SignalDirection::Input),
    };
    return r;
}

}  // namespace

std::vector<SimDeviceRule> phoenix6Rules() {
    return {
        talonFxLike("ctre.talonfx", kTypeTalonFx, "Talon FX"),
        talonFxLike("ctre.talonfxs", kTypeTalonFxs, "Talon FXS"),
        canCoderRule(),
        pigeon2Rule(),
        canDleRule(),
        canRangeRule(),
        canDiRule(),
    };
}

std::vector<SimDeviceRule> phoenix5Rules(const Phoenix5Options& options) {
    namespace sg = sdk::signal;
    const double cpr = options.quadratureCountsPerRev > 0.0 ? options.quadratureCountsPerRev : 4096.0;
    std::vector<SimDeviceRule> rules;

    // Names below come from the Phoenix 5 WPI wrappers as enumerated live in ADR 0002; the Phoenix 5 vendor
    // library is not available in this repository's caches, so value names/directions beyond the ADR table are
    // kept data-driven: unknown values are simply not bound, HAL directions are never overridden except limits.
    const auto motorRoot = [&](const char* name, const char* type, const char* model, CapabilitySet caps, bool withSensors) {
        const std::string pattern = withSensors ? "(?:CANMotor|CANEncoder|CANDIO):" + std::string(model) +
                                                      R"(\[(\d+)\](?:/(Quad Encoder|Fwd Limit|Rev Limit))?)"
                                                : "CANMotor:" + std::string(model) + R"(\[(\d+)\])";
        SimDeviceRule r = base(name, kFamilyPhoenix5, type, caps, pattern, withSensors ? 2 : 0);
        r.bindings = {
            binding("", "busVoltage", sg::kSupplyVoltage, "V"),
            binding("", "supplyCurrent", vsignal::kSupplyCurrentInput, "A"),
            binding("", "motorCurrent", vsignal::kMotorCurrentInput, "A"),
            binding("", "percentOutput", sg::kDutyCycle, ""),
            binding("", "motorOutputLeadVoltage", sg::kMotorVoltage, "V", 1.0, std::nullopt, sg::kVelocity),
        };
        if (withSensors) {
            r.bindings.push_back(binding("Quad Encoder", "rawPositionInput", sg::kPosition, "rot", 1.0 / cpr));
            r.bindings.push_back(binding("Quad Encoder", "velocity", sg::kVelocity, "rps", 10.0 / cpr));  // counts / 100 ms
            r.bindings.push_back(binding("Quad Encoder", "position", vsignal::kPositionReported, "rot", 1.0 / cpr));
            r.bindings.push_back(binding("Fwd Limit", "value", sg::kForwardLimit, "", 1.0, SignalDirection::Input));
            r.bindings.push_back(binding("Rev Limit", "value", sg::kReverseLimit, "", 1.0, SignalDirection::Input));
        }
        return r;
    };

    const CapabilitySet srxCaps = Capability::MotorOutput | Capability::PositionSensor | Capability::VelocitySensor |
                                  Capability::LimitInputs | Capability::SupplyVoltage;
    rules.push_back(motorRoot("ctre5.talonsrx", kTypeTalonSrx, "Talon SRX", srxCaps, true));
    {
        // Analog / pulse-width sensors: raw sensor units, exposed under their native names (no canonical mapping).
        SimDeviceRule r = base("ctre5.talonsrx.sensors", kFamilyPhoenix5, kTypeTalonSrx, srxCaps,
                               R"(CANEncoder:Talon SRX\[(\d+)\]/(Analog In|Pulse Width Input))", 2);
        rules.push_back(std::move(r));  // empty bindings => auto-bind "<child>/<value>"
    }
    rules.push_back(motorRoot("ctre5.victorspx", kTypeVictorSpx, "Victor SPX",
                              Capability::MotorOutput | Capability::SupplyVoltage, false));
    {
        // Legacy Pigeon IMU: names/directions not verified => discover-only (no capabilities), native names.
        SimDeviceRule r = base("ctre5.pigeonimu", kFamilyPhoenix5, kTypePigeonImu, CapabilitySet{},
                               R"(CANGyro:Pigeon IMU\[(\d+)\])", 0);
        rules.push_back(std::move(r));
    }
    return rules;
}

GuardOptions phoenix6GuardOptions() {
    GuardOptions g;
    const std::string anyV6 = R"(CAN\w*:[^/]+ \(v6\)\[\d+\](?:/.+)?)";
    g.guarded.push_back({anyV6, "rawYawInput", true});
    g.guarded.push_back({anyV6, "rawPositionInput", false});
    return g;
}

}  // namespace wpiemu::bridges::vendor::ctre
