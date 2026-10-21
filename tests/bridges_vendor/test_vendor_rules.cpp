#include <doctest/doctest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "fakes.h"
#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/bridges/vendor/vendor_bridges.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::vendor_tests;
namespace ctre = wpiemu::bridges::vendor::ctre;
namespace sg = wpiemu::sdk::signal;
using wpiemu::bridges::vendor::NullSimDeviceBackend;
using wpiemu::bridges::vendor::VendorSimBridgeOptions;
using wpiemu::bridges::vendor::VendorSimDeviceBridge;

namespace {

struct Rig {
    FakeSimBackend backend;
    FakeHost host;
    std::unique_ptr<SimDeviceBridge> bridge;

    explicit Rig(std::vector<SimDeviceRule> rules) {
        bridge = std::make_unique<SimDeviceBridge>("test", std::move(rules), backend);
    }
    void start() { REQUIRE(bridge->start(host).ok()); }
};

double outputOf(const std::vector<SignalSample>& samples, FakeHost& host, DeviceId id, const std::string& name) {
    const std::uint16_t idx = host.signalIndex(id, name);
    for (const auto& s : samples)
        if (s.device == id && s.signal == idx) return s.value;
    FAIL("signal not collected: " << name);
    return 0.0;
}

bool hasOutput(const std::vector<SignalSample>& samples, FakeHost& host, DeviceId id, const std::string& name) {
    const std::uint16_t idx = host.signalIndex(id, name);
    for (const auto& s : samples)
        if (s.device == id && s.signal == idx) return true;
    return false;
}

}  // namespace

TEST_CASE("vendor rules: every rule table compiles and never fabricates a bus") {
    auto all = ctre::phoenix6Rules();
    for (auto& r : ctre::phoenix5Rules()) all.push_back(r);
    REQUIRE_FALSE(all.empty());
    for (const auto& r : all) {
        CAPTURE(r.name);
        CHECK_FALSE(r.busIsStatic);
        CHECK(r.vendor == "ctre");
        CHECK_NOTHROW(std::regex(r.nameRegex, std::regex::ECMAScript));
        CHECK(r.idGroup == 1);
    }
    // And the SDK bridge accepts them.
    FakeSimBackend backend;
    SimDeviceBridge bridge("t", all, backend);
    FakeHost host;
    CHECK(bridge.start(host).ok());
}

TEST_CASE("vendor rules: TalonFX: tree grouped into one device with canonical signals and directions") {
    Rig rig(ctre::phoenix6Rules());
    const auto t = addTalonFx(rig.backend, 4);
    rig.start();
    rig.bridge->poll();

    REQUIRE(rig.host.announced.size() == 1);  // motor + rotor + 2 limits folded together
    const auto& [id, ann] = *rig.host.announced.begin();
    CHECK(ann.descriptor.family == "phoenix6");
    CHECK(ann.descriptor.type == "talonfx");
    CHECK(ann.descriptor.vendor == "ctre");
    CHECK(ann.descriptor.id == 4);
    CHECK(ann.descriptor.bus == wpiemu::sdk::kBusUnknown);  // never fabricated (docs/ARCHITECTURE.md)
    CHECK(ann.descriptor.nativeName == "CANMotor:Talon FX (v6)[4]");
    CHECK(ann.descriptor.capabilities.has(Capability::MotorOutput));
    CHECK(ann.descriptor.capabilities.has(Capability::RotorFeedback));
    CHECK(ann.descriptor.capabilities.has(Capability::LimitInputs));
    CHECK(ann.descriptor.capabilities.has(Capability::SupplyVoltage));

    const auto dir = [&](const char* n) {
        const SignalSpec* s = rig.host.signalSpec(id, n);
        REQUIRE(s != nullptr);
        return s->direction;
    };
    CHECK(dir(sg::kDutyCycle) == SignalDirection::Output);
    CHECK(dir(sg::kMotorVoltage) == SignalDirection::Output);
    CHECK(dir(ctre::vsignal::kTorqueCurrentReported) == SignalDirection::Output);
    CHECK(rig.host.signalSpec(id, sg::kTorqueCurrent) == nullptr);
    CHECK(dir(sg::kSupplyCurrentOut) == SignalDirection::Output);
    CHECK(dir(sg::kSupplyVoltage) == SignalDirection::Input);
    CHECK(dir(sg::kRotorPosition) == SignalDirection::Input);
    CHECK(dir(sg::kRotorVelocity) == SignalDirection::Input);
    CHECK(dir(sg::kRotorAcceleration) == SignalDirection::Input);
    CHECK(dir(ctre::vsignal::kRotorPositionReported) == SignalDirection::Output);
    CHECK(dir(sg::kForwardLimit) == SignalDirection::Input);  // HAL says Bidir, we only write it
    CHECK(dir(sg::kReverseLimit) == SignalDirection::Input);
    CHECK(rig.host.signalSpec(id, "init") == nullptr);  // helper values are not bridged
    CHECK(rig.host.signalSpec(id, "input") == nullptr);

    // robot -> sim
    rig.backend.poke(t.motor, "motorVoltage", 7.5);
    rig.backend.poke(t.motor, "dutyCycle", 0.625);
    rig.backend.poke(t.motor, "torqueCurrent", 33.0);
    rig.backend.poke(t.rotor, "position", 12.5);
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(outputOf(out, rig.host, id, sg::kMotorVoltage) == doctest::Approx(7.5));
    CHECK(outputOf(out, rig.host, id, sg::kDutyCycle) == doctest::Approx(0.625));
    CHECK(outputOf(out, rig.host, id, ctre::vsignal::kTorqueCurrentReported) ==
          doctest::Approx(33.0));
    CHECK(outputOf(out, rig.host, id, ctre::vsignal::kRotorPositionReported) == doctest::Approx(12.5));
    CHECK_FALSE(hasOutput(out, rig.host, id, sg::kRotorPosition));  // inputs are never read back as outputs

    // sim -> robot
    std::vector<SignalSample> in{
        {id, rig.host.signalIndex(id, sg::kRotorPosition), 3.25},
        {id, rig.host.signalIndex(id, sg::kRotorVelocity), 50.0},
        {id, rig.host.signalIndex(id, sg::kRotorAcceleration), -2.0},
        {id, rig.host.signalIndex(id, sg::kSupplyVoltage), 11.4},
        {id, rig.host.signalIndex(id, sg::kForwardLimit), 1.0},
        {id, rig.host.signalIndex(id, sg::kMotorVoltage), 99.0},  // an Output: must be ignored
    };
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(t.rotor, "rawPositionInput") == doctest::Approx(3.25));
    CHECK(rig.backend.get(t.rotor, "velocity") == doctest::Approx(50.0));
    CHECK(rig.backend.get(t.rotor, "acceleration") == doctest::Approx(-2.0));
    CHECK(rig.backend.get(t.motor, "supplyVoltage") == doctest::Approx(11.4));
    CHECK(rig.backend.get(t.fwd, "value") == doctest::Approx(1.0));
    CHECK(rig.backend.get(t.rev, "value") == doctest::Approx(0.0));
    CHECK(rig.backend.get(t.motor, "motorVoltage") == doctest::Approx(7.5));
}

TEST_CASE("vendor rules: Phoenix Java and C++ motor roots bind outputs with their rotor children") {
    bool cppMotorName = false;
    SUBCASE("Java motor root") {}
    SUBCASE("C++ motor root") {
        cppMotorName = true;
    }
    for (const char* model : {"Talon FX", "Talon FXS"}) {
        CAPTURE(model);
        Rig rig(ctre::phoenix6Rules());
        const auto tree = addTalonFx(rig.backend, 1, model, cppMotorName);
        rig.start();
        rig.bridge->poll();
        REQUIRE(rig.host.announced.size() == 1);
        const auto& [id, announcement] = *rig.host.announced.begin();
        CHECK(announcement.descriptor.bus == wpiemu::sdk::kBusUnknown);
        CHECK(announcement.descriptor.nativeName ==
              std::string(cppMotorName ? "CANMotor: " : "CANMotor:") + model + " (v6)[1]");
        REQUIRE(rig.host.signalSpec(id, sg::kMotorVoltage) != nullptr);
        REQUIRE(rig.host.signalSpec(id, sg::kDutyCycle) != nullptr);
        REQUIRE(rig.host.signalSpec(id, sg::kRotorPosition) != nullptr);
        rig.backend.poke(tree.motor, "motorVoltage", 3.0);
        rig.backend.poke(tree.motor, "dutyCycle", 0.25);
        std::vector<SignalSample> outputs;
        rig.bridge->collectOutputs(outputs);
        CHECK(outputOf(outputs, rig.host, id, sg::kMotorVoltage) == doctest::Approx(3.0));
        CHECK(outputOf(outputs, rig.host, id, sg::kDutyCycle) == doctest::Approx(0.25));
        const std::vector<SignalSample> inputs{
            {id, rig.host.signalIndex(id, sg::kRotorPosition), 2.0},
            {id, rig.host.signalIndex(id, sg::kSupplyVoltage), 11.4},
        };
        rig.bridge->applyInputs(inputs);
        CHECK(rig.backend.get(tree.rotor, "rawPositionInput") == doctest::Approx(2.0));
        CHECK(rig.backend.get(tree.motor, "supplyVoltage") == doctest::Approx(11.4));
    }
}

TEST_CASE("vendor rules: TalonFX loopback echoes motor voltage to rotor velocity") {
    Rig rig(ctre::phoenix6Rules());
    const auto t = addTalonFx(rig.backend, 9);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("talonfx", 9);
    rig.backend.poke(t.motor, "motorVoltage", 6.0);
    rig.bridge->applyLoopback(id);
    CHECK(rig.backend.get(t.rotor, "velocity") == doctest::Approx(6.0));
}

TEST_CASE("vendor rules: TalonFX vs TalonFXS names are distinguished, FXS mirrors the FX table") {
    Rig rig(ctre::phoenix6Rules());
    addTalonFx(rig.backend, 1, "Talon FX");
    addTalonFx(rig.backend, 2, "Talon FXS");
    rig.start();
    rig.bridge->poll();
    REQUIRE(rig.host.announced.size() == 2);
    const DeviceId fx = rig.host.idOf("talonfx", 1);
    const DeviceId fxs = rig.host.idOf("talonfxs", 2);
    CHECK(fx.valid());
    CHECK(fxs.valid());
    CHECK(rig.host.signalSpec(fxs, sg::kMotorVoltage) != nullptr);
    CHECK(rig.host.signalSpec(fxs, sg::kRotorPosition) != nullptr);
    CHECK(rig.host.announced.at(fxs).descriptor.nativeName == "CANMotor:Talon FXS (v6)[2]");
}

TEST_CASE("vendor rules: TalonFX tolerates missing value names and missing children") {
    Rig rig(ctre::phoenix6Rules());
    // Motor device without 'torqueCurrent' and without any child sensors/limits (older/newer Phoenix).
    const SimDeviceHandle motor = rig.backend.addDevice("CANMotor:Talon FX (v6)[7]",
                                                        {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                                                         {"dutyCycle", kOut},
                                                         {"motorVoltage", kOut},
                                                         {"someFutureValue", kOut}});
    rig.start();
    rig.bridge->poll();
    REQUIRE(rig.host.announced.size() == 1);
    const DeviceId id = rig.host.idOf("talonfx", 7);
    CHECK(rig.host.signalSpec(id, sg::kMotorVoltage) != nullptr);
    CHECK(rig.host.signalSpec(id, sg::kTorqueCurrent) == nullptr);
    CHECK(rig.host.signalSpec(id, sg::kRotorPosition) == nullptr);
    CHECK(rig.host.signalSpec(id, "someFutureValue") == nullptr);  // unbound values are ignored, not exposed

    // The rotor sensor child shows up later: the same device is re-announced with extra signals (append-only).
    const SimDeviceHandle rotor =
        rig.backend.addDevice("CANEncoder:Talon FX (v6)[7]/Rotor Sensor", {{"rawPositionInput", kIn}, {"velocity", kIn}});
    rig.bridge->poll();
    CHECK(rig.host.announced.size() == 1);
    CHECK(rig.host.signalSpec(id, sg::kRotorPosition) != nullptr);
    CHECK(rig.host.signalSpec(id, sg::kRotorAcceleration) == nullptr);  // 'acceleration' missing: just absent
    (void)motor;
    (void)rotor;
}

TEST_CASE("vendor rules: CANcoder maps position, velocity and supply voltage") {
    Rig rig(ctre::phoenix6Rules());
    const SimDeviceHandle cc = addCanCoder(rig.backend, 12);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("cancoder", 12);
    REQUIRE(id.valid());
    const auto& d = rig.host.announced.at(id).descriptor;
    CHECK(d.capabilities.has(Capability::PositionSensor));
    CHECK(d.capabilities.has(Capability::VelocitySensor));
    CHECK_FALSE(d.capabilities.has(Capability::MotorOutput));

    std::vector<SignalSample> in{{id, rig.host.signalIndex(id, sg::kPosition), 0.25},
                                 {id, rig.host.signalIndex(id, sg::kVelocity), -1.5},
                                 {id, rig.host.signalIndex(id, sg::kSupplyVoltage), 12.3}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(cc, "rawPositionInput") == doctest::Approx(0.25));
    CHECK(rig.backend.get(cc, "velocity") == doctest::Approx(-1.5));
    CHECK(rig.backend.get(cc, "supplyVoltage") == doctest::Approx(12.3));

    rig.backend.poke(cc, "position", 0.75);
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(outputOf(out, rig.host, id, ctre::vsignal::kPositionReported) == doctest::Approx(0.75));
}

TEST_CASE("vendor rules: Pigeon 2 converts degrees to canonical radians and back") {
    Rig rig(ctre::phoenix6Rules());
    const SimDeviceHandle pg = addPigeon2(rig.backend, 13);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("pigeon2", 13);
    REQUIRE(id.valid());
    CHECK(rig.host.announced.at(id).descriptor.capabilities.has(Capability::Imu));
    CHECK(rig.host.signalSpec(id, sg::kAccelX) == nullptr);  // CTRE sim has no accelerometer inputs

    const double pi = wpiemu::sdk::kPi;
    std::vector<SignalSample> in{{id, rig.host.signalIndex(id, sg::kYaw), pi / 2},
                                 {id, rig.host.signalIndex(id, sg::kPitch), pi / 6},
                                 {id, rig.host.signalIndex(id, sg::kRoll), -pi / 6},
                                 {id, rig.host.signalIndex(id, sg::kAngularVelX), pi},
                                 {id, rig.host.signalIndex(id, sg::kAngularVelY), 2 * pi},
                                 {id, rig.host.signalIndex(id, sg::kAngularVelZ), -pi}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(pg, "rawYawInput") == doctest::Approx(90.0));
    CHECK(rig.backend.get(pg, "pitch") == doctest::Approx(30.0));
    CHECK(rig.backend.get(pg, "roll") == doctest::Approx(-30.0));
    CHECK(rig.backend.get(pg, "angularVelX") == doctest::Approx(180.0));
    CHECK(rig.backend.get(pg, "angularVelY") == doctest::Approx(360.0));
    CHECK(rig.backend.get(pg, "angularVelZ") == doctest::Approx(-180.0));

    rig.backend.poke(pg, "yaw", 45.0);  // robot-visible yaw (degrees) -> rad
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(outputOf(out, rig.host, id, ctre::vsignal::kYawReported) == doctest::Approx(pi / 4));
}

TEST_CASE("vendor rules: CANdle is observed: supply/5V/current/temperature in, vbat modulation out") {
    Rig rig(ctre::phoenix6Rules());
    const SimDeviceHandle cd = addCanDle(rig.backend, 3);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("candle", 3);
    REQUIRE(id.valid());
    CHECK(rig.host.announced.at(id).descriptor.capabilities.has(Capability::Led));
    CHECK(rig.host.signalSpec(id, ctre::vsignal::kVBatModulation)->direction == SignalDirection::Output);
    CHECK(rig.host.signalSpec(id, ctre::vsignal::kFiveVRail)->direction == SignalDirection::Input);
    rig.backend.poke(cd, "vbatModulation", 0.8);
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(outputOf(out, rig.host, id, ctre::vsignal::kVBatModulation) == doctest::Approx(0.8));
    std::vector<SignalSample> in{{id, rig.host.signalIndex(id, ctre::vsignal::kTemperature), 41.0}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(cd, "temperature") == doctest::Approx(41.0));
}

TEST_CASE("vendor rules: CANrange: distance is the only writable measurement, detection is firmware-derived") {
    Rig rig(ctre::phoenix6Rules());
    const SimDeviceHandle cr = addCanRange(rig.backend, 21);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("canrange", 21);
    REQUIRE(id.valid());
    CHECK(rig.host.announced.at(id).descriptor.capabilities.has(Capability::Distance));
    CHECK(rig.host.signalSpec(id, sg::kDistance)->direction == SignalDirection::Input);
    CHECK(rig.host.signalSpec(id, ctre::vsignal::kDetectedReported)->direction == SignalDirection::Output);
    std::vector<SignalSample> in{{id, rig.host.signalIndex(id, sg::kDistance), 0.42}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(cr, "distance") == doctest::Approx(0.42));
}

TEST_CASE("vendor rules: CANdi: children are folded, S1/S2 are writable despite the kOutput label") {
    Rig rig(ctre::phoenix6Rules());
    FakeSimBackend& b = rig.backend;
    const SimDeviceHandle root = b.addDevice("CAN:CANdi (v6)[5]", {{"supplyVoltage", kIn, SignalType::Double, 12.0},
                                                                    {"requestedOutputCurrent", kIn},
                                                                    {"outputCurrent", kOut}});
    const SimDeviceHandle quad = b.addDevice("CANEncoder:CANdi (v6)[5]/Quadrature",
                                             {{"init", kOut, SignalType::Bool, 1}, {"position", kOut}, {"rawPositionInput", kIn}, {"velocity", kIn}});
    const SimDeviceHandle s1 = b.addDevice("CANDIO:CANdi (v6)[5]/S1", {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kOut, SignalType::Bool}});
    const SimDeviceHandle s2 = b.addDevice("CANDIO:CANdi (v6)[5]/S2", {{"init", kOut, SignalType::Bool, 1}, {"input", kOut, SignalType::Bool, 1}, {"value", kOut, SignalType::Bool}});
    const SimDeviceHandle pwm1 = b.addDevice("CANDutyCycle:CANdi (v6)[5]/PWM1", {{"position", kIn}, {"connected", kIn, SignalType::Bool, 1}, {"velocity", kIn}, {"riseToRise", kIn, SignalType::Double, 0.004}, {"riseToFall", kIn}});
    rig.start();
    rig.bridge->poll();
    REQUIRE(rig.host.announced.size() == 1);
    const DeviceId id = rig.host.idOf("candi", 5);
    REQUIRE(id.valid());
    CHECK(rig.host.announced.at(id).descriptor.capabilities.has(Capability::DigitalInput));

    std::vector<SignalSample> in{{id, rig.host.signalIndex(id, sg::kDigitalIn), 1.0},
                                 {id, rig.host.signalIndex(id, ctre::vsignal::kDigitalIn2), 1.0},
                                 {id, rig.host.signalIndex(id, sg::kPosition), 4.0},
                                 {id, rig.host.signalIndex(id, ctre::vsignal::kPwm1Position), 0.5}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(s1, "value") == doctest::Approx(1.0));
    CHECK(rig.backend.get(s2, "value") == doctest::Approx(1.0));
    CHECK(rig.backend.get(quad, "rawPositionInput") == doctest::Approx(4.0));
    CHECK(rig.backend.get(pwm1, "position") == doctest::Approx(0.5));
    (void)root;
}

TEST_CASE("vendor rules: Phoenix 5: Talon SRX quadrature scaling, limits, Victor SPX, legacy Pigeon is discover-only") {
    ctre::Phoenix5Options opts;
    opts.quadratureCountsPerRev = 2048.0;
    Rig rig(ctre::phoenix5Rules(opts));
    FakeSimBackend& b = rig.backend;
    const SimDeviceHandle srx = b.addDevice("CANMotor:Talon SRX[2]", {{"busVoltage", kIn, SignalType::Double, 12.0},
                                                                       {"supplyCurrent", kIn},
                                                                       {"motorCurrent", kIn},
                                                                       {"percentOutput", kOut},
                                                                       {"motorOutputLeadVoltage", kOut}});
    const SimDeviceHandle quad = b.addDevice("CANEncoder:Talon SRX[2]/Quad Encoder", {{"position", kOut}, {"rawPositionInput", kIn}, {"velocity", kIn}});
    const SimDeviceHandle analog = b.addDevice("CANEncoder:Talon SRX[2]/Analog In", {{"position", kOut}, {"rawPositionInput", kIn}});
    const SimDeviceHandle fwd = b.addDevice("CANDIO:Talon SRX[2]/Fwd Limit", {{"value", kBidir, SignalType::Bool}});
    const SimDeviceHandle victor = b.addDevice("CANMotor:Victor SPX[6]", {{"busVoltage", kIn, SignalType::Double, 12.0},
                                                                           {"percentOutput", kOut},
                                                                           {"motorOutputLeadVoltage", kOut}});
    const SimDeviceHandle pig = b.addDevice("CANGyro:Pigeon IMU[1]", {{"headingRaw", kIn}, {"heading", kOut}});
    rig.start();
    rig.bridge->poll();
    REQUIRE(rig.host.announced.size() == 3);

    const DeviceId s = rig.host.idOf("talonsrx", 2);
    REQUIRE(s.valid());
    CHECK(rig.host.announced.at(s).descriptor.family == "phoenix5");
    CHECK(rig.host.announced.at(s).descriptor.capabilities.has(Capability::MotorOutput));
    CHECK(rig.host.signalSpec(s, "Analog In/rawPositionInput") != nullptr);  // auto-bound under native names

    rig.backend.poke(srx, "motorOutputLeadVoltage", 6.0);
    rig.backend.poke(srx, "percentOutput", 0.5);
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(outputOf(out, rig.host, s, sg::kMotorVoltage) == doctest::Approx(6.0));
    CHECK(outputOf(out, rig.host, s, sg::kDutyCycle) == doctest::Approx(0.5));

    // 1 rev = 2048 counts, velocity in counts per 100 ms.
    std::vector<SignalSample> in{{s, rig.host.signalIndex(s, sg::kPosition), 2.0},
                                 {s, rig.host.signalIndex(s, sg::kVelocity), 10.0},
                                 {s, rig.host.signalIndex(s, sg::kForwardLimit), 1.0},
                                 {s, rig.host.signalIndex(s, sg::kSupplyVoltage), 10.5}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend.get(quad, "rawPositionInput") == doctest::Approx(4096.0));
    CHECK(rig.backend.get(quad, "velocity") == doctest::Approx(10.0 * 2048.0 / 10.0));
    CHECK(rig.backend.get(fwd, "value") == doctest::Approx(1.0));
    CHECK(rig.backend.get(srx, "busVoltage") == doctest::Approx(10.5));

    const DeviceId v = rig.host.idOf("victorspx", 6);
    REQUIRE(v.valid());
    CHECK(rig.host.announced.at(v).descriptor.capabilities.has(Capability::MotorOutput));
    CHECK_FALSE(rig.host.announced.at(v).descriptor.capabilities.has(Capability::PositionSensor));

    const DeviceId p = rig.host.idOf("pigeonimu", 1);
    REQUIRE(p.valid());
    CHECK(rig.host.announced.at(p).descriptor.capabilities == CapabilitySet{});
    CHECK(rig.host.signalSpec(p, "headingRaw") != nullptr);
    (void)analog;
    (void)victor;
    (void)pig;
}

TEST_CASE("vendor rules: unrelated devices and Phoenix 5 names are ignored by the Phoenix 6 table") {
    Rig rig(ctre::phoenix6Rules());
    rig.backend.addDevice("CANMotor:SPARK MAX[4]", {{"appliedOutput", kOut}});
    rig.backend.addDevice("Gyro:ADXRS450[0]", {{"angle", kIn}});
    rig.backend.addDevice("CANMotor:Talon SRX[3]", {{"busVoltage", kIn}});  // P5 name: not matched by the P6 table
    rig.start();
    rig.bridge->poll();
    CHECK(rig.host.announced.empty());
}

TEST_CASE("vendor rules: devices that disappear are retired and recreated devices are announced again") {
    Rig rig(ctre::phoenix6Rules());
    const TalonFxTree t = addTalonFx(rig.backend, 11);
    rig.start();
    rig.bridge->poll();
    REQUIRE(rig.host.announced.size() == 1);
    rig.backend.removeDevice(t.motor);
    rig.backend.removeDevice(t.rotor);
    rig.backend.removeDevice(t.fwd);
    rig.backend.removeDevice(t.rev);
    rig.bridge->poll();
    CHECK(rig.host.announced.empty());
    CHECK(rig.host.retired.size() == 1);
    addTalonFx(rig.backend, 11);
    rig.bridge->poll();
    CHECK(rig.host.announced.size() == 1);
}

TEST_CASE("vendor rules: same id on different device types are different physical devices") {
    Rig rig(ctre::phoenix6Rules());
    addTalonFx(rig.backend, 5);
    addCanCoder(rig.backend, 5);
    addPigeon2(rig.backend, 5);
    rig.start();
    rig.bridge->poll();
    CHECK(rig.host.announced.size() == 3);
    CHECK(rig.host.idOf("talonfx", 5) != rig.host.idOf("cancoder", 5));
}
