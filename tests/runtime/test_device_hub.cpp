#include <doctest/doctest.h>

#include <algorithm>

#include "wpiemu/core/runtime/device_hub.h"
#include "wpiemu/sdk/devices/signals.h"

using namespace wpiemu::sdk;
using wpiemu::core::runtime::DeviceHub;

namespace {

SignalSpec sig(const char* name, SignalDirection dir) { return SignalSpec{name, dir, SignalType::Double, ""}; }

// TalonFX-like: outputs duty/voltage/torque, inputs rotor + limits.
proto::DeviceAnnounce talon(std::uint32_t id, int canId, const std::string& bus = kBusUnknown) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{id};
    a.descriptor.family = "phoenix6";
    a.descriptor.type = "talonfx";
    a.descriptor.id = canId;
    a.descriptor.bus = bus;
    a.descriptor.capabilities = Capability::MotorOutput | Capability::RotorFeedback | Capability::PositionSensor;
    a.signalSpecs = {sig(signal::kDutyCycle, SignalDirection::Output),   sig(signal::kMotorVoltage, SignalDirection::Output),
                     sig(signal::kTorqueCurrent, SignalDirection::Output), sig(signal::kRotorPosition, SignalDirection::Input),
                     sig(signal::kRotorVelocity, SignalDirection::Input),  sig(signal::kRotorAcceleration, SignalDirection::Input),
                     sig(signal::kForwardLimit, SignalDirection::Bidirectional)};
    return a;
}

proto::DeviceAnnounce pwm(std::uint32_t id, int channel) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{id};
    a.descriptor.family = "hal";
    a.descriptor.type = "pwm";
    a.descriptor.id = channel;
    a.descriptor.bus = "rio";
    a.descriptor.capabilities = CapabilitySet(Capability::MotorOutput) | CapabilitySet(Capability::PwmOutput);
    a.signalSpecs = {sig(signal::kPwmSpeed, SignalDirection::Output)};
    return a;
}

proto::DeviceAnnounce pigeon(std::uint32_t id, int canId) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{id};
    a.descriptor.family = "phoenix6";
    a.descriptor.type = "pigeon2";
    a.descriptor.id = canId;
    a.descriptor.bus = kBusUnknown;
    a.descriptor.capabilities = CapabilitySet(Capability::Imu);
    for (const char* n : {signal::kYaw, signal::kPitch, signal::kRoll, signal::kAngularVelX, signal::kAngularVelY, signal::kAngularVelZ,
                          signal::kAccelX, signal::kAccelY, signal::kAccelZ})
        a.signalSpecs.push_back(sig(n, SignalDirection::Input));
    return a;
}

proto::DeviceAnnounce dio(std::uint32_t id, int channel) {
    proto::DeviceAnnounce a;
    a.id = DeviceId{id};
    a.descriptor.family = "hal";
    a.descriptor.type = "dio";
    a.descriptor.id = channel;
    a.descriptor.bus = "rio";
    a.descriptor.capabilities = CapabilitySet(Capability::DigitalInput) | CapabilitySet(Capability::DigitalOutput);
    a.signalSpecs = {sig(signal::kDigitalIn, SignalDirection::Input), sig(signal::kDigitalOut, SignalDirection::Output)};
    return a;
}

proto::ActuatorFrame frame(std::initializer_list<SignalSample> samples) {
    proto::ActuatorFrame f;
    f.samples = samples;
    return f;
}

bool hasWarning(const DeviceHub& hub, const char* code) {
    const auto w = hub.warnings();
    return std::any_of(w.begin(), w.end(), [&](const HubWarning& x) { return x.code == code; });
}

const DeviceKey kTalon1{"phoenix6", "talonfx", 1, "rio", -1};
const SlotAddress kSlotA{"arm", "motor"};
const SlotAddress kSlotB{"arm", "encoder"};

}  // namespace

TEST_CASE("hub: an announced, unmapped device is looped back with a warning until it is bound") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    REQUIRE(hub.discovered().size() == 1);
    CHECK_FALSE(hub.discovered()[0].mapped);
    CHECK(hub.loopbackSet().loopback == std::vector<DeviceId>{DeviceId{1}});
    CHECK(hasWarning(hub, "unmapped_loopback"));
    CHECK_FALSE(hub.isMapped(DeviceId{1}));

    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    CHECK(hub.isMapped(DeviceId{1}));
    CHECK(hub.loopbackSet().loopback.empty());
    CHECK_FALSE(hasWarning(hub, "unmapped_loopback"));  // derived warnings clear themselves
    REQUIRE(hub.resolve(kSlotA).has_value());
    CHECK(*hub.resolve(kSlotA) == DeviceId{1});

    hub.unbind(kSlotA);
    CHECK(hasWarning(hub, "unmapped_loopback"));
    CHECK_FALSE(hub.resolve(kSlotA).has_value());
}

TEST_CASE("hub: the declared bus never takes part in resolving a device") {
    DeviceHub hub;
    hub.onAnnounce(talon(7, 4));
    const DeviceKey onCanivore{"phoenix6", "talonfx", 4, "canivore", -1};
    const DeviceKey onRio{"phoenix6", "talonfx", 4, "rio", -1};
    REQUIRE(hub.find(onCanivore) != nullptr);
    CHECK(hub.find(onCanivore) == hub.find(onRio));
    CHECK(hub.find(onRio)->id == DeviceId{7});
    CHECK(hub.findPhysical(onRio.physical()) == hub.find(onRio));
    CHECK(hub.find(DeviceKey{"phoenix6", "talonfx", 5, "rio", -1}) == nullptr);
    CHECK(hub.find(DeviceId{99}) == nullptr);
    CHECK(hub.find(DeviceId{7}) != nullptr);
}

TEST_CASE("hub: saved mappings bind before the device exists and take effect when it appears") {
    DeviceHub hub;
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    CHECK_FALSE(hub.resolve(kSlotA).has_value());
    CHECK(hub.motorOutput(kSlotA).valid == false);
    hub.onAnnounce(talon(3, 1));
    CHECK(hub.discovered()[0].mapped);
    CHECK(*hub.resolve(kSlotA) == DeviceId{3});
    CHECK(hub.loopbackSet().loopback.empty());
}

TEST_CASE("hub: one device, one slot, rebinding the same pair is a no-op") {
    DeviceHub hub;
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    const auto again = hub.bind(kSlotA, kTalon1);
    CHECK(again.ok());
    const auto stolen = hub.bind(kSlotB, kTalon1);
    REQUIRE_FALSE(stolen.ok());
    CHECK(stolen.error().code == ErrorCode::AlreadyExists);
    CHECK(hub.bind(SlotAddress{"", "x"}, kTalon1).error().code == ErrorCode::InvalidArgument);
    REQUIRE(hub.bind(kSlotA, DeviceKey{"phoenix6", "talonfx", 2, "rio", -1}).ok());  // slot re-pointed
    CHECK(hub.boundDevice(kSlotA)->id == 2);
    CHECK_FALSE(hub.boundDevice(kSlotB).has_value());
}

TEST_CASE("hub: actuator frames become typed motor output (voltage falls back to duty x battery)") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    hub.onAnnounce(pwm(2, 0));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    REQUIRE(hub.bind({"drive", "motor"}, DeviceKey{"hal", "pwm", 0, "rio", -1}).ok());
    hub.setBatteryVoltage(12.0);
    CHECK(hub.batteryVoltage() == 12.0);

    CHECK_FALSE(hub.motorOutput(kSlotA).valid);  // nothing received yet

    hub.onActuatorFrame(frame({{DeviceId{1}, 0, 0.5}, {DeviceId{1}, 1, 5.5}, {DeviceId{2}, 0, -0.25}}));
    const MotorOutput talonOut = hub.motorOutput(kSlotA);
    CHECK(talonOut.valid);
    CHECK(talonOut.dutyCycle == 0.5);
    CHECK(talonOut.voltageV == 5.5);  // the device's own motorVoltage wins over duty x battery
    CHECK_FALSE(talonOut.hasTorqueCurrent);

    const MotorOutput pwmOut = hub.motorOutput({"drive", "motor"});
    CHECK(pwmOut.valid);
    CHECK(pwmOut.dutyCycle == -0.25);
    CHECK(pwmOut.voltageV == doctest::Approx(-3.0));  // no voltage signal: duty * battery

    hub.onActuatorFrame(frame({{DeviceId{1}, 2, 12.5}}));
    CHECK(hub.motorOutput(kSlotA).hasTorqueCurrent);
    CHECK(hub.motorOutput(kSlotA).torqueCurrentA == 12.5);
    CHECK(hub.motorOutput(kSlotA).dutyCycle == 0.5);  // partial frame keeps earlier values
}

TEST_CASE("hub: digital and solenoid outputs") {
    DeviceHub hub;
    hub.onAnnounce(dio(5, 3));
    REQUIRE(hub.bind({"claw", "out"}, DeviceKey{"hal", "dio", 3, "rio", -1}).ok());
    CHECK_FALSE(hub.digitalOutput({"claw", "out"}));
    hub.onActuatorFrame(frame({{DeviceId{5}, 1, 1.0}}));
    CHECK(hub.digitalOutput({"claw", "out"}));
    hub.onActuatorFrame(frame({{DeviceId{5}, 1, 0.0}}));
    CHECK_FALSE(hub.digitalOutput({"claw", "out"}));
    CHECK_FALSE(hub.digitalOutput({"nobody", "out"}));
}

TEST_CASE("hub: invalid actuator samples are dropped and counted, never crash") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    hub.onActuatorFrame(frame({{DeviceId{42}, 0, 1.0},     // unknown device
                               {DeviceId{1}, 99, 1.0},     // signal index out of range
                               {DeviceId{1}, 3, 1.0}}));   // rotorPosition is an Input signal: the robot cannot write it
    CHECK(hub.droppedSamples() == 3);
    CHECK_FALSE(hub.motorOutput(kSlotA).valid);
}

TEST_CASE("hub: sensor writes are delta-compressed, full frames resend everything written") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());

    proto::SensorFrame out;
    hub.setBatteryVoltage(11.5);
    hub.buildSensorFrame(out, false);
    CHECK(out.samples.empty());
    CHECK(out.batteryVoltageV == 11.5);

    MotorFeedback fb;
    fb.rotorPositionRot = 1.5;
    fb.rotorVelocityRps = 2.0;
    fb.forwardLimit = true;
    hub.writeMotorFeedback(kSlotA, fb);
    hub.buildSensorFrame(out, false);
    REQUIRE(out.samples.size() == 4);  // position, velocity, acceleration, forwardLimit
    auto valueOf = [&](std::uint16_t index) {
        for (const auto& s : out.samples)
            if (s.signal == index) return s.value;
        return -999.0;
    };
    CHECK(valueOf(3) == 1.5);
    CHECK(valueOf(4) == 2.0);
    CHECK(valueOf(6) == 1.0);
    for (const auto& s : out.samples) CHECK(s.device == DeviceId{1});

    hub.buildSensorFrame(out, false);
    CHECK(out.samples.empty());  // nothing changed

    fb.rotorPositionRot = 1.75;
    hub.writeMotorFeedback(kSlotA, fb);
    hub.buildSensorFrame(out, false);
    REQUIRE(out.samples.size() == 1);
    CHECK(out.samples[0].signal == 3);

    hub.buildSensorFrame(out, true);
    CHECK(out.full);
    CHECK(out.samples.size() == 4);  // every signal ever written
    CHECK(valueOf(3) == 1.75);
}

TEST_CASE("hub: writes to an unbound or undiscovered slot are ignored") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    hub.writeMotorFeedback(kSlotA, MotorFeedback{});  // not bound
    REQUIRE(hub.bind(kSlotB, DeviceKey{"phoenix6", "talonfx", 8, "rio", -1}).ok());  // bound, device never appears
    hub.writeEncoder(kSlotB, EncoderFeedback{1.0, 1.0});
    proto::SensorFrame out;
    hub.buildSensorFrame(out, true);
    CHECK(out.samples.empty());
}

TEST_CASE("hub: epoch reset invalidates outputs and sends neutral feedback in a full frame") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    hub.onAnnounce(dio(2, 3));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    const SlotAddress digital{"claw", "out"};
    REQUIRE(hub.bind(digital, DeviceKey{"hal", "dio", 3, "rio", -1}).ok());
    hub.markDiscoveryComplete();
    const auto mappings = hub.exportMappings();
    const auto revision = hub.mappingRevision();
    MotorFeedback feedback;
    feedback.rotorPositionRot = 7.0;
    feedback.rotorVelocityRps = 9.0;
    feedback.forwardLimit = true;
    hub.writeMotorFeedback(kSlotA, feedback);
    hub.writeDigital(digital, true);
    hub.onActuatorFrame(
        frame({{DeviceId{1}, 1, 12.0}, {DeviceId{1}, 2, 5.0}, {DeviceId{2}, 1, 1.0}}));
    proto::SensorFrame out;
    hub.buildSensorFrame(out, false);
    REQUIRE_FALSE(out.samples.empty());
    REQUIRE(hub.motorOutput(kSlotA).valid);
    REQUIRE(hub.digitalOutput(digital));

    hub.resetSignalHistory();
    CHECK_FALSE(hub.motorOutput(kSlotA).valid);
    CHECK(hub.motorOutput(kSlotA).voltageV == 0.0);
    CHECK_FALSE(hub.digitalOutput(digital));
    CHECK(hub.exportMappings() == mappings);
    CHECK(hub.mappingRevision() == revision);
    CHECK(hub.discoveryComplete());
    REQUIRE(hub.discovered().size() == 2);
    CHECK(hub.find(DeviceId{1})->signalSpecs == talon(1, 1).signalSpecs);
    hub.buildSensorFrame(out, false);
    CHECK(out.full);
    REQUIRE(out.samples.size() == 5);
    for (const auto& sample : out.samples) CHECK(sample.value == 0.0);
    hub.buildSensorFrame(out, false);
    CHECK_FALSE(out.full);
    CHECK(out.samples.empty());
    // A partial fresh output must not resurrect an old voltage/torque/digital value.
    hub.onActuatorFrame(frame({{DeviceId{1}, 0, 0.25}}));
    CHECK(hub.motorOutput(kSlotA).valid);
    CHECK(hub.motorOutput(kSlotA).dutyCycle == 0.25);
    CHECK(hub.motorOutput(kSlotA).voltageV == 0.0);
    CHECK_FALSE(hub.motorOutput(kSlotA).hasTorqueCurrent);
    CHECK_FALSE(hub.digitalOutput(digital));

    hub.writeMotorFeedback(kSlotA, MotorFeedback{});
    hub.resetSignalHistory();
    hub.buildSensorFrame(out, false);
    CHECK(out.full);
    for (const auto& sample : out.samples) CHECK(sample.value == 0.0);
}
TEST_CASE("hub: encoder / digital / analog / IMU writes land on the canonical signals") {
    DeviceHub hub;
    hub.onAnnounce(pigeon(1, 20));
    hub.onAnnounce(dio(2, 4));
    REQUIRE(hub.bind({"robot", "imu"}, DeviceKey{"phoenix6", "pigeon2", 20, "rio", -1}).ok());
    REQUIRE(hub.bind({"intake", "beam"}, DeviceKey{"hal", "dio", 4, "rio", -1}).ok());

    ImuFeedback imu;
    imu.yawRad = 0.5;
    imu.pitchRad = 0.1;
    imu.angularVelocityRadPerSec = {1.f, 2.f, 3.f};
    imu.accelerationMps2 = {0.f, 0.f, 9.8f};
    hub.writeImu({"robot", "imu"}, imu);
    hub.writeDigital({"intake", "beam"}, true);
    proto::SensorFrame out;
    hub.buildSensorFrame(out, false);
    CHECK(out.samples.size() == 10);  // 9 IMU + 1 digital
    auto value = [&](DeviceId d, std::uint16_t s) {
        for (const auto& x : out.samples)
            if (x.device == d && x.signal == s) return x.value;
        return -999.0;
    };
    CHECK(value(DeviceId{1}, 0) == 0.5);   // yaw
    CHECK(value(DeviceId{1}, 5) == 3.0);   // angularVelZ
    CHECK(value(DeviceId{1}, 8) == doctest::Approx(9.8));
    CHECK(value(DeviceId{2}, 0) == 1.0);
}

TEST_CASE("hub: aliases of one physical device mapped twice are disabled (conflict), one mapping is only a warning") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    const DeviceKey onRio{"phoenix6", "talonfx", 1, "rio", -1};
    const DeviceKey onCanivore{"phoenix6", "talonfx", 1, "canivore", -1};
    REQUIRE(hub.bind(kSlotA, onRio).ok());
    REQUIRE(hub.bind(kSlotB, onCanivore).ok());  // accepted at runtime, the validator is what rejects it

    const DiscoveredDevice* d = hub.find(DeviceId{1});
    REQUIRE(d != nullptr);
    CHECK(d->aliasState == AliasState::Conflict);
    CHECK_FALSE(d->mapped);
    CHECK(d->aliases.size() == 2);
    CHECK(hasWarning(hub, validation_code::kDeviceAliasConflict));
    CHECK_FALSE(hasWarning(hub, "unmapped_loopback"));
    CHECK(hub.loopbackSet().loopback == std::vector<DeviceId>{DeviceId{1}});
    CHECK_FALSE(hub.resolve(kSlotA).has_value());

    hub.onActuatorFrame(frame({{DeviceId{1}, 0, 1.0}}));
    CHECK_FALSE(hub.motorOutput(kSlotA).valid);  // neutral
    hub.writeMotorFeedback(kSlotA, MotorFeedback{1.0, 1.0, 0.0, {}, {}});
    proto::SensorFrame out;
    hub.buildSensorFrame(out, true);
    CHECK(out.samples.empty());  // writes ignored

    hub.unbind(kSlotB);  // the config was fixed: the device comes back to life
    CHECK(hub.find(DeviceId{1})->aliasState == AliasState::None);
    CHECK(hub.isMapped(DeviceId{1}));
    CHECK_FALSE(hasWarning(hub, validation_code::kDeviceAliasConflict));
    CHECK(hub.motorOutput(kSlotA).valid);
}

TEST_CASE("hub: declared aliases from the robot program give Aliased + bus mismatch warnings") {
    DeviceHub hub;
    hub.onAnnounce(pwm(1, 0));  // announces itself on "rio"
    REQUIRE(hub.bind({"a", "motor"}, DeviceKey{"hal", "pwm", 0, "rio", -1}).ok());
    CHECK(hub.find(DeviceId{1})->aliasState == AliasState::None);

    hub.onAnnounce(talon(2, 9));
    REQUIRE(hub.bind({"b", "motor"}, DeviceKey{"phoenix6", "talonfx", 9, "rio", -1}).ok());
    hub.observeBus(DeviceId{2}, "canivore");
    CHECK(hub.find(DeviceId{2})->observedBus == "canivore");
    CHECK(hasWarning(hub, validation_code::kDeviceBusMismatch));
    CHECK(hub.isMapped(DeviceId{2}));  // a warning only: traffic flows

    // same physical device announced on a second bus by the robot + one mapping => Aliased
    proto::DeviceAnnounce second = pwm(1, 0);
    second.descriptor.bus = "other";
    hub.onAnnounce(second);
    // re-announce replaces the declared address; still one declared key + the mapping's key => 2 distinct, 1 mapping
    CHECK(hub.find(DeviceId{1})->aliasState == AliasState::Aliased);
    CHECK(hasWarning(hub, validation_code::kDeviceAliased));
    CHECK(hub.isMapped(DeviceId{1}));

    DeviceHub other;
    other.onAnnounce(talon(5, 5));
    other.observeBus(DeviceId{5}, "canivore");
    other.observeBus(DeviceId{404}, "x");  // unknown device: ignored
    CHECK(other.find(DeviceId{5})->observedBus == "canivore");
    other.onAnnounce([&] {
        auto a = talon(5, 5);
        a.observedBus = "rio";
        return a;
    }());
    CHECK(other.find(DeviceId{5})->observedBus == "rio");
}

TEST_CASE("hub: retired devices leave the loopback set and stop resolving, re-announce revives them") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    hub.onAnnounce(pwm(2, 0));
    CHECK(hub.loopbackSet().loopback == std::vector<DeviceId>{DeviceId{2}});

    auto retired = pwm(2, 0);
    retired.retired = true;
    hub.onAnnounce(retired);
    CHECK_FALSE(hub.find(DeviceId{2})->live);
    CHECK(hub.loopbackSet().loopback.empty());
    CHECK_FALSE(hasWarning(hub, "unmapped_loopback"));

    auto retiredMapped = talon(1, 1);
    retiredMapped.retired = true;
    hub.onAnnounce(retiredMapped);
    CHECK_FALSE(hub.resolve(kSlotA).has_value());

    hub.onAnnounce(pwm(2, 0));
    CHECK(hub.find(DeviceId{2})->live);
    CHECK(hub.discovered().size() == 2);  // idempotent: no duplicate entry
}

TEST_CASE("hub: clearDevices forgets the robot's devices but keeps the mappings, a new session id works") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    hub.onActuatorFrame(frame({{DeviceId{1}, 0, 1.0}}));
    hub.clearDevices();
    CHECK(hub.discovered().empty());
    CHECK(hub.boundDevice(kSlotA).has_value());
    CHECK_FALSE(hub.motorOutput(kSlotA).valid);
    CHECK(hub.loopbackSet().loopback.empty());

    hub.onAnnounce(talon(77, 1));  // robot restarted: same physical device, new runtime id
    CHECK(*hub.resolve(kSlotA) == DeviceId{77});
    CHECK_FALSE(hub.motorOutput(kSlotA).valid);  // outputs of the previous run are gone
    hub.onActuatorFrame(frame({{DeviceId{77}, 0, 0.3}}));
    CHECK(hub.motorOutput(kSlotA).dutyCycle == 0.3);
}

TEST_CASE("hub: a re-announce of the same physical device under a new id re-keys it in place") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    hub.onAnnounce(talon(9, 1));
    REQUIRE(hub.discovered().size() == 1);
    CHECK(hub.discovered()[0].id == DeviceId{9});
    CHECK(hub.find(DeviceId{1}) == nullptr);
}

TEST_CASE("hub: slot_missing_device only after discovery completed, slot_incompatible by declared slot types") {
    DeviceHub hub;
    REQUIRE(hub.bind(kSlotA, kTalon1).ok());
    CHECK_FALSE(hasWarning(hub, "slot_missing_device"));
    hub.markDiscoveryComplete();
    CHECK(hub.discoveryComplete());
    CHECK(hasWarning(hub, "slot_missing_device"));
    hub.onAnnounce(talon(1, 1));
    CHECK_FALSE(hasWarning(hub, "slot_missing_device"));

    hub.declareSlots("arm", {{"motor", "Motor", SlotType::Motor, true, ""}, {"encoder", "Enc", SlotType::LimitSwitch, false, ""}});
    CHECK_FALSE(hasWarning(hub, "slot_incompatible"));
    hub.unbind(kSlotA);
    REQUIRE(hub.bind(kSlotB, kTalon1).ok());  // a TalonFX cannot be a limit switch (no DigitalInput capability)
    CHECK(hasWarning(hub, "slot_incompatible"));
    hub.clearSlotDeclarations();
    CHECK_FALSE(hasWarning(hub, "slot_incompatible"));
}

TEST_CASE("hub: persisted mappings round trip and the revision tracks loopback changes") {
    DeviceHub hub;
    hub.onAnnounce(talon(1, 1));
    const std::uint64_t r0 = hub.mappingRevision();
    const std::vector<SlotMapping> saved = {{kSlotA, kTalon1}, {kSlotB, DeviceKey{"phoenix6", "talonfx", 2, "rio", -1}}};
    REQUIRE(hub.loadMappings(saved).ok());
    CHECK(hub.mappingRevision() > r0);  // device 1 left the loopback set
    CHECK(hub.exportMappings().size() == 2);
    const auto exported = hub.exportMappings();  // ordered by slot address
    REQUIRE(exported.size() == 2);
    CHECK(std::find(exported.begin(), exported.end(), saved[0]) != exported.end());
    CHECK(std::find(exported.begin(), exported.end(), saved[1]) != exported.end());

    const std::uint64_t r1 = hub.mappingRevision();
    hub.unbind(kSlotB);  // device 2 was never discovered: loopback set unchanged
    CHECK(hub.mappingRevision() == r1);

    DeviceHub partial;
    const auto status = partial.loadMappings(std::vector<SlotMapping>{{kSlotA, kTalon1}, {kSlotB, kTalon1}, {{"c", "motor"}, DeviceKey{"hal", "pwm", 1, "rio", -1}}});
    CHECK_FALSE(status.ok());  // second one steals the device
    CHECK(partial.exportMappings().size() == 2);  // the others were still applied
}
