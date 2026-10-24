#include <doctest/doctest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/devices/channel_family_bridge.h"
#include "wpiemu/sdk/devices/device_catalog.h"
#include "wpiemu/sdk/devices/signals.h"
#include "wpiemu/sdk/devices/sim_device_bridge.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/testing/fake_bridge_host.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"
#include "wpiemu/sdk/testing/fake_device_hub.h"
#include "wpiemu/sdk/testing/fake_sim_device_backend.h"

using namespace wpiemu::sdk;

namespace {

using FakeHost = testing::FakeBridgeHost;
using FakeBackend = testing::FakeSimDeviceBackend;

std::vector<SimDeviceRule> talonRules() {
    SimDeviceRule r;
    r.name = "talonfx";
    r.family = "phoenix6";
    r.type = "talonfx";
    r.vendor = "ctre";
    r.nameRegex = R"(CAN\w*:Talon FX \(v6\)\[(\d+)\](?:/(.+))?)";
    r.idGroup = 1;
    r.childGroup = 2;
    r.capabilities = Capability::MotorOutput | Capability::RotorFeedback;
    r.bindings = {
        {"", "motorVoltage", signal::kMotorVoltage, 1.0, 0.0, std::nullopt, "V", ""},
        {"", "supplyVoltage", signal::kSupplyVoltage, 1.0, 0.0, std::nullopt, "V", ""},
        {"Rotor Sensor", "rawPositionInput", signal::kRotorPosition, 1.0, 0.0, std::nullopt, "rot", ""},
        {"Rotor Sensor", "velocity", signal::kRotorVelocity, 2.0, 0.0, std::nullopt, "rps", ""},
    };
    return {r};
}

}  // namespace

TEST_CASE("SimDeviceBridge discovers, groups children, and bridges values by canonical name") {
    FakeBackend backend;
    FakeHost host;
    SimDeviceBridge bridge("ctre.simdevice", talonRules(), backend);
    REQUIRE(bridge.start(host).ok());

    const auto root = backend.addDevice("CANMotor:Talon FX (v6)[4]",
                                        {{"motorVoltage", SignalDirection::Output},
                                         {"supplyVoltage", SignalDirection::Input},
                                         {"ignoredValue", SignalDirection::Output}});
    const auto child = backend.addDevice("CANEncoder:Talon FX (v6)[4]/Rotor Sensor",
                                         {{"rawPositionInput", SignalDirection::Input},
                                          {"velocity", SignalDirection::Input}});
    bridge.poll();

    REQUIRE(host.announced.size() == 1);
    const auto& [id, ann] = *host.announced.begin();
    CHECK(ann.descriptor.type == "talonfx");
    CHECK(ann.descriptor.id == 4);
    CHECK(ann.descriptor.capabilities.has(Capability::MotorOutput));
    REQUIRE(ann.signalSpecs.size() == 4);  // unbound value skipped; both SimDevices folded into one device
    CHECK(bridge.owns(id));

    // robot -> sim
    backend.poke(root, "motorVoltage", 7.5);
    std::vector<SignalSample> out;
    bridge.collectOutputs(out);
    REQUIRE(out.size() == 1);  // only Output signals
    CHECK(out[0].value == 7.5);
    CHECK(ann.signalSpecs[out[0].signal].name == signal::kMotorVoltage);

    // sim -> robot, with scale (canonical 10 rps -> raw 5)
    std::uint16_t velIdx = 0;
    REQUIRE(findSignal(ann.signalSpecs, signal::kRotorVelocity, &velIdx) != nullptr);
    const SignalSample in[] = {{id, velIdx, 10.0}};
    bridge.applyInputs(in);
    CHECK(backend.get(child, "velocity") == doctest::Approx(5.0));

    // idle poll does not re-announce
    bridge.poll();
    CHECK(host.reannounces == 0);
    bridge.stop();
    CHECK_FALSE(bridge.owns(id));
}

TEST_CASE("SimDeviceBridge rejects an invalid rule regex") {
    FakeBackend backend;
    FakeHost host;
    SimDeviceRule bad;
    bad.name = "bad";
    bad.nameRegex = "([";
    SimDeviceBridge bridge("bad", {bad}, backend);
    CHECK(bridge.start(host).error().code == ErrorCode::InvalidArgument);
}

namespace {

class FakeDioBridge final : public ChannelFamilyBridge {
public:
    FakeDioBridge() : ChannelFamilyBridge("hal.dio") {}
    bool init[4] = {false, false, false, false};
    double value[4] = {0, 0, 0, 0};
    int loopbacks = 0;

protected:
    int channelCount() const override { return 4; }
    bool isInitialized(int c) const override { return init[c]; }
    DeviceDescriptor describe(int c) const override {
        DeviceDescriptor d;
        d.family = "hal";
        d.type = "dio";
        d.vendor = "wpilib";
        d.id = c;
        d.capabilities = Capability::DigitalInput | Capability::DigitalOutput;
        return d;
    }
    std::vector<SignalSpec> signalsOf(int) const override {
        return {{signal::kDigitalIn, SignalDirection::Bidirectional, SignalType::Bool, ""}};
    }
    double readSignal(int c, std::size_t) const override { return value[c]; }
    void writeSignal(int c, std::size_t, double v) override { value[c] = v; }
    void loopback(int) override { ++loopbacks; }
};

}  // namespace

TEST_CASE("ChannelFamilyBridge announces on init and retires on free") {
    FakeHost host;
    FakeDioBridge bridge;
    REQUIRE(bridge.start(host).ok());
    bridge.poll();
    CHECK(host.announced.empty());

    bridge.init[2] = true;
    bridge.poll();
    REQUIRE(host.announced.size() == 1);
    const DeviceId id = host.announced.begin()->first;
    CHECK(host.announced.begin()->second.descriptor.id == 2);

    bridge.value[2] = 1.0;
    std::vector<SignalSample> out;
    bridge.collectOutputs(out);
    REQUIRE(out.size() == 1);
    CHECK(out[0].value == 1.0);

    const SignalSample in[] = {{id, 0, 0.0}};
    bridge.applyInputs(in);
    CHECK(bridge.value[2] == 0.0);

    bridge.applyLoopback(id);
    CHECK(bridge.loopbacks == 1);

    bridge.init[2] = false;
    bridge.poll();
    REQUIRE(host.retired.size() == 1);
    CHECK_FALSE(bridge.owns(id));
}

TEST_CASE("ChannelFamilyBridge is safe to poll and collect after stop") {
    FakeHost host;
    FakeDioBridge bridge;
    REQUIRE(bridge.start(host).ok());
    bridge.init[1] = true;
    bridge.poll();
    const DeviceId id = host.announced.begin()->first;

    bridge.stop();
    std::vector<SignalSample> out;
    bridge.collectOutputs(out);
    CHECK(out.empty());
    bridge.poll();
    const SignalSample in[] = {{id, 0, 1.0}};
    bridge.applyInputs(in);
    bridge.applyLoopback(id);
    CHECK_FALSE(bridge.owns(id));
    CHECK(bridge.loopbacks == 0);

    bridge.collectOutputs(out);
    FakeDioBridge fresh;  // never started
    fresh.collectOutputs(out);
    CHECK(out.empty());
}

TEST_CASE("DeviceBridgeRegistry creates every registered bridge in order") {
    DeviceBridgeRegistry registry;
    CHECK(registry.add("hal.dio", {"hal", "DIO", "wpilib"}, [] { return std::make_unique<FakeDioBridge>(); }).ok());
    CHECK(registry.add("hal.dio", {}, [] { return std::make_unique<FakeDioBridge>(); }).error().code ==
          ErrorCode::AlreadyExists);
    auto all = registry.createAll();
    REQUIRE(all.ok());
    CHECK(all.value().size() == 1);
    CHECK(all.value()[0]->name() == "hal.dio");
}

TEST_CASE("slot compatibility follows device capabilities") {
    SlotSpec motorSlot{"motor", "Motor", SlotType::Motor, true, "", "", 0};
    DeviceDescriptor talon;
    talon.capabilities = Capability::MotorOutput | Capability::RotorFeedback;
    DeviceDescriptor dio;
    dio.capabilities = Capability::DigitalInput;
    CHECK(isCompatible(motorSlot, talon));
    CHECK_FALSE(isCompatible(motorSlot, dio));
    CHECK(isCompatible(SlotSpec{"lim", "Limit", SlotType::LimitSwitch, true, "", "", 0}, dio));
}

TEST_CASE("one steering rule and one traction slot generator serve every drivetrain consumer") {
    WheelSpec fixed;
    fixed.name = "a";
    WheelSpec flagged;
    flagged.name = "b";
    flagged.steered = true;
    WheelSpec module;
    module.name = "c";
    module.kind = WheelKind::Swerve;
    module.driveMotorCount = 2;
    CHECK_FALSE(isSteered(fixed));
    CHECK(isSteered(flagged));
    CHECK(isSteered(module));

    DrivetrainSpec spec;
    spec.wheels = {fixed, flagged, module};
    const auto namesOf = [](const std::vector<SlotSpec>& slots) {
        std::vector<std::string> out;
        for (const SlotSpec& s : slots) out.push_back(s.name);
        return out;
    };
    CHECK(namesOf(tractionSlotSpecs(spec, true)) ==
          std::vector<std::string>{"a.drive", "b.drive", "b.steer", "b.steerEncoder", "c.drive", "c.drive2", "c.steer",
                                   "c.steerEncoder"});
    CHECK(namesOf(tractionSlotSpecs(spec, false)) == std::vector<std::string>{"a.drive", "b.drive", "c.drive", "c.drive2"});

    const auto slots = tractionSlotSpecs(spec, true);
    CHECK(slots[0].required);                    // first drive motor
    CHECK_FALSE(slots[5].required);              // c.drive2
    CHECK(slots[5].countKey == "c.driveMotorCount");
    CHECK(slots[5].ordinal == 2);
    CHECK(slots[6].required);                    // steer motor
    CHECK_FALSE(slots[7].required);              // steer encoder
    CHECK(slots[7].type == SlotType::PositionSensor);
    CHECK(steerSlotName("fl") == "fl.steer");
    CHECK(steerEncoderSlotName("fl") == "fl.steerEncoder");

    CHECK(namesOf(drivetrainSlotSpecs(nullptr, spec)) == namesOf(tractionSlotSpecs(spec, true)));   // unknown kind: convention
    DrivetrainInfo custom;
    custom.slotSpecs = [](const DrivetrainSpec&) {
        SlotSpec kiwi;
        kiwi.name = "kiwi.motor";
        kiwi.label = "Kiwi";
        return std::vector<SlotSpec>{kiwi};
    };
    CHECK(namesOf(drivetrainSlotSpecs(&custom, spec)) == std::vector<std::string>{"kiwi.motor"});
    const DrivetrainInfo none;
    CHECK(namesOf(drivetrainSlotSpecs(&none, spec)) == namesOf(tractionSlotSpecs(spec, true)));   // no callback: convention

    const auto robot = robotSlotSpecs();
    REQUIRE(robot.size() == 1);
    CHECK(robot[0].name == slot_owner::kRobotImuSlot);
    CHECK(robot[0].type == SlotType::Imu);
    CHECK_FALSE(robot[0].required);
}

TEST_CASE("FakeDeviceHub behaves like the real hub: unbound slots are neutral") {
    testing::FakeDeviceHub hub;
    const SlotAddress slot{"intake", "motor"};
    hub.setMotorOutput(slot, MotorOutput{true, 0.5, 6.0, 0.0, false});
    CHECK_FALSE(hub.motorOutput(slot).valid);  // unbound => neutral

    const DeviceId id = hub.autoBind(slot, SlotType::Motor, 7);
    CHECK(hub.motorOutput(slot).voltageV == 6.0);
    CHECK(hub.isMapped(id));
    CHECK(hub.resolve(slot) == id);

    hub.writeMotorFeedback(slot, MotorFeedback{1.0, 2.0, 0.0, std::nullopt, std::nullopt});
    CHECK(hub.lastMotorFeedback(slot)->rotorVelocityRps == 2.0);

    // one device : one slot
    CHECK(hub.bind({"other", "motor"}, hub.boundDevice(slot).value()).error().code == ErrorCode::AlreadyExists);
    hub.unbind(slot);
    CHECK_FALSE(hub.isMapped(id));
}

TEST_CASE("SimDeviceBridge never fabricates a bus for rules that cannot know it (SDK 1.1)") {
    FakeBackend backend;
    FakeHost host;
    auto rules = talonRules();
    rules[0].busIsStatic = false;  // Phoenix 6 SimDevice names carry no bus
    SimDeviceBridge bridge("ctre.simdevice", rules, backend);
    REQUIRE(bridge.start(host).ok());
    backend.addDevice("CANMotor:Talon FX (v6)[4]", {{"motorVoltage", SignalDirection::Output}});
    bridge.poll();

    REQUIRE(host.announced.size() == 1);
    const auto& [id, ann] = *host.announced.begin();
    CHECK(ann.descriptor.bus == kBusUnknown);
    CHECK(ann.descriptor.physicalId() == PhysicalDeviceId{"phoenix6", "talonfx", 4, -1});

    // bus arrives only through an explicit observation (L3), and does not change the identity
    CHECK(bridge.observedBus(id).empty());
    CHECK(bridge.observeBus(id, "canivore"));
    CHECK(bridge.observedBus(id) == "canivore");
    REQUIRE(host.observed.size() == 1);
    CHECK(host.observed[0].second == "canivore");
    CHECK_FALSE(bridge.observeBus(DeviceId{99}, "rio"));
    bridge.poll();
    CHECK(host.reannounces == 0);
}

TEST_CASE("SimDeviceBridge keeps the rule bus when it is a fact of the family (default, 1.0 behaviour)") {
    FakeBackend backend;
    FakeHost host;
    SimDeviceBridge bridge("ctre.simdevice", talonRules(), backend);
    REQUIRE(bridge.start(host).ok());
    backend.addDevice("CANMotor:Talon FX (v6)[4]", {{"motorVoltage", SignalDirection::Output}});
    bridge.poll();
    REQUIRE(host.announced.size() == 1);
    CHECK(host.announced.begin()->second.descriptor.bus == "rio");
}

TEST_CASE("SDK slot and device catalogs supply canonical labels and capability sets") {
    CHECK(std::string_view(slotTypeInfo(SlotType::Motor).shortName) == "motor");
    CHECK(std::string_view(slotTypeInfo(SlotType::Motor).displayName) == "Motor controller");
    CHECK(requiredCapabilities(SlotType::Imu) == CapabilitySet(Capability::Imu));

    CHECK(deviceTypeDisplayName(device_type::kTalonFx) == "TalonFX");
    CHECK(deviceTypeDisplayName(device_type::kCancoder) == "CANcoder");
    CHECK(deviceTypeDisplayName("custom_dev") == "custom_dev");
    CHECK(deviceTypeCapabilities(device_type::kTalonFx).has(Capability::MotorOutput));
    CHECK(deviceTypeCapabilities(device_type::kPigeon2).has(Capability::Imu));
    CHECK(findDeviceType("unknown_type") == nullptr);
}
