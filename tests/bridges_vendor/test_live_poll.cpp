#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fakes.h"
#include "wpiemu/bridges/vendor/ctre_native.h"
#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/bridges/vendor/incremental_backend.h"
#include "wpiemu/bridges/vendor/vendor_sim_bridge.h"
#include "wpiemu/sdk/core/units.h"

// Live discovery: the bridge is polled while the robot constructor is still creating SimDevices and values.
using namespace wpiemu::vendor_tests;
namespace vendor = wpiemu::bridges::vendor;
namespace ctre = wpiemu::bridges::vendor::ctre;
namespace sg = wpiemu::sdk::signal;

namespace {

struct LiveRig {
    std::shared_ptr<FakeSimBackend> backend = std::make_shared<FakeSimBackend>();
    FakeHost host;
    std::unique_ptr<vendor::VendorSimDeviceBridge> bridge;

    explicit LiveRig(vendor::VendorSimBridgeOptions options = {}) {
        options.guard = ctre::phoenix6GuardOptions();
        bridge = std::make_unique<vendor::VendorSimDeviceBridge>("ctre.phoenix6", ctre::phoenix6Rules(), backend,
                                                                 std::move(options));
        REQUIRE(bridge->start(host).ok());
    }
};

// Counts config reads and remembers whether the "robot" was busy constructing when one happened.
class BusyAwareNative final : public vendor::ICtreSimNative {
public:
    bool hasConfigReadback() const override { return true; }
    bool hasPhysicsApi() const override { return false; }
    Result<std::string> readConfigs(vendor::CtreDeviceType, int, std::string_view) const override {
        ++reads;
        if (robotBusy.load()) ++readsWhileBusy;
        return std::string("1901,s_0\n");
    }
    int getPhysicsValue(vendor::CtreDeviceType, int, const char*, double*) const override { return -1; }
    int setPhysicsInput(vendor::CtreDeviceType, int, const char*, double) override { return -1; }
    mutable std::atomic<int> reads{0};
    mutable std::atomic<int> readsWhileBusy{0};
    std::atomic<bool> robotBusy{false};
};

}  // namespace

TEST_CASE("vendor live poll: a device seen half-built gains its late values as appended signals") {
    LiveRig rig;
    // The robot created the SimDevice and only the first value so far.
    const SimDeviceHandle motor = rig.backend->addDevice("CANMotor:Talon FX (v6)[4]", {{"supplyVoltage", kIn, SignalType::Double, 12.0}});
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("talonfx", 4);
    REQUIRE(id.valid());
    REQUIRE(rig.host.announced.at(id).signalSpecs.size() == 1);
    const std::uint16_t supplyIndex = rig.host.signalIndex(id, sg::kSupplyVoltage);

    rig.backend->addValue(motor, {"dutyCycle", kOut});
    rig.backend->addValue(motor, {"motorVoltage", kOut});
    rig.bridge->poll();
    CHECK(rig.host.announced.size() == 1);  // same device, re-announced
    CHECK(rig.host.announced.at(id).signalSpecs.size() == 3);
    CHECK(rig.host.signalIndex(id, sg::kSupplyVoltage) == supplyIndex);  // indices are append-only

    rig.backend->poke(motor, "motorVoltage", 5.5);
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    bool saw = false;
    for (const auto& s : out)
        if (s.signal == rig.host.signalIndex(id, sg::kMotorVoltage)) saw = s.value == 5.5;
    CHECK(saw);

    std::vector<SignalSample> in{{id, supplyIndex, 11.0}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend->get(motor, "supplyVoltage") == doctest::Approx(11.0));
}

TEST_CASE("vendor live poll: late values of guarded inputs are still guarded") {
    LiveRig rig;
    const SimDeviceHandle pg = rig.backend->addDevice("CANGyro:Pigeon 2 (v6)[13]", {{"supplyVoltage", kIn, SignalType::Double, 12.0}});
    rig.bridge->poll();
    rig.backend->addValue(pg, {"rawYawInput", kIn});
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("pigeon2", 13);
    REQUIRE(rig.host.signalSpec(id, sg::kYaw) != nullptr);
    const auto yaw = rig.host.signalIndex(id, sg::kYaw);
    const double pi = wpiemu::sdk::kPi;

    std::vector<SignalSample> a{{id, yaw, 170.0 * pi / 180.0}};
    rig.bridge->applyInputs(a);
    std::vector<SignalSample> b{{id, yaw, -170.0 * pi / 180.0}};
    rig.bridge->applyInputs(b);
    CHECK(rig.backend->get(pg, "rawYawInput") == doctest::Approx(190.0));  // unwrapped: the guard knew the late value
}

TEST_CASE("vendor live poll: late-value pseudo devices die with their device and never leak handles") {
    FakeSimBackend inner;
    vendor::IncrementalSimDeviceBackend inc(inner);
    const SimDeviceHandle d = inner.addDevice("CANMotor:Talon FX (v6)[1]", {{"supplyVoltage", kIn}});
    std::vector<SimDeviceInfo> devices;
    inc.listDevices(devices);
    REQUIRE(devices.size() == 1);
    std::vector<SimValueInfo> values;
    inc.listValues(devices[0].handle, values);
    CHECK(values.size() == 1);

    inner.addValue(d, {"motorVoltage", kOut});
    devices.clear();
    inc.listDevices(devices);
    REQUIRE(devices.size() == 2);  // the device + one pseudo device with the same name
    CHECK(devices[1].name == devices[0].name);
    CHECK(inc.lateValueDeviceCount() == 1);
    values.clear();
    inc.listValues(devices[1].handle, values);
    REQUIRE(values.size() == 1);
    CHECK(values[0].name == "motorVoltage");  // only what is new

    devices.clear();
    inc.listDevices(devices);  // nothing new: no further pseudo devices
    CHECK(devices.size() == 2);

    inner.removeDevice(d);
    devices.clear();
    inc.listDevices(devices);
    CHECK(devices.empty());
    CHECK(inc.lateValueDeviceCount() == 0);
    values.clear();
    inc.listValues(SimDeviceHandle{(std::uint64_t{1} << 62) | 1}, values);  // stale pseudo handle: harmless
    CHECK(values.empty());
}

TEST_CASE("vendor live poll: poll never enters the CTRE native sim, quiescent entry points do") {
    auto native = std::make_shared<BusyAwareNative>();
    vendor::VendorSimBridgeOptions options;
    options.nativeProvider = [native] { return native; };
    LiveRig rig(options);
    addTalonFx(*rig.backend, 4);
    addTalonFx(*rig.backend, 5);
    for (int i = 0; i < 20; ++i) rig.bridge->poll();
    CHECK(native->reads == 0);  // robot-concurrent entry point: no native calls at all

    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);  // documented quiescent entry point
    CHECK(native->reads == 1);
    CHECK(rig.bridge->readPendingConfigs(5) == 1);  // the host can drain explicitly when it knows it is quiescent
    CHECK(native->reads == 2);
    CHECK(rig.bridge->readPendingConfigs(5) == 0);
}

// The crash scenario: a robot thread constructs TalonFX trees (device, then values, then children) with no
// synchronisation while the bridge is polled (and tick outputs are collected) as fast as possible.
TEST_CASE("vendor live poll: concurrent construction, nothing lost and no native call while the robot is busy") {
    auto native = std::make_shared<BusyAwareNative>();
    vendor::VendorSimBridgeOptions options;
    options.nativeProvider = [native] { return native; };
    LiveRig rig(options);

    constexpr int kDevices = 30;
    std::atomic<bool> done{false};
    native->robotBusy = true;
    std::thread robot([&] {
        for (int id = 1; id <= kDevices; ++id) {
            const std::string base = "Talon FX (v6)[" + std::to_string(id) + "]";
            const SimDeviceHandle motor = rig.backend->addDevice("CANMotor:" + base, {});
            std::this_thread::yield();
            for (const char* n : {"supplyVoltage", "dutyCycle", "motorVoltage", "torqueCurrent", "supplyCurrent"}) {
                rig.backend->addValue(motor, {n, std::string_view(n) == "supplyVoltage" ? kIn : kOut});
                std::this_thread::yield();
            }
            const SimDeviceHandle rotor = rig.backend->addDevice("CANEncoder:" + base + "/Rotor Sensor", {});
            for (const char* n : {"position", "rawPositionInput", "velocity", "acceleration"}) {
                rig.backend->addValue(rotor, {n, std::string_view(n) == "position" ? kOut : kIn});
                std::this_thread::yield();
            }
            const SimDeviceHandle fwd = rig.backend->addDevice("CANDIO:" + base + "/Fwd Limit", {});
            rig.backend->addValue(fwd, {"value", kBidir, SignalType::Bool});
        }
        done = true;
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    std::vector<SignalSample> out;
    while (!done && std::chrono::steady_clock::now() < deadline) {
        rig.bridge->poll();  // live discovery: concurrent with construction
    }
    robot.join();
    native->robotBusy = false;
    REQUIRE(done);
    rig.bridge->poll();  // quiescent
    CHECK(native->reads == 0);
    CHECK(native->readsWhileBusy == 0);

    REQUIRE(rig.host.announced.size() == static_cast<std::size_t>(kDevices));
    for (int id = 1; id <= kDevices; ++id) {
        const DeviceId dev = rig.host.idOf("talonfx", id);
        REQUIRE(dev.valid());
        CAPTURE(id);
        CHECK(rig.host.announced.at(dev).signalSpecs.size() == 10);  // 5 motor + 4 rotor + fwd limit
        CHECK(rig.host.signalSpec(dev, sg::kForwardLimit) != nullptr);
    }
    for (int i = 0; i < kDevices; ++i) rig.bridge->collectOutputs(out);
    CHECK(native->reads == kDevices);  // all configs read once the robot is quiescent
}
