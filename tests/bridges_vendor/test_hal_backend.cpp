// Exercises the real HAL SimDevice binding (hal_sim_device_backend.cpp) against an in-memory fake of the HALSIM
// C API with the exact HAL signatures. Only built when the WPILib HAL headers were found (WPIEMU_WPILIB_ROOT).
// The fake mimics the real HAL's locking: every callback is invoked WHILE the fake HAL lock is held, on the thread
// that created the device/value, which is what makes the lock order of the binding matter.
#ifdef WPIEMU_VENDOR_HAS_WPILIB

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hal/Types.h"
#include "hal/Value.h"
#include "hal/SimDevice.h"
#include "hal/simulation/SimDeviceData.h"
#include "fakes.h"
#include "wpiemu/bridges/vendor/ctre_rules.h"
#include "wpiemu/bridges/vendor/hal_sim_device_backend.h"
#include "wpiemu/bridges/vendor/vendor_sim_bridge.h"

using namespace wpiemu::bridges::vendor;
namespace sg = wpiemu::sdk::signal;
using wpiemu::sdk::SignalDirection;
using wpiemu::sdk::SignalType;

namespace {

struct FakeHal {
    struct Value {
        std::string name;
        HAL_SimDeviceHandle device;
        int32_t direction;
        HAL_Value value;
    };
    struct Callback {
        std::string prefix;
        HAL_SimDeviceHandle device = 0;
        void* param = nullptr;
        HALSIM_SimDeviceCallback deviceFn = nullptr;
        HALSIM_SimValueCallback valueFn = nullptr;
    };
    std::mutex mutex;  // the "HAL lock": held while callbacks run, exactly like the real HAL
    std::map<HAL_SimDeviceHandle, std::string> devices;
    std::map<HAL_SimValueHandle, Value> values;
    std::map<int32_t, Callback> created, freed, valueCreated;
    int32_t nextHandle = 100;
    int32_t nextUid = 1;

    HAL_SimDeviceHandle createDevice(const std::string& name) {
        std::lock_guard lock(mutex);
        const HAL_SimDeviceHandle h = nextHandle++;
        devices[h] = name;
        for (auto& [uid, cb] : created)
            if (name.rfind(cb.prefix, 0) == 0) cb.deviceFn(name.c_str(), cb.param, h);
        return h;
    }
    HAL_SimValueHandle createValue(HAL_SimDeviceHandle d, const std::string& name, int32_t dir, HAL_Value v) {
        std::lock_guard lock(mutex);
        const HAL_SimValueHandle h = nextHandle++;
        values[h] = {name, d, dir, v};
        for (auto& [uid, cb] : valueCreated)
            if (cb.device == d) cb.valueFn(name.c_str(), cb.param, h, dir, &values[h].value);
        return h;
    }
    void freeDevice(HAL_SimDeviceHandle h) {
        std::lock_guard lock(mutex);
        const std::string name = devices[h];
        devices.erase(h);
        for (auto it = valueCreated.begin(); it != valueCreated.end();)
            it = it->second.device == h ? valueCreated.erase(it) : std::next(it);  // the HAL drops them with the device
        for (auto& [uid, cb] : freed)
            if (name.rfind(cb.prefix, 0) == 0) cb.deviceFn(name.c_str(), cb.param, h);
    }
};

FakeHal* g_hal = nullptr;

int32_t fakeRegisterCreated(const char* prefix, void* param, HALSIM_SimDeviceCallback cb, HAL_Bool) {
    std::lock_guard lock(g_hal->mutex);
    const int32_t uid = g_hal->nextUid++;
    g_hal->created[uid] = {prefix, 0, param, cb, nullptr};
    return uid;
}
void fakeCancelCreated(int32_t uid) {
    std::lock_guard lock(g_hal->mutex);
    g_hal->created.erase(uid);
}
int32_t fakeRegisterFreed(const char* prefix, void* param, HALSIM_SimDeviceCallback cb, HAL_Bool) {
    std::lock_guard lock(g_hal->mutex);
    const int32_t uid = g_hal->nextUid++;
    g_hal->freed[uid] = {prefix, 0, param, cb, nullptr};
    return uid;
}
void fakeCancelFreed(int32_t uid) {
    std::lock_guard lock(g_hal->mutex);
    g_hal->freed.erase(uid);
}
int32_t fakeRegisterValueCreated(HAL_SimDeviceHandle device, void* param, HALSIM_SimValueCallback cb, HAL_Bool) {
    std::lock_guard lock(g_hal->mutex);
    if (g_hal->devices.count(device) == 0) return 0;  // stale handle: the real HAL returns 0 as well
    const int32_t uid = g_hal->nextUid++;
    g_hal->valueCreated[uid] = {"", device, param, nullptr, cb};
    return uid;
}
void fakeCancelValueCreated(int32_t uid) {
    std::lock_guard lock(g_hal->mutex);
    g_hal->valueCreated.erase(uid);
}
void fakeEnumerateDevices(const char* prefix, void* param, HALSIM_SimDeviceCallback cb) {
    std::lock_guard lock(g_hal->mutex);
    for (const auto& [h, name] : g_hal->devices)
        if (name.rfind(prefix, 0) == 0) cb(name.c_str(), param, h);
}
void fakeEnumerateValues(HAL_SimDeviceHandle device, void* param, HALSIM_SimValueCallback cb) {
    std::lock_guard lock(g_hal->mutex);
    for (const auto& [h, v] : g_hal->values)
        if (v.device == device && g_hal->devices.count(device) != 0) cb(v.name.c_str(), param, h, v.direction, &v.value);
}
void fakeGetValue(HAL_SimValueHandle h, HAL_Value* out) {
    std::lock_guard lock(g_hal->mutex);
    const auto it = g_hal->values.find(h);
    if (it != g_hal->values.end()) *out = it->second.value;  // stale handle: untouched, like the real HAL
}
void fakeSetValue(HAL_SimValueHandle h, const HAL_Value* v) {
    std::lock_guard lock(g_hal->mutex);
    const auto it = g_hal->values.find(h);
    if (it != g_hal->values.end()) it->second.value = *v;
}

class FakeHalResolver final : public ISymbolResolver {
public:
    void* resolve(std::string_view s) const override {
        const std::string n(s);
        if (n == "HALSIM_RegisterSimDeviceCreatedCallback") return reinterpret_cast<void*>(&fakeRegisterCreated);
        if (n == "HALSIM_CancelSimDeviceCreatedCallback") return reinterpret_cast<void*>(&fakeCancelCreated);
        if (n == "HALSIM_RegisterSimDeviceFreedCallback") return reinterpret_cast<void*>(&fakeRegisterFreed);
        if (n == "HALSIM_CancelSimDeviceFreedCallback") return reinterpret_cast<void*>(&fakeCancelFreed);
        if (n == "HALSIM_RegisterSimValueCreatedCallback") return reinterpret_cast<void*>(&fakeRegisterValueCreated);
        if (n == "HALSIM_CancelSimValueCreatedCallback") return reinterpret_cast<void*>(&fakeCancelValueCreated);
        if (n == "HALSIM_EnumerateSimDevices") return reinterpret_cast<void*>(&fakeEnumerateDevices);
        if (n == "HALSIM_EnumerateSimValues") return reinterpret_cast<void*>(&fakeEnumerateValues);
        if (!omitValueApi && n == "HAL_GetSimValue") return reinterpret_cast<void*>(&fakeGetValue);
        if (!omitValueApi && n == "HAL_SetSimValue") return reinterpret_cast<void*>(&fakeSetValue);
        return nullptr;
    }
    bool omitValueApi = false;
};

struct HalRig {
    FakeHal hal;
    HalRig() { g_hal = &hal; }
    ~HalRig() { g_hal = nullptr; }
};

}  // namespace

TEST_CASE("vendor HAL binding: compiled in, resolves through the injected symbol table") {
    CHECK(halBindingCompiledIn());
    HalRig rig;
    std::string error;
    CHECK(makeHalSimDeviceBackend(std::make_shared<FakeHalResolver>(), &error) != nullptr);

    auto partial = std::make_shared<FakeHalResolver>();
    partial->omitValueApi = true;
    CHECK(makeHalSimDeviceBackend(partial, &error) == nullptr);
    CHECK_FALSE(error.empty());
    CHECK(rig.hal.created.empty());  // nothing registered when the binding is incomplete
}

TEST_CASE("vendor HAL binding: devices, directions, types and conversions") {
    HalRig rig;
    auto backend = makeHalSimDeviceBackend(std::make_shared<FakeHalResolver>());
    REQUIRE(backend != nullptr);
    CHECK(rig.hal.created.size() == 1);  // created + freed callbacks registered with the "CAN" prefix
    CHECK(rig.hal.freed.size() == 1);

    const auto before = backend->generation();
    const HAL_SimDeviceHandle motor = rig.hal.createDevice("CANMotor:Talon FX (v6)[4]");
    rig.hal.createDevice("ADXRS450[0]");  // not a CAN device: filtered by prefix
    CHECK(backend->generation() != before);

    // A device is reported the moment it exists, even before it has a single value (live discovery).
    std::vector<wpiemu::sdk::SimDeviceInfo> devices;
    backend->listDevices(devices);
    REQUIRE(devices.size() == 1);
    CHECK(devices[0].name == "CANMotor:Talon FX (v6)[4]");
    CHECK(rig.hal.valueCreated.size() == 1);  // watching this device for values created later

    const auto genBeforeValue = backend->generation();
    const HAL_SimValueHandle volts = rig.hal.createValue(motor, "motorVoltage", HAL_SimValueOutput, HAL_MakeDouble(0.0));
    CHECK(backend->generation() != genBeforeValue);  // a late value wakes the consumer up
    const HAL_SimValueHandle supply = rig.hal.createValue(motor, "supplyVoltage", HAL_SimValueInput, HAL_MakeDouble(12.0));
    const HAL_SimValueHandle flag = rig.hal.createValue(motor, "flag", HAL_SimValueBidir, HAL_MakeBoolean(0));
    const HAL_SimValueHandle count = rig.hal.createValue(motor, "count", HAL_SimValueInput, HAL_MakeInt(3));

    std::vector<wpiemu::sdk::SimValueInfo> values;
    backend->listValues(devices[0].handle, values);
    REQUIRE(values.size() == 4);
    const auto find = [&](const char* n) -> const wpiemu::sdk::SimValueInfo& {
        for (const auto& v : values)
            if (v.name == n) return v;
        FAIL("missing value " << n);
        return values[0];
    };
    CHECK(find("motorVoltage").direction == SignalDirection::Output);
    CHECK(find("supplyVoltage").direction == SignalDirection::Input);
    CHECK(find("flag").direction == SignalDirection::Bidirectional);
    CHECK(find("flag").type == SignalType::Bool);
    CHECK(find("count").type == SignalType::Int);
    CHECK(find("motorVoltage").type == SignalType::Double);

    CHECK(backend->getValue(wpiemu::sdk::SimValueHandle(supply)) == doctest::Approx(12.0));
    CHECK(backend->getValue(wpiemu::sdk::SimValueHandle(count)) == doctest::Approx(3.0));
    backend->setValue(wpiemu::sdk::SimValueHandle(supply), 10.5);
    backend->setValue(wpiemu::sdk::SimValueHandle(flag), 1.0);
    backend->setValue(wpiemu::sdk::SimValueHandle(count), 7.0);
    CHECK(rig.hal.values.at(supply).value.type == HAL_DOUBLE);
    CHECK(rig.hal.values.at(supply).value.data.v_double == doctest::Approx(10.5));
    CHECK(rig.hal.values.at(flag).value.type == HAL_BOOLEAN);  // type preserved: never writes a double into a bool
    CHECK(rig.hal.values.at(flag).value.data.v_boolean == 1);
    CHECK(rig.hal.values.at(count).value.type == HAL_INT);
    CHECK(rig.hal.values.at(count).value.data.v_int == 7);
    (void)volts;

    const auto gen = backend->generation();
    rig.hal.freeDevice(motor);
    CHECK(backend->generation() != gen);
    devices.clear();
    backend->listDevices(devices);
    CHECK(devices.empty());

    backend.reset();
    CHECK(rig.hal.created.empty());  // callbacks are cancelled (RAII)
    CHECK(rig.hal.freed.empty());
}

TEST_CASE("vendor HAL binding: stale handles are harmless") {
    HalRig rig;
    auto backend = makeHalSimDeviceBackend(std::make_shared<FakeHalResolver>());
    REQUIRE(backend != nullptr);
    const HAL_SimDeviceHandle d = rig.hal.createDevice("CANMotor:Talon FX (v6)[1]");
    const HAL_SimValueHandle v = rig.hal.createValue(d, "motorVoltage", HAL_SimValueOutput, HAL_MakeDouble(3.0));
    std::vector<wpiemu::sdk::SimDeviceInfo> devices;
    backend->listDevices(devices);
    rig.hal.freeDevice(d);
    rig.hal.values.erase(v);  // the value is gone with its device

    std::vector<wpiemu::sdk::SimValueInfo> values;
    backend->listValues(devices[0].handle, values);  // device freed since it was listed
    CHECK(values.empty());
    CHECK(backend->getValue(wpiemu::sdk::SimValueHandle(v)) == doctest::Approx(0.0));
    backend->setValue(wpiemu::sdk::SimValueHandle(v), 1.0);  // no crash, no effect
    backend->setValue(wpiemu::sdk::SimValueHandle(9999), 1.0);
    CHECK(backend->getValue(wpiemu::sdk::SimValueHandle(9999)) == doctest::Approx(0.0));
}

TEST_CASE("vendor HAL binding: end to end with the Phoenix 6 table") {
    HalRig rig;
    std::shared_ptr<wpiemu::sdk::ISimDeviceBackend> backend = makeHalSimDeviceBackend(std::make_shared<FakeHalResolver>());
    REQUIRE(backend != nullptr);

    const HAL_SimDeviceHandle motor = rig.hal.createDevice("CANMotor:Talon FX (v6)[4]");
    rig.hal.createValue(motor, "supplyVoltage", HAL_SimValueInput, HAL_MakeDouble(12.0));
    rig.hal.createValue(motor, "motorVoltage", HAL_SimValueOutput, HAL_MakeDouble(6.0));
    const HAL_SimDeviceHandle rotor = rig.hal.createDevice("CANEncoder:Talon FX (v6)[4]/Rotor Sensor");
    const HAL_SimValueHandle vel = rig.hal.createValue(rotor, "velocity", HAL_SimValueInput, HAL_MakeDouble(0.0));

    VendorSimBridgeOptions options;
    options.guard = ctre::phoenix6GuardOptions();
    VendorSimDeviceBridge bridge("ctre.phoenix6", ctre::phoenix6Rules(), backend, options);
    wpiemu::vendor_tests::FakeHost host;
    REQUIRE(bridge.start(host).ok());
    bridge.poll();  // a single poll is enough: no settling delay (program-start announce must be complete)
    const wpiemu::sdk::DeviceId id = host.idOf("talonfx", 4);
    REQUIRE(id.valid());

    std::vector<wpiemu::sdk::SignalSample> out;
    bridge.collectOutputs(out);
    bool sawVoltage = false;
    for (const auto& s : out)
        if (s.signal == host.signalIndex(id, sg::kMotorVoltage)) sawVoltage = s.value == doctest::Approx(6.0);
    CHECK(sawVoltage);

    std::vector<wpiemu::sdk::SignalSample> in{{id, host.signalIndex(id, sg::kRotorVelocity), 33.0}};
    bridge.applyInputs(in);
    CHECK(rig.hal.values.at(vel).value.data.v_double == doctest::Approx(33.0));
    bridge.stop();
}

// Regression for the access violation seen when the bridge was polled while the robot constructor was still creating
// devices (docs/reference/bridges.md): a "robot" thread creates TalonFX trees with
// the HAL lock semantics of the real HAL while the main thread polls as fast as it can.
TEST_CASE("vendor HAL binding: polling while another thread constructs devices is safe and loses nothing") {
    HalRig rig;
    std::shared_ptr<wpiemu::sdk::ISimDeviceBackend> backend = makeHalSimDeviceBackend(std::make_shared<FakeHalResolver>());
    REQUIRE(backend != nullptr);
    VendorSimBridgeOptions options;
    options.guard = ctre::phoenix6GuardOptions();
    VendorSimDeviceBridge bridge("ctre.phoenix6", ctre::phoenix6Rules(), backend, options);
    wpiemu::vendor_tests::FakeHost host;
    REQUIRE(bridge.start(host).ok());

    constexpr int kDevices = 24;
    std::atomic<bool> done{false};
    std::thread robot([&] {
        for (int id = 1; id <= kDevices; ++id) {
            const std::string base = "Talon FX (v6)[" + std::to_string(id) + "]";
            const HAL_SimDeviceHandle motor = rig.hal.createDevice("CANMotor:" + base);
            std::this_thread::yield();
            for (const char* n : {"supplyVoltage", "dutyCycle", "motorVoltage", "torqueCurrent", "supplyCurrent"}) {
                rig.hal.createValue(motor, n, std::string_view(n) == "supplyVoltage" ? HAL_SimValueInput : HAL_SimValueOutput,
                                    HAL_MakeDouble(0.0));
                std::this_thread::yield();
            }
            const HAL_SimDeviceHandle rotor = rig.hal.createDevice("CANEncoder:" + base + "/Rotor Sensor");
            std::this_thread::yield();
            for (const char* n : {"position", "rawPositionInput", "velocity", "acceleration"}) {
                rig.hal.createValue(rotor, n, std::string_view(n) == "position" ? HAL_SimValueOutput : HAL_SimValueInput,
                                    HAL_MakeDouble(0.0));
                std::this_thread::yield();
            }
        }
        done = true;
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    std::vector<wpiemu::sdk::SignalSample> out;
    while (!done && std::chrono::steady_clock::now() < deadline) {
        bridge.poll();
        out.clear();
        bridge.collectOutputs(out);  // reads only values of devices already announced
    }
    robot.join();
    REQUIRE(done);
    bridge.poll();  // quiescent poll after construction

    REQUIRE(host.announced.size() == static_cast<std::size_t>(kDevices));
    for (int id = 1; id <= kDevices; ++id) {
        const wpiemu::sdk::DeviceId dev = host.idOf("talonfx", id);
        REQUIRE(dev.valid());
        CAPTURE(id);
        // 5 motor values + 4 rotor values + the synthesized torqueCurrent, whatever the interleaving of the polls
        // with the construction.
        CHECK(host.announced.at(dev).signalSpecs.size() == 10);
        CHECK(host.signalSpec(dev, sg::kSupplyCurrentOut) != nullptr);
        CHECK(host.signalSpec(dev, sg::kRotorAcceleration) != nullptr);
    }
    bridge.stop();
}

#endif  // WPIEMU_VENDOR_HAS_WPILIB
