#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <vector>

#include "fakes.h"
#include "wpiemu/bridges/vendor/hal_sim_device_backend.h"
#include "wpiemu/bridges/vendor/vendor_bridges.h"
#include "wpiemu/sdk/core/units.h"

using namespace wpiemu::vendor_tests;
namespace vendor = wpiemu::bridges::vendor;
namespace ctre = wpiemu::bridges::vendor::ctre;
namespace sg = wpiemu::sdk::signal;

namespace {

class FakeNative final : public vendor::ICtreSimNative {
public:
    bool hasConfigReadback() const override { return configApi; }
    bool hasPhysicsApi() const override { return physicsApi; }
    Result<std::string> readConfigs(vendor::CtreDeviceType type, int id, std::string_view bus) const override {
        ++reads;
        lastBus = std::string(bus);
        lastType = type;
        lastId = id;
        if (failWith) return Error{*failWith, "forced failure"};
        return text;
    }
    int getPhysicsValue(vendor::CtreDeviceType, int, const char*, double*) const override { return -1; }
    int setPhysicsInput(vendor::CtreDeviceType, int id, const char* key, double value) override {
        physicsWrites.push_back({id, key, value});
        return physicsStatus;
    }

    struct PhysicsWrite {
        int id;
        std::string key;
        double value;
    };
    bool configApi = true;
    bool physicsApi = false;
    int physicsStatus = 0;
    std::vector<PhysicsWrite> physicsWrites;
    std::string text = "1901,s_1\n1918,s_1070945649\n1919,s_1065353216\n1920,s_1\n1921,s_7\n";
    std::optional<ErrorCode> failWith;
    mutable int reads = 0;
    mutable std::string lastBus;
    mutable vendor::CtreDeviceType lastType = vendor::CtreDeviceType::P6_TalonFX;
    mutable int lastId = -1;
};

class FakeHooks final : public vendor::ICtreHookProvider {
public:
    std::string_view name() const override { return "fake-hooks"; }
    Status install(vendor::ICtreObservationSink& s) override {
        ++installs;
        sink = &s;
        return okStatus();
    }
    void uninstall() noexcept override {
        ++uninstalls;
        sink = nullptr;
    }
    int installs = 0;
    int uninstalls = 0;
    vendor::ICtreObservationSink* sink = nullptr;
};

struct BridgeRig {
    std::shared_ptr<FakeSimBackend> backend = std::make_shared<FakeSimBackend>();
    FakeHost host;
    std::unique_ptr<vendor::VendorSimDeviceBridge> bridge;

    explicit BridgeRig(vendor::VendorSimBridgeOptions options = defaultOptions()) {
        bridge = std::make_unique<vendor::VendorSimDeviceBridge>("ctre.phoenix6", ctre::phoenix6Rules(), backend,
                                                                 std::move(options));
    }
    static vendor::VendorSimBridgeOptions defaultOptions() {
        vendor::VendorSimBridgeOptions o;
        o.guard = ctre::phoenix6GuardOptions();
        return o;
    }
    void start() { REQUIRE(bridge->start(host).ok()); }
};

}  // namespace

TEST_CASE("vendor registration: both CTRE bridges are registered and created from one registry") {
    DeviceBridgeRegistry registry;
    vendor::VendorBridgeOptions options;
    auto backend = std::make_shared<FakeSimBackend>();
    options.backend = backend;
    vendor::registerVendorBridges(registry, options);
    CHECK(registry.contains("ctre.phoenix6"));
    CHECK(registry.contains("ctre.phoenix5"));
    CHECK(registry.info("ctre.phoenix6")->family == "phoenix6");
    CHECK(registry.info("ctre.phoenix5")->vendor == "ctre");

    addTalonFx(*backend, 4);
    backend->addDevice("CANMotor:Talon SRX[2]", {{"busVoltage", kIn}, {"percentOutput", kOut}, {"motorOutputLeadVoltage", kOut}});

    auto bridges = registry.createAll();
    REQUIRE(bridges.ok());
    REQUIRE(bridges.value().size() == 2);
    FakeHost host;
    for (auto& b : bridges.value()) {
        REQUIRE(b->start(host).ok());
        b->poll();
    }
    CHECK(host.idOf("talonfx", 4).valid());
    CHECK(host.idOf("talonsrx", 2).valid());
    CHECK(bridges.value()[0]->owns(host.idOf("talonfx", 4)));
    CHECK_FALSE(bridges.value()[1]->owns(host.idOf("talonfx", 4)));  // each device has exactly one owner
    for (auto& b : bridges.value()) b->stop();
}

TEST_CASE("vendor registration: duplicates are reported by the checked variant and options can drop a family") {
    DeviceBridgeRegistry registry;
    vendor::VendorBridgeOptions options;
    options.backend = std::make_shared<FakeSimBackend>();
    CHECK(vendor::registerVendorBridgesChecked(registry, options).ok());
    const Status again = vendor::registerVendorBridgesChecked(registry, options);
    REQUIRE_FALSE(again.ok());
    CHECK(again.error().code == ErrorCode::AlreadyExists);

    DeviceBridgeRegistry only6;
    options.enablePhoenix5 = false;
    vendor::registerVendorBridges(only6, options);
    CHECK(only6.size() == 1);
    CHECK(only6.contains("ctre.phoenix6"));
}

TEST_CASE("vendor bridge: without a HAL binding the bridge is inert and says so") {
    FakeHost host;
    vendor::VendorSimBridgeOptions options;
    options.backendAvailable = false;
    // macOS may retain the transitively linked wpiHal without initializing it. Model its absence
    // explicitly rather than attaching to whatever native libraries the test runner has loaded.
    vendor::VendorSimDeviceBridge bridge("ctre.phoenix6", ctre::phoenix6Rules(), nullptr, options);
    REQUIRE(bridge.start(host).ok());
    bridge.poll();
    bridge.poll();
    CHECK(host.announced.empty());
    CHECK(host.log.count(LogLevel::Warn) == 1);
    bridge.stop();
}

TEST_CASE("vendor bridge: yaw reaches the Pigeon once per tick and survives a vendor mirror via re-assert") {
    BridgeRig rig;
    const SimDeviceHandle pg = addPigeon2(*rig.backend, 13);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("pigeon2", 13);
    REQUIRE(id.valid());
    const auto yaw = rig.host.signalIndex(id, sg::kYaw);

    const auto tick = [&](double rad) {
        std::vector<SignalSample> in{{id, yaw, rad}};
        rig.bridge->applyInputs(in);
    };
    tick(0.5);
    CHECK(rig.backend->get(pg, "rawYawInput") == doctest::Approx(0.5 * 180.0 / wpiemu::sdk::kPi));
    CHECK(rig.backend->writesTo(pg, "rawYawInput") == 1);
    tick(0.5);
    CHECK(rig.backend->writesTo(pg, "rawYawInput") == 1);  // unchanged: no redundant write

    rig.backend->poke(pg, "rawYawInput", 0.0);  // vendor mirror corrupts raw before the robot reads
    CHECK(rig.bridge->reassertAfterVendorMirror() == 1);
    CHECK(rig.backend->get(pg, "rawYawInput") == doctest::Approx(0.5 * 180.0 / wpiemu::sdk::kPi));
    CHECK(rig.bridge->inputDisturbances().size() == 1);

    rig.bridge->resetInputHistory();
    CHECK(rig.bridge->inputDisturbances().empty());
}

TEST_CASE("vendor bridge: emulator owns the device can be switched off per device") {
    BridgeRig rig;
    const TalonFxTree t = addTalonFx(*rig.backend, 1);
    const SimDeviceHandle cc = addCanCoder(*rig.backend, 2);
    rig.start();
    rig.bridge->poll();
    const DeviceId talon = rig.host.idOf("talonfx", 1);
    const DeviceId coder = rig.host.idOf("cancoder", 2);

    rig.bridge->setInputWritesEnabled(talon, false);
    CHECK_FALSE(rig.bridge->inputWritesEnabled(talon));
    std::vector<SignalSample> in{{talon, rig.host.signalIndex(talon, sg::kRotorVelocity), 40.0},
                                 {coder, rig.host.signalIndex(coder, sg::kPosition), 1.5}};
    rig.bridge->applyInputs(in);
    CHECK(rig.backend->get(t.rotor, "velocity") == doctest::Approx(0.0));  // robot-side sim keeps control
    CHECK(rig.backend->get(cc, "rawPositionInput") == doctest::Approx(1.5));

    rig.backend->poke(t.motor, "motorVoltage", 5.0);
    rig.bridge->applyLoopback(talon);
    CHECK(rig.backend->get(t.rotor, "velocity") == doctest::Approx(0.0));
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);  // outputs are still readable
    bool sawVoltage = false;
    for (const auto& s : out)
        if (s.device == talon && s.signal == rig.host.signalIndex(talon, sg::kMotorVoltage)) sawVoltage = s.value == 5.0;
    CHECK(sawVoltage);

    rig.bridge->setInputWritesEnabled(talon, true);
    rig.bridge->applyInputs(in);
    CHECK(rig.backend->get(t.rotor, "velocity") == doctest::Approx(40.0));
}

TEST_CASE(
    "vendor bridge: generic history reset rebases wrapped yaw without touching firmware offsets") {
    BridgeRig rig;
    const auto pigeon = addPigeon2(*rig.backend, 13);
    rig.start();
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("pigeon2", 13);
    const auto yaw = rig.host.signalIndex(id, sg::kYaw);
    const auto raw = rig.backend->handle(pigeon, "rawYawInput");
    double lastRaw = 0.0;
    double reported = 0.0;
    double firmwareOffset = 0.0;
    rig.backend->onWrite = [&](SimValueHandle handle, double value) {
        if (handle != raw) return;
        reported += value - lastRaw;
        lastRaw = value;
    };
    wpiemu::sdk::IDeviceBridge& generic = *rig.bridge;
    const auto tick = [&](double degrees) {
        const std::vector<SignalSample> samples{{id, yaw, degrees * wpiemu::sdk::kPi / 180.0}};
        generic.applyInputs(samples);
    };
    for (int transition = 0; transition != 3; ++transition) {
        tick(0);
        tick(170);
        tick(-170);
        CHECK(rig.backend->get(pigeon, "rawYawInput") == doctest::Approx(190));
        firmwareOffset += 35;  // robot setYaw changes firmware state, not physical raw input
        reported -= 35;
        generic.resetInputHistory();
        tick(0);
        CHECK(rig.backend->get(pigeon, "rawYawInput") == doctest::Approx(0));
        CHECK(reported == doctest::Approx(-firmwareOffset));
    }
    tick(30);
    generic.resetInputHistory();  // nonwrapped control, including unchanged first feedback
    tick(30);
    CHECK(reported == doctest::Approx(30 - firmwareOffset));
}

TEST_CASE("vendor bridge: CANcoder inputs undo firmware direction and magnet offset") {
    for (const bool clockwise : {false, true}) {
        auto native = std::make_shared<FakeNative>();
        native->text = std::string("828,s_") + (clockwise ? "1" : "0") + "\n987,s_-1105788928\n";
        auto options = BridgeRig::defaultOptions();
        options.nativeProvider = [native] { return native; };
        BridgeRig rig(options);
        const auto cc = addCanCoder(*rig.backend, 12);
        rig.start();
        rig.bridge->poll();
        const auto id = rig.host.idOf("cancoder", 12);
        REQUIRE(rig.bridge->readPendingConfigs() == 1);
        const std::vector<SignalSample> samples{{id, rig.host.signalIndex(id, sg::kPosition), 0.2},
                                                {id, rig.host.signalIndex(id, sg::kVelocity), 1.5}};
        rig.bridge->applyInputs(samples);
        const double sign = clockwise ? -1.0 : 1.0;
        CHECK(rig.backend->get(cc, "rawPositionInput") ==
              doctest::Approx(sign * (0.2 + 0.1474609375)));
        CHECK(rig.backend->get(cc, "velocity") == doctest::Approx(sign * 1.5));
        rig.bridge->resetInputHistory();
        rig.bridge->applyInputs(samples);
        CHECK(rig.backend->get(cc, "rawPositionInput") ==
              doctest::Approx(sign * (0.2 + 0.1474609375)));
        rig.bridge->setInputWritesEnabled(id, false);
        auto changed = samples;
        changed.front().value = 0.4;
        rig.bridge->applyInputs(changed);
        CHECK(rig.backend->get(cc, "rawPositionInput") ==
              doctest::Approx(sign * (0.2 + 0.1474609375)));
    }
}

TEST_CASE("vendor bridge: missing CANcoder calibration warns and leaves raw feedback unchanged") {
    auto native = std::make_shared<FakeNative>();
    native->text = "828,s_1\n";
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [native] { return native; };
    SUBCASE("incomplete calibration") {}
    SUBCASE("native API unavailable") {
        native->configApi = false;
    }
    SUBCASE("config read-back disabled") {
        options.nativeProvider = {};
    }
    BridgeRig rig(options);
    const auto cc = addCanCoder(*rig.backend, 12);
    rig.start();
    rig.bridge->poll();
    const auto id = rig.host.idOf("cancoder", 12);
    const std::size_t expectedReads = options.nativeProvider && native->configApi ? 1 : 0;
    REQUIRE(rig.bridge->readPendingConfigs() == expectedReads);
    rig.bridge->readPendingConfigs();
    CHECK(rig.host.log.count(LogLevel::Warn) == 1);
    rig.bridge->applyInputs(
        std::vector<SignalSample>{{id, rig.host.signalIndex(id, sg::kPosition), 0.2}});
    CHECK(rig.backend->get(cc, "rawPositionInput") == doctest::Approx(0.2));
}

TEST_CASE("vendor bridge: the firmware sim is told which motor the plant simulates") {
    auto native = std::make_shared<FakeNative>();
    native->physicsApi = true;
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [native] { return native; };
    BridgeRig rig(options);
    addTalonFx(*rig.backend, 3);
    addTalonFx(*rig.backend, 4);
    addTalonFx(*rig.backend, 5);
    rig.start();
    rig.bridge->poll();
    const auto send = [&](int id, wpiemu::sdk::MotorFamily family) {
        const auto dev = rig.host.idOf("talonfx", id);
        REQUIRE(dev.valid());
        REQUIRE(rig.host.signalSpec(dev, sg::kMotorFamily) != nullptr);
        rig.bridge->applyInputs(std::vector<SignalSample>{
            {dev, rig.host.signalIndex(dev, sg::kMotorFamily), static_cast<double>(static_cast<int>(family))}});
    };
    std::vector<SignalSample> out;
    send(3, wpiemu::sdk::MotorFamily::KrakenX44);
    send(4, wpiemu::sdk::MotorFamily::KrakenX60);
    send(5, wpiemu::sdk::MotorFamily::Falcon500);  // the firmware sim has no such motor: default stays
    rig.bridge->collectOutputs(out);
    rig.bridge->collectOutputs(out);  // written once per change, not every tick

    int x44 = 0, x60 = 0;
    for (const auto& w : native->physicsWrites) {
        CHECK(w.key == vendor::kPhysicsMotorType);
        if (w.id == 3 && w.value == vendor::sim_motor_type::kKrakenX44) ++x44;
        if (w.id == 4 && w.value == vendor::sim_motor_type::kKrakenX60) ++x60;
        CHECK(w.id != 5);
    }
    CHECK(x44 == 1);
    CHECK(x60 == 1);
    CHECK(native->physicsWrites.size() == 2);

    send(3, wpiemu::sdk::MotorFamily::KrakenX60);  // the plant changed its mind
    rig.bridge->collectOutputs(out);
    CHECK(native->physicsWrites.size() == 3);
    CHECK(native->physicsWrites.back().value == vendor::sim_motor_type::kKrakenX60);
}

TEST_CASE("vendor bridge: without the CTRE physics API the motor type is left alone") {
    auto native = std::make_shared<FakeNative>();  // physicsApi = false
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [native] { return native; };
    BridgeRig rig(options);
    addTalonFx(*rig.backend, 3);
    rig.start();
    rig.bridge->poll();
    const auto dev = rig.host.idOf("talonfx", 3);
    rig.bridge->applyInputs(std::vector<SignalSample>{
        {dev, rig.host.signalIndex(dev, sg::kMotorFamily),
         static_cast<double>(static_cast<int>(wpiemu::sdk::MotorFamily::KrakenX44))}});
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);
    CHECK(native->physicsWrites.empty());
}

TEST_CASE("vendor bridge: time-sync advice appears while Phoenix devices are live") {
    BridgeRig rig;
    rig.start();
    CHECK_FALSE(rig.bridge->timeSyncAdvice().paceToWallClock);
    addTalonFx(*rig.backend, 1);
    rig.bridge->poll();
    const auto advice = rig.bridge->timeSyncAdvice();
    CHECK(advice.paceToWallClock);
    CHECK(advice.maxRealtimeFactor == doctest::Approx(1.0));
    CHECK(advice.pauseUnsafe);
    CHECK_FALSE(advice.reason.empty());

    auto ignore = BridgeRig::defaultOptions();
    ignore.timeSync = vendor::TimeSyncPolicy::Ignore;
    BridgeRig quiet(ignore);
    addTalonFx(*quiet.backend, 1);
    quiet.start();
    quiet.bridge->poll();
    CHECK_FALSE(quiet.bridge->timeSyncAdvice().paceToWallClock);
}

TEST_CASE("vendor bridge: L3 hooks are optional, session-gated, and only feed the bus label") {
    auto hooks = std::make_shared<FakeHooks>();
    auto options = BridgeRig::defaultOptions();
    options.hooks = hooks;

    {  // session did not enable optional hooks: never installed
        BridgeRig rig(options);
        rig.host.hooksEnabled = false;
        rig.start();
        CHECK(hooks->installs == 0);
    }
    BridgeRig rig(options);
    rig.host.hooksEnabled = true;
    rig.start();
    REQUIRE(hooks->installs == 1);
    REQUIRE(hooks->sink != nullptr);

    // The constructor hook fires before the SimDevice exists: the observation is retried until announced.
    hooks->sink->onDeviceObserved({vendor::CtreDeviceType::P6_TalonFX, 4, "canivore"});
    rig.bridge->poll();
    CHECK(rig.host.observed.empty());
    addTalonFx(*rig.backend, 4);
    rig.bridge->poll();
    const DeviceId id = rig.host.idOf("talonfx", 4);
    REQUIRE(id.valid());
    REQUIRE(rig.host.observed.size() == 1);
    CHECK(rig.host.observed[0].first == id);
    CHECK(rig.host.observed[0].second == "canivore");
    CHECK(rig.bridge->observedBus(id) == "canivore");
    CHECK(rig.host.announced.at(id).descriptor.bus == wpiemu::sdk::kBusUnknown);  // static table never learns it

    rig.bridge->stop();
    CHECK(hooks->uninstalls == 1);
    rig.bridge->stop();  // idempotent
    CHECK(hooks->uninstalls == 1);
}

TEST_CASE("vendor bridge: read-only config read-back fills a typed snapshot after announce") {
    auto native = std::make_shared<FakeNative>();
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [native] { return native; };
    std::vector<int> reported;
    options.onConfig = [&](DeviceId, const DeviceDescriptor& d, const vendor::CtreConfigSnapshot& c) {
        reported.push_back(d.id);
        CHECK(c.sensorToMechanismRatio().value() == doctest::Approx(1.66667).epsilon(1e-5));
    };
    BridgeRig rig(options);
    addTalonFx(*rig.backend, 4);
    addCanCoder(*rig.backend, 9);  // CANcoder: P6 too, has a config as well
    rig.start();
    rig.bridge->poll();
    CHECK(native->reads == 0);  // discovery (poll) never enters CTRE's native sim
    std::vector<SignalSample> out;
    rig.bridge->collectOutputs(out);  // quiescent entry point: one device per call keeps ticks short
    rig.bridge->collectOutputs(out);
    rig.bridge->collectOutputs(out);
    CHECK(native->reads == 2);
    CHECK(native->lastBus == "rio");
    const DeviceId id = rig.host.idOf("talonfx", 4);
    const vendor::CtreConfigSnapshot* cfg = rig.bridge->configOf(id);
    REQUIRE(cfg != nullptr);
    CHECK(cfg->clockwisePositive().value() == true);
    CHECK(cfg->rotorToSensorRatio().value() == doctest::Approx(1.0));
    CHECK(cfg->feedbackSensorSource().value() == 1);
    CHECK(cfg->remoteSensorId().value() == 7);
    CHECK(reported.size() == 2);
    CHECK(rig.host.log.count(LogLevel::Info) >= 1);
}

TEST_CASE("vendor bridge: config read-back waits for the CTRE module and gives up politely") {
    auto native = std::make_shared<FakeNative>();
    bool moduleLoaded = false;
    int providerCalls = 0;
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [&]() -> std::shared_ptr<vendor::ICtreSimNative> {
        ++providerCalls;
        return moduleLoaded ? native : nullptr;
    };
    BridgeRig rig(options);
    addTalonFx(*rig.backend, 4);
    rig.start();
    for (int i = 0; i < 10; ++i) rig.bridge->poll();
    std::vector<SignalSample> out;
    for (int i = 0; i < 10; ++i) rig.bridge->collectOutputs(out);
    CHECK(providerCalls == 1);  // back-off: no hammering while the module is absent
    CHECK(native->reads == 0);
    CHECK(rig.bridge->configOf(rig.host.idOf("talonfx", 4)) == nullptr);

    moduleLoaded = true;
    for (int i = 0; i < 80; ++i) rig.bridge->collectOutputs(out);
    CHECK(native->reads == 1);
    CHECK(rig.bridge->configOf(rig.host.idOf("talonfx", 4)) != nullptr);
}

TEST_CASE("vendor bridge: failing or missing config API is tolerated") {
    auto native = std::make_shared<FakeNative>();
    native->failWith = ErrorCode::IoError;
    auto options = BridgeRig::defaultOptions();
    options.nativeProvider = [native] { return native; };
    options.configReadAttempts = 3;
    BridgeRig rig(options);
    addTalonFx(*rig.backend, 4);
    rig.start();
    std::vector<SignalSample> out;
    for (int i = 0; i < 10; ++i) {
        rig.bridge->poll();
        rig.bridge->collectOutputs(out);
    }
    CHECK(native->reads == 3);  // bounded retries
    CHECK(rig.bridge->configOf(rig.host.idOf("talonfx", 4)) == nullptr);

    auto missing = std::make_shared<FakeNative>();
    missing->configApi = false;
    auto options2 = BridgeRig::defaultOptions();
    options2.nativeProvider = [missing] { return missing; };
    BridgeRig rig2(options2);
    addTalonFx(*rig2.backend, 4);
    rig2.start();
    for (int i = 0; i < 5; ++i) {
        rig2.bridge->poll();
        rig2.bridge->collectOutputs(out);
    }
    CHECK(missing->reads == 0);
    CHECK(rig2.host.announced.size() == 1);  // discovery itself is unaffected
}

TEST_CASE("vendor bridge: no native provider means no L2 activity at all") {
    BridgeRig rig;  // default options: provider empty
    addTalonFx(*rig.backend, 4);
    rig.start();
    rig.bridge->poll();
    CHECK(rig.bridge->configOf(rig.host.idOf("talonfx", 4)) == nullptr);
}

TEST_CASE("vendor bridge: stop is idempotent and clears the session") {
    BridgeRig rig;
    addTalonFx(*rig.backend, 4);
    rig.start();
    rig.bridge->poll();
    CHECK(rig.bridge->liveDeviceCount() == 1);
    rig.bridge->stop();
    rig.bridge->stop();
    CHECK(rig.bridge->liveDeviceCount() == 0);
    rig.bridge->poll();  // after stop: harmless
    CHECK(rig.host.announced.size() == 1);
}
