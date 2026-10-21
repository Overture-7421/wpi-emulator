#include <doctest/doctest.h>

#include <atomic>
#include <thread>

#include "fakes.h"
#include "hal/HAL.h"
#include "hal/simulation/DIOData.h"
#include "hal/simulation/DriverStationData.h"
#include "hal/simulation/MockHooks.h"
#include "hal/simulation/PWMData.h"
#include "hal/simulation/RoboRioData.h"
#include "wpiemu/halsim_ext/hal_sim_platform.h"
#include "wpiemu/halsim_ext/os_process.h"
#include "wpiemu/sdk/testing/fake_emulator.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

namespace {

// A bridge over the real HALSIM_* data for PWM[0] (output) and DIO[1] (input).
class HalProbeBridge final : public sdk::IDeviceBridge {
public:
    std::string_view name() const override { return "test.hal_probe"; }
    sdk::Status start(sdk::IBridgeHost& host) override {
        host_ = &host;
        return sdk::okStatus();
    }
    void poll() override {
        if (!pwm_.valid() && HALSIM_GetPWMInitialized(0)) {
            sdk::DeviceDescriptor d;
            d.family = "hal";
            d.type = "pwm";
            d.id = 0;
            pwm_ = host_->announce(d, {{"speed", sdk::SignalDirection::Output, sdk::SignalType::Double, ""}});
        }
        if (!dio_.valid() && HALSIM_GetDIOInitialized(1)) {
            sdk::DeviceDescriptor d;
            d.family = "hal";
            d.type = "dio";
            d.id = 1;
            dio_ = host_->announce(d, {{"value", sdk::SignalDirection::Input, sdk::SignalType::Bool, ""}});
        }
    }
    void collectOutputs(std::vector<sdk::SignalSample>& out) override {
        if (pwm_.valid()) out.push_back({pwm_, 0, HALSIM_GetPWMSpeed(0)});
    }
    void applyInputs(std::span<const sdk::SignalSample> samples) override {
        for (const auto& s : samples)
            if (s.device == dio_) HALSIM_SetDIOValue(1, s.value != 0.0);
    }
    void applyLoopback(sdk::DeviceId) override {}
    bool owns(sdk::DeviceId id) const override { return id == pwm_ || id == dio_; }
    void stop() noexcept override {}

    sdk::DeviceId pwm() const { return pwm_; }
    sdk::DeviceId dio() const { return dio_; }

private:
    sdk::IBridgeHost* host_ = nullptr;
    sdk::DeviceId pwm_;
    sdk::DeviceId dio_;
};

// What a TimedRobot does through the C HAL: create devices, observe program start, then run a 20 ms
// notifier loop that reads the DS enable bit, drives a PWM and samples a DIO.
class MiniRobot {
public:
    MiniRobot() : thread_([this] { run(); }) {}
    ~MiniRobot() {
        stop_ = true;
        std::int32_t status = 0;
        if (notifier_ != 0) HAL_StopNotifier(notifier_, &status);
        thread_.join();
    }
    std::atomic<int> loops{0};
    std::atomic<bool> dioValue{true};
    std::atomic<bool> enabledSeen{false};
    std::atomic<std::uint64_t> startFpgaUs{0};

private:
    void run() {
        std::int32_t status = 0;
        const HAL_DigitalHandle pwm = HAL_InitializePWMPort(HAL_GetPort(0), "test", &status);
        HAL_SetPWMConfigMicroseconds(pwm, 2000, 1501, 1500, 1499, 1000, &status);
        const HAL_DigitalHandle dio = HAL_InitializeDIOPort(HAL_GetPort(1), true, "test", &status);
        notifier_ = HAL_InitializeNotifier(&status);
        HAL_ObserveUserProgramStarting();
        std::uint64_t next = HAL_GetFPGATime(&status) + 20000;
        startFpgaUs = next - 20000;
        while (!stop_) {
            HAL_UpdateNotifierAlarm(notifier_, next, &status);
            if (HAL_WaitForNotifierAlarm(notifier_, &status) == 0) break;
            ++loops;
            const bool enabled = HALSIM_GetDriverStationEnabled() != 0;
            if (enabled) enabledSeen = true;
            HAL_SetPWMSpeed(pwm, enabled ? 0.5 : 0.0, &status);
            dioValue = HAL_GetDIO(dio, &status) != 0;
            next += 20000;
        }
        HAL_CleanNotifier(notifier_);
        HAL_FreePWMPort(pwm);
        HAL_FreeDIOPort(dio);
    }

    std::atomic<bool> stop_{false};
    HAL_NotifierHandle notifier_ = 0;
    std::thread thread_;
};

struct RealRig {
    explicit RealRig(proto::HelloAck ack = defaultAck()) : config(Rig::defaultConfig()) {
        codec = std::make_shared<sdk::testing::MemoryMessageCodec>();
        auto pair = sdk::testing::makeInMemoryTransportPair();
        client.push(std::move(pair.b));
        emulator = std::make_unique<sdk::testing::FakeEmulator>(std::move(pair.a), codec);
        emulator->sessionId = ack.sessionId;
        emulator->autoAck(std::move(ack));
        auto probe = std::make_unique<HalProbeBridge>();
        bridge = probe.get();
        bridges.push_back(std::move(probe));
    }
    sdk::Status start() {
        RuntimeDeps deps{config,
                         HalSimPlatform::describeRobot(config),
                         HalPorts{platform.clock(), platform.driverStation(), platform.power(), process},
                         client,
                         codec,
                         std::move(bridges),
                         [](std::string_view) {}};
        runtime = std::make_unique<ExtensionRuntime>(std::move(deps));
        return runtime->start();
    }
    ~RealRig() {
        if (runtime) runtime->stop();
    }

    ExtensionConfig config;
    std::shared_ptr<sdk::testing::MemoryMessageCodec> codec;
    sdk::testing::QueuedTransportClient client;
    std::unique_ptr<sdk::testing::FakeEmulator> emulator;
    HalSimPlatform platform;
    FakeProcess process;
    HalProbeBridge* bridge = nullptr;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    std::unique_ptr<ExtensionRuntime> runtime;
};

}  // namespace

TEST_CASE("real HAL: describeRobot reports the runtime and port counts") {
    ExtensionConfig cfg;
    const RobotInfo info = HalSimPlatform::describeRobot(cfg);
    CHECK(info.pid == currentProcessId());
    CHECK(info.wpilibYear == 2026);
    CHECK(info.language == proto::RobotLanguage::Cpp);
    CHECK_FALSE(info.programName.empty());
    bool hasPwm = false;
    for (const auto& [name, count] : info.halPortCounts)
        if (name == "pwm") hasPwm = count > 0;
    CHECK(hasPwm);
}

TEST_CASE("real HAL: lock-step drives the robot loop deterministically and exchanges device values") {
    RealRig rig;
    REQUIRE(rig.start());
    CHECK(HALSIM_IsTimingPaused());  // paused before any robot code exists

    MiniRobot robot;
    REQUIRE(rig.emulator->waitForLifecycle(5000ms));
    const auto pwmAnnounce = rig.emulator->take<proto::DeviceAnnounce>(2000ms, [](const proto::DeviceAnnounce& a) { return a.descriptor.type == "pwm"; });
    const auto dioAnnounce = rig.emulator->take<proto::DeviceAnnounce>(2000ms, [](const proto::DeviceAnnounce& a) { return a.descriptor.type == "dio"; });
    REQUIRE(pwmAnnounce.has_value());
    REQUIRE(dioAnnounce.has_value());

    proto::DsState ds;
    ds.enabled = true;
    ds.joysticks[0].present = true;
    ds.joysticks[0].axes = {0.25f};
    REQUIRE(rig.emulator->sendDs(ds));

    std::int32_t status = 0;
    const std::uint64_t t0 = HAL_GetFPGATime(&status);
    bool sawPwmHalf = false;
    for (int i = 0; i < 100; ++i) {
        std::vector<sdk::SignalSample> samples;
        if (i == 0) samples.push_back({dioAnnounce->id, 0, 0.0});  // sim drives the DIO low
        const auto ack = rig.emulator->tick(std::move(samples), 5000, 12.2);
        REQUIRE(ack.has_value());
        REQUIRE(ack->result == proto::TickResult::Completed);
        for (const auto& s : ack->samples)
            if (s.device == pwmAnnounce->id && s.value > 0.45 && s.value < 0.55) sawPwmHalf = true;  // the HAL quantises through the pulse width
    }
    const std::uint64_t t1 = HAL_GetFPGATime(&status);

    CHECK(t1 - t0 == 500000);                     // exactly 100 x 5 ms of sim time
    CHECK(robot.loops == 25);                     // 20 ms TimedRobot period => 25 loops, deterministic
    CHECK(robot.enabledSeen);                     // DS state reached the robot
    CHECK(sawPwmHalf);                            // robot output reached the emulator
    CHECK_FALSE(robot.dioValue);                  // emulator input reached the robot
    CHECK(HALSIM_GetRoboRioVInVoltage() == doctest::Approx(12.2));
    HAL_JoystickAxes axes{};
    HALSIM_GetJoystickAxes(0, &axes);
    CHECK(axes.count == 1);
    CHECK(axes.axes[0] == doctest::Approx(0.25f));

    rig.runtime->stop();
    CHECK_FALSE(HALSIM_IsTimingPaused());
}

TEST_CASE("real HAL: a vanished emulator never leaves the robot frozen") {
    RealRig rig;
    REQUIRE(rig.start());
    CHECK(HALSIM_IsTimingPaused());
    MiniRobot robot;
    REQUIRE(rig.emulator->waitForLifecycle(5000ms));
    REQUIRE(rig.emulator->tick());

    rig.emulator->disconnect();
    REQUIRE(rig.runtime->waitFinished(3000ms));
    CHECK_FALSE(HALSIM_IsTimingPaused());

    // Real time flows again: the robot loop keeps running on its own.
    const int before = robot.loops;
    CHECK(waitUntil([&] { return robot.loops > before; }, 3000ms));
}
