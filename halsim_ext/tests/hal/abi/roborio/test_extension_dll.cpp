#include <doctest/doctest.h>

#include <atomic>
#include <thread>

#include "fakes.h"
#include "hal/HAL.h"
#include "hal/Extensions.h"
#include "hal/simulation/DriverStationData.h"
#include "hal/simulation/MockHooks.h"
#include "scoped_env.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/testing/fake_emulator.h"

// Loads the REAL halsim_wpiemu shared library through the HAL's own extension loader (exactly what
// HALSIM_EXTENSIONS does in a robot process), connects it to a fake emulator over a real named pipe /
// unix socket with the real binary codec, and lets it drive the real built-in HAL bridges.

using namespace wpiemu;
using namespace wpiemu::halsim::testing;
using wpiemu::test_support::ScopedEnv;

namespace {

class DllRobot {
public:
    DllRobot() : thread_([this] { run(); }) {}
    ~DllRobot() {
        stop_ = true;
        std::int32_t status = 0;
        if (notifier_ != 0) HAL_StopNotifier(notifier_, &status);
        thread_.join();
    }
    std::atomic<int> loops{0};
    std::atomic<bool> enabledSeen{false};

private:
    void run() {
        std::int32_t status = 0;
        const HAL_DigitalHandle pwm = HAL_InitializePWMPort(HAL_GetPort(2), "dll-test", &status);
        HAL_SetPWMConfigMicroseconds(pwm, 2000, 1501, 1500, 1499, 1000, &status);
        notifier_ = HAL_InitializeNotifier(&status);
        HAL_ObserveUserProgramStarting();
        std::uint64_t next = HAL_GetFPGATime(&status) + 20000;
        while (!stop_) {
            HAL_UpdateNotifierAlarm(notifier_, next, &status);
            if (HAL_WaitForNotifierAlarm(notifier_, &status) == 0) break;
            ++loops;
            const bool enabled = HALSIM_GetDriverStationEnabled() != 0;
            if (enabled) enabledSeen = true;
            HAL_SetPWMSpeed(pwm, enabled ? 1.0 : 0.0, &status);
            next += 20000;
        }
        HAL_CleanNotifier(notifier_);
        HAL_FreePWMPort(pwm);
    }
    std::atomic<bool> stop_{false};
    HAL_NotifierHandle notifier_ = 0;
    std::thread thread_;
};

}  // namespace

TEST_CASE("real extension library: loads through the HAL extension loader and runs the robot in lock-step over a real pipe") {
    const ipc::ConnectionEndpoint endpoint = ipc::ConnectionEndpoint::generate();
    auto server = ipc::makeLocalServer(endpoint);
    REQUIRE(server);
    REQUIRE(server.value()->listen());

    auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::unique_ptr<sdk::testing::FakeEmulator> emulator;
    std::thread accepting([&] {
        auto transport = server.value()->accept(std::chrono::seconds(20));
        if (transport && transport.value()) {
            emulator = std::make_unique<sdk::testing::FakeEmulator>(std::move(transport).value(), codec);
            proto::HelloAck ack = defaultAck();
            ack.watchdogTimeoutMs = 5000;
            emulator->autoAck(ack);
        }
    });

    const ScopedEnv endpointVar("WPIEMU_IPC_ENDPOINT", endpoint.address());
    const ScopedEnv logLevelVar("WPIEMU_LOG_LEVEL", "debug");
    const int rc = HAL_LoadOneExtension(WPIEMU_EXTENSION_PATH);
    accepting.join();
    REQUIRE(rc == 0);
    REQUIRE(emulator != nullptr);
    CHECK(HALSIM_IsTimingPaused());  // the extension took the clock before any robot code ran

    const auto hello = emulator->take<proto::Hello>(2000ms);
    REQUIRE(hello.has_value());
    CHECK(hello->wpilibYear == std::atoi(WPIEMU_WPILIB_VERSION));
    CHECK(hello->language == proto::RobotLanguage::Cpp);
    CHECK(hello->halPortCounts.size() > 5);
    CHECK(hello->pid > 0);

    {
        DllRobot robot;
        REQUIRE(emulator->waitForLifecycle(10000ms));
        const auto pwm = emulator->take<proto::DeviceAnnounce>(2000ms, [](const proto::DeviceAnnounce& a) {
            return a.descriptor.family == "hal" && a.descriptor.type == "pwm" && a.descriptor.id == 2;
        });
        REQUIRE(pwm.has_value());
        CHECK_FALSE(pwm->signalSpecs.empty());

        proto::DsState ds;
        ds.enabled = true;
        REQUIRE(emulator->sendDs(ds));

        std::int32_t status = 0;
        const std::uint64_t t0 = HAL_GetFPGATime(&status);
        bool sawFullSpeed = false;
        for (int i = 0; i < 40; ++i) {
            const auto ack = emulator->tick({}, 5000, 12.4, 5000ms);
            REQUIRE(ack.has_value());
            REQUIRE(ack->result == proto::TickResult::Completed);
            for (const auto& s : ack->samples)
                if (s.device == pwm->id && s.value > 0.9) sawFullSpeed = true;
        }
        CHECK(HAL_GetFPGATime(&status) - t0 == 200000);
        CHECK(robot.loops == 10);
        CHECK(robot.enabledSeen);
        CHECK(sawFullSpeed);
        // Stop is honoured: timing is released and the extension says Goodbye.
        REQUIRE(emulator->sendControl(proto::SimCommand::Stop, 77));
        CHECK(emulator->take<proto::SimControlAck>(3000ms).has_value());
        // Goodbye follows the final timing resume, so the next test can safely own HAL's clock.
        REQUIRE(emulator->take<proto::Goodbye>(3000ms).has_value());
        CHECK(waitUntil([] { return !HALSIM_IsTimingPaused(); }, 3000ms));
    }
}
