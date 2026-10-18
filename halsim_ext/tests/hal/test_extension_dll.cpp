#include <doctest/doctest.h>

#include <atomic>
#include <cstdlib>
#include <thread>

#include "fakes.h"
#include "hal/HAL.h"
#include "hal/Extensions.h"
#include "hal/simulation/DriverStationData.h"
#include "hal/simulation/MockHooks.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/local_ipc.h"

// Loads the REAL halsim_wpiemu shared library through the HAL's own extension loader (exactly what
// HALSIM_EXTENSIONS does in a robot process), connects it to a fake emulator over a real named pipe /
// unix socket with the real binary codec, and lets it drive the real built-in HAL bridges.

using namespace wpiemu;
using namespace wpiemu::halsim::testing;

namespace {

void setEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

// The camera setup and a camera frame, so that the real binary codec and the real library see
// vision on the wire.
sdk::VisionSetup cameraSetup() {
    sdk::CameraSetup camera;
    camera.cameraId = camera.ntName = "photon-front";
    camera.widthPx = 1280;
    camera.heightPx = 800;
    camera.fxPx = camera.fyPx = 640.0;
    camera.cxPx = 640.0;
    camera.cyPx = 400.0;
    camera.latencyUs = 25000;
    sdk::VisionSetup setup;
    setup.cameras.push_back(camera);
    return setup;
}

sdk::VisionFrame cameraFrame(std::uint64_t sequence) {
    sdk::FiducialObservation tag;
    tag.id = 25;
    tag.areaPct = 1.5;
    tag.distanceM = 2.0;
    tag.cornersPx = {{{600.0, 420.0}, {660.0, 420.0}, {660.0, 360.0}, {600.0, 360.0}}};
    tag.cameraToTagBest.position = {2.0f, 0.0f, 0.0f};
    tag.cameraToTagAlt = tag.cameraToTagBest;
    sdk::VisionFrame frame;
    frame.cameraId = frame.ntName = "photon-front";
    frame.sequence = sequence;
    frame.latencyUs = 25000;
    frame.observations.push_back(tag);
    return frame;
}

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
    std::unique_ptr<FakeEmulator> emulator;
    std::thread accepting([&] {
        auto transport = server.value()->accept(std::chrono::seconds(20));
        if (transport && transport.value()) {
            emulator = std::make_unique<FakeEmulator>(std::move(transport).value(), codec);
            proto::HelloAck ack = defaultAck();
            ack.watchdogTimeoutMs = 5000;
            // Camera setup and frames cross the real codec and the real library.
            ack.vision = cameraSetup();
            emulator->autoAck(ack);
        }
    });

    setEnv("WPIEMU_IPC_ENDPOINT", endpoint.address());
    setEnv("WPIEMU_LOG_LEVEL", "debug");
    const int rc = HAL_LoadOneExtension(WPIEMU_EXTENSION_PATH);
    accepting.join();
    REQUIRE(rc == 0);
    REQUIRE(emulator != nullptr);
    CHECK(HALSIM_IsTimingPaused());  // the extension took the clock before any robot code ran

    const auto hello = emulator->take<proto::Hello>(2000ms);
    REQUIRE(hello.has_value());
    CHECK(hello->wpilibYear == 2026);
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
            if (i % 4 == 0) emulator->nextVision = {cameraFrame(static_cast<std::uint64_t>(i) + 1)};
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
        CHECK(waitUntil([] { return !HALSIM_IsTimingPaused(); }, 3000ms));
    }
}
