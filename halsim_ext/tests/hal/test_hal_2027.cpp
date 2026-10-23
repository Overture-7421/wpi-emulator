#include <doctest/doctest.h>

#include <atomic>
#include <limits>
#include <thread>

#include "fakes.h"
#include "scoped_env.h"
#include "wpi/hal/CAN.h"
#include "wpi/hal/CANAPITypes.h"
#include "wpi/hal/DIO.h"
#include "wpi/hal/DriverStation.h"
#include "wpi/hal/Encoder.h"
#include "wpi/hal/Extensions.h"
#include "wpi/hal/HAL.h"
#include "wpi/hal/IMU.h"
#include "wpi/hal/Notifier.h"
#include "wpi/hal/PWM.h"
#include "wpi/hal/SimDevice.h"
#include "wpi/hal/simulation/DIOData.h"
#include "wpi/hal/simulation/DriverStationData.h"
#include "wpi/hal/simulation/EncoderData.h"
#include "wpi/hal/simulation/MockHooks.h"
#include "wpi/hal/simulation/PWMData.h"
#include "wpi/util/Synchronization.h"
#include "wpiemu/halsim_ext/hal_sim_platform.h"
#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/ipc/endpoint.h"
#include "wpiemu/ipc/local_ipc.h"
#include "wpiemu/sdk/testing/fake_emulator.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

TEST_CASE("2027 HAL: driver station converts modes, joystick masks and POV hats") {
    HalSimPlatform platform;
    proto::DsState ds;
    ds.enabled = true;
    ds.mode = proto::RobotMode::Autonomous;
    ds.station = proto::AllianceStation::Blue2;
    ds.gameSpecificMessage = "ABC";
    auto& joystick = ds.joysticks[0];
    joystick.present = true;
    joystick.axes = {-1.0f, 0.5f};
    joystick.povs = {-1, 45, 180, 315};
    joystick.buttonCount = 32;
    joystick.buttons = 0x80000001u;
    platform.driverStation().apply(ds);
    CHECK(HALSIM_GetDriverStationRobotMode() == HAL_ROBOT_MODE_AUTONOMOUS);
    CHECK(HALSIM_GetDriverStationEnabled());
    CHECK(HALSIM_GetDriverStationAllianceStationId() == HAL_ALLIANCE_STATION_BLUE_2);
    HAL_JoystickAxes axes{};
    HALSIM_GetJoystickAxes(0, &axes);
    CHECK(axes.available == 3);
    CHECK(axes.axes[1] == doctest::Approx(0.5));
    HAL_JoystickPOVs povs{};
    HALSIM_GetJoystickPOVs(0, &povs);
    CHECK(povs.available == 15);
    CHECK(povs.povs[0] == HAL_JOYSTICK_POV_CENTERED);
    CHECK(povs.povs[1] == HAL_JOYSTICK_POV_RIGHT_UP);
    CHECK(povs.povs[2] == HAL_JOYSTICK_POV_DOWN);
    CHECK(povs.povs[3] == HAL_JOYSTICK_POV_LEFT_UP);
    HAL_JoystickButtons buttons{};
    HALSIM_GetJoystickButtons(0, &buttons);
    CHECK(buttons.available == 0xffffffffu);
    CHECK(buttons.buttons == 0x80000001u);
    HAL_GameData game{};
    HALSIM_GetGameData(&game);
    CHECK(std::string(game.gameData) == "ABC");
    ds.mode = proto::RobotMode::Test;
    ds.eStop = true;
    joystick.present = false;
    platform.driverStation().apply(ds);
    CHECK(HALSIM_GetDriverStationRobotMode() == HAL_ROBOT_MODE_UTILITY);
    CHECK_FALSE(HALSIM_GetDriverStationEnabled());
    HALSIM_GetJoystickButtons(0, &buttons);
    CHECK(buttons.available == 0);
}

namespace {
class Robot2027 {
public:
    Robot2027() {
        std::int32_t status = 0;
        notifier_ = HAL_CreateNotifier(&status);
        thread_ = std::thread([this] { run(); });
    }
    ~Robot2027() {
        stop_ = true;
        HAL_DestroyNotifier(notifier_);
        thread_.join();
    }
    std::atomic<int> loops{0};
    std::atomic<bool> inputSeen{false};
    std::atomic<bool> enabledSeen{false};

private:
    void run() {
        std::int32_t status = 0;
        const auto pwm = HAL_InitializePWMPort(2, "2027 robot", &status);
        const auto dio = HAL_InitializeDIOPort(1, true, "2027 robot", &status);
        HAL_ObserveUserProgramStarting();
        HAL_SetNotifierAlarm(notifier_, 20000000, 20000000, false, true, &status);
        while (!stop_ && WPI_WaitForObject(notifier_)) {
            if (stop_) break;
            const bool enabled = HALSIM_GetDriverStationEnabled() != 0;
            enabledSeen = enabled;
            HAL_SetPWMPulseTimeMicroseconds(pwm, enabled ? 1750 : 1500, &status);
            inputSeen = HAL_GetDIO(dio, &status) == 0;
            ++loops;
            HAL_SimPeriodicAfter();
            HAL_AcknowledgeNotifierAlarm(notifier_, &status);
        }
        HAL_FreePWMPort(pwm);
        HAL_FreeDIOPort(dio);
    }
    HAL_NotifierHandle notifier_ = 0;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};
}

TEST_CASE("2027 extension: real library handshakes and exchanges lockstep ticks over local IPC") {
    const auto endpoint = ipc::ConnectionEndpoint::generate();
    auto server = ipc::makeLocalServer(endpoint);
    REQUIRE(server);
    REQUIRE(server.value()->listen());
    auto codec = std::make_shared<ipc::BinaryMessageCodec>();
    std::unique_ptr<sdk::testing::FakeEmulator> emulator;
    std::thread accepting([&] {
        auto transport = server.value()->accept(10s);
        if (transport && transport.value()) {
            emulator = std::make_unique<sdk::testing::FakeEmulator>(std::move(transport).value(), codec);
            auto ack = defaultAck();
            ack.watchdogTimeoutMs = 5000;
            emulator->autoAck(ack);
        }
    });
    const test_support::ScopedEnv endpointVar("WPIEMU_IPC_ENDPOINT", endpoint.address());
    const int rc = HAL_LoadOneExtension(WPIEMU_EXTENSION_PATH);
    accepting.join();
    REQUIRE(rc == 0);
    REQUIRE(emulator);
    const auto hello = emulator->take<proto::Hello>(2s);
    REQUIRE(hello);
    CHECK(hello->wpilibYear == WPIEMU_WPILIB_YEAR);
    CHECK(hello->wpilibVersion == WPIEMU_WPILIB_VERSION);
    CHECK(HALSIM_IsTimingPaused());
    Robot2027 robot;
    REQUIRE(emulator->waitForLifecycle(5s));
    const auto dio = emulator->take<proto::DeviceAnnounce>(2s, [](const auto& a) {
        return a.descriptor.type == "dio" && a.descriptor.id == 1;
    });
    const auto pwm = emulator->take<proto::DeviceAnnounce>(2s, [](const auto& a) {
        return a.descriptor.type == "pwm" && a.descriptor.id == 2;
    });
    REQUIRE(dio);
    REQUIRE(pwm);
    proto::DsState ds;
    ds.enabled = true;
    ds.mode = proto::RobotMode::Teleop;
    REQUIRE(emulator->sendDs(ds));
    const auto start = HAL_GetMonotonicTime();
    bool outputSeen = false;
    for (int i = 0; i < 40; ++i) {
        const auto ack = emulator->tick({{dio->id, 0, 0.0}}, 5000, 12.0, 5s);
        REQUIRE(ack);
        REQUIRE(ack->result == proto::TickResult::Completed);
        for (const auto& sample : ack->samples)
            if (sample.device == pwm->id && sample.value == 1750) outputSeen = true;
    }
    CHECK(HAL_GetMonotonicTime() - start == 200000000);
    CHECK(robot.loops == 10);
    CHECK(robot.inputSeen);
    CHECK(robot.enabledSeen);
    CHECK(outputSeen);
    emulator->disconnect();
    REQUIRE(waitUntil([] { return !HALSIM_IsTimingPaused(); }, 5s));
    const auto before = robot.loops.load();
    CHECK(waitUntil([&] { return robot.loops > before; }, 3s));
}
