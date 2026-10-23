#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#if WPIEMU_WPILIB_YEAR >= 2027
#include "wpi/hal/HAL.h"
#include "wpi/hal/simulation/DriverStationData.h"
#include "wpi/hal/simulation/MockHooks.h"
#include "wpi/hal/simulation/NotifierData.h"
#include "wpi/hal/simulation/RoboRioData.h"
#else
#include "hal/HAL.h"
#include "hal/simulation/DriverStationData.h"
#include "hal/simulation/MockHooks.h"
#include "hal/simulation/NotifierData.h"
#include "hal/simulation/RoboRioData.h"
#endif
#include "wpiemu/halsim_ext/hal_sim_platform.h"
#include "wpiemu/halsim_ext/os_process.h"

#ifndef WPIEMU_WPILIB_VERSION
#define WPIEMU_WPILIB_VERSION "unknown"
#endif

namespace wpiemu::halsim {

namespace proto = sdk::proto;

// ---- clock -------------------------------------------------------------------------------------------
class HalSimPlatform::Clock final : public IHalClock {
public:
    Clock() { loopCallbackUid_ = HALSIM_RegisterSimPeriodicAfterCallback(&Clock::onLoopEnd, this); }
    ~Clock() override { HALSIM_CancelSimPeriodicAfterCallback(loopCallbackUid_); }

    void pauseTiming() override { HALSIM_PauseTiming(); }
    void resumeTiming() override { HALSIM_ResumeTiming(); }
    bool isTimingPaused() const override { return HALSIM_IsTimingPaused() != 0; }
    bool programStarted() const override { return HALSIM_GetProgramStarted() != 0; }
    bool schedulerArmed() const override { return HALSIM_GetNextNotifierTimeout() != std::numeric_limits<std::uint64_t>::max(); }
    void stepTiming(std::uint64_t deltaUs) override { HALSIM_StepTiming(deltaUs); }
    std::uint64_t robotLoopCount() const override { return loops_.load(std::memory_order_relaxed); }

private:
    static void onLoopEnd(void* self) { static_cast<Clock*>(self)->loops_.fetch_add(1, std::memory_order_relaxed); }

    std::atomic<std::uint64_t> loops_{0};
    std::int32_t loopCallbackUid_ = 0;
};

// ---- driver station ---------------------------------------------------------------------------------------
class HalSimPlatform::DriverStation final : public IDriverStationSink {
public:
    void apply(const proto::DsState& s) override {
        HALSIM_SetDriverStationDsAttached(s.dsAttached);
        HALSIM_SetDriverStationFmsAttached(s.fmsAttached);
        HALSIM_SetDriverStationEStop(s.eStop);
        HALSIM_SetDriverStationAutonomous(s.mode == proto::RobotMode::Autonomous);
        HALSIM_SetDriverStationTest(s.mode == proto::RobotMode::Test);
        HALSIM_SetDriverStationAllianceStationId(allianceStation(s.station));
        HALSIM_SetDriverStationMatchTime(s.matchTimeS);
        if (!gameMessageValid_ || s.gameSpecificMessage != gameMessage_) {
            gameMessage_ = s.gameSpecificMessage;
            gameMessageValid_ = true;
            const WPI_String message = wpi::make_string(gameMessage_);
            HALSIM_SetGameSpecificMessage(&message);
        }
        for (std::size_t i = 0; i < s.joysticks.size() && i < HAL_kMaxJoysticks; ++i) applyJoystick(static_cast<int>(i), s.joysticks[i]);
        // The enable bit goes last so the robot never sees "enabled" with a half-updated mode.
        HALSIM_SetDriverStationEnabled(s.enabled && !s.eStop && s.dsAttached);
        HALSIM_NotifyDriverStationNewData();
    }

private:
    static HAL_AllianceStationID allianceStation(proto::AllianceStation station) {
        switch (station) {
            case proto::AllianceStation::Red1: return HAL_AllianceStationID_kRed1;
            case proto::AllianceStation::Red2: return HAL_AllianceStationID_kRed2;
            case proto::AllianceStation::Red3: return HAL_AllianceStationID_kRed3;
            case proto::AllianceStation::Blue1: return HAL_AllianceStationID_kBlue1;
            case proto::AllianceStation::Blue2: return HAL_AllianceStationID_kBlue2;
            case proto::AllianceStation::Blue3: return HAL_AllianceStationID_kBlue3;
        }
        return HAL_AllianceStationID_kUnknown;
    }

    static bool sameJoystick(const proto::JoystickState& a, const proto::JoystickState& b) {
        return a.present == b.present && a.axes == b.axes && a.povs == b.povs && a.buttons == b.buttons &&
               a.buttonCount == b.buttonCount;
    }

    void applyJoystick(int index, const proto::JoystickState& j) {
        proto::JoystickState& last = lastJoysticks_[static_cast<std::size_t>(index)];
        if (lastValid_[static_cast<std::size_t>(index)] && sameJoystick(last, j)) return;
        last = j;
        lastValid_[static_cast<std::size_t>(index)] = true;

        HAL_JoystickAxes axes{};
        HAL_JoystickPOVs povs{};
        HAL_JoystickButtons buttons{};
        HAL_JoystickDescriptor descriptor{};
        if (j.present) {
            axes.count = static_cast<std::int16_t>(std::min<std::size_t>(j.axes.size(), HAL_kMaxJoystickAxes));
            for (int a = 0; a < axes.count; ++a) {
                const float v = std::clamp(j.axes[static_cast<std::size_t>(a)], -1.0f, 1.0f);
                axes.axes[a] = v;
                axes.raw[a] = static_cast<std::uint8_t>(std::lround((v + 1.0f) * 127.5f));
            }
            povs.count = static_cast<std::int16_t>(std::min<std::size_t>(j.povs.size(), HAL_kMaxJoystickPOVs));
            for (int p = 0; p < povs.count; ++p) povs.povs[p] = j.povs[static_cast<std::size_t>(p)];
            buttons.count = static_cast<std::uint8_t>(std::min<int>(j.buttonCount, 32));
            buttons.buttons = j.buttons;
            descriptor.isXbox = 1;
            descriptor.type = 21;  // kHIDGamepad
            std::snprintf(descriptor.name, sizeof(descriptor.name), "WPI Emulator gamepad %d", index);
            descriptor.axisCount = static_cast<std::uint8_t>(axes.count);
            descriptor.buttonCount = buttons.count;
            descriptor.povCount = static_cast<std::uint8_t>(povs.count);
        }
        HALSIM_SetJoystickDescriptor(index, &descriptor);
        HALSIM_SetJoystickAxes(index, &axes);
        HALSIM_SetJoystickPOVs(index, &povs);
        HALSIM_SetJoystickButtons(index, &buttons);
    }

    std::string gameMessage_;
    bool gameMessageValid_ = false;
    std::array<proto::JoystickState, HAL_kMaxJoysticks> lastJoysticks_{};
    std::array<bool, HAL_kMaxJoysticks> lastValid_{};
};

// ---- roboRIO power ------------------------------------------------------------------------------------------
class HalSimPlatform::Power final : public IRobotPowerSink {
public:
    void setBattery(double volts, double amps) override {
        HALSIM_SetRoboRioVInVoltage(volts);
        HALSIM_SetRoboRioVInCurrent(amps);
    }
};

HalSimPlatform::HalSimPlatform()
    : clock_(std::make_unique<Clock>()), driverStation_(std::make_unique<DriverStation>()), power_(std::make_unique<Power>()) {}

HalSimPlatform::~HalSimPlatform() = default;

IHalClock& HalSimPlatform::clock() { return *clock_; }
IDriverStationSink& HalSimPlatform::driverStation() { return *driverStation_; }
IRobotPowerSink& HalSimPlatform::power() { return *power_; }

RobotInfo HalSimPlatform::describeRobot(const ExtensionConfig& config) {
    RobotInfo info;
    info.language = detectRobotLanguage();
    info.wpilibVersion = WPIEMU_WPILIB_VERSION;
    info.wpilibYear = std::atoi(info.wpilibVersion.c_str());  // "2026.2.1" -> 2026; 0 when unknown
    info.extensionVersion = "wpiemu-halsim-1";
    info.pid = currentProcessId();
    info.programName = config.programName.empty() ? executableName() : config.programName;
    info.halPortCounts = {
        {"pwm", HAL_GetNumPWMChannels()},
        {"dio", HAL_GetNumDigitalChannels()},
        {"analog_in", HAL_GetNumAnalogInputs()},
        {"analog_out", HAL_GetNumAnalogOutputs()},
        {"relay", HAL_GetNumRelayHeaders()},
        {"encoder", HAL_GetNumEncoders()},
        {"counter", HAL_GetNumCounters()},
        {"duty_cycle", HAL_GetNumDutyCycles()},
        {"ctre_pcm", HAL_GetNumCTREPCMModules()},
        {"rev_ph", HAL_GetNumREVPHModules()},
        {"ctre_pdp", HAL_GetNumCTREPDPModules()},
        {"rev_pdh", HAL_GetNumREVPDHModules()},
        {"addressable_led", HAL_GetNumAddressableLEDs()},
    };
    return info;
}

}  // namespace wpiemu::halsim
