#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "hal_platform_abi.h"
#include "wpiemu/hal_abi/hal_includes.h"
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
    bool schedulerArmed() const override { return abi::schedulerArmed(); }
    void stepTiming(std::uint64_t deltaUs) override { abi::stepTiming(deltaUs); }
    std::uint64_t robotLoopCount() const override { return loops_.load(std::memory_order_relaxed); }

private:
    static void onLoopEnd(void* self) { static_cast<Clock*>(self)->loops_.fetch_add(1, std::memory_order_relaxed); }

    std::atomic<std::uint64_t> loops_{0};
    std::int32_t loopCallbackUid_ = 0;
};

// ---- driver station ---------------------------------------------------------------------------------------
class HalSimPlatform::DriverStation final : public IDriverStationSink {
public:
    DriverStation() : lastJoysticks_(abi::maxJoysticks()), lastValid_(abi::maxJoysticks(), false) {}

    void apply(const proto::DsState& s) override {
        HALSIM_SetDriverStationDsAttached(s.dsAttached);
        HALSIM_SetDriverStationFmsAttached(s.fmsAttached);
        HALSIM_SetDriverStationEStop(s.eStop);
        abi::setRobotMode(s.mode);
        abi::setAllianceStation(s.station);
        HALSIM_SetDriverStationMatchTime(s.matchTimeS);
        if (!gameMessageValid_ || s.gameSpecificMessage != gameMessage_) {
            gameMessage_ = s.gameSpecificMessage;
            gameMessageValid_ = true;
            abi::setGameMessage(gameMessage_);
        }
        for (std::size_t i = 0; i < s.joysticks.size() && i < lastJoysticks_.size(); ++i) applyJoystick(i, s.joysticks[i]);
        // The enable bit goes last so the robot never sees "enabled" with a half-updated mode.
        HALSIM_SetDriverStationEnabled(s.enabled && !s.eStop && s.dsAttached);
        HALSIM_NotifyDriverStationNewData();
    }

private:
    static bool sameJoystick(const proto::JoystickState& a, const proto::JoystickState& b) {
        return a.present == b.present && a.axes == b.axes && a.povs == b.povs && a.buttons == b.buttons &&
               a.buttonCount == b.buttonCount;
    }

    void applyJoystick(std::size_t index, const proto::JoystickState& j) {
        if (lastValid_[index] && sameJoystick(lastJoysticks_[index], j)) return;
        lastJoysticks_[index] = j;
        lastValid_[index] = true;
        abi::setJoystick(static_cast<int>(index), j);
    }

    std::string gameMessage_;
    bool gameMessageValid_ = false;
    std::vector<proto::JoystickState> lastJoysticks_;
    std::vector<bool> lastValid_;
};

// ---- roboRIO power ------------------------------------------------------------------------------------------
class HalSimPlatform::Power final : public IRobotPowerSink {
public:
    void setBattery(double volts, double amps) override {
        HALSIM_SetRoboRioVInVoltage(volts);
        abi::setBatteryCurrent(amps);
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
    };
    for (auto& family : abi::extraPortCounts()) info.halPortCounts.push_back(std::move(family));
    const std::vector<std::pair<std::string, std::int32_t>> shared{
        {"encoder", HAL_GetNumEncoders()},
        {"counter", HAL_GetNumCounters()},
        {"duty_cycle", HAL_GetNumDutyCycles()},
        {"ctre_pcm", HAL_GetNumCTREPCMModules()},
        {"rev_ph", HAL_GetNumREVPHModules()},
        {"ctre_pdp", HAL_GetNumCTREPDPModules()},
        {"rev_pdh", HAL_GetNumREVPDHModules()},
        {"addressable_led", HAL_GetNumAddressableLEDs()},
    };
    info.halPortCounts.insert(info.halPortCounts.end(), shared.begin(), shared.end());
    return info;
}

}  // namespace wpiemu::halsim
