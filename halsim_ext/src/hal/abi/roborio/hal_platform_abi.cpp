// Platform HAL calls of the roborio generation (WPILib 2026): microsecond clock, boolean autonomous/test mode flags,
// 8-bit joystick axes, XInput-style descriptors and the roboRIO's input current and port families.
#include "hal_platform_abi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "wpiemu/hal_abi/hal_includes.h"

namespace wpiemu::halsim::abi {

namespace proto = sdk::proto;

bool schedulerArmed() { return HALSIM_GetNextNotifierTimeout() != std::numeric_limits<std::uint64_t>::max(); }

void stepTiming(std::uint64_t deltaUs) { HALSIM_StepTiming(deltaUs); }

void setRobotMode(proto::RobotMode mode) {
    HALSIM_SetDriverStationAutonomous(mode == proto::RobotMode::Autonomous);
    HALSIM_SetDriverStationTest(mode == proto::RobotMode::Test);
}

void setAllianceStation(proto::AllianceStation station) {
    HAL_AllianceStationID id = HAL_AllianceStationID_kUnknown;
    switch (station) {
        case proto::AllianceStation::Red1: id = HAL_AllianceStationID_kRed1; break;
        case proto::AllianceStation::Red2: id = HAL_AllianceStationID_kRed2; break;
        case proto::AllianceStation::Red3: id = HAL_AllianceStationID_kRed3; break;
        case proto::AllianceStation::Blue1: id = HAL_AllianceStationID_kBlue1; break;
        case proto::AllianceStation::Blue2: id = HAL_AllianceStationID_kBlue2; break;
        case proto::AllianceStation::Blue3: id = HAL_AllianceStationID_kBlue3; break;
    }
    HALSIM_SetDriverStationAllianceStationId(id);
}

void setGameMessage(const std::string& message) {
    const WPI_String text = wpi::make_string(message);
    HALSIM_SetGameSpecificMessage(&text);
}

std::size_t maxJoysticks() { return HAL_kMaxJoysticks; }

void setJoystick(int index, const proto::JoystickState& j) {
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
        descriptor.axisCount = static_cast<std::uint8_t>(axes.count);
        descriptor.buttonCount = buttons.count;
        descriptor.povCount = static_cast<std::uint8_t>(povs.count);
        std::snprintf(descriptor.name, sizeof(descriptor.name), "WPI Emulator gamepad %d", index);
    }
    HALSIM_SetJoystickDescriptor(index, &descriptor);
    HALSIM_SetJoystickAxes(index, &axes);
    HALSIM_SetJoystickPOVs(index, &povs);
    HALSIM_SetJoystickButtons(index, &buttons);
}

void setBatteryCurrent(double amps) { HALSIM_SetRoboRioVInCurrent(amps); }

std::vector<std::pair<std::string, std::int32_t>> extraPortCounts() {
    return {{"analog_out", HAL_GetNumAnalogOutputs()}, {"relay", HAL_GetNumRelayHeaders()}};
}

}  // namespace wpiemu::halsim::abi
