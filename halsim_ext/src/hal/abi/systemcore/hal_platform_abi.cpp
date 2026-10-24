// Platform HAL calls of the systemcore generation (WPILib 2027): nanosecond clock, a robot-mode enum, 16-bit availability
// masks for axes/POVs/buttons, no controller input current, and CAN-bus and smart-IO port families.
#include "hal_platform_abi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "wpiemu/hal_abi/hal_includes.h"

namespace wpiemu::halsim::abi {

namespace proto = sdk::proto;

bool schedulerArmed() { return HALSIM_GetNextNotifierTimeout() != std::numeric_limits<std::int64_t>::max(); }

void stepTiming(std::uint64_t deltaUs) {
    constexpr auto limit = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
    if (deltaUs > limit) throw std::overflow_error("HAL timing step exceeds nanosecond range");
    HALSIM_StepTiming(static_cast<std::int64_t>(deltaUs) * 1000);
}

void setRobotMode(proto::RobotMode mode) {
    HALSIM_SetDriverStationRobotMode(mode == proto::RobotMode::Autonomous ? HAL_ROBOT_MODE_AUTONOMOUS
                                     : mode == proto::RobotMode::Test     ? HAL_ROBOT_MODE_UTILITY
                                                                          : HAL_ROBOT_MODE_TELEOPERATED);
}

void setAllianceStation(proto::AllianceStation station) {
    HAL_AllianceStationID id = HAL_ALLIANCE_STATION_UNKNOWN;
    switch (station) {
        case proto::AllianceStation::Red1: id = HAL_ALLIANCE_STATION_RED_1; break;
        case proto::AllianceStation::Red2: id = HAL_ALLIANCE_STATION_RED_2; break;
        case proto::AllianceStation::Red3: id = HAL_ALLIANCE_STATION_RED_3; break;
        case proto::AllianceStation::Blue1: id = HAL_ALLIANCE_STATION_BLUE_1; break;
        case proto::AllianceStation::Blue2: id = HAL_ALLIANCE_STATION_BLUE_2; break;
        case proto::AllianceStation::Blue3: id = HAL_ALLIANCE_STATION_BLUE_3; break;
    }
    HALSIM_SetDriverStationAllianceStationId(id);
}

void setGameMessage(const std::string& message) {
    const WPI_String text{message.data(), message.size()};
    HALSIM_SetGameDataString(&text);
}

std::size_t maxJoysticks() { return HAL_MAX_JOYSTICKS; }

void setJoystick(int index, const proto::JoystickState& j) {
    HAL_JoystickAxes axes{};
    HAL_JoystickPOVs povs{};
    HAL_JoystickButtons buttons{};
    HAL_JoystickDescriptor descriptor{};
    if (j.present) {
        const auto axisCount = std::min<std::size_t>(j.axes.size(), HAL_MAX_JOYSTICK_AXES);
        axes.available = static_cast<std::uint16_t>((1u << axisCount) - 1u);
        for (std::size_t a = 0; a < axisCount; ++a) {
            const float value = std::clamp(j.axes[a], -1.0f, 1.0f);
            axes.axes[a] = value;
            axes.raw[a] = static_cast<std::int16_t>(std::lround(value * 32767.0f));
        }
        const auto povCount = std::min<std::size_t>(j.povs.size(), HAL_MAX_JOYSTICK_POVS);
        povs.available = static_cast<std::uint8_t>((1u << povCount) - 1u);
        constexpr HAL_JoystickPOV hats[]{HAL_JOYSTICK_POV_UP,         HAL_JOYSTICK_POV_RIGHT_UP, HAL_JOYSTICK_POV_RIGHT,
                                         HAL_JOYSTICK_POV_RIGHT_DOWN, HAL_JOYSTICK_POV_DOWN,     HAL_JOYSTICK_POV_LEFT_DOWN,
                                         HAL_JOYSTICK_POV_LEFT,       HAL_JOYSTICK_POV_LEFT_UP};
        for (std::size_t p = 0; p < povCount; ++p) {
            const int degrees = j.povs[p];
            povs.povs[p] = degrees >= 0 && degrees < 360 ? hats[((degrees + 22) / 45) % 8] : HAL_JOYSTICK_POV_CENTERED;
        }
        const auto buttonCount = std::clamp<int>(j.buttonCount, 0, 32);
        buttons.available = (std::uint64_t{1} << buttonCount) - 1;
        buttons.buttons = j.buttons & buttons.available;
        descriptor.isGamepad = 1;
        std::snprintf(descriptor.name, sizeof(descriptor.name), "WPI Emulator gamepad %d", index);
    }
    HALSIM_SetJoystickDescriptor(index, &descriptor);
    HALSIM_SetJoystickAxes(index, &axes);
    HALSIM_SetJoystickPOVs(index, &povs);
    HALSIM_SetJoystickButtons(index, &buttons);
}

void setBatteryCurrent(double) {}   // the controller has no HAL input-current channel

std::vector<std::pair<std::string, std::int32_t>> extraPortCounts() {
    return {{"can_bus", HAL_GetNumCanBuses()}, {"smart_io", HAL_GetNumSmartIo()}};
}

}  // namespace wpiemu::halsim::abi
