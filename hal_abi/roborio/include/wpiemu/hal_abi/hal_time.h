#pragma once

// The HAL clock as the vision bridges read it, for the roborio ABI: HAL_GetFPGATime(int32_t* status) in microseconds.
// No WPILib header is needed; the function is resolved from the loaded HAL module by name.

#include <cstdint>

namespace wpiemu::hal_abi {

using HalTimeFn = std::uint64_t (*)(std::int32_t* status);
inline constexpr const char* kHalTimeSymbol = "HAL_GetFPGATime";
inline constexpr std::int64_t kNativeTimePerUs = 1;   // ntcore and the HAL clock tick in microseconds

// Microseconds from the HAL clock; false when the HAL reports an error.
inline bool halTimeUs(HalTimeFn clock, std::int64_t* us) {
    std::int32_t status = 0;
    const std::uint64_t now = clock(&status);
    if (status != 0) return false;
    *us = static_cast<std::int64_t>(now);
    return true;
}

}  // namespace wpiemu::hal_abi
