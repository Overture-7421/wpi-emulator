#pragma once

// The HAL clock as the vision bridges read it, for the systemcore ABI: HAL_GetMonotonicTime() in nanoseconds.
// No WPILib header is needed; the function is resolved from the loaded HAL module by name.

#include <cstdint>

namespace wpiemu::hal_abi {

using HalTimeFn = std::int64_t (*)();
inline constexpr const char* kHalTimeSymbol = "HAL_GetMonotonicTime";
inline constexpr std::int64_t kNativeTimePerUs = 1000;   // ntcore and the HAL clock tick in nanoseconds

// Microseconds from the HAL clock; the nanosecond clock cannot report an error.
inline bool halTimeUs(HalTimeFn clock, std::int64_t* us) {
    *us = clock() / kNativeTimePerUs;
    return true;
}

}  // namespace wpiemu::hal_abi
