#pragma once

#include <cstdint>
#include <string>

namespace wpiemu::bridges::hal {

// Fields of a FRC CAN arbitration id (29 bit): devType[28:24] manufacturer[23:16] api[15:6] deviceId[5:0].
struct CanArbitrationId {
    int deviceType = 0;
    int manufacturer = 0;
    int apiId = 0;
    int deviceId = 0;

    static CanArbitrationId decode(std::uint32_t id) {
        return {static_cast<int>((id >> 24) & 0x1F), static_cast<int>((id >> 16) & 0xFF),
                static_cast<int>((id >> 6) & 0x3FF), static_cast<int>(id & 0x3F)};
    }
};

// Lower-case snake_case names from hal/CANAPITypes.h (2026.2.1); unknown values become "mfr<N>" / "type<N>".
std::string canManufacturerName(int manufacturer);
std::string canDeviceTypeName(int deviceType);

}  // namespace wpiemu::bridges::hal
