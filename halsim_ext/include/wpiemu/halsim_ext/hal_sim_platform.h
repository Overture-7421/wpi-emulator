#pragma once

#include <memory>

#include "wpiemu/halsim_ext/config.h"
#include "wpiemu/halsim_ext/hal_ports.h"

namespace wpiemu::halsim {

// The real IHalClock / IDriverStationSink / IRobotPowerSink over WPILib's HALSIM_* C API (2026.2.1,
// ADR 0001; times are microseconds). This header is HAL-free; the implementation file is only
// compiled when the WPILib HAL was found at configure time.
class HalSimPlatform {
public:
    HalSimPlatform();
    ~HalSimPlatform();
    HalSimPlatform(const HalSimPlatform&) = delete;
    HalSimPlatform& operator=(const HalSimPlatform&) = delete;

    IHalClock& clock();
    IDriverStationSink& driverStation();
    IRobotPowerSink& power();

    // Hello contents: runtime language, WPILib version this extension was built against, pid, HAL port counts.
    static RobotInfo describeRobot(const ExtensionConfig& config);

private:
    class Clock;
    class DriverStation;
    class Power;
    std::unique_ptr<Clock> clock_;
    std::unique_ptr<DriverStation> driverStation_;
    std::unique_ptr<Power> power_;
};

}  // namespace wpiemu::halsim
