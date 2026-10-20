#pragma once

// The seams between the extension logic and the robot process. Everything that touches HALSIM_*
// or the OS lives behind these interfaces (implemented by hal_sim_platform.h / os_process.h), so
// the lock-step logic is unit-testable with fakes and no WPILib.

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::halsim {

// HALSIM_*Timing. Called on the extension clock thread unless noted.
class IHalClock {
    WPIEMU_INTERFACE(IHalClock)
public:
    virtual void pauseTiming() = 0;
    // Any thread (the watchdog unblocks the robot with it). Idempotent.
    virtual void resumeTiming() = 0;
    virtual bool isTimingPaused() const = 0;
    // True once robot code called HAL_ObserveUserProgramStarting. Polled (not blocked on) so that
    // the wait can be interrupted when the emulator goes away.
    virtual bool programStarted() const = 0;
    // True once some robot notifier has an alarm armed. TimedRobot arms its first alarm right AFTER
    // it observed program start, so stepping earlier would shift the loop phase (nondeterministic).
    virtual bool schedulerArmed() const = 0;
    // HALSIM_StepTiming: returns when every notifier woken inside the window has finished its work
    // (the whole TimedRobot loop body). Blocks for as long as the robot takes.
    virtual void stepTiming(std::uint64_t deltaUs) = 0;
    // Robot loop iterations completed (HALSIM sim-periodic-after callbacks), diagnostics only.
    virtual std::uint64_t robotLoopCount() const = 0;
};

// The extension plays the Driver Station (no halsim_gui is loaded, docs/reference/extension-ipc.md).
class IDriverStationSink {
    WPIEMU_INTERFACE(IDriverStationSink)
public:
    virtual void apply(const sdk::proto::DsState& state) = 0;
};

// roboRIO battery rail (HALSIM_SetRoboRioVIn*).
class IRobotPowerSink {
    WPIEMU_INTERFACE(IRobotPowerSink)
public:
    virtual void setBattery(double volts, double amps) = 0;
};

class IProcessControl {
    WPIEMU_INTERFACE(IProcessControl)
public:
    // Ends the robot process without running static destructors (WatchdogPolicy::Terminate). The
    // real implementation never returns; fakes record the call and return, so callers must cope.
    virtual void terminate(int exitCode) = 0;
};

// What the Hello message is built from.
struct RobotInfo {
    sdk::proto::RobotLanguage language = sdk::proto::RobotLanguage::Unknown;
    std::int32_t wpilibYear = 0;
    std::string wpilibVersion;
    std::string extensionVersion;
    std::int64_t pid = 0;
    std::string programName;
    std::vector<std::pair<std::string, std::int32_t>> halPortCounts;
};

// Bundle handed to the runtime: non-owning references; the composition root owns the objects and
// keeps them alive for the life of the process.
struct HalPorts {
    IHalClock& clock;
    IDriverStationSink& driverStation;
    IRobotPowerSink& power;
    IProcessControl& process;
};

}  // namespace wpiemu::halsim
