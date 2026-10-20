#pragma once

// The thin seam between the HAL bridges and WPILib's HALSIM_* C API. Bridges depend ONLY on these
// small per-family interfaces (interface segregation); the real binding (hal_sim_binding.h) is
// compiled only when the WPILib HAL headers are available, and unit tests use FakeHalApi.
//
// Conventions (all match HALSIM 2026.2.1, docs/reference/extension-ipc.md):
//   * `channelCount()` is HAL_GetNum*(); every other index argument is guaranteed by the bridges to
//     be in [0, count). Implementations must still tolerate out-of-range indices (return neutral
//     values / ignore writes) because HALSIM_* itself does not range-check.
//   * Getters are plain reads (robot -> sim), setters are the sim -> robot write-backs.
//   * Nothing here may block or step sim time; it is called on the extension's clock thread while
//     the robot is quiescent (CAN is the one exception: its listener is called on robot threads).

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::bridges::hal {

class IPwmHal {
    WPIEMU_INTERFACE(IPwmHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int channel) const = 0;
    virtual double speed(int channel) const = 0;            // HALSIM_GetPWMSpeed, [-1, 1]
    virtual double position(int channel) const = 0;         // HALSIM_GetPWMPosition, [0, 1] (servos)
    virtual int pulseMicroseconds(int channel) const = 0;   // HALSIM_GetPWMPulseMicrosecond
};

class IDioHal {
    WPIEMU_INTERFACE(IDioHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int channel) const = 0;
    virtual bool isInput(int channel) const = 0;
    virtual bool value(int channel) const = 0;
    virtual void setValue(int channel, bool value) = 0;
};

class IAnalogInHal {
    WPIEMU_INTERFACE(IAnalogInHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int channel) const = 0;
    virtual double voltage(int channel) const = 0;
    virtual void setVoltage(int channel, double volts) = 0;
};

class IAnalogOutHal {
    WPIEMU_INTERFACE(IAnalogOutHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int channel) const = 0;
    virtual double voltage(int channel) const = 0;
};

// HAL encoder (quadrature A/B pair). Sim state is "count" (distance / distancePerPulse) and "period"
// (seconds per count, signed; infinity when stopped).
class IEncoderHal {
    WPIEMU_INTERFACE(IEncoderHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int index) const = 0;
    virtual int digitalChannelA(int index) const = 0;
    virtual int digitalChannelB(int index) const = 0;
    virtual double distancePerPulse(int index) const = 0;
    virtual bool reverseDirection(int index) const = 0;
    virtual int count(int index) const = 0;
    virtual void setCount(int index, int count) = 0;
    virtual void setPeriod(int index, double secondsPerCount) = 0;
    virtual void setDirection(int index, bool positive) = 0;
};

// DutyCycle input (REV Through Bore / absolute encoders): output is the duty ratio in [0, 1].
class IDutyCycleHal {
    WPIEMU_INTERFACE(IDutyCycleHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int index) const = 0;
    virtual int digitalChannel(int index) const = 0;
    virtual double output(int index) const = 0;
    virtual void setOutput(int index, double ratio) = 0;
    virtual int frequency(int index) const = 0;
    virtual void setFrequency(int index, int hertz) = 0;
};

// Forward and reverse relay outputs are initialised (and freed) independently.
class IRelayHal {
    WPIEMU_INTERFACE(IRelayHal)
public:
    virtual int headerCount() const = 0;
    virtual bool forwardInitialized(int header) const = 0;
    virtual bool reverseInitialized(int header) const = 0;
    virtual bool forward(int header) const = 0;
    virtual bool reverse(int header) const = 0;
};

// CTRE PCM and REV PH share one shape; the real binding is instantiated once per technology.
class IPneumaticsHal {
    WPIEMU_INTERFACE(IPneumaticsHal)
public:
    virtual int moduleCount() const = 0;
    virtual int solenoidChannelCount() const = 0;
    virtual bool moduleInitialized(int module) const = 0;
    virtual bool solenoidOutput(int module, int channel) const = 0;
    virtual bool compressorOn(int module) const = 0;
    // PCM: closed-loop flag. PH: compressor config type != disabled.
    virtual bool closedLoopEnabled(int module) const = 0;
    virtual bool pressureSwitch(int module) const = 0;
    virtual void setPressureSwitch(int module, bool closed) = 0;
    virtual double compressorCurrent(int module) const = 0;
    virtual void setCompressorCurrent(int module, double amps) = 0;
};

// PDP / PDH. The sim HAL does not say which one the robot created, so they are one family.
class IPowerDistributionHal {
    WPIEMU_INTERFACE(IPowerDistributionHal)
public:
    virtual int moduleCount() const = 0;
    virtual int channelCount() const = 0;
    virtual bool initialized(int module) const = 0;
    virtual double voltage(int module) const = 0;
    virtual void setVoltage(int module, double volts) = 0;
    virtual double temperature(int module) const = 0;
    virtual void setTemperature(int module, double celsius) = 0;
    virtual double current(int module, int channel) const = 0;
    virtual void setCurrent(int module, int channel, double amps) = 0;
};

struct LedPixel {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

class IAddressableLedHal {
    WPIEMU_INTERFACE(IAddressableLedHal)
public:
    virtual int channelCount() const = 0;
    virtual bool initialized(int index) const = 0;
    virtual int outputPort(int index) const = 0;
    virtual int length(int index) const = 0;
    virtual bool running(int index) const = 0;
    virtual void readPixels(int index, std::vector<LedPixel>& out) const = 0;
};

class IRoboRioHal {
    WPIEMU_INTERFACE(IRoboRioHal)
public:
    virtual double vInVoltage() const = 0;
    virtual void setVInVoltage(double volts) = 0;
    virtual double vInCurrent() const = 0;
    virtual void setVInCurrent(double amps) = 0;
    virtual double brownoutVoltage() const = 0;
};

// Built-in roboRIO accelerometer. Values are in g, like HALSIM_SetAccelerometer{X,Y,Z}.
class IAccelerometerHal {
    WPIEMU_INTERFACE(IAccelerometerHal)
public:
    virtual bool active() const = 0;
    virtual void setAcceleration(double xG, double yG, double zG) = 0;
};

struct CanFrame {
    std::uint32_t arbitrationId = 0;
    std::array<std::uint8_t, 8> data{};
    std::uint8_t size = 0;
    std::int32_t periodMs = 0;  // 0 = one shot; > 0 repeating send; HAL_CAN_SEND_PERIOD_STOP_REPEATING (-1) = stop
};

// Called on whatever ROBOT thread sent the frame; must be quick and non-blocking.
class ICanFrameListener {
    WPIEMU_INTERFACE(ICanFrameListener)
public:
    virtual void onFrame(const CanFrame& frame) = 0;
};

// RAII subscription: destroying it unregisters the HAL callback.
class ICanSubscription {
    WPIEMU_INTERFACE(ICanSubscription)
};

class ICanHal {
    WPIEMU_INTERFACE(ICanHal)
public:
    // Observe-only (HALSIM_RegisterCanSendMessageCallback). The listener must outlive the subscription.
    virtual std::unique_ptr<ICanSubscription> subscribe(ICanFrameListener& listener) = 0;
};

// Aggregate handed to registerHalBridges. Accessors never return null.
class IHalApi {
    WPIEMU_INTERFACE(IHalApi)
public:
    virtual IPwmHal& pwm() = 0;
    virtual IDioHal& dio() = 0;
    virtual IAnalogInHal& analogIn() = 0;
    virtual IAnalogOutHal& analogOut() = 0;
    virtual IEncoderHal& encoder() = 0;
    virtual IDutyCycleHal& dutyCycle() = 0;
    virtual IRelayHal& relay() = 0;
    virtual IPneumaticsHal& ctrePcm() = 0;
    virtual IPneumaticsHal& revPh() = 0;
    virtual IPowerDistributionHal& powerDistribution() = 0;
    virtual IAddressableLedHal& addressableLed() = 0;
    virtual IRoboRioHal& roboRio() = 0;
    virtual IAccelerometerHal& accelerometer() = 0;
    virtual ICanHal& can() = 0;
};

}  // namespace wpiemu::bridges::hal
