// HALSIM binding of the systemcore HAL generation (WPILib 2027): the device families and calls that differ from roborio.
// Families the generation removed (analog output, relay, accelerometer) are absent; PWM exposes a throttle SimValue, the
// addressable LED has no output port or running state, the controller has no input current, and a built-in IMU exists.
#include "hal_sim_abi.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <wpi/hal/AddressableLEDTypes.h>
#include <wpi/hal/CANAPITypes.h>
#include <wpi/hal/SimDevice.h>
#include <wpi/hal/simulation/AddressableLEDData.h>
#include <wpi/hal/simulation/CanData.h>
#include <wpi/hal/simulation/IMUData.h>
#include <wpi/hal/simulation/PWMData.h>
#include <wpi/hal/simulation/SimDeviceData.h>

#include "wpiemu/hal_abi/hal_includes.h"

namespace wpiemu::bridges::hal::abi {
namespace {

class SimPwm final : public IPwmHal {
public:
    int channelCount() const override { return HAL_GetNumPWMChannels(); }
    bool initialized(int c) const override { return inRange(c, channelCount()) && HALSIM_GetPWMInitialized(c); }
    bool hasSpeed(int c) const override { return throttle(c) != 0; }
    bool hasPosition() const override { return false; }
    double speed(int c) const override {
        const auto handle = throttle(c);
        return handle != 0 ? HAL_GetSimValueDouble(handle) : 0.0;
    }
    double position(int) const override { throw std::logic_error("PWM position is not exposed by this WPILib generation"); }
    int pulseMicroseconds(int c) const override { return inRange(c, channelCount()) ? HALSIM_GetPWMPulseMicrosecond(c) : 0; }

private:
    static HAL_SimValueHandle throttle(int c) {
        if (!inRange(c, HAL_GetNumPWMChannels())) return 0;
        const auto device = HALSIM_GetPWMSimDevice(c);
        return device != 0 ? HALSIM_GetSimValueHandle(device, "Throttle") : 0;
    }
};

class SimAddressableLed final : public IAddressableLedHal {
public:
    bool hasOutputPort() const override { return false; }
    bool hasRunning() const override { return false; }
    int channelCount() const override { return HAL_GetNumAddressableLEDs(); }
    bool initialized(int i) const override { return inRange(i, channelCount()) && HALSIM_GetAddressableLEDInitialized(i); }
    int outputPort(int) const override { throw std::logic_error("this WPILib generation has no LED output port"); }
    int length(int i) const override { return inRange(i, channelCount()) ? HALSIM_GetAddressableLEDLength(i) : 0; }
    bool running(int) const override { throw std::logic_error("this WPILib generation has no LED running state"); }
    void readPixels(int i, std::vector<LedPixel>& out) const override {
        out.clear();
        if (!inRange(i, channelCount())) return;
        buffer_.resize(static_cast<std::size_t>(std::max(0, length(i))));
        const int n = std::clamp(HALSIM_GetAddressableLEDData(HALSIM_GetAddressableLEDStart(i), length(i), buffer_.data()),
                                 0, static_cast<int>(buffer_.size()));
        out.reserve(static_cast<std::size_t>(n));
        for (int k = 0; k < n; ++k) {
            const HAL_AddressableLEDData& p = buffer_[static_cast<std::size_t>(k)];
            out.push_back(LedPixel{p.r, p.g, p.b});
        }
    }

private:
    mutable std::vector<HAL_AddressableLEDData> buffer_;
};

class SimRoboRio final : public IRoboRioHal {
public:
    bool hasInputCurrent() const override { return false; }
    double vInVoltage() const override { return HALSIM_GetRoboRioVInVoltage(); }
    void setVInVoltage(double v) override { HALSIM_SetRoboRioVInVoltage(v); }
    double vInCurrent() const override { throw std::logic_error("this WPILib generation has no controller input current"); }
    void setVInCurrent(double) override { throw std::logic_error("this WPILib generation has no controller input current"); }
    double brownoutVoltage() const override { return HALSIM_GetRoboRioBrownoutVoltage(); }
};

class SimImu final : public IImuHal {
public:
    void setAcceleration(double x, double y, double z) override {
        HALSIM_SetIMUAccelX(x);
        HALSIM_SetIMUAccelY(y);
        HALSIM_SetIMUAccelZ(z);
    }
    void setAngles(double x, double y, double z) override {
        HALSIM_SetIMUAngleX(x);
        HALSIM_SetIMUAngleY(y);
        HALSIM_SetIMUAngleZ(z);
        HALSIM_SetIMUYaw(z);
    }
    void setRates(double x, double y, double z) override {
        HALSIM_SetIMUGyroRateX(x);
        HALSIM_SetIMUGyroRateY(y);
        HALSIM_SetIMUGyroRateZ(z);
    }
};

class SimCan final : public ICanHal {
public:
    std::unique_ptr<ICanSubscription> subscribe(ICanFrameListener& listener) override {
        auto sub = std::make_unique<Subscription>(listener);
        return sub->valid() ? std::unique_ptr<ICanSubscription>(std::move(sub)) : nullptr;
    }

private:
    class Subscription final : public ICanSubscription {
    public:
        explicit Subscription(ICanFrameListener& listener)
            : uid_(HALSIM_RegisterCanSendMessageCallback(&Subscription::trampoline, &listener)) {}
        ~Subscription() override {
            if (uid_ != 0) HALSIM_CancelCanSendMessageCallback(uid_);
        }
        bool valid() const { return uid_ != 0; }

    private:
        static void trampoline(const char*, void* param, int32_t busId, uint32_t id, const HAL_CANMessage* message,
                               int32_t periodMs, int32_t*) {
            const auto* data = message != nullptr ? message->data : nullptr;
            const auto size = message != nullptr ? message->dataSize : 0;
            CanFrame frame;
            frame.busId = busId;
            frame.arbitrationId = id;
            frame.size = static_cast<std::uint8_t>(std::min<std::size_t>(size, frame.data.size()));
            if (data != nullptr && frame.size > 0) std::memcpy(frame.data.data(), data, frame.size);
            frame.periodMs = periodMs;
            static_cast<ICanFrameListener*>(param)->onFrame(frame);
        }
        int32_t uid_;
    };
};

}  // namespace

std::unique_ptr<IPwmHal> makePwm() { return std::make_unique<SimPwm>(); }
std::unique_ptr<IAddressableLedHal> makeAddressableLed() { return std::make_unique<SimAddressableLed>(); }
std::unique_ptr<IRoboRioHal> makeRoboRio() { return std::make_unique<SimRoboRio>(); }
std::unique_ptr<ICanHal> makeCan() { return std::make_unique<SimCan>(); }
std::unique_ptr<IAnalogOutHal> makeAnalogOut() { return nullptr; }
std::unique_ptr<IRelayHal> makeRelay() { return nullptr; }
std::unique_ptr<IAccelerometerHal> makeAccelerometer() { return nullptr; }
std::unique_ptr<IImuHal> makeImu() { return std::make_unique<SimImu>(); }

void setEncoderPeriod(int encoder, double periodS) {
    HALSIM_SetEncoderRate(encoder, std::isfinite(periodS) && periodS != 0.0 ? HALSIM_GetEncoderDistancePerPulse(encoder) / periodS : 0.0);
}
int dutyCycleDigitalChannel(int dutyCycle) { return dutyCycle; }
bool revPhClosedLoopEnabled(int module) {
    return HALSIM_GetREVPHCompressorConfigType(module) != HAL_REVPH_COMPRESSOR_CONFIG_DISABLED;
}

}  // namespace wpiemu::bridges::hal::abi
