// HALSIM binding of the roborio HAL generation (WPILib 2026): the device families and calls that differ from systemcore.
#include "hal_sim_abi.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include <hal/AddressableLEDTypes.h>
#include <hal/simulation/AccelerometerData.h>
#include <hal/simulation/AddressableLEDData.h>
#include <hal/simulation/AnalogOutData.h>
#include <hal/simulation/CanData.h>
#include <hal/simulation/PWMData.h>
#include <hal/simulation/RelayData.h>

#include "wpiemu/hal_abi/hal_includes.h"

namespace wpiemu::bridges::hal::abi {
namespace {

class SimPwm final : public IPwmHal {
public:
    int channelCount() const override { return HAL_GetNumPWMChannels(); }
    bool initialized(int c) const override { return inRange(c, channelCount()) && HALSIM_GetPWMInitialized(c); }
    double speed(int c) const override { return inRange(c, channelCount()) ? HALSIM_GetPWMSpeed(c) : 0.0; }
    double position(int c) const override { return inRange(c, channelCount()) ? HALSIM_GetPWMPosition(c) : 0.0; }
    int pulseMicroseconds(int c) const override { return inRange(c, channelCount()) ? HALSIM_GetPWMPulseMicrosecond(c) : 0; }
};

class SimAnalogOut final : public IAnalogOutHal {
public:
    int channelCount() const override { return HAL_GetNumAnalogOutputs(); }
    bool initialized(int c) const override { return inRange(c, channelCount()) && HALSIM_GetAnalogOutInitialized(c); }
    double voltage(int c) const override { return inRange(c, channelCount()) ? HALSIM_GetAnalogOutVoltage(c) : 0.0; }
};

class SimRelay final : public IRelayHal {
public:
    int headerCount() const override { return HAL_GetNumRelayHeaders(); }
    bool forwardInitialized(int h) const override { return inRange(h, headerCount()) && HALSIM_GetRelayInitializedForward(h); }
    bool reverseInitialized(int h) const override { return inRange(h, headerCount()) && HALSIM_GetRelayInitializedReverse(h); }
    bool forward(int h) const override { return inRange(h, headerCount()) && HALSIM_GetRelayForward(h); }
    bool reverse(int h) const override { return inRange(h, headerCount()) && HALSIM_GetRelayReverse(h); }
};

class SimAccelerometer final : public IAccelerometerHal {
public:
    bool active() const override { return HALSIM_GetAccelerometerActive(0); }
    void setAcceleration(double x, double y, double z) override {
        HALSIM_SetAccelerometerX(0, x);
        HALSIM_SetAccelerometerY(0, y);
        HALSIM_SetAccelerometerZ(0, z);
    }
};

class SimAddressableLed final : public IAddressableLedHal {
public:
    int channelCount() const override { return HAL_GetNumAddressableLEDs(); }
    bool initialized(int i) const override { return inRange(i, channelCount()) && HALSIM_GetAddressableLEDInitialized(i); }
    int outputPort(int i) const override { return inRange(i, channelCount()) ? HALSIM_GetAddressableLEDOutputPort(i) : 0; }
    int length(int i) const override { return inRange(i, channelCount()) ? HALSIM_GetAddressableLEDLength(i) : 0; }
    bool running(int i) const override { return inRange(i, channelCount()) && HALSIM_GetAddressableLEDRunning(i); }
    void readPixels(int i, std::vector<LedPixel>& out) const override {
        out.clear();
        if (!inRange(i, channelCount())) return;
        buffer_.resize(HAL_kAddressableLEDMaxLength);
        const int n = std::clamp(HALSIM_GetAddressableLEDData(i, buffer_.data()), 0, static_cast<int>(buffer_.size()));
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
    double vInVoltage() const override { return HALSIM_GetRoboRioVInVoltage(); }
    void setVInVoltage(double v) override { HALSIM_SetRoboRioVInVoltage(v); }
    double vInCurrent() const override { return HALSIM_GetRoboRioVInCurrent(); }
    void setVInCurrent(double a) override { HALSIM_SetRoboRioVInCurrent(a); }
    double brownoutVoltage() const override { return HALSIM_GetRoboRioBrownoutVoltage(); }
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
        static void trampoline(const char*, void* param, uint32_t id, const uint8_t* data, uint8_t size, int32_t periodMs,
                               int32_t*) {
            CanFrame frame;
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
std::unique_ptr<IAnalogOutHal> makeAnalogOut() { return std::make_unique<SimAnalogOut>(); }
std::unique_ptr<IRelayHal> makeRelay() { return std::make_unique<SimRelay>(); }
std::unique_ptr<IAccelerometerHal> makeAccelerometer() { return std::make_unique<SimAccelerometer>(); }
std::unique_ptr<IImuHal> makeImu() { return nullptr; }

void setEncoderPeriod(int encoder, double periodS) { HALSIM_SetEncoderPeriod(encoder, periodS); }
int dutyCycleDigitalChannel(int dutyCycle) { return HALSIM_GetDutyCycleDigitalChannel(dutyCycle); }
bool revPhClosedLoopEnabled(int module) {
    return HALSIM_GetREVPHCompressorConfigType(module) != HAL_REVPHCompressorConfigType_kDisabled;
}

}  // namespace wpiemu::bridges::hal::abi
