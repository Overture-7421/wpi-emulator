#include "wpiemu/bridges/hal/hal_sim_binding.h"

#include <algorithm>
#include <cstring>

#if WPIEMU_WPILIB_YEAR >= 2027
#include <wpi/hal/AddressableLEDTypes.h>
#include <wpi/hal/Ports.h>
#include <wpi/hal/REVPH.h>
#include <wpi/hal/simulation/AddressableLEDData.h>
#include <wpi/hal/simulation/AnalogInData.h>
#include <wpi/hal/simulation/CTREPCMData.h>
#include <wpi/hal/simulation/CanData.h>
#include <wpi/hal/simulation/DIOData.h>
#include <wpi/hal/simulation/DutyCycleData.h>
#include <wpi/hal/simulation/EncoderData.h>
#include <wpi/hal/simulation/PWMData.h>
#include <wpi/hal/simulation/PowerDistributionData.h>
#include <wpi/hal/simulation/REVPHData.h>
#include <wpi/hal/simulation/RoboRioData.h>
#else
#include <hal/AddressableLEDTypes.h>
#include <hal/Ports.h>
#include <hal/REVPH.h>
#include <hal/simulation/AccelerometerData.h>
#include <hal/simulation/AddressableLEDData.h>
#include <hal/simulation/AnalogInData.h>
#include <hal/simulation/AnalogOutData.h>
#include <hal/simulation/CTREPCMData.h>
#include <hal/simulation/CanData.h>
#include <hal/simulation/DIOData.h>
#include <hal/simulation/DutyCycleData.h>
#include <hal/simulation/EncoderData.h>
#include <hal/simulation/PWMData.h>
#include <hal/simulation/PowerDistributionData.h>
#include <hal/simulation/REVPHData.h>
#include <hal/simulation/RelayData.h>
#include <hal/simulation/RoboRioData.h>
#endif

// HALSIM_* does not range-check its index argument, so every accessor guards it here.
namespace wpiemu::bridges::hal {
namespace {

bool in(int index, int count) { return index >= 0 && index < count; }

class SimPwm final : public IPwmHal {
public:
    int channelCount() const override { return HAL_GetNumPWMChannels(); }
    bool initialized(int c) const override { return in(c, channelCount()) && HALSIM_GetPWMInitialized(c); }
    double speed(int c) const override { return in(c, channelCount()) ? HALSIM_GetPWMSpeed(c) : 0.0; }
    double position(int c) const override { return in(c, channelCount()) ? HALSIM_GetPWMPosition(c) : 0.0; }
    int pulseMicroseconds(int c) const override { return in(c, channelCount()) ? HALSIM_GetPWMPulseMicrosecond(c) : 0; }
};

class SimDio final : public IDioHal {
public:
    int channelCount() const override { return HAL_GetNumDigitalChannels(); }
    bool initialized(int c) const override { return in(c, channelCount()) && HALSIM_GetDIOInitialized(c); }
    bool isInput(int c) const override { return !in(c, channelCount()) || HALSIM_GetDIOIsInput(c); }
    bool value(int c) const override { return in(c, channelCount()) && HALSIM_GetDIOValue(c); }
    void setValue(int c, bool v) override {
        if (in(c, channelCount())) HALSIM_SetDIOValue(c, v);
    }
};

class SimAnalogIn final : public IAnalogInHal {
public:
    int channelCount() const override { return HAL_GetNumAnalogInputs(); }
    bool initialized(int c) const override { return in(c, channelCount()) && HALSIM_GetAnalogInInitialized(c); }
    double voltage(int c) const override { return in(c, channelCount()) ? HALSIM_GetAnalogInVoltage(c) : 0.0; }
    void setVoltage(int c, double v) override {
        if (in(c, channelCount())) HALSIM_SetAnalogInVoltage(c, v);
    }
};

class SimAnalogOut final : public IAnalogOutHal {
public:
    int channelCount() const override { return HAL_GetNumAnalogOutputs(); }
    bool initialized(int c) const override { return in(c, channelCount()) && HALSIM_GetAnalogOutInitialized(c); }
    double voltage(int c) const override { return in(c, channelCount()) ? HALSIM_GetAnalogOutVoltage(c) : 0.0; }
};

class SimEncoder final : public IEncoderHal {
public:
    int channelCount() const override { return HAL_GetNumEncoders(); }
    bool initialized(int i) const override { return in(i, channelCount()) && HALSIM_GetEncoderInitialized(i); }
    int digitalChannelA(int i) const override { return in(i, channelCount()) ? HALSIM_GetEncoderDigitalChannelA(i) : -1; }
    int digitalChannelB(int i) const override { return in(i, channelCount()) ? HALSIM_GetEncoderDigitalChannelB(i) : -1; }
    double distancePerPulse(int i) const override { return in(i, channelCount()) ? HALSIM_GetEncoderDistancePerPulse(i) : 0.0; }
    bool reverseDirection(int i) const override { return in(i, channelCount()) && HALSIM_GetEncoderReverseDirection(i); }
    int count(int i) const override { return in(i, channelCount()) ? HALSIM_GetEncoderCount(i) : 0; }
    void setCount(int i, int c) override {
        if (in(i, channelCount())) HALSIM_SetEncoderCount(i, c);
    }
    void setPeriod(int i, double p) override {
        if (in(i, channelCount())) HALSIM_SetEncoderPeriod(i, p);
    }
    void setDirection(int i, bool positive) override {
        if (in(i, channelCount())) HALSIM_SetEncoderDirection(i, positive);
    }
};

class SimDutyCycle final : public IDutyCycleHal {
public:
    int channelCount() const override { return HAL_GetNumDutyCycles(); }
    bool initialized(int i) const override { return in(i, channelCount()) && HALSIM_GetDutyCycleInitialized(i); }
    int digitalChannel(int i) const override { return in(i, channelCount()) ? HALSIM_GetDutyCycleDigitalChannel(i) : -1; }
    double output(int i) const override { return in(i, channelCount()) ? HALSIM_GetDutyCycleOutput(i) : 0.0; }
    void setOutput(int i, double v) override {
        if (in(i, channelCount())) HALSIM_SetDutyCycleOutput(i, v);
    }
    int frequency(int i) const override { return in(i, channelCount()) ? HALSIM_GetDutyCycleFrequency(i) : 0; }
    void setFrequency(int i, int hz) override {
        if (in(i, channelCount())) HALSIM_SetDutyCycleFrequency(i, hz);
    }
};

class SimRelay final : public IRelayHal {
public:
    int headerCount() const override { return HAL_GetNumRelayHeaders(); }
    bool forwardInitialized(int h) const override { return in(h, headerCount()) && HALSIM_GetRelayInitializedForward(h); }
    bool reverseInitialized(int h) const override { return in(h, headerCount()) && HALSIM_GetRelayInitializedReverse(h); }
    bool forward(int h) const override { return in(h, headerCount()) && HALSIM_GetRelayForward(h); }
    bool reverse(int h) const override { return in(h, headerCount()) && HALSIM_GetRelayReverse(h); }
};

class SimCtrePcm final : public IPneumaticsHal {
public:
    int moduleCount() const override { return HAL_GetNumCTREPCMModules(); }
    int solenoidChannelCount() const override { return HAL_GetNumCTRESolenoidChannels(); }
    bool moduleInitialized(int m) const override { return in(m, moduleCount()) && HALSIM_GetCTREPCMInitialized(m); }
    bool solenoidOutput(int m, int c) const override {
        return in(m, moduleCount()) && in(c, solenoidChannelCount()) && HALSIM_GetCTREPCMSolenoidOutput(m, c);
    }
    bool compressorOn(int m) const override { return in(m, moduleCount()) && HALSIM_GetCTREPCMCompressorOn(m); }
    bool closedLoopEnabled(int m) const override { return in(m, moduleCount()) && HALSIM_GetCTREPCMClosedLoopEnabled(m); }
    bool pressureSwitch(int m) const override { return in(m, moduleCount()) && HALSIM_GetCTREPCMPressureSwitch(m); }
    void setPressureSwitch(int m, bool v) override {
        if (in(m, moduleCount())) HALSIM_SetCTREPCMPressureSwitch(m, v);
    }
    double compressorCurrent(int m) const override { return in(m, moduleCount()) ? HALSIM_GetCTREPCMCompressorCurrent(m) : 0.0; }
    void setCompressorCurrent(int m, double a) override {
        if (in(m, moduleCount())) HALSIM_SetCTREPCMCompressorCurrent(m, a);
    }
};

class SimRevPh final : public IPneumaticsHal {
public:
    int moduleCount() const override { return HAL_GetNumREVPHModules(); }
    int solenoidChannelCount() const override { return HAL_GetNumREVPHChannels(); }
    bool moduleInitialized(int m) const override { return in(m, moduleCount()) && HALSIM_GetREVPHInitialized(m); }
    bool solenoidOutput(int m, int c) const override {
        return in(m, moduleCount()) && in(c, solenoidChannelCount()) && HALSIM_GetREVPHSolenoidOutput(m, c);
    }
    bool compressorOn(int m) const override { return in(m, moduleCount()) && HALSIM_GetREVPHCompressorOn(m); }
    bool closedLoopEnabled(int m) const override {
        return in(m, moduleCount()) && HALSIM_GetREVPHCompressorConfigType(m) != HAL_REVPHCompressorConfigType_kDisabled;
    }
    bool pressureSwitch(int m) const override { return in(m, moduleCount()) && HALSIM_GetREVPHPressureSwitch(m); }
    void setPressureSwitch(int m, bool v) override {
        if (in(m, moduleCount())) HALSIM_SetREVPHPressureSwitch(m, v);
    }
    double compressorCurrent(int m) const override { return in(m, moduleCount()) ? HALSIM_GetREVPHCompressorCurrent(m) : 0.0; }
    void setCompressorCurrent(int m, double a) override {
        if (in(m, moduleCount())) HALSIM_SetREVPHCompressorCurrent(m, a);
    }
};

class SimPowerDistribution final : public IPowerDistributionHal {
public:
    // PDH has the most channels; the sim HAL does not say which kind the robot created.
    int moduleCount() const override { return HAL_GetNumREVPDHModules(); }
    int channelCount() const override { return std::max(HAL_GetNumREVPDHChannels(), HAL_GetNumCTREPDPChannels()); }
    bool initialized(int m) const override { return in(m, moduleCount()) && HALSIM_GetPowerDistributionInitialized(m); }
    double voltage(int m) const override { return in(m, moduleCount()) ? HALSIM_GetPowerDistributionVoltage(m) : 0.0; }
    void setVoltage(int m, double v) override {
        if (in(m, moduleCount())) HALSIM_SetPowerDistributionVoltage(m, v);
    }
    double temperature(int m) const override { return in(m, moduleCount()) ? HALSIM_GetPowerDistributionTemperature(m) : 0.0; }
    void setTemperature(int m, double t) override {
        if (in(m, moduleCount())) HALSIM_SetPowerDistributionTemperature(m, t);
    }
    double current(int m, int c) const override {
        return in(m, moduleCount()) && in(c, channelCount()) ? HALSIM_GetPowerDistributionCurrent(m, c) : 0.0;
    }
    void setCurrent(int m, int c, double a) override {
        if (in(m, moduleCount()) && in(c, channelCount())) HALSIM_SetPowerDistributionCurrent(m, c, a);
    }
};

class SimAddressableLed final : public IAddressableLedHal {
public:
    int channelCount() const override { return HAL_GetNumAddressableLEDs(); }
    bool initialized(int i) const override { return in(i, channelCount()) && HALSIM_GetAddressableLEDInitialized(i); }
    int outputPort(int i) const override { return in(i, channelCount()) ? HALSIM_GetAddressableLEDOutputPort(i) : 0; }
    int length(int i) const override { return in(i, channelCount()) ? HALSIM_GetAddressableLEDLength(i) : 0; }
    bool running(int i) const override { return in(i, channelCount()) && HALSIM_GetAddressableLEDRunning(i); }
    void readPixels(int i, std::vector<LedPixel>& out) const override {
        out.clear();
        if (!in(i, channelCount())) return;
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

class SimAccelerometer final : public IAccelerometerHal {
public:
    bool active() const override { return HALSIM_GetAccelerometerActive(0); }
    void setAcceleration(double x, double y, double z) override {
        HALSIM_SetAccelerometerX(0, x);
        HALSIM_SetAccelerometerY(0, y);
        HALSIM_SetAccelerometerZ(0, z);
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

class SimHalApi final : public IHalApi {
public:
    IPwmHal& pwm() override { return pwm_; }
    IDioHal& dio() override { return dio_; }
    IAnalogInHal& analogIn() override { return analogIn_; }
    IAnalogOutHal& analogOut() override { return analogOut_; }
    IEncoderHal& encoder() override { return encoder_; }
    IDutyCycleHal& dutyCycle() override { return dutyCycle_; }
    IRelayHal& relay() override { return relay_; }
    IPneumaticsHal& ctrePcm() override { return pcm_; }
    IPneumaticsHal& revPh() override { return ph_; }
    IPowerDistributionHal& powerDistribution() override { return pd_; }
    IAddressableLedHal& addressableLed() override { return led_; }
    IRoboRioHal& roboRio() override { return rio_; }
    IAccelerometerHal& accelerometer() override { return accel_; }
    ICanHal& can() override { return can_; }

private:
    SimPwm pwm_;
    SimDio dio_;
    SimAnalogIn analogIn_;
    SimAnalogOut analogOut_;
    SimEncoder encoder_;
    SimDutyCycle dutyCycle_;
    SimRelay relay_;
    SimCtrePcm pcm_;
    SimRevPh ph_;
    SimPowerDistribution pd_;
    SimAddressableLed led_;
    SimRoboRio rio_;
    SimAccelerometer accel_;
    SimCan can_;
};

}  // namespace

std::shared_ptr<IHalApi> makeHalSimApi() { return std::make_shared<SimHalApi>(); }

sdk::Status registerHalBridges(sdk::DeviceBridgeRegistry& registry) { return registerHalBridges(registry, makeHalSimApi()); }

}  // namespace wpiemu::bridges::hal
