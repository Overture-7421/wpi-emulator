#include "wpiemu/bridges/hal/hal_sim_binding.h"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <stdexcept>

#include "hal_sim_abi.h"
#include "wpiemu/hal_abi/hal_includes.h"

// HALSIM_* does not range-check its index argument, so every accessor guards it here.
namespace wpiemu::bridges::hal {
namespace {

bool in(int index, int count) { return index >= 0 && index < count; }

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
        if (!in(i, channelCount())) return;
        abi::setEncoderPeriod(i, p);
    }
    void setDirection(int i, bool positive) override {
        if (in(i, channelCount())) HALSIM_SetEncoderDirection(i, positive);
    }
};

class SimDutyCycle final : public IDutyCycleHal {
public:
    int channelCount() const override { return HAL_GetNumDutyCycles(); }
    bool initialized(int i) const override { return in(i, channelCount()) && HALSIM_GetDutyCycleInitialized(i); }
    int digitalChannel(int i) const override {
        return in(i, channelCount()) ? abi::dutyCycleDigitalChannel(i) : -1;
    }
    double output(int i) const override { return in(i, channelCount()) ? HALSIM_GetDutyCycleOutput(i) : 0.0; }
    void setOutput(int i, double v) override {
        if (in(i, channelCount())) HALSIM_SetDutyCycleOutput(i, v);
    }
    int frequency(int i) const override { return in(i, channelCount()) ? static_cast<int>(HALSIM_GetDutyCycleFrequency(i)) : 0; }
    void setFrequency(int i, int hz) override {
        if (in(i, channelCount())) HALSIM_SetDutyCycleFrequency(i, hz);
    }
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
        return in(m, moduleCount()) && abi::revPhClosedLoopEnabled(m);
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

class SimHalApi final : public IHalApi {
public:
    IPwmHal& pwm() override { return *pwm_; }
    IDioHal& dio() override { return dio_; }
    IAnalogInHal& analogIn() override { return analogIn_; }
    IAnalogOutHal* analogOut() override { return analogOut_.get(); }
    IEncoderHal& encoder() override { return encoder_; }
    IDutyCycleHal& dutyCycle() override { return dutyCycle_; }
    IRelayHal* relay() override { return relay_.get(); }
    IPneumaticsHal& ctrePcm() override { return pcm_; }
    IPneumaticsHal& revPh() override { return ph_; }
    IPowerDistributionHal& powerDistribution() override { return pd_; }
    IAddressableLedHal& addressableLed() override { return *led_; }
    IRoboRioHal& roboRio() override { return *rio_; }
    IAccelerometerHal* accelerometer() override { return accelerometer_.get(); }
    IImuHal* imu() override { return imu_.get(); }
    ICanHal& can() override { return *can_; }

private:
    // The families of the selected HAL generation come from its abi/<abi>/ source; the rest is shared.
    std::unique_ptr<IPwmHal> pwm_ = abi::makePwm();
    SimDio dio_;
    SimAnalogIn analogIn_;
    std::unique_ptr<IAnalogOutHal> analogOut_ = abi::makeAnalogOut();
    std::unique_ptr<IRelayHal> relay_ = abi::makeRelay();
    std::unique_ptr<IAccelerometerHal> accelerometer_ = abi::makeAccelerometer();
    std::unique_ptr<IImuHal> imu_ = abi::makeImu();
    SimEncoder encoder_;
    SimDutyCycle dutyCycle_;
    SimCtrePcm pcm_;
    SimRevPh ph_;
    SimPowerDistribution pd_;
    std::unique_ptr<IAddressableLedHal> led_ = abi::makeAddressableLed();
    std::unique_ptr<IRoboRioHal> rio_ = abi::makeRoboRio();
    std::unique_ptr<ICanHal> can_ = abi::makeCan();
};

}  // namespace

std::shared_ptr<IHalApi> makeHalSimApi() { return std::make_shared<SimHalApi>(); }

sdk::Status registerHalBridges(sdk::DeviceBridgeRegistry& registry) { return registerHalBridges(registry, makeHalSimApi()); }

}  // namespace wpiemu::bridges::hal
