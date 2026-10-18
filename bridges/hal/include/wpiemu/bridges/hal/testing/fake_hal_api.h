#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::bridges::hal::testing {

class FakePwmHal final : public IPwmHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int channel) const override { return at(init, channel); }
    double speed(int channel) const override { return at(spd, channel); }
    double position(int channel) const override { return at(pos, channel); }
    int pulseMicroseconds(int channel) const override { return at(pulse, channel); }

    std::array<bool, 20> init{};
    std::array<double, 20> spd{};
    std::array<double, 20> pos{};
    std::array<int, 20> pulse{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
};

class FakeDioHal final : public IDioHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int channel) const override { return at(init, channel); }
    bool isInput(int channel) const override { return at(input, channel); }
    bool value(int channel) const override { return at(val, channel); }
    void setValue(int channel, bool value) override { set(val, channel, value); }

    std::array<bool, 26> init{};
    std::array<bool, 26> input = [] {
        std::array<bool, 26> values{};
        values.fill(true);
        return values;
    }();
    std::array<bool, 26> val{};

private:
    template <std::size_t N>
    static bool at(const std::array<bool, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : false;
    }
    template <std::size_t N>
    static void set(std::array<bool, N>& values, int index, bool value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }
};

class FakeAnalogInHal final : public IAnalogInHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int channel) const override { return at(init, channel); }
    double voltage(int channel) const override { return at(volts, channel); }
    void setVoltage(int channel, double value) override { set(volts, channel, value); }

    std::array<bool, 8> init{};
    std::array<double, 8> volts{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
    template <class T, std::size_t N>
    static void set(std::array<T, N>& values, int index, T value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }
};

class FakeAnalogOutHal final : public IAnalogOutHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int channel) const override { return at(init, channel); }
    double voltage(int channel) const override { return at(volts, channel); }

    std::array<bool, 8> init{};
    std::array<double, 8> volts{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
};

class FakeEncoderHal final : public IEncoderHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int index) const override { return at(init, index); }
    int digitalChannelA(int index) const override { return at(chanA, index); }
    int digitalChannelB(int index) const override { return at(chanB, index); }
    double distancePerPulse(int index) const override { return at(dpp, index); }
    bool reverseDirection(int index) const override { return at(reversed, index); }
    int count(int index) const override { return at(cnt, index); }
    void setCount(int index, int value) override { set(cnt, index, value); }
    void setPeriod(int index, double value) override { set(per, index, value); }
    void setDirection(int index, bool value) override { set(dir, index, value); }

    std::array<bool, 8> init{};
    std::array<int, 8> chanA{};
    std::array<int, 8> chanB{};
    std::array<double, 8> dpp{};
    std::array<bool, 8> reversed{};
    std::array<int, 8> cnt{};
    std::array<double, 8> per{};
    std::array<bool, 8> dir{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
    template <class T, std::size_t N>
    static void set(std::array<T, N>& values, int index, T value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }
};

class FakeDutyCycleHal final : public IDutyCycleHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int index) const override { return at(init, index); }
    int digitalChannel(int index) const override { return at(chan, index); }
    double output(int index) const override { return at(out, index); }
    void setOutput(int index, double value) override { set(out, index, value); }
    int frequency(int index) const override { return at(freq, index); }
    void setFrequency(int index, int value) override { set(freq, index, value); }

    std::array<bool, 8> init{};
    std::array<int, 8> chan{};
    std::array<double, 8> out{};
    std::array<int, 8> freq{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
    template <class T, std::size_t N>
    static void set(std::array<T, N>& values, int index, T value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }
};

class FakeRelayHal final : public IRelayHal {
public:
    int headerCount() const override { return static_cast<int>(fwdInit.size()); }
    bool forwardInitialized(int header) const override { return at(fwdInit, header); }
    bool reverseInitialized(int header) const override { return at(revInit, header); }
    bool forward(int header) const override { return at(fwd, header); }
    bool reverse(int header) const override { return at(rev, header); }

    std::array<bool, 8> fwdInit{};
    std::array<bool, 8> revInit{};
    std::array<bool, 8> fwd{};
    std::array<bool, 8> rev{};

private:
    template <std::size_t N>
    static bool at(const std::array<bool, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : false;
    }
};

class FakePneumaticsHal final : public IPneumaticsHal {
public:
    explicit FakePneumaticsHal(int channels) : channels_(channels) {}

    int moduleCount() const override { return static_cast<int>(init.size()); }
    int solenoidChannelCount() const override { return channels_; }
    bool moduleInitialized(int module) const override { return at(init, module); }
    bool solenoidOutput(int module, int channel) const override {
        return valid(module, channel) ? solenoids[static_cast<std::size_t>(module)][static_cast<std::size_t>(channel)] : false;
    }
    bool compressorOn(int module) const override { return at(compressor, module); }
    bool closedLoopEnabled(int module) const override { return at(closed, module); }
    bool pressureSwitch(int module) const override { return at(pressure, module); }
    void setPressureSwitch(int module, bool value) override { set(pressure, module, value); }
    double compressorCurrent(int module) const override { return at(current, module); }
    void setCompressorCurrent(int module, double value) override { set(current, module, value); }
    void setSolenoid(int module, int channel, bool value) {
        if (valid(module, channel)) solenoids[static_cast<std::size_t>(module)][static_cast<std::size_t>(channel)] = value;
    }

    std::array<bool, 63> init{};
    std::array<bool, 63> compressor{};
    std::array<bool, 63> closed{};
    std::array<bool, 63> pressure{};
    std::array<double, 63> current{};
    std::array<std::array<bool, 16>, 63> solenoids{};

private:
    bool valid(int module, int channel) const {
        return module >= 0 && static_cast<std::size_t>(module) < init.size() && channel >= 0 && channel < channels_;
    }
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
    template <class T, std::size_t N>
    static void set(std::array<T, N>& values, int index, T value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }

    int channels_;
};

class FakePowerDistributionHal final : public IPowerDistributionHal {
public:
    int moduleCount() const override { return static_cast<int>(init.size()); }
    int channelCount() const override { return 24; }
    bool initialized(int module) const override { return at(init, module); }
    double voltage(int module) const override { return at(volts, module); }
    void setVoltage(int module, double value) override { set(volts, module, value); }
    double temperature(int module) const override { return at(temp, module); }
    void setTemperature(int module, double value) override { set(temp, module, value); }
    double current(int module, int channel) const override {
        return valid(module, channel) ? amps[static_cast<std::size_t>(module)][static_cast<std::size_t>(channel)] : 0.0;
    }
    void setCurrent(int module, int channel, double value) override {
        if (valid(module, channel)) amps[static_cast<std::size_t>(module)][static_cast<std::size_t>(channel)] = value;
    }

    std::array<bool, 64> init{};
    std::array<double, 64> volts{};
    std::array<double, 64> temp{};
    std::array<std::array<double, 24>, 64> amps{};

private:
    bool valid(int module, int channel) const {
        return module >= 0 && static_cast<std::size_t>(module) < init.size() && channel >= 0 && channel < channelCount();
    }
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
    template <class T, std::size_t N>
    static void set(std::array<T, N>& values, int index, T value) {
        if (index >= 0 && static_cast<std::size_t>(index) < N) values[static_cast<std::size_t>(index)] = value;
    }
};

class FakeAddressableLedHal final : public IAddressableLedHal {
public:
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int index) const override { return at(init, index); }
    int outputPort(int index) const override { return at(port, index); }
    int length(int index) const override { return at(len, index); }
    bool running(int index) const override { return at(run, index); }
    void readPixels(int index, std::vector<LedPixel>& out) const override {
        if (index >= 0 && static_cast<std::size_t>(index) < pixels.size()) out = pixels[static_cast<std::size_t>(index)];
        else out.clear();
    }

    std::array<bool, 8> init{};
    std::array<int, 8> port{};
    std::array<int, 8> len{};
    std::array<bool, 8> run{};
    std::array<std::vector<LedPixel>, 8> pixels{};

private:
    template <class T, std::size_t N>
    static T at(const std::array<T, N>& values, int index) {
        return index >= 0 && static_cast<std::size_t>(index) < N ? values[static_cast<std::size_t>(index)] : T{};
    }
};

class FakeRoboRioHal final : public IRoboRioHal {
public:
    double vInVoltage() const override { return vIn; }
    void setVInVoltage(double value) override { vIn = value; }
    double vInCurrent() const override { return vInAmps; }
    void setVInCurrent(double value) override { vInAmps = value; }
    double brownoutVoltage() const override { return brownout; }

    double vIn = 12.0;
    double vInAmps = 0.0;
    double brownout = 6.75;
};

class FakeAccelerometerHal final : public IAccelerometerHal {
public:
    bool active() const override { return isActive; }
    void setAcceleration(double xG, double yG, double zG) override {
        x = xG;
        y = yG;
        z = zG;
    }

    bool isActive = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class FakeCanHal final : public ICanHal {
public:
    std::unique_ptr<ICanSubscription> subscribe(ICanFrameListener& listener) override {
        std::lock_guard lock(mutex_);
        if (refuseSubscribe) return nullptr;
        listener_ = &listener;
        ++subscriptions;
        return std::make_unique<Subscription>(*this, listener);
    }

    void send(const CanFrame& frame) {
        std::lock_guard lock(mutex_);
        if (listener_ != nullptr) listener_->onFrame(frame);
    }

    bool refuseSubscribe = false;
    int subscriptions = 0;

private:
    class Subscription final : public ICanSubscription {
    public:
        Subscription(FakeCanHal& owner, ICanFrameListener& listener) : owner_(owner), listener_(listener) {}
        ~Subscription() override {
            std::lock_guard lock(owner_.mutex_);
            if (owner_.listener_ == &listener_) {
                owner_.listener_ = nullptr;
                --owner_.subscriptions;
            }
        }

    private:
        FakeCanHal& owner_;
        ICanFrameListener& listener_;
    };

    std::mutex mutex_;
    ICanFrameListener* listener_ = nullptr;
};

class FakeHalApi final : public IHalApi {
public:
    IPwmHal& pwm() override { return pwmHal; }
    IDioHal& dio() override { return dioHal; }
    IAnalogInHal& analogIn() override { return analogInHal; }
    IAnalogOutHal& analogOut() override { return analogOutHal; }
    IEncoderHal& encoder() override { return encoderHal; }
    IDutyCycleHal& dutyCycle() override { return dutyCycleHal; }
    IRelayHal& relay() override { return relayHal; }
    IPneumaticsHal& ctrePcm() override { return pcmHal; }
    IPneumaticsHal& revPh() override { return phHal; }
    IPowerDistributionHal& powerDistribution() override { return pdHal; }
    IAddressableLedHal& addressableLed() override { return ledHal; }
    IRoboRioHal& roboRio() override { return rioHal; }
    IAccelerometerHal& accelerometer() override { return accelHal; }
    ICanHal& can() override { return canHal; }

    FakePwmHal pwmHal;
    FakeDioHal dioHal;
    FakeAnalogInHal analogInHal;
    FakeAnalogOutHal analogOutHal;
    FakeEncoderHal encoderHal;
    FakeDutyCycleHal dutyCycleHal;
    FakeRelayHal relayHal;
    FakePneumaticsHal pcmHal{8};
    FakePneumaticsHal phHal{16};
    FakePowerDistributionHal pdHal;
    FakeAddressableLedHal ledHal;
    FakeRoboRioHal rioHal;
    FakeAccelerometerHal accelHal;
    FakeCanHal canHal;
};

}  // namespace wpiemu::bridges::hal::testing
