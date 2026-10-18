#pragma once

// In-memory IHalApi for unit tests (bridges here, and any module that wants a HAL without WPILib).
// Every field is public so a test can arrange "the robot did X" and assert "the bridge wrote Y".
// Out-of-range indices behave like a HAL with fewer ports: reads return neutral values, writes are ignored.

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "wpiemu/bridges/hal/hal_api.h"

namespace wpiemu::bridges::hal::testing {

namespace detail {
template <class T>
T at(const std::vector<T>& v, int i, T fallback = T{}) {
    if (i < 0 || static_cast<std::size_t>(i) >= v.size()) return fallback;
    return v[static_cast<std::size_t>(i)];
}
template <class T>
void put(std::vector<T>& v, int i, T value) {
    if (i >= 0 && static_cast<std::size_t>(i) < v.size()) v[static_cast<std::size_t>(i)] = value;
}
}  // namespace detail

class FakePwm final : public IPwmHal {
public:
    explicit FakePwm(std::size_t n = 20) : init(n, false), spd(n, 0.0), pos(n, 0.0), pulse(n, 0) {}
    std::vector<bool> init;
    std::vector<double> spd, pos;
    std::vector<int> pulse;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int c) const override { return detail::at(init, c, false); }
    double speed(int c) const override { return detail::at(spd, c); }
    double position(int c) const override { return detail::at(pos, c); }
    int pulseMicroseconds(int c) const override { return detail::at(pulse, c); }
};

class FakeDio final : public IDioHal {
public:
    explicit FakeDio(std::size_t n = 26) : init(n, false), input(n, true), val(n, false) {}
    std::vector<bool> init, input, val;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int c) const override { return detail::at(init, c, false); }
    bool isInput(int c) const override { return detail::at(input, c, true); }
    bool value(int c) const override { return detail::at(val, c, false); }
    void setValue(int c, bool v) override { detail::put(val, c, v); }
};

class FakeAnalogIn final : public IAnalogInHal {
public:
    explicit FakeAnalogIn(std::size_t n = 8) : init(n, false), volts(n, 0.0) {}
    std::vector<bool> init;
    std::vector<double> volts;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int c) const override { return detail::at(init, c, false); }
    double voltage(int c) const override { return detail::at(volts, c); }
    void setVoltage(int c, double v) override { detail::put(volts, c, v); }
};

class FakeAnalogOut final : public IAnalogOutHal {
public:
    explicit FakeAnalogOut(std::size_t n = 2) : init(n, false), volts(n, 0.0) {}
    std::vector<bool> init;
    std::vector<double> volts;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int c) const override { return detail::at(init, c, false); }
    double voltage(int c) const override { return detail::at(volts, c); }
};

class FakeEncoder final : public IEncoderHal {
public:
    explicit FakeEncoder(std::size_t n = 8)
        : init(n, false), chanA(n, 0), chanB(n, 1), dpp(n, 1.0), reversed(n, false), cnt(n, 0),
          per(n, std::numeric_limits<double>::infinity()), dir(n, true) {}
    std::vector<bool> init;
    std::vector<int> chanA, chanB;
    std::vector<double> dpp;
    std::vector<bool> reversed;
    std::vector<int> cnt;
    std::vector<double> per;
    std::vector<bool> dir;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int i) const override { return detail::at(init, i, false); }
    int digitalChannelA(int i) const override { return detail::at(chanA, i, -1); }
    int digitalChannelB(int i) const override { return detail::at(chanB, i, -1); }
    double distancePerPulse(int i) const override { return detail::at(dpp, i); }
    bool reverseDirection(int i) const override { return detail::at(reversed, i, false); }
    int count(int i) const override { return detail::at(cnt, i); }
    void setCount(int i, int c) override { detail::put(cnt, i, c); }
    void setPeriod(int i, double p) override { detail::put(per, i, p); }
    void setDirection(int i, bool positive) override { detail::put(dir, i, positive); }
};

class FakeDutyCycle final : public IDutyCycleHal {
public:
    explicit FakeDutyCycle(std::size_t n = 8) : init(n, false), chan(n, 0), out(n, 0.0), freq(n, 0) {}
    std::vector<bool> init;
    std::vector<int> chan;
    std::vector<double> out;
    std::vector<int> freq;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int i) const override { return detail::at(init, i, false); }
    int digitalChannel(int i) const override { return detail::at(chan, i, -1); }
    double output(int i) const override { return detail::at(out, i); }
    void setOutput(int i, double v) override { detail::put(out, i, v); }
    int frequency(int i) const override { return detail::at(freq, i); }
    void setFrequency(int i, int hz) override { detail::put(freq, i, hz); }
};

class FakeRelay final : public IRelayHal {
public:
    explicit FakeRelay(std::size_t n = 4) : fwdInit(n, false), revInit(n, false), fwd(n, false), rev(n, false) {}
    std::vector<bool> fwdInit, revInit, fwd, rev;
    int headerCount() const override { return static_cast<int>(fwdInit.size()); }
    bool forwardInitialized(int h) const override { return detail::at(fwdInit, h, false); }
    bool reverseInitialized(int h) const override { return detail::at(revInit, h, false); }
    bool forward(int h) const override { return detail::at(fwd, h, false); }
    bool reverse(int h) const override { return detail::at(rev, h, false); }
};

class FakePneumatics final : public IPneumaticsHal {
public:
    FakePneumatics(int modules, int channels)
        : channels_(channels),
          init(static_cast<std::size_t>(modules), false),
          solenoids(static_cast<std::size_t>(modules * channels), false),
          compressor(static_cast<std::size_t>(modules), false),
          closed(static_cast<std::size_t>(modules), false),
          pressure(static_cast<std::size_t>(modules), false),
          current(static_cast<std::size_t>(modules), 0.0) {}
    int channels_;
    std::vector<bool> init;
    std::vector<bool> solenoids;
    std::vector<bool> compressor, closed, pressure;
    std::vector<double> current;
    int moduleCount() const override { return static_cast<int>(init.size()); }
    int solenoidChannelCount() const override { return channels_; }
    bool moduleInitialized(int m) const override { return detail::at(init, m, false); }
    bool solenoidOutput(int m, int c) const override {
        return m >= 0 && m < moduleCount() && c >= 0 && c < channels_ && solenoids[static_cast<std::size_t>(m * channels_ + c)];
    }
    void setSolenoid(int m, int c, bool v) {
        if (m >= 0 && m < moduleCount() && c >= 0 && c < channels_) solenoids[static_cast<std::size_t>(m * channels_ + c)] = v;
    }
    bool compressorOn(int m) const override { return detail::at(compressor, m, false); }
    bool closedLoopEnabled(int m) const override { return detail::at(closed, m, false); }
    bool pressureSwitch(int m) const override { return detail::at(pressure, m, false); }
    void setPressureSwitch(int m, bool v) override { detail::put(pressure, m, v); }
    double compressorCurrent(int m) const override { return detail::at(current, m); }
    void setCompressorCurrent(int m, double a) override { detail::put(current, m, a); }
};

class FakePowerDistribution final : public IPowerDistributionHal {
public:
    FakePowerDistribution(int modules, int channels)
        : channels_(channels),
          init(static_cast<std::size_t>(modules), false),
          volts(static_cast<std::size_t>(modules), 12.0),
          temp(static_cast<std::size_t>(modules), 25.0),
          amps(static_cast<std::size_t>(modules * channels), 0.0) {}
    int channels_;
    std::vector<bool> init;
    std::vector<double> volts, temp, amps;
    int moduleCount() const override { return static_cast<int>(init.size()); }
    int channelCount() const override { return channels_; }
    bool initialized(int m) const override { return detail::at(init, m, false); }
    double voltage(int m) const override { return detail::at(volts, m); }
    void setVoltage(int m, double v) override { detail::put(volts, m, v); }
    double temperature(int m) const override { return detail::at(temp, m); }
    void setTemperature(int m, double t) override { detail::put(temp, m, t); }
    double current(int m, int c) const override {
        return (m >= 0 && m < moduleCount() && c >= 0 && c < channels_) ? amps[static_cast<std::size_t>(m * channels_ + c)] : 0.0;
    }
    void setCurrent(int m, int c, double a) override {
        if (m >= 0 && m < moduleCount() && c >= 0 && c < channels_) amps[static_cast<std::size_t>(m * channels_ + c)] = a;
    }
};

class FakeAddressableLed final : public IAddressableLedHal {
public:
    explicit FakeAddressableLed(std::size_t n = 1) : init(n, false), port(n, 0), len(n, 0), run(n, false), pixels(n) {}
    std::vector<bool> init;
    std::vector<int> port, len;
    std::vector<bool> run;
    std::vector<std::vector<LedPixel>> pixels;
    int channelCount() const override { return static_cast<int>(init.size()); }
    bool initialized(int i) const override { return detail::at(init, i, false); }
    int outputPort(int i) const override { return detail::at(port, i); }
    int length(int i) const override { return detail::at(len, i); }
    bool running(int i) const override { return detail::at(run, i, false); }
    void readPixels(int i, std::vector<LedPixel>& out) const override {
        out.clear();
        if (i >= 0 && static_cast<std::size_t>(i) < pixels.size()) out = pixels[static_cast<std::size_t>(i)];
    }
};

class FakeRoboRio final : public IRoboRioHal {
public:
    double vIn = 12.0, vInAmps = 0.0, brownout = 6.75;
    double vInVoltage() const override { return vIn; }
    void setVInVoltage(double v) override { vIn = v; }
    double vInCurrent() const override { return vInAmps; }
    void setVInCurrent(double a) override { vInAmps = a; }
    double brownoutVoltage() const override { return brownout; }
};

class FakeAccelerometer final : public IAccelerometerHal {
public:
    bool isActive = false;
    double x = 0.0, y = 0.0, z = 0.0;  // g
    bool active() const override { return isActive; }
    void setAcceleration(double xG, double yG, double zG) override {
        x = xG;
        y = yG;
        z = zG;
    }
};

class FakeCan final : public ICanHal {
public:
    // Simulates the robot sending a frame; ignored when nobody is subscribed.
    void send(const CanFrame& frame) {
        if (listener != nullptr) listener->onFrame(frame);
    }
    int subscriptions = 0;
    bool refuseSubscribe = false;
    ICanFrameListener* listener = nullptr;

    std::unique_ptr<ICanSubscription> subscribe(ICanFrameListener& l) override {
        if (refuseSubscribe) return nullptr;
        listener = &l;
        ++subscriptions;
        return std::make_unique<Subscription>(*this);
    }

private:
    class Subscription final : public ICanSubscription {
    public:
        explicit Subscription(FakeCan& owner) : owner_(owner) {}
        ~Subscription() override {
            owner_.listener = nullptr;
            --owner_.subscriptions;
        }

    private:
        FakeCan& owner_;
    };
};

// Defaults match a roboRIO sim HAL: 20 PWM, 26 DIO, 8 analog in, 2 analog out, 8 encoders, 8 duty
// cycles, 4 relays, 63 PCM x 8 solenoids, 63 PH x 16, 63 PD x 24, 1 LED.
class FakeHalApi final : public IHalApi {
public:
    FakePwm pwmHal;
    FakeDio dioHal;
    FakeAnalogIn analogInHal;
    FakeAnalogOut analogOutHal;
    FakeEncoder encoderHal;
    FakeDutyCycle dutyCycleHal;
    FakeRelay relayHal;
    FakePneumatics pcmHal{63, 8};
    FakePneumatics phHal{63, 16};
    FakePowerDistribution pdHal{63, 24};
    FakeAddressableLed ledHal;
    FakeRoboRio rioHal;
    FakeAccelerometer accelHal;
    FakeCan canHal;

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
};

}  // namespace wpiemu::bridges::hal::testing
