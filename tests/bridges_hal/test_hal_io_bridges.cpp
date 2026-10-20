#include <doctest/doctest.h>

#include <cmath>
#include <limits>

#include "hal_test_rig.h"

using namespace hal_test;
using sdk::Capability;
namespace sig = wpiemu::sdk::signal;

TEST_CASE("PWM: announces on init, reports normalised speed, retires on free") {
    Rig rig;
    rig.poll();
    CHECK_FALSE(rig.find("pwm", 3).has_value());

    rig.api->pwmHal.init[3] = true;
    rig.api->pwmHal.spd[3] = 0.5;
    rig.api->pwmHal.pulse[3] = 1750;
    rig.poll();

    const auto id = rig.find("pwm", 3);
    REQUIRE(id.has_value());
    const auto& d = rig.info(*id).descriptor;
    CHECK(d.family == "hal");
    CHECK(d.channel == -1);
    CHECK(d.capabilities.has(Capability::MotorOutput));
    CHECK(d.capabilities.has(Capability::PwmOutput));
    CHECK(rig.output(*id, sig::kDutyCycle).value() == doctest::Approx(0.5));
    CHECK(rig.output(*id, sig::kPwmSpeed).value() == doctest::Approx(0.5));
    CHECK(rig.output(*id, "pulseMicroseconds").value() == 1750.0);

    rig.api->pwmHal.spd[3] = -1.0;
    CHECK(rig.output(*id, sig::kDutyCycle).value() == doctest::Approx(-1.0));

    rig.api->pwmHal.init[3] = false;
    rig.poll();
    REQUIRE(rig.host.retired.size() == 1);
    CHECK(rig.host.retired[0] == *id);
    CHECK_FALSE(rig.bridge("hal.pwm").owns(*id));
}

TEST_CASE("PWM: out-of-range speeds and non-finite values are clamped, outputs are read-only") {
    Rig rig;
    rig.api->pwmHal.init[0] = true;
    rig.poll();
    const auto id = rig.find("pwm", 0).value();

    rig.api->pwmHal.spd[0] = 7.5;
    CHECK(rig.output(id, sig::kDutyCycle).value() == 1.0);
    rig.api->pwmHal.spd[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK(rig.output(id, sig::kDutyCycle).value() == 0.0);

    // The sim must never write a robot output: all PWM signals are Output direction.
    for (const auto& spec : rig.info(id).signalSpecs) CHECK(spec.direction == sdk::SignalDirection::Output);
    rig.api->pwmHal.spd[0] = 0.3;
    CHECK(rig.write(id, sig::kDutyCycle, 1.0));
    CHECK(rig.api->pwmHal.spd[0] == 0.3);
}

TEST_CASE("PWM: channels beyond the HAL's count are never announced") {
    Rig rig;
    rig.api->pwmHal.init[19] = true;  // last valid
    rig.poll();
    CHECK(rig.find("pwm", 19).has_value());
    CHECK_FALSE(rig.find("pwm", 20).has_value());
    CHECK_FALSE(rig.api->pwmHal.initialized(20));
    CHECK_FALSE(rig.api->pwmHal.initialized(-1));
    CHECK(rig.countType("pwm") == 1);
}

TEST_CASE("DIO: inputs are driven by the sim and expose digitalIn") {
    Rig rig;
    rig.api->dioHal.init[4] = true;  // default: input
    rig.poll();
    const auto id = rig.find("dio", 4).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::DigitalInput));
    CHECK_FALSE(rig.info(id).descriptor.capabilities.has(Capability::DigitalOutput));
    REQUIRE(rig.info(id).signalSpecs.size() == 1);
    CHECK(rig.info(id).signalSpecs[0].name == sig::kDigitalIn);
    CHECK(rig.info(id).signalSpecs[0].direction == sdk::SignalDirection::Input);

    CHECK(rig.write(id, sig::kDigitalIn, 1.0));
    CHECK(rig.api->dioHal.val[4]);
    CHECK(rig.write(id, sig::kDigitalIn, 0.0));
    CHECK_FALSE(rig.api->dioHal.val[4]);
    CHECK_FALSE(rig.output(id, sig::kDigitalIn).has_value());  // input signals are not collected
}

TEST_CASE("DIO: outputs are observed and never overwritten by the sim") {
    Rig rig;
    rig.api->dioHal.init[9] = true;
    rig.api->dioHal.input[9] = false;
    rig.poll();
    const auto id = rig.find("dio", 9).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::DigitalOutput));
    CHECK_FALSE(rig.info(id).descriptor.capabilities.has(Capability::DigitalInput));

    CHECK(rig.output(id, sig::kDigitalOut).value() == 0.0);
    rig.api->dioHal.val[9] = true;
    CHECK(rig.output(id, sig::kDigitalOut).value() == 1.0);

    // Even a hostile sample for an output pin is ignored.
    rig.apply({{id, 0, 0.0}});
    CHECK(rig.api->dioHal.val[9]);
}

TEST_CASE("DIO: samples with an out-of-range signal index or unknown device are ignored") {
    Rig rig;
    rig.api->dioHal.init[1] = true;
    rig.poll();
    const auto id = rig.find("dio", 1).value();
    rig.apply({{id, 7, 1.0}});
    CHECK_FALSE(rig.api->dioHal.val[1]);
    rig.apply({{sdk::DeviceId{9999}, 0, 1.0}});
    CHECK_FALSE(rig.api->dioHal.val[1]);
    for (auto& b : rig.bridges) CHECK_FALSE(b->owns(sdk::DeviceId{9999}));
}

TEST_CASE("AnalogIn: the sim writes voltage, AnalogOut is observed") {
    Rig rig;
    rig.api->analogInHal.init[2] = true;
    rig.api->analogOutHal.init[1] = true;
    rig.api->analogOutHal.volts[1] = 3.3;
    rig.poll();

    const auto in = rig.find("analog_in", 2).value();
    CHECK(rig.info(in).descriptor.capabilities.has(Capability::AnalogInput));
    CHECK(rig.write(in, sig::kAnalogVoltage, 2.75));
    CHECK(rig.api->analogInHal.volts[2] == 2.75);

    const auto out = rig.find("analog_out", 1).value();
    CHECK(rig.output(out, sig::kAnalogVoltage).value() == doctest::Approx(3.3));
    CHECK_FALSE(rig.write(out, "nope", 1.0));
    rig.apply({{out, 0, 9.0}});  // Output-direction signal: ignored
    CHECK(rig.api->analogOutHal.volts[1] == 3.3);
}

TEST_CASE("Encoder: A/B pairing is reported and position/velocity map onto HAL counts") {
    Rig rig;
    auto& e = rig.api->encoderHal;
    e.init[1] = true;
    e.chanA[1] = 6;
    e.chanB[1] = 7;
    e.dpp[1] = 0.01;
    rig.poll();

    const auto id = rig.find("encoder", 1).value();
    const auto& d = rig.info(id).descriptor;
    CHECK(d.capabilities.has(Capability::PositionSensor));
    CHECK(d.capabilities.has(Capability::VelocitySensor));
    CHECK(d.displayName.find("DIO 6/7") != std::string::npos);
    CHECK(rig.output(id, "digitalChannelA").value() == 6.0);
    CHECK(rig.output(id, "digitalChannelB").value() == 7.0);
    CHECK(rig.output(id, "distancePerPulse").value() == doctest::Approx(0.01));
    CHECK(rig.output(id, "reverseDirection").value() == 0.0);

    CHECK(rig.write(id, sig::kPosition, 1.5));  // 2048 counts / rev
    CHECK(e.cnt[1] == 3072);
    CHECK(rig.write(id, sig::kVelocity, 2.0));
    CHECK(e.per[1] == doctest::Approx(1.0 / 4096.0));
    CHECK(e.dir[1]);
    CHECK(rig.write(id, sig::kVelocity, -1.0));
    CHECK(e.per[1] == doctest::Approx(-1.0 / 2048.0));
    CHECK_FALSE(e.dir[1]);
    CHECK(rig.write(id, sig::kVelocity, 0.0));
    CHECK(std::isinf(e.per[1]));
    CHECK_FALSE(e.dir[1]);  // direction is kept while stopped
    CHECK(rig.write(id, "count", 123.0));
    CHECK(e.cnt[1] == 123);
}

TEST_CASE("Encoder: a robot-reversed encoder reads negative like real hardware") {
    Rig rig;
    auto& e = rig.api->encoderHal;
    e.init[0] = true;
    e.reversed[0] = true;
    rig.poll();
    const auto id = rig.find("encoder", 0).value();
    CHECK(rig.output(id, "reverseDirection").value() == 1.0);
    CHECK(rig.write(id, sig::kPosition, 1.0));
    CHECK(e.cnt[0] == -2048);
    CHECK(rig.write(id, sig::kVelocity, 1.0));
    CHECK(e.per[0] < 0.0);
    CHECK_FALSE(e.dir[0]);
}

TEST_CASE("Encoder: counts per revolution is configurable and huge positions saturate") {
    hal::HalBridgeOptions opts;
    opts.encoderCountsPerRevolution = 4096.0;
    Rig rig(opts);
    rig.api->encoderHal.init[0] = true;
    rig.poll();
    const auto id = rig.find("encoder", 0).value();
    CHECK(rig.write(id, sig::kPosition, 0.5));
    CHECK(rig.api->encoderHal.cnt[0] == 2048);
    CHECK(rig.write(id, sig::kPosition, 1e12));
    CHECK(rig.api->encoderHal.cnt[0] == std::numeric_limits<int>::max());
    CHECK(rig.write(id, sig::kPosition, std::nan("")));
    CHECK(rig.api->encoderHal.cnt[0] == 0);
}

TEST_CASE("DutyCycle: connects with a default frequency, wraps position, restores on stop") {
    Rig rig;
    auto& dc = rig.api->dutyCycleHal;
    dc.init[2] = true;
    dc.chan[2] = 9;
    rig.poll();
    const auto id = rig.find("duty_cycle", 2).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::PositionSensor));
    CHECK(dc.freq[2] == 975);
    CHECK(rig.output(id, "digitalChannel").value() == 9.0);

    CHECK(rig.write(id, sig::kPosition, 3.25));
    CHECK(dc.out[2] == doctest::Approx(0.25));
    CHECK(rig.write(id, sig::kPosition, -0.1));
    CHECK(dc.out[2] == doctest::Approx(0.9));

    rig.bridge("hal.duty_cycle").stop();
    CHECK(dc.freq[2] == 0);
}

TEST_CASE("DutyCycle: a frequency the robot or sim already set is left alone") {
    hal::HalBridgeOptions opts;
    opts.dutyCycleDefaultFrequencyHz = 500;
    Rig rig(opts);
    auto& dc = rig.api->dutyCycleHal;
    dc.init[0] = true;
    dc.freq[0] = 1200;
    dc.init[1] = true;
    rig.poll();
    CHECK(dc.freq[0] == 1200);
    CHECK(dc.freq[1] == 500);

    const auto id = rig.find("duty_cycle", 1).value();
    CHECK(rig.write(id, "frequency", 800.0));
    CHECK(dc.freq[1] == 800);
    rig.bridge("hal.duty_cycle").stop();
    CHECK(dc.freq[0] == 1200);
    CHECK(dc.freq[1] == 800);  // sim overrode our default, so it is not ours to undo
}

TEST_CASE("Relay: forward and reverse are separate devices on one header") {
    Rig rig;
    auto& r = rig.api->relayHal;
    r.fwdInit[1] = true;
    r.revInit[1] = true;
    r.fwdInit[3] = true;  // forward only
    r.fwd[1] = true;
    rig.poll();

    REQUIRE(rig.countType("relay") == 3);
    const auto fwd = rig.find("relay", 1, 0).value();
    const auto rev = rig.find("relay", 1, 1).value();
    CHECK(rig.find("relay", 3, 0).has_value());
    CHECK_FALSE(rig.find("relay", 3, 1).has_value());
    CHECK(rig.info(fwd).descriptor.capabilities.has(Capability::DigitalOutput));
    CHECK(rig.output(fwd, sig::kDigitalOut).value() == 1.0);
    CHECK(rig.output(rev, sig::kDigitalOut).value() == 0.0);

    r.rev[1] = true;
    CHECK(rig.output(rev, sig::kDigitalOut).value() == 1.0);

    r.revInit[1] = false;
    rig.poll();
    REQUIRE(rig.host.retired.size() == 1);
    CHECK(rig.host.retired[0] == rev);
    CHECK(rig.bridge("hal.relay").owns(fwd));
}
