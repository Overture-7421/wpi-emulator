#include <doctest/doctest.h>

#include <algorithm>

#include "hal_test_rig.h"

using namespace hal_test;
using sdk::Capability;
namespace sig = wpiemu::sdk::signal;

TEST_CASE("PCM: module device carries compressor state and sim-driven pressure switch") {
    Rig rig;
    auto& p = rig.api->pcmHal;
    p.init[2] = true;
    p.compressor[2] = true;
    p.closed[2] = true;
    rig.poll();

    const auto id = rig.find("ctre_pcm", 2).value();
    CHECK(rig.info(id).descriptor.vendor == "ctre");
    CHECK(rig.output(id, "compressorOn").value() == 1.0);
    CHECK(rig.output(id, "compressorClosedLoop").value() == 1.0);

    CHECK(rig.write(id, "pressureSwitch", 1.0));
    CHECK(p.pressure[2]);
    CHECK(rig.write(id, "compressorCurrent", 7.5));
    CHECK(p.current[2] == 7.5);
    p.compressor[2] = false;
    CHECK(rig.output(id, "compressorOn").value() == 0.0);

    p.init[2] = false;
    rig.poll();
    CHECK(rig.host.retired.size() == 1);
}

TEST_CASE("PCM: a solenoid channel is announced when first commanded on and stays latched") {
    Rig rig;
    auto& p = rig.api->pcmHal;
    p.init[0] = true;
    rig.poll();
    CHECK(rig.countType("ctre_pcm_solenoid") == 0);

    p.setSolenoid(0, 5, true);
    rig.poll();
    const auto id = rig.find("ctre_pcm_solenoid", 0, 5).value();
    const auto& d = rig.info(id).descriptor;
    CHECK(d.capabilities.has(Capability::Solenoid));
    CHECK(d.id == 0);
    CHECK(d.channel == 5);
    CHECK(rig.output(id, sig::kSolenoid).value() == 1.0);

    p.setSolenoid(0, 5, false);
    rig.poll();
    CHECK(rig.output(id, sig::kSolenoid).value() == 0.0);
    CHECK(rig.host.retired.empty());  // latched: a closed valve is still a known device
    CHECK(rig.countType("ctre_pcm_solenoid") == 1);

    // Freeing the module retires the channel and clears the latch.
    p.init[0] = false;
    rig.poll();
    CHECK(rig.host.retired.size() == 2);  // module + solenoid
    p.init[0] = true;
    rig.poll();
    CHECK(rig.countType("ctre_pcm_solenoid") == 1);  // not re-announced until commanded again
    CHECK(rig.host.retired.size() == 2);
}

TEST_CASE("PCM: solenoids of a module that was never initialised are ignored") {
    Rig rig;
    rig.api->pcmHal.setSolenoid(4, 0, true);  // module 4 not initialised
    rig.poll();
    CHECK(rig.countType("ctre_pcm_solenoid") == 0);
    rig.api->pcmHal.setSolenoid(63, 0, true);   // out-of-range module: ignored by the fake, never crashes
    rig.api->pcmHal.setSolenoid(0, 8, true);    // out-of-range channel
    rig.poll();
    CHECK(rig.countType("ctre_pcm_solenoid") == 0);
}

TEST_CASE("PCM: announceAllSolenoidChannels announces every channel of a live module") {
    hal::HalBridgeOptions opts;
    opts.announceAllSolenoidChannels = true;
    Rig rig(opts);
    rig.api->pcmHal.init[1] = true;
    rig.api->phHal.init[0] = true;
    rig.poll();
    CHECK(rig.countType("ctre_pcm_solenoid") == 8);
    CHECK(rig.countType("rev_ph_solenoid") == 16);
    CHECK(rig.find("rev_ph_solenoid", 0, 15).has_value());
    CHECK_FALSE(rig.find("rev_ph_solenoid", 0, 16).has_value());
}

TEST_CASE("PH: uses its own families and 16 channels per module") {
    Rig rig;
    auto& ph = rig.api->phHal;
    ph.init[1] = true;
    ph.setSolenoid(1, 15, true);
    ph.compressor[1] = true;
    rig.poll();
    const auto mod = rig.find("rev_ph", 1).value();
    const auto sol = rig.find("rev_ph_solenoid", 1, 15).value();
    CHECK(rig.info(mod).descriptor.vendor == "rev");
    CHECK(rig.output(mod, "compressorOn").value() == 1.0);
    CHECK(rig.output(sol, sig::kSolenoid).value() == 1.0);
    CHECK(rig.bridge("hal.rev_ph.solenoid").owns(sol));
    CHECK_FALSE(rig.bridge("hal.ctre_pcm.solenoid").owns(sol));
}

TEST_CASE("PowerDistribution: sim drives bus voltage, temperature and per-channel current") {
    Rig rig;
    auto& pd = rig.api->pdHal;
    pd.init[0] = true;
    rig.poll();
    const auto id = rig.find("power_distribution", 0).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::SupplyVoltage));
    REQUIRE(rig.info(id).specs.size() == 2u + 24u);

    CHECK(rig.write(id, sig::kSupplyVoltage, 11.2));
    CHECK(pd.volts[0] == 11.2);
    CHECK(rig.write(id, "temperature", 41.0));
    CHECK(pd.temp[0] == 41.0);
    CHECK(rig.write(id, "channelCurrent23", 30.0));
    CHECK(pd.current(0, 23) == 30.0);
    CHECK(rig.write(id, "channelCurrent0", 1.5));
    CHECK(pd.current(0, 0) == 1.5);
    CHECK_FALSE(rig.write(id, "channelCurrent24", 5.0));  // no such channel
}

TEST_CASE("roboRIO: battery voltage and current are written back, brownout is observed") {
    Rig rig;
    rig.poll();
    const auto id = rig.find("roborio", 0).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::SupplyVoltage));
    CHECK(rig.write(id, sig::kSupplyVoltage, 10.4));
    CHECK(rig.api->rioHal.vIn == 10.4);
    CHECK(rig.write(id, "vInCurrent", 88.0));
    CHECK(rig.api->rioHal.vInAmps == 88.0);

    CHECK(rig.output(id, "brownoutVoltage").value() == 6.75);
    rig.api->rioHal.brownout = 6.3;
    CHECK(rig.output(id, "brownoutVoltage").value() == 6.3);
}

TEST_CASE("Accelerometer: appears when the robot activates it and converts m/s^2 to g") {
    Rig rig;
    rig.poll();
    CHECK_FALSE(rig.find("accelerometer", 0).has_value());
    rig.api->accelHal.isActive = true;
    rig.poll();
    const auto id = rig.find("accelerometer", 0).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::Imu));

    CHECK(rig.write(id, sig::kAccelZ, 9.80665));
    CHECK(rig.api->accelHal.z == doctest::Approx(1.0));
    CHECK(rig.write(id, sig::kAccelX, -4.903325));
    CHECK(rig.api->accelHal.x == doctest::Approx(-0.5));
    CHECK(rig.api->accelHal.z == doctest::Approx(1.0));  // components are independent

    rig.api->accelHal.isActive = false;
    rig.poll();
    CHECK(rig.host.retired.size() == 1);
}

TEST_CASE("Addressable LED: strip state is observed as a summary") {
    Rig rig;
    auto& led = rig.api->ledHal;
    led.init[0] = true;
    led.run[0] = true;
    led.len[0] = 4;
    led.port[0] = 9;
    led.pixels[0] = {{255, 0, 0}, {255, 0, 0}, {0, 0, 255}, {0, 0, 255}};
    rig.poll();

    const auto id = rig.find("addressable_led", 0).value();
    CHECK(rig.info(id).descriptor.capabilities.has(Capability::Led));
    CHECK(rig.output(id, "running").value() == 1.0);
    CHECK(rig.output(id, "length").value() == 4.0);
    CHECK(rig.output(id, "outputPort").value() == 9.0);
    CHECK(rig.output(id, "avgRed").value() == doctest::Approx(127.5));
    CHECK(rig.output(id, "avgGreen").value() == 0.0);
    CHECK(rig.output(id, "avgBlue").value() == doctest::Approx(127.5));
    CHECK(rig.output(id, "firstPixelRgb").value() == double(0xFF0000));

    led.pixels[0].clear();
    CHECK(rig.output(id, "avgRed").value() == 0.0);  // empty strip
    for (const auto& spec : rig.info(id).specs) CHECK(spec.direction == sdk::SignalDirection::Output);
}

namespace {
wpiemu::bridges::hal::CanFrame frameFor(int mfr, int devType, int api, int devId, std::initializer_list<int> bytes,
                                        int period = 0) {
    wpiemu::bridges::hal::CanFrame f;
    f.arbitrationId = (static_cast<std::uint32_t>(devType) << 24) | (static_cast<std::uint32_t>(mfr) << 16) |
                      (static_cast<std::uint32_t>(api) << 6) | static_cast<std::uint32_t>(devId);
    for (int b : bytes) f.data[f.size++] = static_cast<std::uint8_t>(b);
    f.periodMs = period;
    return f;
}
}  // namespace

TEST_CASE("CAN: arbitration ids decode into manufacturer / device type / api / id") {
    using wpiemu::bridges::hal::CanArbitrationId;
    // Source of truth is hal/CANAPITypes.h: kCTRE = 4, kMotorController = 2.
    const auto d = CanArbitrationId::decode(frameFor(4, 2, 0x155, 9, {}).arbitrationId);
    CHECK(d.manufacturer == 4);
    CHECK(d.deviceType == 2);
    CHECK(d.apiId == 0x155);
    CHECK(d.deviceId == 9);
}

TEST_CASE("CAN: frames create observe-only devices keyed by manufacturer, type and id") {
    Rig rig;
    auto& can = rig.api->canHal;
    CHECK(can.subscriptions == 1);

    can.send(frameFor(5, 2, 0x10, 12, {0x01, 0x02, 0x03, 0x04, 0x05}, 20));
    can.send(frameFor(5, 2, 0x11, 12, {0xFF}, 0));
    can.send(frameFor(4, 8, 0x01, 0, {0xAA, 0xBB}));
    can.send(frameFor(5, 2, 0x10, 13, {}));
    CHECK(rig.countType("can_rev_motor_controller") == 0);  // nothing announced before poll()
    rig.poll();

    CHECK(rig.countType("can_rev_motor_controller") == 2);
    CHECK(rig.countType("can_ctre_power_distribution") == 1);
    const auto id = rig.find("can_rev_motor_controller", 12).value();
    CHECK(rig.info(id).descriptor.capabilities.bits == 0u);
    CHECK(rig.output(id, "frameCount").value() == 2.0);
    CHECK(rig.output(id, "lastApiId").value() == 0x11);
    CHECK(rig.output(id, "lastPeriodMs").value() == 0.0);
    CHECK(rig.output(id, "lastDataLow").value() == 0xFF);
    CHECK(rig.output(id, "lastDataHigh").value() == 0.0);
    CHECK(rig.bridge("hal.can").owns(id));

    can.send(frameFor(5, 2, 0x10, 12, {1, 2, 3, 4, 5, 6, 7, 8}, 20));
    CHECK(rig.output(id, "frameCount").value() == 3.0);
    CHECK(rig.output(id, "lastDataLow").value() == double(0x04030201u));
    CHECK(rig.output(id, "lastDataHigh").value() == double(0x08070605u));
    CHECK(rig.output(id, "lastPeriodMs").value() == 20.0);

    // Nothing is ever written back to the bus.
    for (const auto& spec : rig.info(id).specs) CHECK(spec.direction == sdk::SignalDirection::Output);
}

TEST_CASE("CAN: broadcast frames and unknown ids are handled safely") {
    Rig rig;
    auto& can = rig.api->canHal;
    can.send(frameFor(0, 0, 0, 0, {1}));      // broadcast
    can.send(frameFor(200, 30, 1, 63, {1}));  // unknown manufacturer / device type, highest id
    rig.poll();
    CHECK(rig.countType("can_mfr200_type30") == 1);
    CHECK(rig.host.announced.size() == 2);  // + roboRIO only
    CHECK(wpiemu::bridges::hal::canManufacturerName(5) == "rev");
    CHECK(wpiemu::bridges::hal::canDeviceTypeName(31) == "firmware_update");
}

TEST_CASE("CAN: stop unsubscribes and a refused subscription fails start") {
    Rig rig;
    CHECK(rig.api->canHal.subscriptions == 1);
    rig.bridge("hal.can").stop();
    CHECK(rig.api->canHal.subscriptions == 0);
    rig.api->canHal.send(frameFor(4, 2, 1, 1, {1}));  // no listener: dropped
    rig.poll();
    CHECK(rig.countType("can_ctre_motor_controller") == 0);

    auto refusing = std::make_shared<hal::testing::FakeHalApi>();
    refusing->canHal.refuseSubscribe = true;
    sdk::DeviceBridgeRegistry r2;
    REQUIRE(hal::registerHalBridges(r2, refusing).ok());
    auto b2 = r2.create("hal.can");
    REQUIRE(b2.ok());
    RecordingHost host;
    CHECK(b2.value()->start(host).error().code == sdk::ErrorCode::Unsupported);
    b2.value()->poll();
    CHECK(host.announced.empty());
}

TEST_CASE("every bridge survives a full lifecycle: start, poll, collect, apply, stop twice") {
    Rig rig;
    auto& a = *rig.api;
    a.pwmHal.init[0] = a.dioHal.init[0] = a.analogInHal.init[0] = a.encoderHal.init[0] = true;
    a.pcmHal.init[0] = a.phHal.init[0] = a.pdHal.init[0] = a.ledHal.init[0] = true;
    a.pcmHal.setSolenoid(0, 0, true);
    rig.poll();
    CHECK_FALSE(rig.outputs().empty());
    for (auto& b : rig.bridges) {
        b->stop();
        b->stop();
        b->poll();  // after stop there is no host: must be a no-op
    }
    CHECK(rig.api->canHal.subscriptions == 0);
}
