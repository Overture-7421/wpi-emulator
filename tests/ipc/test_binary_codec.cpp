#include <doctest/doctest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>

#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;

namespace {

std::vector<std::byte> bytesOf(std::initializer_list<int> v) {
    std::vector<std::byte> out;
    for (int b : v) out.push_back(static_cast<std::byte>(b));
    return out;
}

std::vector<std::byte> encodeOk(const Message& m) {
    std::vector<std::byte> out;
    const Status s = BinaryMessageCodec().encode(m, out);
    REQUIRE_MESSAGE(s.ok(), (s.ok() ? std::string() : s.error().message));
    return out;
}

Message decodeOk(const std::vector<std::byte>& frame) {
    auto r = BinaryMessageCodec().decode(frame);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? std::string() : r.error().message));
    return std::move(r).value();
}

Hello sampleHello() {
    Hello h;
    h.language = RobotLanguage::Java;
    h.wpilibYear = 2026;
    h.wpilibVersion = "2026.2.1";
    h.extensionVersion = "0.1.0";
    h.pid = 1234567890123LL;
    h.programName = "Robot \xC3\xA9 name";
    h.halPortCounts = {{"pwm", 20}, {"dio", 26}, {"can", -1}};
    h.proposedWatchdogMs = 750;
    h.supportsArmed = true;
    return h;
}

HelloAck sampleHelloAck() {
    HelloAck a;
    a.accepted = false;
    a.reason = "no";
    a.clockMode = ClockMode::Realtime;
    a.stepUs = 2500;
    a.sessionId = 0x1122334455667788ULL;
    a.discoveryOnly = true;
    a.enableL3Hooks = true;
    a.runEpoch = 9;
    a.watchdogTimeoutMs = 4321;
    a.watchdogPolicy = WatchdogPolicy::Terminate;
    return a;
}

DeviceAnnounce sampleAnnounce() {
    DeviceAnnounce d;
    d.id = DeviceId{42};
    d.descriptor.family = "phoenix6";
    d.descriptor.type = "talonfx";
    d.descriptor.vendor = "ctre";
    d.descriptor.id = 7;
    d.descriptor.bus = "canivore";
    d.descriptor.channel = -1;
    d.descriptor.capabilities = Capability::MotorOutput | Capability::RotorFeedback;
    d.descriptor.nativeName = "CANMotor:Talon FX 7";
    d.descriptor.displayName = "Left drive";
    d.signalSpecs = {{"motorVoltage", SignalDirection::Output, SignalType::Double, "V"},
                     {"rotorPosition", SignalDirection::Input, SignalType::Double, "rot"},
                     {"fwdLimit", SignalDirection::Bidirectional, SignalType::Bool, ""},
                     {"count", SignalDirection::Output, SignalType::Int, ""}};
    d.retired = true;
    d.observedBus = "CANivore1";
    return d;
}

std::vector<SignalSample> sampleSignals(std::size_t n) {
    std::vector<SignalSample> s;
    for (std::size_t i = 0; i < n; ++i)
        s.push_back({DeviceId{static_cast<std::uint32_t>(i % 5 + 1)}, static_cast<std::uint16_t>(i % 3), 0.125 * static_cast<double>(i) - 3.0});
    return s;
}

SensorFrame sampleSensor() {
    SensorFrame f;
    f.tick = 0xFFFFFFFFFFULL;
    f.simTimeUs = 123456789012LL;
    f.stepUs = 5000;
    f.batteryVoltageV = 11.875;
    f.batteryCurrentA = 123.5;
    f.full = true;
    f.samples = sampleSignals(40);
    f.sessionId = 3;
    f.runEpoch = 4;
    return f;
}

ActuatorFrame sampleActuator() {
    ActuatorFrame f;
    f.tick = 17;
    f.simTimeUs = -5;
    f.robotLoopCount = 99;
    f.full = true;
    f.samples = sampleSignals(7);
    f.sessionId = 3;
    f.runEpoch = 4;
    f.ackTick = 16;
    f.result = TickResult::RobotStalled;
    return f;
}

DsState sampleDs() {
    DsState d;
    d.dsAttached = false;
    d.fmsAttached = true;
    d.enabled = true;
    d.eStop = true;
    d.mode = RobotMode::Test;
    d.station = AllianceStation::Blue3;
    d.matchTimeS = 134.5;
    d.gameSpecificMessage = "RBG";
    d.joysticks[0].present = true;
    d.joysticks[0].axes = {0.0f, -1.0f, 0.5f, 1.0f, -0.25f, 0.75f};
    d.joysticks[0].povs = {-1, 0, 315};
    d.joysticks[0].buttons = 0xA5A5A5A5u;
    d.joysticks[0].buttonCount = 12;
    d.joysticks[5].present = true;
    d.joysticks[5].buttons = 1;
    d.joysticks[5].buttonCount = 1;
    d.sessionId = 5;
    d.runEpoch = 6;
    return d;
}

SimControl sampleControl() {
    SimControl c;
    c.command = SimCommand::SetTimeScale;
    c.stepCount = 3;
    c.timeScale = 0.25;
    c.commandId = 77;
    c.sessionId = 5;
    c.runEpoch = 6;
    return c;
}

SimControlAck sampleControlAck() {
    SimControlAck a;
    a.commandId = 77;
    a.command = SimCommand::Reset;
    a.result = ControlResult::StaleEpoch;
    a.sessionId = 5;
    a.runEpoch = 6;
    a.tick = 1000;
    a.detail = "Reset must carry a newer runEpoch";
    return a;
}

TagDetection sampleTag(int id) {
    TagDetection t;
    t.id = id;
    t.cameraToTag = Pose3({2.5f + static_cast<float>(id), -0.25f, 0.5f}, {0.f, 0.f, 0.70710678f, 0.70710678f});
    t.yawDeg = -12.5 + id;
    t.pitchDeg = 3.25;
    t.areaPercent = 0.0625;
    t.ambiguity = 0.0;
    for (std::size_t i = 0; i < t.cornersPx.size(); ++i) t.cornersPx[i] = 100.5 + 10.0 * static_cast<double>(i) + id;
    t.distanceM = 2.75 + id;
    return t;
}

CameraFrame sampleCameraFrame(std::uint32_t index, std::size_t tags, bool hasPose) {
    CameraFrame f;
    f.cameraIndex = index;
    f.sequence = 0x100000001ULL + index;
    f.captureTimeUs = 1000000 + index;
    f.publishTimeUs = 1040000 + index;
    f.robotToCamera = Pose3({0.3f, -0.1f, 0.6f}, {0.f, 0.258819f, 0.f, 0.965926f});
    for (std::size_t i = 0; i < tags; ++i) f.tags.push_back(sampleTag(static_cast<int>(i) + 1));
    f.hasRobotPose = hasPose;
    if (hasPose) f.robotPoseField = Pose3({8.25f, 4.0f, 0.f}, {0.f, 0.f, 0.382683f, 0.92388f});
    f.avgTagDistM = tags > 0 ? 3.5 : 0.0;
    f.avgTagAreaPercent = tags > 0 ? 0.125 : 0.0;
    f.tagSpanM = tags > 1 ? 1.625 : 0.0;
    return f;
}

CameraConfig sampleCameraConfig(int n) {
    CameraConfig c;
    c.id = "camera-" + std::to_string(n);
    c.label = "Front \xC3\xA9 cam";
    c.name = n == 0 ? "" : "cam" + std::to_string(n);
    c.kind = n % 2 == 0 ? CameraKind::Limelight : CameraKind::PhotonVision;
    c.attachment.parentMechanismId = n == 0 ? std::string(kChassisParent) : "turret";
    c.attachment.parentLink = n == 0 ? std::string(kRootLink) : "pitch_link";
    c.local = Pose3({0.25f, -0.125f, 0.5f}, {0.f, 0.f, 0.382683f, 0.92388f});
    c.hFovDeg = 62.5;
    c.vFovDeg = 48.75;
    c.maxRangeM = 7.5;
    c.maxIncidenceDeg = 70.0;
    c.fps = 45.0;
    c.latencyMs = 33.5;
    c.widthPx = 960 + n;
    c.heightPx = 720 + n;
    c.positionNoiseM = 0.015;
    c.yawNoiseDeg = 0.75;
    return c;
}

VisionSetup sampleVisionSetup() {
    VisionSetup v;
    for (int i = 0; i < 3; ++i) v.cameras.push_back(CameraSetup{sampleCameraConfig(i)});
    v.photonVersion = "v2026.3.4";
    v.layoutName = "2026 Rebuilt (welded)";
    return v;
}

std::vector<CameraFrame> sampleVision() {
    return {sampleCameraFrame(0, 3, true), sampleCameraFrame(1, 0, false), sampleCameraFrame(2, 1, true)};
}

SensorFrame sensorWithVision() {
    SensorFrame f = sampleSensor();
    f.vision = sampleVision();
    return f;
}

std::vector<Message> allSamples() {
    Log log;
    log.simTimeUs = 42;
    log.level = LogLevel::Warn;
    log.source = "bridge.phoenix6";
    log.message = "unmapped device";
    LoopbackSet lb;
    lb.loopback = {DeviceId{1}, DeviceId{9}, DeviceId{4000000000u}};
    Goodbye bye;
    bye.reason = "done";
    return {sampleHello(),   sampleHelloAck(), sampleAnnounce(), lb,  sampleSensor(), sampleActuator(),
            sampleDs(),      sampleControl(),  sampleControlAck(), log, bye, sampleVisionSetup(),
            sensorWithVision()};
}

bool sameSamples(const std::vector<SignalSample>& a, const std::vector<SignalSample>& b) { return a == b; }

}  // namespace

TEST_CASE("codec: every message type is represented in the sample set") {
    std::vector<bool> seen(std::variant_size_v<Message> + 1, false);
    for (const Message& m : allSamples()) seen[m.index() + 1] = true;
    for (std::size_t i = 1; i < seen.size(); ++i) CHECK_MESSAGE(seen[i], "message type " << i << " has no sample");
}

TEST_CASE("codec: round trip is exact for every message (re-encoding is byte-identical)") {
    for (const Message& m : allSamples()) {
        const std::vector<std::byte> a = encodeOk(m);
        const Message back = decodeOk(a);
        CHECK(back.index() == m.index());
        CHECK(typeOf(back) == typeOf(m));
        CHECK(encodeOk(back) == a);
    }
}

TEST_CASE("codec: Hello fields survive") {
    const auto h = std::get<Hello>(decodeOk(encodeOk(sampleHello())));
    const Hello e = sampleHello();
    CHECK(h.protocolVersion == e.protocolVersion);
    CHECK(h.language == RobotLanguage::Java);
    CHECK(h.wpilibYear == 2026);
    CHECK(h.wpilibVersion == "2026.2.1");
    CHECK(h.extensionVersion == "0.1.0");
    CHECK(h.pid == 1234567890123LL);
    CHECK(h.programName == e.programName);
    CHECK(h.halPortCounts == e.halPortCounts);
    CHECK(h.proposedWatchdogMs == 750);
    CHECK(h.supportsArmed);
}

TEST_CASE("codec: a v2 Hello without the Armed tail defaults to unsupported") {
    Hello hello = sampleHello();
    hello.protocolVersion = 2;
    auto bytes = encodeOk(hello);
    bytes.pop_back();  // v2 ended at proposedWatchdogMs
    const auto decoded = std::get<Hello>(decodeOk(bytes));
    CHECK(decoded.protocolVersion == 2);
    CHECK(decoded.proposedWatchdogMs == 750);
    CHECK_FALSE(decoded.supportsArmed);
}

TEST_CASE("codec: HelloAck, ActuatorFrame, SensorFrame, SimControl(Ack) fields survive") {
    const auto ack = std::get<HelloAck>(decodeOk(encodeOk(sampleHelloAck())));
    CHECK_FALSE(ack.accepted);
    CHECK(ack.reason == "no");
    CHECK(ack.clockMode == ClockMode::Realtime);
    CHECK(ack.stepUs == 2500);
    CHECK(ack.sessionId == 0x1122334455667788ULL);
    CHECK(ack.discoveryOnly);
    CHECK(ack.enableL3Hooks);
    CHECK(ack.runEpoch == 9);
    CHECK(ack.watchdogTimeoutMs == 4321);
    CHECK(ack.watchdogPolicy == WatchdogPolicy::Terminate);

    const auto act = std::get<ActuatorFrame>(decodeOk(encodeOk(sampleActuator())));
    CHECK(act.tick == 17);
    CHECK(act.simTimeUs == -5);
    CHECK(act.robotLoopCount == 99);
    CHECK(act.full);
    CHECK(sameSamples(act.samples, sampleSignals(7)));
    CHECK(act.sessionId == 3);
    CHECK(act.runEpoch == 4);
    CHECK(act.ackTick == 16);
    CHECK(act.ackedTick() == 16);
    CHECK(act.result == TickResult::RobotStalled);

    const auto sf = std::get<SensorFrame>(decodeOk(encodeOk(sampleSensor())));
    CHECK(sf.tick == 0xFFFFFFFFFFULL);
    CHECK(sf.simTimeUs == 123456789012LL);
    CHECK(sf.batteryVoltageV == 11.875);
    CHECK(sf.batteryCurrentA == 123.5);
    CHECK(sameSamples(sf.samples, sampleSignals(40)));
    CHECK(sf.sessionId == 3);
    CHECK(sf.runEpoch == 4);

    const auto sc = std::get<SimControl>(decodeOk(encodeOk(sampleControl())));
    CHECK(sc.command == SimCommand::SetTimeScale);
    CHECK(sc.stepCount == 3);
    CHECK(sc.timeScale == 0.25);
    CHECK(sc.commandId == 77);
    CHECK(sc.runEpoch == 6);

    const auto sa = std::get<SimControlAck>(decodeOk(encodeOk(sampleControlAck())));
    CHECK(sa.commandId == 77);
    CHECK(sa.command == SimCommand::Reset);
    CHECK(sa.result == ControlResult::StaleEpoch);
    CHECK(sa.tick == 1000);
    CHECK(sa.detail == "Reset must carry a newer runEpoch");
}

TEST_CASE("codec: DeviceAnnounce and DsState fields survive") {
    const auto d = std::get<DeviceAnnounce>(decodeOk(encodeOk(sampleAnnounce())));
    const DeviceAnnounce e = sampleAnnounce();
    CHECK(d.id == e.id);
    CHECK(d.descriptor == e.descriptor);
    CHECK(d.signalSpecs == e.signalSpecs);
    CHECK(d.retired);
    CHECK(d.observedBus == "CANivore1");

    const auto ds = std::get<DsState>(decodeOk(encodeOk(sampleDs())));
    CHECK_FALSE(ds.dsAttached);
    CHECK(ds.fmsAttached);
    CHECK(ds.enabled);
    CHECK(ds.eStop);
    CHECK(ds.mode == RobotMode::Test);
    CHECK(ds.station == AllianceStation::Blue3);
    CHECK(ds.matchTimeS == 134.5);
    CHECK(ds.gameSpecificMessage == "RBG");
    CHECK(ds.joysticks[0].present);
    CHECK(ds.joysticks[0].axes == sampleDs().joysticks[0].axes);
    CHECK(ds.joysticks[0].povs == sampleDs().joysticks[0].povs);
    CHECK(ds.joysticks[0].buttons == 0xA5A5A5A5u);
    CHECK(ds.joysticks[0].buttonCount == 12);
    CHECK_FALSE(ds.joysticks[3].present);
    CHECK(ds.joysticks[5].present);
    CHECK(ds.sessionId == 5);
    CHECK(ds.runEpoch == 6);
}

TEST_CASE("codec: special floating point values and extreme integers round trip") {
    SensorFrame f;
    f.tick = ~std::uint64_t{0};
    f.simTimeUs = std::numeric_limits<std::int64_t>::min();
    f.batteryVoltageV = std::numeric_limits<double>::infinity();
    f.samples = {{DeviceId{0xFFFFFFFFu}, 0xFFFF, -0.0}, {DeviceId{1}, 1, std::numeric_limits<double>::quiet_NaN()}};
    const auto back = std::get<SensorFrame>(decodeOk(encodeOk(f)));
    CHECK(back.tick == ~std::uint64_t{0});
    CHECK(back.simTimeUs == std::numeric_limits<std::int64_t>::min());
    CHECK(std::isinf(back.batteryVoltageV));
    CHECK(back.samples[0].device.value == 0xFFFFFFFFu);
    CHECK(back.samples[0].signal == 0xFFFF);
    CHECK(std::signbit(back.samples[0].value));
    CHECK(std::isnan(back.samples[1].value));
}

TEST_CASE("codec: empty collections and strings round trip") {
    CHECK(std::get<LoopbackSet>(decodeOk(encodeOk(LoopbackSet{}))).loopback.empty());
    CHECK(std::get<Goodbye>(decodeOk(encodeOk(Goodbye{}))).reason.empty());
    CHECK(std::get<SensorFrame>(decodeOk(encodeOk(SensorFrame{}))).samples.empty());
    const auto h = std::get<Hello>(decodeOk(encodeOk(Hello{})));
    CHECK(h.programName.empty());
    CHECK(h.halPortCounts.empty());
}

TEST_CASE("codec: wire layout is little-endian with a 3-byte header") {
    Goodbye g;
    g.reason = "hi";
    const std::vector<std::byte> f = encodeOk(g);
    CHECK(f == bytesOf({kWireFormatVersion, 10, 0, 2, 'h', 'i'}));

    SimControlAck a;
    a.commandId = 0x0102030405060708ULL;
    const std::vector<std::byte> fa = encodeOk(a);
    REQUIRE(fa.size() > 11);
    CHECK(fa[0] == std::byte{kWireFormatVersion});
    CHECK(fa[1] == std::byte{11});  // MessageType::SimControlAck
    CHECK(fa[2] == std::byte{0});
    CHECK(fa[3] == std::byte{0x08});  // least-significant byte first
    CHECK(fa[10] == std::byte{0x01});
}

TEST_CASE("codec: encode appends and never disturbs earlier bytes, failures leave the buffer untouched") {
    std::vector<std::byte> out = bytesOf({0xAA, 0xBB});
    REQUIRE(BinaryMessageCodec().encode(Goodbye{"x"}, out).ok());
    CHECK(out[0] == std::byte{0xAA});
    CHECK(out[1] == std::byte{0xBB});
    CHECK(out.size() == 2 + 3 + 2);

    Goodbye huge;
    huge.reason.assign(kMaxStringBytes + 1, 'x');
    const std::size_t before = out.size();
    const Status s = BinaryMessageCodec().encode(huge, out);
    CHECK_FALSE(s.ok());
    CHECK(s.error().code == ErrorCode::InvalidArgument);
    CHECK(out.size() == before);

    Goodbye big;
    big.reason.assign(2000, 'y');
    const Status small = BinaryMessageCodec(1000).encode(big, out);
    CHECK_FALSE(small.ok());
    CHECK(out.size() == before);
}

TEST_CASE("codec: protocol v1 peers (frames without the appended fields) decode with defaults") {
    // Hello v1: no proposedWatchdogMs.
    std::vector<std::byte> hello = encodeOk(sampleHello());
    hello.resize(hello.size() - 5);
    const auto h = std::get<Hello>(decodeOk(hello));
    CHECK(h.proposedWatchdogMs == 0);
    CHECK_FALSE(h.supportsArmed);
    CHECK(h.wpilibYear == 2026);

    // HelloAck v1: no runEpoch / watchdog fields.
    std::vector<std::byte> ack = encodeOk(sampleHelloAck());
    ack.resize(ack.size() - (8 + 4 + 1));
    const auto a = std::get<HelloAck>(decodeOk(ack));
    CHECK(a.runEpoch == kFirstRunEpoch);
    CHECK(a.watchdogTimeoutMs == kDefaultWatchdogTimeoutMs);
    CHECK(a.watchdogPolicy == WatchdogPolicy::ResumeRealtime);
    CHECK(a.sessionId == 0x1122334455667788ULL);

    // ActuatorFrame v1.
    std::vector<std::byte> act = encodeOk(sampleActuator());
    act.resize(act.size() - (8 + 8 + 8 + 1));
    const auto af = std::get<ActuatorFrame>(decodeOk(act));
    CHECK(af.sessionId == kUnspecified);
    CHECK(af.runEpoch == kUnspecified);
    CHECK(af.ackTick == kNoAckTick);
    CHECK(af.ackedTick() == 17);
    CHECK(af.result == TickResult::Completed);

    // SensorFrame / DsState / SimControl v1.
    std::vector<std::byte> sf = encodeOk(sampleSensor());
    sf.resize(sf.size() - 17);
    CHECK(std::get<SensorFrame>(decodeOk(sf)).runEpoch == kUnspecified);
    std::vector<std::byte> ds = encodeOk(sampleDs());
    ds.resize(ds.size() - 16);
    CHECK(std::get<DsState>(decodeOk(ds)).sessionId == kUnspecified);
    std::vector<std::byte> sc = encodeOk(sampleControl());
    sc.resize(sc.size() - 24);
    const auto c = std::get<SimControl>(decodeOk(sc));
    CHECK(c.commandId == kUnspecified);
    CHECK(c.command == SimCommand::SetTimeScale);

    // DeviceAnnounce v1: no observedBus.
    std::vector<std::byte> da = encodeOk(sampleAnnounce());
    da.resize(da.size() - (1 + 9));
    CHECK(std::get<DeviceAnnounce>(decodeOk(da)).observedBus.empty());
}

TEST_CASE("codec: a newer peer's extra trailing bytes (appended fields) are ignored") {
    for (const Message& m : allSamples()) {
        std::vector<std::byte> f = encodeOk(m);
        const std::vector<std::byte> original = f;
        for (int i = 0; i < 17; ++i) f.push_back(static_cast<std::byte>(0xC0 + i));
        auto r = BinaryMessageCodec().decode(f);
        REQUIRE_MESSAGE(r.ok(), "type " << m.index() + 1);
        CHECK(encodeOk(r.value()) == original);
    }
}

TEST_CASE("codec: a newer peer's extra fields inside length-prefixed records are skipped") {
    // Grow the DeviceDescriptor record of an announce by 3 unknown bytes and patch its u16 length.
    DeviceAnnounce d = sampleAnnounce();
    std::vector<std::byte> f = encodeOk(d);
    // header(3) + deviceId(4) -> record length at [7..8]
    const std::uint16_t len = static_cast<std::uint16_t>(static_cast<std::uint8_t>(f[7]) | (static_cast<std::uint8_t>(f[8]) << 8));
    f.insert(f.begin() + 9 + len, 3, std::byte{0x5A});
    const std::uint16_t grown = static_cast<std::uint16_t>(len + 3);
    f[7] = static_cast<std::byte>(grown & 0xFF);
    f[8] = static_cast<std::byte>(grown >> 8);
    const auto back = std::get<DeviceAnnounce>(decodeOk(f));
    CHECK(back.descriptor == d.descriptor);
    CHECK(back.signalSpecs == d.signalSpecs);
    CHECK(back.observedBus == "CANivore1");
}

TEST_CASE("codec: unknown message types are skippable, not fatal") {
    BinaryMessageCodec codec;
    for (int type : {0, 13, 200, 65535}) {
        const auto r = codec.decode(bytesOf({kWireFormatVersion, type & 0xFF, (type >> 8) & 0xFF, 1, 2, 3}));
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == ErrorCode::Unsupported);
        CHECK(isSkippableDecodeError(r.error()));
    }
    CHECK_FALSE(isSkippableDecodeError(makeError(ErrorCode::ProtocolError, "x")));
}

TEST_CASE("codec: header violations") {
    BinaryMessageCodec codec;
    CHECK(codec.decode({}).error().code == ErrorCode::ProtocolError);
    CHECK(codec.decode(bytesOf({kWireFormatVersion})).error().code == ErrorCode::ProtocolError);
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 10})).error().code == ErrorCode::ProtocolError);
    CHECK(codec.decode(bytesOf({0, 10, 0, 0})).error().code == ErrorCode::VersionMismatch);
    CHECK(codec.decode(bytesOf({kWireFormatVersion + 1, 10, 0, 0})).error().code == ErrorCode::VersionMismatch);
    std::vector<std::byte> big(100, std::byte{0});
    big[0] = std::byte{kWireFormatVersion};
    CHECK(BinaryMessageCodec(50).decode(big).error().code == ErrorCode::ProtocolError);
}

TEST_CASE("codec: hostile lengths and counts are rejected without allocating") {
    BinaryMessageCodec codec;
    // Goodbye with a 4 GiB string length.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 10, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F})).error().code == ErrorCode::ProtocolError);
    // Goodbye whose string claims more bytes than the frame holds.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 10, 0, 5, 'a', 'b'})).error().code == ErrorCode::ProtocolError);
    // LoopbackSet claiming 2^40 ids.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 4, 0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20})).error().code == ErrorCode::ProtocolError);
    // Count that does not fit the remaining bytes (3 ids announced, 1 present).
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 4, 0, 3, 1, 0, 0, 0})).error().code == ErrorCode::ProtocolError);
    // Varint with 11 continuation bytes / overflowing 10th byte.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 4, 0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x01})).error().code ==
          ErrorCode::ProtocolError);
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 4, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F})).error().code ==
          ErrorCode::ProtocolError);
    // Truncated varint.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 4, 0, 0x80})).error().code == ErrorCode::ProtocolError);
}

TEST_CASE("codec: record length beyond the frame and out-of-range enums are rejected") {
    BinaryMessageCodec codec;
    std::vector<std::byte> f = encodeOk(sampleAnnounce());
    f[7] = std::byte{0xFF};
    f[8] = std::byte{0xFF};
    CHECK(codec.decode(f).error().code == ErrorCode::ProtocolError);

    DsState ds;
    std::vector<std::byte> d = encodeOk(ds);
    d[3 + 4] = std::byte{9};  // RobotMode
    CHECK(codec.decode(d).error().code == ErrorCode::ProtocolError);
    d = encodeOk(ds);
    d[3 + 5] = std::byte{6};  // AllianceStation
    CHECK(codec.decode(d).error().code == ErrorCode::ProtocolError);

    Log log;
    std::vector<std::byte> l = encodeOk(log);
    l[3 + 8] = std::byte{5};  // LogLevel
    CHECK(codec.decode(l).error().code == ErrorCode::ProtocolError);

    HelloAck ack;
    std::vector<std::byte> a = encodeOk(ack);
    a[3 + 1 + 1 + 4] = std::byte{2};  // ClockMode (accepted, empty reason, protocolVersion, then clockMode)
    CHECK(codec.decode(a).error().code == ErrorCode::ProtocolError);

    // Lenient enums (peer may be newer): passed through for the consumer's default path.
    SimControl c;
    std::vector<std::byte> cf = encodeOk(c);
    cf[3] = std::byte{200};
    const auto sc = std::get<SimControl>(decodeOk(cf));
    CHECK(static_cast<int>(sc.command) == 200);
}

TEST_CASE("codec: every truncation of every message fails cleanly or yields a valid message of the same type") {
    BinaryMessageCodec codec;
    for (const Message& m : allSamples()) {
        const std::vector<std::byte> full = encodeOk(m);
        for (std::size_t len = 0; len < full.size(); ++len) {
            const auto r = codec.decode(std::span<const std::byte>(full.data(), len));
            if (r.ok()) CHECK(r.value().index() == m.index());  // only possible at an appended-field boundary
        }
        CHECK(codec.decode(full).ok());
    }
}

TEST_CASE("codec: truncation is rejected for messages without optional tails") {
    BinaryMessageCodec codec;
    for (const Message& m : {Message{sampleControlAck()}, Message{Log{}}, Message{LoopbackSet{{DeviceId{1}, DeviceId{2}}}}}) {
        const std::vector<std::byte> full = encodeOk(m);
        for (std::size_t len = 0; len < full.size(); ++len)
            CHECK_FALSE(codec.decode(std::span<const std::byte>(full.data(), len)).ok());
    }
}

TEST_CASE("codec: fuzz - bit flips and random corruption never crash and stay consistent") {
    BinaryMessageCodec codec;
    std::mt19937_64 rng(0xC0FFEE);
    const std::vector<Message> samples = allSamples();
    std::size_t decodedOk = 0;
    for (int iteration = 0; iteration < 6000; ++iteration) {
        std::vector<std::byte> f = encodeOk(samples[rng() % samples.size()]);
        const int edits = 1 + static_cast<int>(rng() % 4);
        for (int e = 0; e < edits && !f.empty(); ++e) {
            switch (rng() % 4) {
                case 0: f[rng() % f.size()] ^= static_cast<std::byte>(1u << (rng() % 8)); break;
                case 1: f[rng() % f.size()] = static_cast<std::byte>(rng() & 0xFF); break;
                case 2: f.resize(rng() % (f.size() + 1)); break;
                default: f.insert(f.begin() + static_cast<std::ptrdiff_t>(rng() % (f.size() + 1)), static_cast<std::byte>(rng() & 0xFF)); break;
            }
        }
        const auto r = codec.decode(f);
        if (r.ok()) {
            ++decodedOk;
            std::vector<std::byte> again;
            REQUIRE(codec.encode(r.value(), again).ok());  // whatever decoded must be encodable and decode again
            CHECK(codec.decode(again).ok());
        }
    }
    CHECK(decodedOk > 0);
}

TEST_CASE("codec: fuzz - random garbage never crashes") {
    BinaryMessageCodec codec;
    std::mt19937_64 rng(12345);
    for (int iteration = 0; iteration < 20000; ++iteration) {
        std::vector<std::byte> f(rng() % 80);
        for (auto& b : f) b = static_cast<std::byte>(rng() & 0xFF);
        if (f.size() >= 3 && (rng() & 1)) {  // plausible header so the body parsers are exercised
            f[0] = std::byte{kWireFormatVersion};
            f[1] = static_cast<std::byte>(1 + rng() % 11);
            f[2] = std::byte{0};
        }
        (void)codec.decode(f);
    }
    CHECK(true);  // reaching here without crashing is the assertion
}

TEST_CASE("codec: a large SensorFrame round trips") {
    SensorFrame f;
    f.samples = sampleSignals(50000);
    f.tick = 1;
    const auto back = std::get<SensorFrame>(decodeOk(encodeOk(f)));
    CHECK(back.samples.size() == 50000);
    CHECK(sameSamples(back.samples, f.samples));
}

TEST_CASE("codec: sizes are compact") {
    SensorFrame f;
    f.samples = sampleSignals(10);
    CHECK(encodeOk(f).size() <= 3 + 8 + 8 + 4 + 8 + 8 + 1 + 1 + 10 * 14 + 16 + 1);
    ActuatorFrame a;
    CHECK(encodeOk(a).size() < 64);
}

TEST_CASE("codec: VisionSetup round trips with every camera field") {
    const VisionSetup in = sampleVisionSetup();
    const auto bytes = encodeOk(in);
    CHECK(bytes[1] == std::byte{12});  // MessageType::VisionSetup
    CHECK(bytes[2] == std::byte{0});
    const Message decoded = decodeOk(bytes);
    CHECK(typeOf(decoded) == MessageType::VisionSetup);
    const auto& out = std::get<VisionSetup>(decoded);
    REQUIRE(out.cameras.size() == in.cameras.size());
    for (std::size_t i = 0; i < in.cameras.size(); ++i) CHECK(out.cameras[i].config == in.cameras[i].config);
    CHECK(out.cameras[1].config.kind == CameraKind::PhotonVision);
    CHECK(out.cameras[1].config.attachment.parentMechanismId == "turret");
    CHECK(out.cameras[1].config.attachment.parentLink == "pitch_link");
    CHECK(out.cameras[2].config.local == in.cameras[2].config.local);
    CHECK(out.photonVersion == "v2026.3.4");
    CHECK(out.layoutName == "2026 Rebuilt (welded)");

    const auto empty = std::get<VisionSetup>(decodeOk(encodeOk(VisionSetup{})));
    CHECK(empty.cameras.empty());
    CHECK(empty.photonVersion.empty());
    CHECK(empty.layoutName.empty());
}

TEST_CASE("codec: VisionSetup without the trailing strings decodes with empty strings") {
    VisionSetup in = sampleVisionSetup();
    in.photonVersion.clear();
    in.layoutName.clear();
    std::vector<std::byte> bytes = encodeOk(in);
    bytes.resize(bytes.size() - 2);  // both empty strings are one length byte each
    const auto out = std::get<VisionSetup>(decodeOk(bytes));
    CHECK(out.cameras.size() == 3);
    CHECK(out.photonVersion.empty());
    CHECK(out.layoutName.empty());
}

TEST_CASE("codec: SensorFrame carries camera frames with tags, empty frames and optional robot poses") {
    const SensorFrame in = sensorWithVision();
    const auto out = std::get<SensorFrame>(decodeOk(encodeOk(in)));
    CHECK(out.tick == in.tick);
    CHECK(out.runEpoch == in.runEpoch);
    CHECK(sameSamples(out.samples, in.samples));
    REQUIRE(out.vision.size() == 3);
    for (std::size_t i = 0; i < in.vision.size(); ++i) {
        const CameraFrame& a = in.vision[i];
        const CameraFrame& b = out.vision[i];
        CHECK(b.cameraIndex == a.cameraIndex);
        CHECK(b.sequence == a.sequence);
        CHECK(b.captureTimeUs == a.captureTimeUs);
        CHECK(b.publishTimeUs == a.publishTimeUs);
        CHECK(b.robotToCamera == a.robotToCamera);
        CHECK(b.hasRobotPose == a.hasRobotPose);
        CHECK(b.robotPoseField == a.robotPoseField);
        CHECK(b.avgTagDistM == a.avgTagDistM);
        CHECK(b.avgTagAreaPercent == a.avgTagAreaPercent);
        CHECK(b.tagSpanM == a.tagSpanM);
        REQUIRE(b.tags.size() == a.tags.size());
        for (std::size_t t = 0; t < a.tags.size(); ++t) {
            CHECK(b.tags[t].id == a.tags[t].id);
            CHECK(b.tags[t].cameraToTag == a.tags[t].cameraToTag);
            CHECK(b.tags[t].yawDeg == a.tags[t].yawDeg);
            CHECK(b.tags[t].pitchDeg == a.tags[t].pitchDeg);
            CHECK(b.tags[t].areaPercent == a.tags[t].areaPercent);
            CHECK(b.tags[t].ambiguity == a.tags[t].ambiguity);
            CHECK(b.tags[t].cornersPx == a.tags[t].cornersPx);
            CHECK(b.tags[t].distanceM == a.tags[t].distanceM);
        }
    }
    CHECK(out.vision[0].tags.size() == 3);
    CHECK(out.vision[1].tags.empty());
    CHECK_FALSE(out.vision[1].hasRobotPose);
    CHECK(out.vision[2].hasRobotPose);
}

TEST_CASE("codec: a SensorFrame without the vision tail decodes with no camera frames") {
    std::vector<std::byte> bytes = encodeOk(sampleSensor());
    bytes.resize(bytes.size() - 1);  // the empty vision count
    const auto out = std::get<SensorFrame>(decodeOk(bytes));
    CHECK(out.vision.empty());
    CHECK(out.sessionId == 3);
    CHECK(out.runEpoch == 4);
    CHECK(out.samples.size() == 40);

    // A frame that also lacks the identity tail still decodes.
    bytes.resize(bytes.size() - 16);
    CHECK(std::get<SensorFrame>(decodeOk(bytes)).vision.empty());
}

TEST_CASE("codec: truncated vision data in a SensorFrame is rejected") {
    BinaryMessageCodec codec;
    const std::size_t withoutVision = encodeOk(sampleSensor()).size() - 1;
    const std::vector<std::byte> full = encodeOk(sensorWithVision());
    REQUIRE(full.size() > withoutVision + 1);
    for (std::size_t len = withoutVision + 1; len < full.size(); ++len) {
        const auto r = codec.decode(std::span<const std::byte>(full.data(), len));
        CHECK_FALSE(r.ok());
        if (!r.ok()) CHECK(r.error().code == ErrorCode::ProtocolError);
    }
    CHECK(codec.decode(std::span<const std::byte>(full.data(), withoutVision)).ok());
}

TEST_CASE("codec: truncated VisionSetup cameras are rejected") {
    BinaryMessageCodec codec;
    const std::vector<std::byte> full = encodeOk(sampleVisionSetup());
    const std::size_t tailBytes = 1 + 9 + 1 + 21;  // photonVersion + layoutName with their length prefixes
    for (std::size_t len = 0; len < full.size() - tailBytes; ++len)
        CHECK_FALSE(codec.decode(std::span<const std::byte>(full.data(), len)).ok());
}

TEST_CASE("codec: hostile vision counts and enums are rejected") {
    BinaryMessageCodec codec;
    // VisionSetup claiming 2^40 cameras / more cameras than bytes.
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 12, 0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x20})).error().code ==
          ErrorCode::ProtocolError);
    CHECK(codec.decode(bytesOf({kWireFormatVersion, 12, 0, 200, 1, 2, 3})).error().code == ErrorCode::ProtocolError);

    // SensorFrame claiming a huge camera-frame count with almost no bytes behind it.
    std::vector<std::byte> sf = encodeOk(sampleSensor());
    sf.pop_back();
    for (int b : {0xFF, 0xFF, 0x7F, 1, 2, 3}) sf.push_back(static_cast<std::byte>(b));
    CHECK(codec.decode(sf).error().code == ErrorCode::ProtocolError);

    // A camera frame claiming more tags than the cap, and more than its bytes hold.
    SensorFrame one;
    one.vision.push_back(sampleCameraFrame(0, 0, false));
    const std::vector<std::byte> base = encodeOk(one);
    const std::size_t tagCountAt = base.size() - 1 - 28 - 3 * 8 - 1;
    REQUIRE(base[tagCountAt] == std::byte{0});
    for (std::initializer_list<int> count : {std::initializer_list<int>{0x88, 0x27}, std::initializer_list<int>{0xE8, 0x07}}) {
        std::vector<std::byte> f(base.begin(), base.begin() + static_cast<std::ptrdiff_t>(tagCountAt));
        for (int b : count) f.push_back(static_cast<std::byte>(b));
        f.insert(f.end(), base.begin() + static_cast<std::ptrdiff_t>(tagCountAt) + 1, base.end());
        CHECK(codec.decode(f).error().code == ErrorCode::ProtocolError);
    }

    // Out-of-range CameraKind: the byte after the three strings of the first camera.
    VisionSetup v;
    v.cameras.push_back(CameraSetup{sampleCameraConfig(0)});
    std::vector<std::byte> setup = encodeOk(v);
    const std::size_t kindAt = 3 + 1 + (1 + 8) + (1 + 12) + 1;
    REQUIRE(setup[kindAt] == std::byte{0});
    setup[kindAt] = std::byte{2};
    CHECK(codec.decode(setup).error().code == ErrorCode::ProtocolError);
}

TEST_CASE("codec: vision frames beyond the wire caps fail to encode and leave the buffer untouched") {
    std::vector<std::byte> out = {std::byte{7}};
    SensorFrame f;
    f.vision.push_back(sampleCameraFrame(0, 0, false));
    f.vision.back().tags.resize(1025);
    CHECK_FALSE(BinaryMessageCodec().encode(f, out).ok());
    CHECK(out.size() == 1);

    VisionSetup v;
    v.cameras.resize(257);
    CHECK_FALSE(BinaryMessageCodec().encode(v, out).ok());
    CHECK(out.size() == 1);
}

TEST_CASE("codec: a large SensorFrame with many cameras and tags round trips") {
    SensorFrame f = sampleSensor();
    for (std::uint32_t c = 0; c < 8; ++c) f.vision.push_back(sampleCameraFrame(c, 1000, c % 2 == 0));
    const std::vector<std::byte> bytes = encodeOk(f);
    CHECK(bytes.size() > 1000000);
    const auto back = std::get<SensorFrame>(decodeOk(bytes));
    REQUIRE(back.vision.size() == 8);
    CHECK(back.vision[7].tags.size() == 1000);
    CHECK(back.vision[7].tags[999].id == 1000);
    CHECK(back.vision[6].hasRobotPose);
    CHECK(encodeOk(back) == bytes);
}

TEST_CASE("codec: has a stable name") { CHECK(BinaryMessageCodec().name() == "wpiemu-binary-1"); }
