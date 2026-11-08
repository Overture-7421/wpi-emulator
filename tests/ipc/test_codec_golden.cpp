#include <doctest/doctest.h>

#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;

// One representative instance of every message type, encoded with BinaryMessageCodec and compared against hex captured
// from the codec as it was before Writer/Reader moved into the SDK. The wire format is a compatibility contract with
// robots built against older extensions, so these bytes must never change.
namespace {

struct Golden {
    const char* name;
    Message message;
    const char* hex;
};

std::vector<std::byte> fromHex(std::string_view text) {
    std::vector<std::byte> out;
    int high = -1;
    for (const char c : text) {
        const int v = c <= '9' ? c - '0' : c - 'a' + 10;
        if (high < 0) {
            high = v;
        } else {
            out.push_back(static_cast<std::byte>(high * 16 + v));
            high = -1;
        }
    }
    return out;
}

std::string toHex(const std::vector<std::byte>& bytes) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string s;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        const auto b = static_cast<unsigned>(bytes[i]);
        s += digits[b >> 4];
        s += digits[b & 15];
    }
    return s;
}

Message helloMessage() {
    Hello h;
    h.language = RobotLanguage::Cpp;
    h.wpilibYear = 2026;
    h.wpilibVersion = "2026.2.1";
    h.extensionVersion = "0.1.0";
    h.pid = 1234567890123LL;
    h.programName = "Robot \xC3\xA9";
    h.halPortCounts = {{"pwm", 20}, {"can", -1}};
    h.proposedWatchdogMs = 750;
    h.supportsArmed = true;
    return h;
}

Message helloAckMessage() {
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

Message announceMessage() {
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
                     {"fwdLimit", SignalDirection::Bidirectional, SignalType::Bool, ""}};
    d.retired = true;
    d.observedBus = "CANivore1";
    return d;
}

Message loopbackMessage() {
    LoopbackSet lb;
    lb.loopback = {DeviceId{1}, DeviceId{9}, DeviceId{4000000000u}};
    return lb;
}

std::vector<SignalSample> signals(std::size_t n) {
    std::vector<SignalSample> s;
    for (std::size_t i = 0; i < n; ++i)
        s.push_back({DeviceId{static_cast<std::uint32_t>(i + 1)}, static_cast<std::uint16_t>(i),
                     0.125 * static_cast<double>(i) - 3.0});
    return s;
}

CameraFrame cameraFrame() {
    CameraFrame f;
    f.cameraIndex = 1;
    f.sequence = 0x100000001ULL;
    f.captureTimeUs = 1000000;
    f.publishTimeUs = 1040000;
    f.robotToCamera = Pose3({0.3f, -0.1f, 0.6f}, {0.f, 0.258819f, 0.f, 0.965926f});
    TagDetection t;
    t.id = 3;
    t.cameraToTag = Pose3({2.5f, -0.25f, 0.5f}, {0.f, 0.f, 0.70710678f, 0.70710678f});
    t.yawDeg = -12.5;
    t.pitchDeg = 3.25;
    t.areaPercent = 0.0625;
    for (std::size_t i = 0; i < t.cornersPx.size(); ++i) t.cornersPx[i] = 100.5 + 10.0 * static_cast<double>(i);
    t.distanceM = 2.75;
    f.tags.push_back(t);
    f.hasRobotPose = true;
    f.robotPoseField = Pose3({8.25f, 4.0f, 0.f}, {0.f, 0.f, 0.382683f, 0.92388f});
    f.avgTagDistM = 3.5;
    f.avgTagAreaPercent = 0.125;
    f.tagSpanM = 1.625;
    return f;
}

Message sensorMessage() {
    SensorFrame f;
    f.tick = 0xFFFFFFFFFFULL;
    f.simTimeUs = 123456789012LL;
    f.stepUs = 5000;
    f.batteryVoltageV = 11.875;
    f.batteryCurrentA = 123.5;
    f.full = true;
    f.samples = signals(3);
    f.vision.push_back(cameraFrame());
    f.sessionId = 3;
    f.runEpoch = 4;
    return f;
}

Message actuatorMessage() {
    ActuatorFrame f;
    f.tick = 17;
    f.simTimeUs = -5;
    f.robotLoopCount = 99;
    f.full = true;
    f.samples = signals(2);
    f.sessionId = 3;
    f.runEpoch = 4;
    f.ackTick = 16;
    f.result = TickResult::RobotStalled;
    return f;
}

Message dsMessage() {
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
    d.joysticks[0].axes = {0.0f, -1.0f, 0.5f};
    d.joysticks[0].povs = {-1, 0, 315};
    d.joysticks[0].buttons = 0xA5A5A5A5u;
    d.joysticks[0].buttonCount = 12;
    d.sessionId = 5;
    d.runEpoch = 6;
    return d;
}

Message controlMessage() {
    SimControl c;
    c.command = SimCommand::SetTimeScale;
    c.stepCount = 3;
    c.timeScale = 0.25;
    c.commandId = 77;
    c.sessionId = 5;
    c.runEpoch = 6;
    return c;
}

Message controlAckMessage() {
    SimControlAck a;
    a.commandId = 77;
    a.command = SimCommand::Reset;
    a.result = ControlResult::StaleEpoch;
    a.sessionId = 5;
    a.runEpoch = 6;
    a.tick = 1000;
    a.detail = "stale";
    return a;
}

Message logMessage() {
    Log log;
    log.simTimeUs = 42;
    log.level = LogLevel::Warn;
    log.source = "bridge.phoenix6";
    log.message = "unmapped device";
    return log;
}

Message goodbyeMessage() {
    Goodbye g;
    g.reason = "done";
    return g;
}

Message visionSetupMessage() {
    CameraConfig c;
    c.id = "camera-0";
    c.label = "Front \xC3\xA9 cam";
    c.name = "cam0";
    c.kind = CameraKind::PhotonVision;
    c.attachment.parentMechanismId = "turret";
    c.attachment.parentLink = "pitch_link";
    c.local = Pose3({0.25f, -0.125f, 0.5f}, {0.f, 0.f, 0.382683f, 0.92388f});
    c.hFovDeg = 62.5;
    c.vFovDeg = 48.75;
    c.maxRangeM = 7.5;
    c.maxIncidenceDeg = 70.0;
    c.fps = 45.0;
    c.latencyMs = 33.5;
    c.widthPx = 960;
    c.heightPx = 720;
    c.positionNoiseM = 0.015;
    c.yawNoiseDeg = 0.75;
    VisionSetup v;
    v.cameras.push_back(CameraSetup{c});
    v.photonVersion = "v2026.3.4";
    v.layoutName = "welded";
    v.fieldLengthM = 16.541;
    v.fieldWidthM = 8.069;
    return v;
}

std::vector<Golden> goldens() {
    return {
        {"Hello", helloMessage(),
         "0101000100000002ea07000008323032362e322e3105302e312e30cb04fb711f"
         "01000008526f626f7420c3a9020370776d140000000363616effffffffee0200"
         "0001"},
        {"HelloAck", helloAckMessage(),
         "01020000026e6f0100000001c409000088776655443322110101090000000000"
         "0000e110000001"},
        {"DeviceAnnounce", announceMessage(),
         "0103002a0000004a000870686f656e6978360774616c6f6e6678046374726507"
         "0000000863616e69766f7265ffffffff030000001343414e4d6f746f723a5461"
         "6c6f6e20465820370a4c6566742064726976650211000c6d6f746f72566f6c74"
         "616765000001560c00086677644c696d6974020100010943414e69766f726531"},
        {"LoopbackSet", loopbackMessage(), "01040003010000000900000000286bee"},
        {"SensorFrame", sensorMessage(),
         "010500ffffffffff000000141a99be1c000000881300000000000000c0274000"
         "00000000e05e40010301000000000000000000000008c0020000000100000000"
         "00000007c003000000020000000000000006c003000000000000000400000000"
         "0000000101000000010000000100000040420f000000000080de0f0000000000"
         "9a99993ecdccccbd9a99193f00000000ed83843e00000000ed46773f01030000"
         "0000002040000080be0000003f0000000000000000f304353ff304353f000000"
         "00000029c00000000000000a40000000000000b03f0000000000000000000000"
         "00002059400000000000a05b400000000000205e400000000000506040000000"
         "00009061400000000000d0624000000000001064400000000000506540000000"
         "000000064001000004410000804000000000000000000000000007efc33e6683"
         "6c3f0000000000000c40000000000000c03f000000000000fa3f"},
        {"ActuatorFrame", actuatorMessage(),
         "0106001100000000000000fbffffffffffffff63000000000000000102010000"
         "00000000000000000008c002000000010000000000000007c003000000000000"
         "000400000000000000100000000000000001"},
        {"DsState", dsMessage(),
         "0107000001010102050000000000d06040035242471a00010300000000000080"
         "bf0000003f03ffff00003b01a5a5a5a50c080000000000000000000800000000"
         "0000000000080000000000000000000800000000000000000008000000000000"
         "00000005000000000000000600000000000000"},
        {"SimControl", controlMessage(),
         "0108000503000000000000000000d03f4d000000000000000500000000000000"
         "0600000000000000"},
        {"Log", logMessage(),
         "0109002a00000000000000030f6272696467652e70686f656e6978360f756e6d"
         "617070656420646576696365"},
        {"Goodbye", goodbyeMessage(), "010a0004646f6e65"},
        {"SimControlAck", controlAckMessage(),
         "010b004d00000000000000030205000000000000000600000000000000e80300"
         "0000000000057374616c65"},
        {"VisionSetup", visionSetupMessage(),
         "010c00010863616d6572612d300c46726f6e7420c3a92063616d0463616d3001"
         "067475727265740a70697463685f6c696e6b0000803e000000be0000003f0000"
         "00000000000007efc33e66836c3f0000000000404f4000000000006048400000"
         "000000001e40000000000080514000000000008046400000000000c04040c003"
         "0000d0020000b81e85eb51b88e3f000000000000e83f0976323032362e332e34"
         "0677656c646564d122dbf97e8a304017d9cef753232040"},
    };
}

}  // namespace

TEST_CASE("codec golden: one instance of every message type is covered") {
    std::set<std::size_t> seen;
    for (const Golden& g : goldens()) seen.insert(g.message.index());
    CHECK(seen.size() == std::variant_size_v<Message>);
}

TEST_CASE("codec golden: every message type encodes to its pinned bytes") {
    const BinaryMessageCodec codec;
    for (const Golden& g : goldens()) {
        CAPTURE(g.name);
        std::vector<std::byte> out;
        REQUIRE(codec.encode(g.message, out).ok());
        CHECK(toHex(out) == toHex(fromHex(g.hex)));
    }
}

TEST_CASE("codec golden: the pinned bytes decode and re-encode to themselves") {
    const BinaryMessageCodec codec;
    for (const Golden& g : goldens()) {
        CAPTURE(g.name);
        const std::vector<std::byte> frame = fromHex(g.hex);
        Result<Message> decoded = codec.decode(frame);
        REQUIRE(decoded.ok());
        CHECK(decoded.value().index() == g.message.index());
        std::vector<std::byte> again;
        REQUIRE(codec.encode(decoded.value(), again).ok());
        CHECK(again == frame);
    }
}
