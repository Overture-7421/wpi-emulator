#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/sdk/vision/photon_profiles.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;

namespace {

std::vector<std::byte> encodeOk(const Message& m) {
    std::vector<std::byte> out;
    const Status s = BinaryMessageCodec().encode(m, out);
    REQUIRE_MESSAGE(s.ok(), (s.ok() ? std::string() : s.error().message));
    return out;
}

Message decodeOk(std::span<const std::byte> frame) {
    auto r = BinaryMessageCodec().decode(frame);
    REQUIRE_MESSAGE(r.ok(), (r.ok() ? std::string() : r.error().message));
    return std::move(r).value();
}

ErrorCode decodeError(std::span<const std::byte> frame) {
    const auto r = BinaryMessageCodec().decode(frame);
    REQUIRE_FALSE(r.ok());
    return r.error().code;
}

Pose3 pose(float s) {
    return Pose3{Vec3{s, s + 0.5f, s - 1.25f}, Quat{0.1f * s, -0.2f * s, 0.3f, 0.9f}};
}

FiducialObservation observation(int id) {
    const double d = static_cast<double>(id);
    FiducialObservation o;
    o.id = id;
    for (std::size_t i = 0; i < o.cornersPx.size(); ++i)
        o.cornersPx[i] = {100.0 * d + 10.0 * static_cast<double>(i) + 0.25,
                          200.0 + static_cast<double>(i) - 0.5 * d};
    o.centerPx = {321.5, 123.25};
    o.yawRad = 0.1 * d;
    o.pitchRad = -0.05 * d;
    o.skewRad = 0.01;
    o.areaPct = 1.5 + d;
    o.distanceM = 2.0 + 0.5 * d;
    o.fieldToTag = pose(static_cast<float>(id));
    o.cameraToTagBest = pose(static_cast<float>(id) + 0.5f);
    o.cameraToTagAlt = pose(static_cast<float>(id) + 0.25f);
    o.ambiguity = 0.125;
    o.reprojErrBestPx = 0.25;
    o.reprojErrAltPx = 2.0;
    return o;
}

VisionFrame visionFrame(const std::string& camera, CameraVendor vendor, std::uint64_t sequence,
                        int tags) {
    VisionFrame f;
    f.cameraId = camera;
    f.vendor = vendor;
    f.ntName = vendor == CameraVendor::Limelight ? "limelight-" + camera : camera;
    f.sequence = sequence;
    f.captureTimeUs = 1'000'000 + static_cast<std::int64_t>(sequence) * 33'333;
    f.latencyUs = 25'000;
    f.captureLatencyUs = 10'000;
    f.robotToCamera = pose(0.3f);
    for (int i = 0; i < tags; ++i) f.observations.push_back(observation(i + 1));
    if (tags >= 2) {
        f.multiTag.valid = true;
        f.multiTag.fieldToCamera = pose(7.f);
        f.multiTag.reprojErrPx = 0.75;
        for (int i = 0; i < tags; ++i) f.multiTag.tagIds.push_back(i + 1);
    }
    return f;
}

CameraSetup cameraSetup(const std::string& id, CameraVendor vendor) {
    CameraSetup c;
    c.cameraId = id;
    c.vendor = vendor;
    c.ntName = vendor == CameraVendor::Limelight ? "limelight-" + id : id;
    c.widthPx = 1280;
    c.heightPx = 800;
    c.fxPx = 640.5;
    c.fyPx = 641.5;
    c.cxPx = 639.25;
    c.cyPx = 400.75;
    c.distortion = {0.1, -0.2, 0.001, -0.002, 0.03};
    c.frameRateHz = 45.5;
    c.latencyUs = 30'000;
    c.defaultAssumedRobotToCamera = pose(1.f);
    c.photonMessageUuid = kPhotonMessageUuidV2025_3;
    return c;
}

// Every optional present.
CameraRobotState fullState() {
    CameraRobotState s;
    s.ntName = "limelight-left";
    s.vendor = CameraVendor::Limelight;
    s.cameraId = "left";
    s.present = true;
    s.pipelineIndex = -1;
    s.ledMode = std::numeric_limits<std::int32_t>::max();
    s.imuMode = 4;
    s.driverMode = false;  // present and false, not absent
    s.throttle = 0.0;
    s.fpsLimit = 90.5;
    s.robotOrientationDeg = std::array<double, 6>{10.0, 0.5, -1.0, 0.0, 2.0, -0.25};
    s.cameraPoseOverride = std::array<double, 6>{0.3, -0.2, 0.5, 1.0, 2.0, 3.0};
    s.publishing = true;
    s.status = "foreign_publisher";
    s.framesPublished = ~std::uint64_t{0};
    return s;
}

SensorFrame sensorWith(std::vector<VisionFrame> frames) {
    SensorFrame f;
    f.tick = 42;
    f.simTimeUs = 210'000;
    f.stepUs = 5000;
    f.batteryVoltageV = 12.1;
    f.batteryCurrentA = 3.5;
    f.full = true;
    f.samples = {{DeviceId{1}, 0, 1.5}, {DeviceId{2}, 1, -2.5}};
    f.sessionId = 9;
    f.runEpoch = 3;
    f.vision = std::move(frames);
    return f;
}

ActuatorFrame actuatorWith(std::vector<CameraRobotState> states) {
    ActuatorFrame f;
    f.tick = 42;
    f.simTimeUs = 210'000;
    f.robotLoopCount = 10;
    f.full = true;
    f.samples = {{DeviceId{1}, 0, 1.5}};
    f.sessionId = 9;
    f.runEpoch = 3;
    f.ackTick = 41;
    f.result = TickResult::Completed;
    f.vision = std::move(states);
    return f;
}

HelloAck helloAckWith(std::vector<CameraSetup> cameras) {
    HelloAck a;
    a.sessionId = 77;
    a.runEpoch = 5;
    a.watchdogTimeoutMs = 3000;
    a.watchdogPolicy = WatchdogPolicy::Terminate;
    a.vision.cameras = std::move(cameras);
    return a;
}

std::vector<std::byte> varint(std::uint64_t v) {
    std::vector<std::byte> out;
    while (v >= 0x80) {
        out.push_back(static_cast<std::byte>((v & 0x7F) | 0x80));
        v >>= 7;
    }
    out.push_back(static_cast<std::byte>(v));
    return out;
}

// Where the vision tail starts in an encoded message: every message ends with its vision section,
// and the same message without vision ends with a single 0 count byte.
std::size_t visionTailStart(const Message& withoutVision) {
    return encodeOk(withoutVision).size() - 1;
}

}  // namespace

TEST_CASE("vision codec: a SensorFrame with populated vision frames round trips field for field") {
    const SensorFrame in = sensorWith({visionFrame("front", CameraVendor::PhotonVision, 1, 3),
                                       visionFrame("left", CameraVendor::Limelight, 7, 1),
                                       visionFrame("rear", CameraVendor::PhotonVision, ~0ull, 0)});
    const std::vector<std::byte> bytes = encodeOk(in);
    const auto out = std::get<SensorFrame>(decodeOk(bytes));
    CHECK(out.tick == in.tick);
    CHECK(out.simTimeUs == in.simTimeUs);
    CHECK(out.samples == in.samples);
    CHECK(out.sessionId == 9);
    CHECK(out.runEpoch == 3);
    REQUIRE(out.vision.size() == 3);
    CHECK(out.vision == in.vision);
    CHECK(out.vision[0].observations.size() == 3);
    CHECK(out.vision[0].multiTag.valid);
    CHECK(out.vision[0].multiTag.tagIds == std::vector<int>{1, 2, 3});
    CHECK(out.vision[1].vendor == CameraVendor::Limelight);
    CHECK_FALSE(out.vision[1].multiTag.valid);  // one tag: no multi-tag solution
    CHECK(out.vision[2]
              .observations.empty());  // a processed frame that detected nothing is still a frame
    CHECK(encodeOk(out) == bytes);     // re-encoding is byte-identical

    // a newer peer's fields after the vision tail are ignored
    std::vector<std::byte> longer = bytes;
    for (int i = 0; i < 9; ++i) longer.push_back(static_cast<std::byte>(0xC0 + i));
    CHECK(std::get<SensorFrame>(decodeOk(longer)).vision == in.vision);
}

TEST_CASE("vision codec: a HelloAck carries the camera setup next to the handshake fields") {
    const HelloAck in = helloAckWith({cameraSetup("front", CameraVendor::PhotonVision),
                                      cameraSetup("left", CameraVendor::Limelight)});
    const std::vector<std::byte> bytes = encodeOk(in);
    const auto out = std::get<HelloAck>(decodeOk(bytes));
    CHECK(out.sessionId == 77);
    CHECK(out.runEpoch == 5);
    CHECK(out.watchdogTimeoutMs == 3000);
    CHECK(out.watchdogPolicy == WatchdogPolicy::Terminate);
    REQUIRE(out.vision.cameras.size() == 2);
    CHECK(out.vision == in.vision);
    CHECK(out.vision.cameras[1].vendor == CameraVendor::Limelight);
    CHECK(out.vision.cameras[1].distortion[4] == 0.03);
    CHECK(out.vision.cameras[0].photonMessageUuid == kPhotonMessageUuidV2025_3);
    CHECK_FALSE(out.vision.empty());
    CHECK(encodeOk(out) == bytes);
}

TEST_CASE(
    "vision codec: an ActuatorFrame carries camera state with every optional present or absent") {
    CameraRobotState bare;
    bare.ntName = "OV9281_1";
    CameraRobotState partial;
    partial.ntName = "OV9281_2";
    partial.cameraId = "rear";
    partial.present = false;
    partial.driverMode = true;
    partial.cameraPoseOverride = std::array<double, 6>{0, 0, 0, 0, 0, 0};  // present, all zero
    partial.status = "photon_uuid_unsupported";
    const ActuatorFrame in = actuatorWith({fullState(), bare, partial});

    const std::vector<std::byte> bytes = encodeOk(in);
    const auto out = std::get<ActuatorFrame>(decodeOk(bytes));
    CHECK(out.ackTick == 41);
    CHECK(out.result == TickResult::Completed);
    REQUIRE(out.vision.size() == 3);
    CHECK(out.vision == in.vision);
    CHECK(out.vision[0].driverMode.has_value());
    CHECK_FALSE(*out.vision[0].driverMode);
    CHECK(out.vision[0].throttle.has_value());
    CHECK(*out.vision[0].pipelineIndex == -1);
    CHECK(out.vision[0].framesPublished == ~std::uint64_t{0});
    CHECK_FALSE(out.vision[1].pipelineIndex.has_value());
    CHECK_FALSE(out.vision[1].driverMode.has_value());
    CHECK_FALSE(out.vision[1].robotOrientationDeg.has_value());
    CHECK_FALSE(out.vision[1].cameraPoseOverride.has_value());
    CHECK(out.vision[1].present);
    CHECK(out.vision[2].cameraPoseOverride.has_value());
    CHECK_FALSE(out.vision[2].robotOrientationDeg.has_value());
    CHECK(*out.vision[2].driverMode);
    CHECK_FALSE(out.vision[2].present);
    CHECK(encodeOk(out) == bytes);
}

TEST_CASE("vision codec: empty vision is one count byte and decodes to the defaults") {
    for (const Message& m :
         {Message{sensorWith({})}, Message{actuatorWith({})}, Message{helloAckWith({})}}) {
        const std::vector<std::byte> bytes = encodeOk(m);
        REQUIRE(bytes.back() == std::byte{0});
        const Message back = decodeOk(bytes);
        CHECK(back.index() == m.index());
        CHECK(encodeOk(back) == bytes);
        if (const auto* s = std::get_if<SensorFrame>(&back)) CHECK(s->vision.empty());
        if (const auto* a = std::get_if<ActuatorFrame>(&back)) CHECK(a->vision.empty());
        if (const auto* h = std::get_if<HelloAck>(&back)) CHECK(h->vision.empty());
    }
}

TEST_CASE(
    "vision codec: a payload cut before the vision tail decodes with no vision and every cut "
    "inside it fails") {
    BinaryMessageCodec codec;
    const auto check = [&](const Message& withVision, const Message& withoutVision) {
        const std::vector<std::byte> full = encodeOk(withVision);
        const std::size_t tailStart = visionTailStart(withoutVision);
        REQUIRE(tailStart < full.size() - 1);
        const Message cut = decodeOk(std::span<const std::byte>(full.data(), tailStart));
        CHECK(cut.index() == withVision.index());
        for (std::size_t len = tailStart + 1; len < full.size(); ++len)
            CHECK_FALSE(codec.decode(std::span<const std::byte>(full.data(), len)).ok());
        return cut;
    };

    const auto sensor = std::get<SensorFrame>(check(
        sensorWith({visionFrame("front", CameraVendor::PhotonVision, 1, 2)}), sensorWith({})));
    CHECK(sensor.vision.empty());
    CHECK(sensor.sessionId == 9);  // the tails before the vision tail are intact
    CHECK(sensor.runEpoch == 3);
    CHECK(sensor.samples.size() == 2);

    const auto actuator =
        std::get<ActuatorFrame>(check(actuatorWith({fullState()}), actuatorWith({})));
    CHECK(actuator.vision.empty());
    CHECK(actuator.ackTick == 41);
    CHECK(actuator.runEpoch == 3);

    const auto ack = std::get<HelloAck>(
        check(helloAckWith({cameraSetup("front", CameraVendor::PhotonVision)}), helloAckWith({})));
    CHECK(ack.vision.empty());
    CHECK(ack.runEpoch == 5);
    CHECK(ack.watchdogTimeoutMs == 3000);
    CHECK(ack.watchdogPolicy == WatchdogPolicy::Terminate);
}

TEST_CASE(
    "vision codec: vision vectors at their caps round trip and one more is refused when encoding") {
    BinaryMessageCodec codec;
    const auto refused = [&](const Message& m) {
        std::vector<std::byte> out;
        const Status s = codec.encode(m, out);
        REQUIRE_FALSE(s.ok());
        CHECK(s.error().code == ErrorCode::InvalidArgument);
        CHECK(out.empty());
    };

    std::vector<VisionFrame> frames(kMaxVisionFrames);
    for (std::size_t i = 0; i < frames.size(); ++i) frames[i].sequence = i + 1;
    CHECK(std::get<SensorFrame>(decodeOk(encodeOk(sensorWith(frames)))).vision == frames);
    frames.emplace_back();
    refused(sensorWith(frames));

    VisionFrame crowded = visionFrame("front", CameraVendor::PhotonVision, 1, 0);
    for (std::size_t i = 0; i < kMaxVisionObservations; ++i)
        crowded.observations.push_back(observation(static_cast<int>(i)));
    CHECK(std::get<SensorFrame>(decodeOk(encodeOk(sensorWith({crowded})))).vision[0] == crowded);
    crowded.observations.push_back(observation(1));
    refused(sensorWith({crowded}));

    VisionFrame tagged = visionFrame("front", CameraVendor::PhotonVision, 1, 0);
    for (std::size_t i = 0; i < kMaxVisionTagIds; ++i)
        tagged.multiTag.tagIds.push_back(static_cast<int>(i));
    CHECK(std::get<SensorFrame>(decodeOk(encodeOk(sensorWith({tagged})))).vision[0] == tagged);
    tagged.multiTag.tagIds.push_back(0);
    refused(sensorWith({tagged}));

    std::vector<CameraSetup> setups(kMaxVisionCameras,
                                    cameraSetup("cam", CameraVendor::PhotonVision));
    CHECK(std::get<HelloAck>(decodeOk(encodeOk(helloAckWith(setups)))).vision.cameras == setups);
    setups.push_back(cameraSetup("one more", CameraVendor::Limelight));
    refused(helloAckWith(setups));

    std::vector<CameraRobotState> states(kMaxVisionCameras, fullState());
    CHECK(std::get<ActuatorFrame>(decodeOk(encodeOk(actuatorWith(states)))).vision == states);
    states.emplace_back();
    refused(actuatorWith(states));
}

TEST_CASE("vision codec: hostile vision counts and unknown vendors are protocol errors") {
    // an empty SensorFrame ends with the vision count (0): swap it for a hostile one
    const std::vector<std::byte> empty = encodeOk(sensorWith({}));
    REQUIRE(empty.back() == std::byte{0});
    const auto withCount = [&](std::uint64_t count) {
        std::vector<std::byte> f(empty.begin(), empty.end() - 1);
        const std::vector<std::byte> v = varint(count);
        f.insert(f.end(), v.begin(), v.end());
        return f;
    };
    CHECK(decodeError(withCount(kMaxVisionFrames + 1)) ==
          ErrorCode::ProtocolError);                               // above the cap
    CHECK(decodeError(withCount(5)) == ErrorCode::ProtocolError);  // more frames than bytes
    CHECK(decodeError(withCount(std::uint64_t{1} << 40)) == ErrorCode::ProtocolError);

    // a frame claiming more observations than the cap
    std::vector<std::byte> frame =
        encodeOk(sensorWith({visionFrame("a", CameraVendor::PhotonVision, 1, 0)}));
    const std::size_t obsCountAt = visionTailStart(sensorWith({})) + 1 + (1 + 1) + 1 + (1 + 1) +
                                   4 * 8 + 28;  // count, "a", vendor, ntName, 4 x 8, pose
    REQUIRE(frame[obsCountAt] == std::byte{0});
    std::vector<std::byte> hostile(frame.begin(),
                                   frame.begin() + static_cast<std::ptrdiff_t>(obsCountAt));
    const std::vector<std::byte> huge = varint(kMaxVisionObservations + 1);
    hostile.insert(hostile.end(), huge.begin(), huge.end());
    hostile.insert(hostile.end(), frame.begin() + static_cast<std::ptrdiff_t>(obsCountAt) + 1,
                   frame.end());
    CHECK(decodeError(hostile) == ErrorCode::ProtocolError);

    // CameraVendor is a closed enum in all three records that carry it
    const auto vendorBreaks = [&](const Message& withVision, const Message& withoutVision) {
        std::vector<std::byte> bytes = encodeOk(withVision);
        // after the count comes a one character string, then the vendor
        const std::size_t vendorAt = visionTailStart(withoutVision) + 1 + 2;
        REQUIRE(bytes[vendorAt] == std::byte{1});
        bytes[vendorAt] = std::byte{2};
        CHECK(decodeError(bytes) == ErrorCode::ProtocolError);
    };
    vendorBreaks(sensorWith({visionFrame("a", CameraVendor::Limelight, 1, 0)}), sensorWith({}));
    vendorBreaks(helloAckWith({cameraSetup("a", CameraVendor::Limelight)}), helloAckWith({}));
    CameraRobotState state = fullState();
    state.ntName = "a";
    vendorBreaks(actuatorWith({state}), actuatorWith({}));
}

TEST_CASE("vision codec: extreme integers and non-finite doubles round trip bit for bit") {
    VisionFrame f =
        visionFrame(std::string(300, 'c'), CameraVendor::Limelight, ~std::uint64_t{0}, 1);
    f.captureTimeUs = std::numeric_limits<std::int64_t>::min();
    f.latencyUs = std::numeric_limits<std::int64_t>::max();
    f.observations[0].id = std::numeric_limits<int>::min();
    f.observations[0].areaPct = std::numeric_limits<double>::quiet_NaN();
    f.observations[0].distanceM = std::numeric_limits<double>::infinity();
    f.observations[0].cornersPx[2].x = -0.0;
    f.observations[0].fieldToTag.position.x = std::numeric_limits<float>::infinity();

    const std::vector<std::byte> bytes = encodeOk(sensorWith({f}));
    const auto back = std::get<SensorFrame>(decodeOk(bytes));
    REQUIRE(back.vision.size() == 1);
    CHECK(back.vision[0].cameraId.size() == 300);
    CHECK(back.vision[0].sequence == ~std::uint64_t{0});
    CHECK(back.vision[0].captureTimeUs == std::numeric_limits<std::int64_t>::min());
    CHECK(back.vision[0].latencyUs == std::numeric_limits<std::int64_t>::max());
    CHECK(back.vision[0].observations[0].id == std::numeric_limits<int>::min());
    CHECK(std::isnan(back.vision[0].observations[0].areaPct));
    CHECK(std::isinf(back.vision[0].observations[0].distanceM));
    CHECK(std::signbit(back.vision[0].observations[0].cornersPx[2].x));
    CHECK(std::isinf(back.vision[0].observations[0].fieldToTag.position.x));
    CHECK(encodeOk(back) == bytes);
}

TEST_CASE("vision codec: fuzz - corrupted vision messages never crash and stay consistent") {
    BinaryMessageCodec codec;
    std::mt19937_64 rng(0xB15101);
    const std::vector<Message> samples = {
        sensorWith({visionFrame("front", CameraVendor::PhotonVision, 1, 3),
                    visionFrame("left", CameraVendor::Limelight, 2, 1)}),
        actuatorWith({fullState(), CameraRobotState{}}),
        helloAckWith({cameraSetup("front", CameraVendor::PhotonVision),
                      cameraSetup("left", CameraVendor::Limelight)})};
    std::size_t decodedOk = 0;
    for (int iteration = 0; iteration < 4000; ++iteration) {
        std::vector<std::byte> f = encodeOk(samples[rng() % samples.size()]);
        const int edits = 1 + static_cast<int>(rng() % 3);
        for (int e = 0; e < edits && !f.empty(); ++e) {
            switch (rng() % 3) {
                case 0:
                    f[rng() % f.size()] ^= static_cast<std::byte>(1u << (rng() % 8));
                    break;
                case 1:
                    f[rng() % f.size()] = static_cast<std::byte>(rng() & 0xFF);
                    break;
                default:
                    f.resize(rng() % (f.size() + 1));
                    break;
            }
        }
        const auto r = codec.decode(f);
        if (!r.ok()) continue;
        ++decodedOk;
        std::vector<std::byte> again;
        REQUIRE(codec.encode(r.value(), again).ok());  // whatever decoded can be encoded again
        CHECK(codec.decode(again).ok());
    }
    CHECK(decodedOk > 0);
}
