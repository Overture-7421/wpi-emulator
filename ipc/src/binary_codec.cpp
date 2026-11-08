#include "wpiemu/ipc/binary_codec.h"

#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <type_traits>
#include <utility>

#include "match_codec.h"
#include "wpiemu/sdk/core/wire_codec.h"
#include "wpiemu/sdk/core/wire_fields.h"

namespace wpiemu::ipc {

using namespace sdk;
using namespace sdk::proto;

namespace {

// Sanity caps on element counts; they bound the memory a hostile frame can make the decoder allocate.
constexpr std::size_t kMaxSignalSpecs = 65536;  // SignalSample::signal is a u16 index
constexpr std::size_t kMaxSamples = std::size_t{1} << 20;
constexpr std::size_t kMaxLoopbackIds = std::size_t{1} << 20;
constexpr std::size_t kMaxPortCounts = 4096;
constexpr std::size_t kMaxJoystickElements = 1024;
constexpr std::size_t kMaxCameras = 256;
constexpr std::size_t kMaxVisionFrames = 1024;  // camera frames carried by one SensorFrame
constexpr std::size_t kMaxTagsPerFrame = 1024;
constexpr std::size_t kTagWireBytes = 4 + 28 + 4 * 8 + 8 * 8 + 8;
// Smallest encoding of a CameraFrame (no tags) / CameraSetup (empty strings): bounds counts by the bytes left.
constexpr std::size_t kCameraFrameMinWireBytes = 4 + 8 + 8 + 8 + 28 + 1 + 1 + 28 + 3 * 8;
constexpr std::size_t kCameraSetupMinWireBytes = 3 + 1 + 2 + 28 + 6 * 8 + 2 * 4 + 2 * 8;
constexpr std::size_t kHeaderBytes = 3;  // wireFormat + messageType
constexpr std::size_t kSampleWireBytes = 14;

// The field order of every struct is written once, as a `fields(codec, value)` function below, for the symmetric
// Writer/Reader of sdk/core/wire_codec.h (scalars and geometry live in wire_fields.h). Wire layout:
// docs/reference/extension-ipc.md.

// Names this namespace as the home of the `fields` functions below, which the Writer and Reader find by ADL.
struct CodecDomain {};
using Writer = wire::Writer<CodecDomain>;
using Reader = wire::Reader<CodecDomain>;
using wire::Is;
using wire::ReadState;

// ---- shared records ---------------------------------------------------------------------------------------

template <class C, Is<std::pair<std::string, std::int32_t>> M>
void fields(C& c, M& portCount) {
    c.str(portCount.first);
    c.i32(portCount.second);
}

template <class C, Is<DeviceId> M>
void fields(C& c, M& id) {
    c.u32(id.value);
}

template <class C, Is<DeviceDescriptor> M>
void fields(C& c, M& d) {
    c.record([&] {
        c.str(d.family);
        c.str(d.type);
        c.str(d.vendor);
        c.i32(d.id);
        c.str(d.bus);
        c.i32(d.channel);
        c.u32(d.capabilities.bits);
        c.str(d.nativeName);
        c.str(d.displayName);
    });
}

template <class C, Is<SignalSpec> M>
void fields(C& c, M& s) {
    c.record([&] {
        c.str(s.name);
        c.enumeration(s.direction, 2, "invalid SignalDirection");
        c.enumeration(s.type, 2, "invalid SignalType");
        c.str(s.unit);
    });
}

template <class C, Is<SignalSample> M>
void fields(C& c, M& s) {
    fields(c, s.device);
    c.u16(s.signal);
    c.f64(s.value);
}

template <class C, Is<JoystickState> M>
void fields(C& c, M& j) {
    c.record([&] {
        c.boolean(j.present);
        c.list(j.axes, kMaxJoystickElements, 4);
        c.list(j.povs, kMaxJoystickElements, 2);
        c.u32(j.buttons);
        c.u8(j.buttonCount);
    });
}

// ---- vision records (flat: sizes are bounded by the counts below, not by the 16-bit record length) -------------

template <class C, Is<TagDetection> M>
void fields(C& c, M& t) {
    c.i32(t.id);
    fields(c, t.cameraToTag);
    c.f64(t.yawDeg);
    c.f64(t.pitchDeg);
    c.f64(t.areaPercent);
    c.f64(t.ambiguity);
    for (auto& corner : t.cornersPx) c.f64(corner);
    c.f64(t.distanceM);
}

template <class C, Is<CameraFrame> M>
void fields(C& c, M& f) {
    c.u32(f.cameraIndex);
    c.u64(f.sequence);
    c.i64(f.captureTimeUs);
    c.i64(f.publishTimeUs);
    fields(c, f.robotToCamera);
    c.list(f.tags, kMaxTagsPerFrame, kTagWireBytes);
    c.boolean(f.hasRobotPose);
    fields(c, f.robotPoseField);
    c.f64(f.avgTagDistM);
    c.f64(f.avgTagAreaPercent);
    c.f64(f.tagSpanM);
}

template <class C, Is<CameraConfig> M>
void fields(C& c, M& cfg) {
    c.str(cfg.id);
    c.str(cfg.label);
    c.str(cfg.name);
    c.enumeration(cfg.kind, 1, "invalid CameraKind");
    c.str(cfg.attachment.parentMechanismId);
    c.str(cfg.attachment.parentLink);
    fields(c, cfg.local);
    c.f64(cfg.hFovDeg);
    c.f64(cfg.vFovDeg);
    c.f64(cfg.maxRangeM);
    c.f64(cfg.maxIncidenceDeg);
    c.f64(cfg.fps);
    c.f64(cfg.latencyMs);
    c.i32(cfg.widthPx);
    c.i32(cfg.heightPx);
    c.f64(cfg.positionNoiseM);
    c.f64(cfg.yawNoiseDeg);
}

template <class C, Is<CameraSetup> M>
void fields(C& c, M& setup) {
    fields(c, setup.config);
}

// ---- messages: base fields first, later fields in an optional tail ---------------------------------------------
// `if (!c.more()) return;` ends a message that an older peer sent without the appended fields below it: they keep
// their defaults. A newer peer's extra bytes after the last field are ignored.

template <class C, Is<Hello> M>
void fields(C& c, M& m) {
    c.u32(m.protocolVersion);
    c.enumeration(m.language);
    c.i32(m.wpilibYear);
    c.str(m.wpilibVersion);
    c.str(m.extensionVersion);
    c.i64(m.pid);
    c.str(m.programName);
    c.list(m.halPortCounts, kMaxPortCounts, 5);
    if (!c.more()) return;
    c.u32(m.proposedWatchdogMs);
    if (!c.more()) return;
    c.boolean(m.supportsArmed);
}

template <class C, Is<HelloAck> M>
void fields(C& c, M& m) {
    c.boolean(m.accepted);
    c.str(m.reason);
    c.u32(m.protocolVersion);
    c.enumeration(m.clockMode, 1, "invalid ClockMode");
    c.u32(m.stepUs);
    c.u64(m.sessionId);
    c.boolean(m.discoveryOnly);
    c.boolean(m.enableL3Hooks);
    if (!c.more()) return;
    c.u64(m.runEpoch);
    c.u32(m.watchdogTimeoutMs);
    c.enumeration(m.watchdogPolicy);
}

template <class C, Is<DeviceAnnounce> M>
void fields(C& c, M& m) {
    fields(c, m.id);
    fields(c, m.descriptor);
    c.list(m.signalSpecs, kMaxSignalSpecs, 2);
    c.boolean(m.retired);
    if (!c.more()) return;
    c.str(m.observedBus);
}

template <class C, Is<LoopbackSet> M>
void fields(C& c, M& m) {
    c.list(m.loopback, kMaxLoopbackIds, 4);
}

template <class C, Is<SensorFrame> M>
void fields(C& c, M& m) {
    c.u64(m.tick);
    c.i64(m.simTimeUs);
    c.u32(m.stepUs);
    c.f64(m.batteryVoltageV);
    c.f64(m.batteryCurrentA);
    c.boolean(m.full);
    c.list(m.samples, kMaxSamples, kSampleWireBytes);
    if (!c.more()) return;
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    if (!c.more()) return;
    c.list(m.vision, kMaxVisionFrames, kCameraFrameMinWireBytes);
}

template <class C, Is<ActuatorFrame> M>
void fields(C& c, M& m) {
    c.u64(m.tick);
    c.i64(m.simTimeUs);
    c.u64(m.robotLoopCount);
    c.boolean(m.full);
    c.list(m.samples, kMaxSamples, kSampleWireBytes);
    if (!c.more()) return;
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.u64(m.ackTick);
    c.enumeration(m.result);
}

template <class C, Is<DsState> M>
void fields(C& c, M& m) {
    c.boolean(m.dsAttached);
    c.boolean(m.fmsAttached);
    c.boolean(m.enabled);
    c.boolean(m.eStop);
    c.enumeration(m.mode, 2, "invalid RobotMode");
    c.enumeration(m.station, 5, "invalid AllianceStation");
    c.f64(m.matchTimeS);
    c.str(m.gameSpecificMessage);
    for (auto& joystick : m.joysticks) fields(c, joystick);
    if (!c.more()) return;
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
}

template <class C, Is<SimControl> M>
void fields(C& c, M& m) {
    c.enumeration(m.command);  // unknown commands are answered with ControlResult::Unsupported
    c.u32(m.stepCount);
    c.f64(m.timeScale);
    if (!c.more()) return;
    c.u64(m.commandId);
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
}

template <class C, Is<SimControlAck> M>
void fields(C& c, M& m) {
    c.u64(m.commandId);
    c.enumeration(m.command);
    c.enumeration(m.result);
    c.u64(m.sessionId);
    c.u64(m.runEpoch);
    c.u64(m.tick);
    c.str(m.detail);
}

template <class C, Is<Log> M>
void fields(C& c, M& m) {
    c.i64(m.simTimeUs);
    c.enumeration(m.level, 4, "invalid LogLevel");
    c.str(m.source);
    c.str(m.message);
}

template <class C, Is<Goodbye> M>
void fields(C& c, M& m) {
    c.str(m.reason);
}

template <class C, Is<VisionSetup> M>
void fields(C& c, M& m) {
    c.list(m.cameras, kMaxCameras, kCameraSetupMinWireBytes);
    if (!c.more()) return;
    c.str(m.photonVersion);
    if (!c.more()) return;
    c.str(m.layoutName);
    if (!c.more()) return;
    c.f64(m.fieldLengthM);
    c.f64(m.fieldWidthM);
}

template <class T>
Result<Message> decodeBody(std::span<const std::byte> body) {
    ReadState state;
    Reader r(body, &state);
    T value;
    fields(r, value);
    if (!state.ok) return makeError(ErrorCode::ProtocolError, std::string("malformed frame: ") + state.what);
    return Message{std::move(value)};
}

}  // namespace

Status BinaryMessageCodec::encode(const Message& message, std::vector<std::byte>& out) const {
    const std::size_t start = out.size();
    try {
        Writer w(out);
        w.u8(kWireFormatVersion);
        w.u16(static_cast<std::uint16_t>(typeOf(message)));
        bool bad = false;
        if (isMatchMessage(message)) {
            bad = !detail::encodeMatchBody(message, out);
        } else {
            std::visit(
                [&w](const auto& m) {
                    if constexpr (requires { fields(w, m); }) fields(w, m);
                },
                message);
            bad = w.bad();
        }
        if (bad) {
            out.resize(start);
            return makeError(ErrorCode::InvalidArgument, "message field exceeds the wire limits");
        }
        if (out.size() - start > maxFrameBytes_) {
            out.resize(start);
            return makeError(ErrorCode::InvalidArgument, "encoded message exceeds the maximum frame size");
        }
    } catch (const std::bad_alloc&) {
        out.resize(start);
        return makeError(ErrorCode::Internal, "out of memory while encoding");
    }
    return okStatus();
}

Result<Message> BinaryMessageCodec::decode(std::span<const std::byte> frame) const {
    if (frame.size() < kHeaderBytes) return makeError(ErrorCode::ProtocolError, "frame shorter than its header");
    if (frame.size() > maxFrameBytes_) return makeError(ErrorCode::ProtocolError, "frame exceeds the maximum size");
    const auto wire = static_cast<std::uint8_t>(frame[0]);
    if (wire != kWireFormatVersion)
        return makeError(ErrorCode::VersionMismatch, "unsupported wire format " + std::to_string(wire));
    const auto type = static_cast<std::uint16_t>(static_cast<std::uint8_t>(frame[1]) |
                                                 (static_cast<std::uint16_t>(static_cast<std::uint8_t>(frame[2])) << 8));
    const std::span<const std::byte> body = frame.subspan(kHeaderBytes);
    try {
        switch (static_cast<MessageType>(type)) {
            case MessageType::Hello: return decodeBody<Hello>(body);
            case MessageType::HelloAck: return decodeBody<HelloAck>(body);
            case MessageType::DeviceAnnounce: return decodeBody<DeviceAnnounce>(body);
            case MessageType::LoopbackSet: return decodeBody<LoopbackSet>(body);
            case MessageType::SensorFrame: return decodeBody<SensorFrame>(body);
            case MessageType::ActuatorFrame: return decodeBody<ActuatorFrame>(body);
            case MessageType::DsState: return decodeBody<DsState>(body);
            case MessageType::SimControl: return decodeBody<SimControl>(body);
            case MessageType::Log: return decodeBody<Log>(body);
            case MessageType::Goodbye: return decodeBody<Goodbye>(body);
            case MessageType::SimControlAck: return decodeBody<SimControlAck>(body);
            case MessageType::VisionSetup: return decodeBody<VisionSetup>(body);
            case MessageType::Join:
            case MessageType::JoinAccept:
            case MessageType::JoinReject:
            case MessageType::Roster:
            case MessageType::Leave:
            case MessageType::MatchInputPacket:
            case MessageType::MatchSnapshot:
            case MessageType::MatchSnapshotAck:
            case MessageType::MatchControl:
            case MessageType::RegisterRoom:
            case MessageType::RoomRegistered:
            case MessageType::RoomRejected:
            case MessageType::JoinRoom:
            case MessageType::JoinResult:
            case MessageType::PeerJoined:
            case MessageType::PeerLeft: return detail::decodeMatchBody(static_cast<MessageType>(type), body);
        }
    } catch (const std::bad_alloc&) {
        return makeError(ErrorCode::Internal, "out of memory while decoding");
    }
    return makeError(ErrorCode::Unsupported, "unknown message type " + std::to_string(type));
}

Result<bool> receiveSkippingUnknown(MessageChannel& channel, Message& out, std::chrono::milliseconds timeout,
                                    std::size_t* skipped) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
        Result<bool> got = channel.receive(out, proto::timeLeft(deadline));
        if (got || !isSkippableDecodeError(got.error())) return got;
        if (skipped != nullptr) ++*skipped;
    }
}

}  // namespace wpiemu::ipc
