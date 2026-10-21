#include "wpiemu/ipc/binary_codec.h"

#include <bit>
#include <concepts>
#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <type_traits>
#include <utility>

#include "wpiemu/sdk/core/byte_order.h"

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

// The field order of every struct is written once, as a `fields(codec, value)` function below. The Writer and the
// Reader offer the same operations (a writer takes the value, a reader fills it in), so one description encodes and
// decodes and the two cannot drift apart. Wire layout: docs/reference/extension-ipc.md.

class Writer : public LittleEndianWriter<std::byte> {
public:
    explicit Writer(std::vector<std::byte>& out) : LittleEndianWriter(out) {}

    bool bad() const { return bad_; }
    // A writer always has more: fields after an `if (!c.more()) return;` are the optional appended tail.
    constexpr bool more() const { return true; }

    // The bound only matters when reading.
    template <class E>
    void enumeration(E v, std::uint8_t /*maxValue*/ = 0xFF, const char* /*what*/ = "") {
        u8(static_cast<std::uint8_t>(v));
    }

    void str(const std::string& s) {
        if (s.size() > kMaxStringBytes) {
            bad_ = true;
            return;
        }
        varint(s.size());
        const auto* p = reinterpret_cast<const std::byte*>(s.data());
        out_.insert(out_.end(), p, p + s.size());
    }

    // Counted list, each element written by its own `fields` (found by argument-dependent lookup). The cap and
    // the minimum element size are the decoder's limits: a list it would refuse is not worth sending.
    template <class Items>
    void list(const Items& items, std::size_t cap, std::size_t /*minElementBytes*/) {
        if (items.size() > cap) {
            bad_ = true;
            return;
        }
        varint(items.size());
        for (const auto& item : items) fields(*this, item);
    }

    // u16 byte-length prefix, so the record can grow fields later and older readers skip them.
    template <class Body>
    void record(Body body) {
        const std::size_t at = out_.size();
        u16(0);
        body();
        const std::size_t len = out_.size() - at - 2;
        if (len > 0xFFFF) {
            bad_ = true;
            return;
        }
        out_[at] = static_cast<std::byte>(len & 0xFF);
        out_[at + 1] = static_cast<std::byte>((len >> 8) & 0xFF);
    }

private:
    void varint(std::uint64_t v) {
        while (v >= 0x80) {
            u8(static_cast<std::uint8_t>(v | 0x80));
            v >>= 7;
        }
        u8(static_cast<std::uint8_t>(v));
    }

    bool bad_ = false;
};

struct ReadState {
    bool ok = true;
    const char* what = "";
};

class Reader {
public:
    Reader(std::span<const std::byte> data, ReadState* state) : d_(data), s_(state) {}

    bool ok() const { return s_->ok; }
    // True while there is unread data: absent trailing (appended) fields mean an older peer, so they keep their defaults.
    bool more() const { return s_->ok && pos_ < d_.size(); }
    void fail(const char* what) {
        if (s_->ok) {
            s_->ok = false;
            s_->what = what;
        }
    }

    void u8(std::uint8_t& v) { v = static_cast<std::uint8_t>(le(1)); }
    void u16(std::uint16_t& v) { v = static_cast<std::uint16_t>(le(2)); }
    void u32(std::uint32_t& v) { v = static_cast<std::uint32_t>(le(4)); }
    void u64(std::uint64_t& v) { v = le(8); }
    void i16(std::int16_t& v) { v = static_cast<std::int16_t>(le(2)); }
    void i32(std::int32_t& v) { v = static_cast<std::int32_t>(le(4)); }
    void i64(std::int64_t& v) { v = static_cast<std::int64_t>(le(8)); }
    void f32(float& v) { v = std::bit_cast<float>(static_cast<std::uint32_t>(le(4))); }
    void f64(double& v) { v = std::bit_cast<double>(le(8)); }
    void boolean(bool& v) { v = le(1) != 0; }

    // Unbounded, any value is accepted (the receiver answers unknown commands itself); with a bound, a larger
    // value is a malformed frame.
    template <class E>
    void enumeration(E& v, std::uint8_t maxValue = 0xFF, const char* what = "") {
        std::uint8_t raw = 0;
        u8(raw);
        if (raw > maxValue) {
            fail(what);
            return;
        }
        v = static_cast<E>(raw);
    }

    void str(std::string& s) {
        const std::uint64_t n = varint();
        if (!s_->ok) return;
        if (n > kMaxStringBytes || n > remaining()) {
            fail("string length out of range");
            return;
        }
        s.assign(reinterpret_cast<const char*>(d_.data() + pos_), static_cast<std::size_t>(n));
        pos_ += static_cast<std::size_t>(n);
    }

    // Counted list; the count is checked against the cap and against the bytes left (`minElementBytes` each).
    template <class Items>
    void list(Items& items, std::size_t cap, std::size_t minElementBytes) {
        items.resize(count(minElementBytes, cap));
        for (auto& item : items) {
            if (!ok()) break;
            fields(*this, item);
        }
    }

    // The body reads inside the record; this reader then continues after it, whatever the body consumed.
    template <class Body>
    void record(Body body) {
        std::uint16_t n = 0;
        u16(n);
        if (!s_->ok) return;
        if (n > remaining()) {
            fail("record length out of range");
            return;
        }
        const std::span<const std::byte> outer = d_;
        const std::size_t resume = pos_ + n;
        d_ = outer.subspan(pos_, n);
        pos_ = 0;
        body();
        d_ = outer;
        pos_ = resume;
    }

private:
    std::size_t remaining() const { return d_.size() - pos_; }

    std::uint64_t varint() {
        std::uint64_t v = 0;
        for (int i = 0; i < 10; ++i) {
            if (remaining() == 0) {
                fail("truncated varint");
                return 0;
            }
            const auto b = static_cast<std::uint8_t>(d_[pos_++]);
            if (i == 9 && b > 1) {
                fail("varint overflow");
                return 0;
            }
            v |= static_cast<std::uint64_t>(b & 0x7F) << (7 * i);
            if ((b & 0x80) == 0) return v;
        }
        fail("varint too long");
        return 0;
    }

    // Element count of a vector whose elements occupy at least `minElemBytes` on the wire.
    std::size_t count(std::size_t minElemBytes, std::size_t cap) {
        const std::uint64_t n = varint();
        if (!s_->ok) return 0;
        if (n > cap) {
            fail("element count exceeds limit");
            return 0;
        }
        if (minElemBytes > 0 && n > remaining() / minElemBytes) {
            fail("element count exceeds frame");
            return 0;
        }
        return static_cast<std::size_t>(n);
    }

    std::uint64_t le(int bytes) {
        if (!s_->ok) return 0;
        if (remaining() < static_cast<std::size_t>(bytes)) {
            fail("truncated field");
            return 0;
        }
        const std::uint64_t v = readLittleEndian(d_, pos_, bytes);
        pos_ += static_cast<std::size_t>(bytes);
        return v;
    }

    std::span<const std::byte> d_;
    std::size_t pos_ = 0;
    ReadState* s_;
};

// T, possibly const: the writer visits a const message, the reader a mutable one.
template <class T, class U>
concept Is = std::same_as<std::remove_const_t<T>, U>;

// ---- scalars and shared records ---------------------------------------------------------------------------

template <class C, Is<float> M>
void fields(C& c, M& v) {
    c.f32(v);
}

template <class C, Is<std::int16_t> M>
void fields(C& c, M& v) {
    c.i16(v);
}

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

template <class C, Is<Pose3> M>
void fields(C& c, M& p) {
    c.f32(p.position.x);
    c.f32(p.position.y);
    c.f32(p.position.z);
    c.f32(p.rotation.x);
    c.f32(p.rotation.y);
    c.f32(p.rotation.z);
    c.f32(p.rotation.w);
}

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
        std::visit([&w](const auto& m) { fields(w, m); }, message);
        if (w.bad()) {
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
