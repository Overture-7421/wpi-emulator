#include "wpiemu/ipc/binary_codec.h"

#include <array>
#include <bit>
#include <cstring>
#include <limits>
#include <new>
#include <string>

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
constexpr std::size_t kHeaderBytes = 3;  // wireFormat + messageType
constexpr std::size_t kSampleWireBytes = 14;

// Fixed wire sizes of the vision records (see ipc/README.md); decode uses them to bound element
// counts by the bytes left.
constexpr std::size_t kPoseWireBytes = 7 * 4;
constexpr std::size_t kPixelWireBytes = 2 * 8;
constexpr std::size_t kObservationWireBytes =
    4 + 5 * kPixelWireBytes + 5 * 8 + 3 * kPoseWireBytes + 3 * 8;
constexpr std::size_t kTagIdWireBytes = 4;
constexpr std::size_t kMinMultiTagWireBytes = 1 + kPoseWireBytes + 8 + 1;
constexpr std::size_t kMinVisionFrameWireBytes =
    1 + 1 + 1 + 3 * 8 + 8 + kPoseWireBytes + 1 + kMinMultiTagWireBytes;
constexpr std::size_t kMinCameraSetupWireBytes =
    1 + 1 + 1 + 2 * 4 + 4 * 8 + 5 * 8 + 8 + 8 + kPoseWireBytes + 1;
constexpr std::size_t kMinCameraStateWireBytes = 1 + 1 + 1 + 1 + 1 + 1 + 1 + 8;

// CameraRobotState: which optional fields follow, in this bit order.
constexpr std::uint8_t kHasPipelineIndex = 1u << 0;
constexpr std::uint8_t kHasLedMode = 1u << 1;
constexpr std::uint8_t kHasImuMode = 1u << 2;
constexpr std::uint8_t kHasDriverMode = 1u << 3;
constexpr std::uint8_t kHasThrottle = 1u << 4;
constexpr std::uint8_t kHasFpsLimit = 1u << 5;
constexpr std::uint8_t kHasRobotOrientation = 1u << 6;
constexpr std::uint8_t kHasCameraPoseOverride = 1u << 7;

class Writer {
public:
    explicit Writer(std::vector<std::byte>& out) : out_(out) {}

    bool bad() const { return bad_; }
    void fail() { bad_ = true; }

    void u8(std::uint8_t v) { out_.push_back(static_cast<std::byte>(v)); }
    void u16(std::uint16_t v) { le(v, 2); }
    void u32(std::uint32_t v) { le(v, 4); }
    void u64(std::uint64_t v) { le(v, 8); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void f32(float v) { u32(std::bit_cast<std::uint32_t>(v)); }
    void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
    void boolean(bool v) { u8(v ? 1 : 0); }

    void varint(std::uint64_t v) {
        while (v >= 0x80) {
            u8(static_cast<std::uint8_t>(v | 0x80));
            v >>= 7;
        }
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

    std::size_t beginRecord() {
        const std::size_t at = out_.size();
        u16(0);
        return at;
    }

    void endRecord(std::size_t at) {
        const std::size_t len = out_.size() - at - 2;
        if (len > 0xFFFF) {
            bad_ = true;
            return;
        }
        out_[at] = static_cast<std::byte>(len & 0xFF);
        out_[at + 1] = static_cast<std::byte>((len >> 8) & 0xFF);
    }

private:
    void le(std::uint64_t v, int bytes) {
        for (int i = 0; i < bytes; ++i) out_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFF));
    }

    std::vector<std::byte>& out_;
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
    // True while there is unread data: used to detect absent trailing (appended) fields.
    bool more() const { return s_->ok && pos_ < d_.size(); }
    std::size_t remaining() const { return d_.size() - pos_; }
    void fail(const char* what) {
        if (s_->ok) {
            s_->ok = false;
            s_->what = what;
        }
    }

    std::uint8_t u8() { return static_cast<std::uint8_t>(le(1)); }
    std::uint16_t u16() { return static_cast<std::uint16_t>(le(2)); }
    std::uint32_t u32() { return static_cast<std::uint32_t>(le(4)); }
    std::uint64_t u64() { return le(8); }
    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    std::int64_t i64() { return static_cast<std::int64_t>(u64()); }
    float f32() { return std::bit_cast<float>(u32()); }
    double f64() { return std::bit_cast<double>(u64()); }
    bool boolean() { return u8() != 0; }

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

    std::string str() {
        const std::uint64_t n = varint();
        if (!s_->ok) return {};
        if (n > kMaxStringBytes || n > remaining()) {
            fail("string length out of range");
            return {};
        }
        std::string s(reinterpret_cast<const char*>(d_.data() + pos_), static_cast<std::size_t>(n));
        pos_ += static_cast<std::size_t>(n);
        return s;
    }

    // Nested length-prefixed record; the parent continues after it, whatever the child consumed.
    Reader record() {
        const std::uint16_t n = u16();
        if (!s_->ok) return Reader({}, s_);
        if (n > remaining()) {
            fail("record length out of range");
            return Reader({}, s_);
        }
        Reader sub(d_.subspan(pos_, n), s_);
        pos_ += n;
        return sub;
    }

private:
    std::uint64_t le(int bytes) {
        if (!s_->ok) return 0;
        if (remaining() < static_cast<std::size_t>(bytes)) {
            fail("truncated field");
            return 0;
        }
        std::uint64_t v = 0;
        for (int i = 0; i < bytes; ++i) v |= static_cast<std::uint64_t>(static_cast<std::uint8_t>(d_[pos_ + i])) << (8 * i);
        pos_ += static_cast<std::size_t>(bytes);
        return v;
    }

    std::span<const std::byte> d_;
    std::size_t pos_ = 0;
    ReadState* s_;
};

template <class E>
E enumUpTo(Reader& r, std::uint8_t maxValue, const char* what) {
    const std::uint8_t v = r.u8();
    if (v > maxValue) {
        r.fail(what);
        return E{};
    }
    return static_cast<E>(v);
}

// ---- shared records ---------------------------------------------------------------------------------

void write(Writer& w, const DeviceDescriptor& d) {
    const std::size_t at = w.beginRecord();
    w.str(d.family);
    w.str(d.type);
    w.str(d.vendor);
    w.i32(d.id);
    w.str(d.bus);
    w.i32(d.channel);
    w.u32(d.capabilities.bits);
    w.str(d.nativeName);
    w.str(d.displayName);
    w.endRecord(at);
}

void read(Reader& outer, DeviceDescriptor& d) {
    Reader r = outer.record();
    d.family = r.str();
    d.type = r.str();
    d.vendor = r.str();
    d.id = r.i32();
    d.bus = r.str();
    d.channel = r.i32();
    d.capabilities = CapabilitySet{r.u32()};
    d.nativeName = r.str();
    d.displayName = r.str();
}

void write(Writer& w, const SignalSpec& s) {
    const std::size_t at = w.beginRecord();
    w.str(s.name);
    w.u8(static_cast<std::uint8_t>(s.direction));
    w.u8(static_cast<std::uint8_t>(s.type));
    w.str(s.unit);
    w.endRecord(at);
}

void read(Reader& outer, SignalSpec& s) {
    Reader r = outer.record();
    s.name = r.str();
    s.direction = enumUpTo<SignalDirection>(r, 2, "invalid SignalDirection");
    s.type = enumUpTo<SignalType>(r, 2, "invalid SignalType");
    s.unit = r.str();
}

void write(Writer& w, const std::vector<SignalSample>& samples) {
    w.varint(samples.size());
    for (const SignalSample& s : samples) {
        w.u32(s.device.value);
        w.u16(s.signal);
        w.f64(s.value);
    }
}

void read(Reader& r, std::vector<SignalSample>& samples) {
    const std::size_t n = r.count(kSampleWireBytes, kMaxSamples);
    samples.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) {
        SignalSample s;
        s.device = DeviceId{r.u32()};
        s.signal = r.u16();
        s.value = r.f64();
        samples.push_back(s);
    }
}

void write(Writer& w, const JoystickState& j) {
    const std::size_t at = w.beginRecord();
    w.boolean(j.present);
    w.varint(j.axes.size());
    for (float a : j.axes) w.f32(a);
    w.varint(j.povs.size());
    for (std::int16_t p : j.povs) w.u16(static_cast<std::uint16_t>(p));
    w.u32(j.buttons);
    w.u8(j.buttonCount);
    w.endRecord(at);
}

void read(Reader& outer, JoystickState& j) {
    Reader r = outer.record();
    j.present = r.boolean();
    const std::size_t axes = r.count(4, kMaxJoystickElements);
    j.axes.reserve(axes);
    for (std::size_t i = 0; i < axes && r.ok(); ++i) j.axes.push_back(r.f32());
    const std::size_t povs = r.count(2, kMaxJoystickElements);
    j.povs.reserve(povs);
    for (std::size_t i = 0; i < povs && r.ok(); ++i) j.povs.push_back(static_cast<std::int16_t>(r.u16()));
    j.buttons = r.u32();
    j.buttonCount = r.u8();
}

// ---- vision records (sdk/vision/vision_types.h): fixed layouts, no length prefix -----------------

void write(Writer& w, const Pose3& p) {
    w.f32(p.position.x);
    w.f32(p.position.y);
    w.f32(p.position.z);
    w.f32(p.rotation.x);
    w.f32(p.rotation.y);
    w.f32(p.rotation.z);
    w.f32(p.rotation.w);
}

void read(Reader& r, Pose3& p) {
    p.position.x = r.f32();
    p.position.y = r.f32();
    p.position.z = r.f32();
    p.rotation.x = r.f32();
    p.rotation.y = r.f32();
    p.rotation.z = r.f32();
    p.rotation.w = r.f32();
}

void write(Writer& w, const PixelPoint& p) {
    w.f64(p.x);
    w.f64(p.y);
}

void read(Reader& r, PixelPoint& p) {
    p.x = r.f64();
    p.y = r.f64();
}

void write(Writer& w, const FiducialObservation& o) {
    w.i32(o.id);
    for (const PixelPoint& corner : o.cornersPx) write(w, corner);
    write(w, o.centerPx);
    w.f64(o.yawRad);
    w.f64(o.pitchRad);
    w.f64(o.skewRad);
    w.f64(o.areaPct);
    w.f64(o.distanceM);
    write(w, o.fieldToTag);
    write(w, o.cameraToTagBest);
    write(w, o.cameraToTagAlt);
    w.f64(o.ambiguity);
    w.f64(o.reprojErrBestPx);
    w.f64(o.reprojErrAltPx);
}

void read(Reader& r, FiducialObservation& o) {
    o.id = r.i32();
    for (PixelPoint& corner : o.cornersPx) read(r, corner);
    read(r, o.centerPx);
    o.yawRad = r.f64();
    o.pitchRad = r.f64();
    o.skewRad = r.f64();
    o.areaPct = r.f64();
    o.distanceM = r.f64();
    read(r, o.fieldToTag);
    read(r, o.cameraToTagBest);
    read(r, o.cameraToTagAlt);
    o.ambiguity = r.f64();
    o.reprojErrBestPx = r.f64();
    o.reprojErrAltPx = r.f64();
}

void write(Writer& w, const MultiTagSolution& m) {
    w.boolean(m.valid);
    write(w, m.fieldToCamera);
    w.f64(m.reprojErrPx);
    if (m.tagIds.size() > kMaxVisionTagIds) w.fail();
    w.varint(m.tagIds.size());
    for (int id : m.tagIds) w.i32(id);
}

void read(Reader& r, MultiTagSolution& m) {
    m.valid = r.boolean();
    read(r, m.fieldToCamera);
    m.reprojErrPx = r.f64();
    const std::size_t n = r.count(kTagIdWireBytes, kMaxVisionTagIds);
    m.tagIds.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) m.tagIds.push_back(r.i32());
}

void write(Writer& w, const VisionFrame& f) {
    w.str(f.cameraId);
    w.u8(static_cast<std::uint8_t>(f.vendor));
    w.str(f.ntName);
    w.u64(f.sequence);
    w.i64(f.captureTimeUs);
    w.i64(f.latencyUs);
    w.i64(f.captureLatencyUs);
    write(w, f.robotToCamera);
    if (f.observations.size() > kMaxVisionObservations) w.fail();
    w.varint(f.observations.size());
    for (const FiducialObservation& o : f.observations) write(w, o);
    write(w, f.multiTag);
}

void read(Reader& r, VisionFrame& f) {
    f.cameraId = r.str();
    f.vendor = enumUpTo<CameraVendor>(r, 1, "invalid CameraVendor");
    f.ntName = r.str();
    f.sequence = r.u64();
    f.captureTimeUs = r.i64();
    f.latencyUs = r.i64();
    f.captureLatencyUs = r.i64();
    read(r, f.robotToCamera);
    const std::size_t n = r.count(kObservationWireBytes, kMaxVisionObservations);
    f.observations.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) read(r, f.observations.emplace_back());
    read(r, f.multiTag);
}

void write(Writer& w, const std::vector<VisionFrame>& frames) {
    if (frames.size() > kMaxVisionFrames) w.fail();
    w.varint(frames.size());
    for (const VisionFrame& f : frames) write(w, f);
}

void read(Reader& r, std::vector<VisionFrame>& frames) {
    const std::size_t n = r.count(kMinVisionFrameWireBytes, kMaxVisionFrames);
    frames.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) read(r, frames.emplace_back());
}

void write(Writer& w, const CameraSetup& c) {
    w.str(c.cameraId);
    w.u8(static_cast<std::uint8_t>(c.vendor));
    w.str(c.ntName);
    w.i32(c.widthPx);
    w.i32(c.heightPx);
    w.f64(c.fxPx);
    w.f64(c.fyPx);
    w.f64(c.cxPx);
    w.f64(c.cyPx);
    for (double k : c.distortion) w.f64(k);
    w.f64(c.frameRateHz);
    w.i64(c.latencyUs);
    write(w, c.defaultAssumedRobotToCamera);
    w.str(c.photonMessageUuid);
}

void read(Reader& r, CameraSetup& c) {
    c.cameraId = r.str();
    c.vendor = enumUpTo<CameraVendor>(r, 1, "invalid CameraVendor");
    c.ntName = r.str();
    c.widthPx = r.i32();
    c.heightPx = r.i32();
    c.fxPx = r.f64();
    c.fyPx = r.f64();
    c.cxPx = r.f64();
    c.cyPx = r.f64();
    for (double& k : c.distortion) k = r.f64();
    c.frameRateHz = r.f64();
    c.latencyUs = r.i64();
    read(r, c.defaultAssumedRobotToCamera);
    c.photonMessageUuid = r.str();
}

void write(Writer& w, const VisionSetup& s) {
    if (s.cameras.size() > kMaxVisionCameras) w.fail();
    w.varint(s.cameras.size());
    for (const CameraSetup& c : s.cameras) write(w, c);
}

void read(Reader& r, VisionSetup& s) {
    const std::size_t n = r.count(kMinCameraSetupWireBytes, kMaxVisionCameras);
    s.cameras.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) read(r, s.cameras.emplace_back());
}

void write(Writer& w, const CameraRobotState& s) {
    w.str(s.ntName);
    w.u8(static_cast<std::uint8_t>(s.vendor));
    w.str(s.cameraId);
    w.boolean(s.present);
    std::uint8_t mask = 0;
    if (s.pipelineIndex) mask |= kHasPipelineIndex;
    if (s.ledMode) mask |= kHasLedMode;
    if (s.imuMode) mask |= kHasImuMode;
    if (s.driverMode) mask |= kHasDriverMode;
    if (s.throttle) mask |= kHasThrottle;
    if (s.fpsLimit) mask |= kHasFpsLimit;
    if (s.robotOrientationDeg) mask |= kHasRobotOrientation;
    if (s.cameraPoseOverride) mask |= kHasCameraPoseOverride;
    w.u8(mask);
    if (s.pipelineIndex) w.i32(*s.pipelineIndex);
    if (s.ledMode) w.i32(*s.ledMode);
    if (s.imuMode) w.i32(*s.imuMode);
    if (s.driverMode) w.boolean(*s.driverMode);
    if (s.throttle) w.f64(*s.throttle);
    if (s.fpsLimit) w.f64(*s.fpsLimit);
    if (s.robotOrientationDeg)
        for (double v : *s.robotOrientationDeg) w.f64(v);
    if (s.cameraPoseOverride)
        for (double v : *s.cameraPoseOverride) w.f64(v);
    w.boolean(s.publishing);
    w.str(s.status);
    w.u64(s.framesPublished);
}

void read(Reader& r, CameraRobotState& s) {
    s.ntName = r.str();
    s.vendor = enumUpTo<CameraVendor>(r, 1, "invalid CameraVendor");
    s.cameraId = r.str();
    s.present = r.boolean();
    const std::uint8_t mask = r.u8();
    if (mask & kHasPipelineIndex) s.pipelineIndex = r.i32();
    if (mask & kHasLedMode) s.ledMode = r.i32();
    if (mask & kHasImuMode) s.imuMode = r.i32();
    if (mask & kHasDriverMode) s.driverMode = r.boolean();
    if (mask & kHasThrottle) s.throttle = r.f64();
    if (mask & kHasFpsLimit) s.fpsLimit = r.f64();
    if (mask & kHasRobotOrientation) {
        std::array<double, 6> v{};
        for (double& x : v) x = r.f64();
        s.robotOrientationDeg = v;
    }
    if (mask & kHasCameraPoseOverride) {
        std::array<double, 6> v{};
        for (double& x : v) x = r.f64();
        s.cameraPoseOverride = v;
    }
    s.publishing = r.boolean();
    s.status = r.str();
    s.framesPublished = r.u64();
}

void write(Writer& w, const std::vector<CameraRobotState>& states) {
    if (states.size() > kMaxVisionCameras) w.fail();
    w.varint(states.size());
    for (const CameraRobotState& s : states) write(w, s);
}

void read(Reader& r, std::vector<CameraRobotState>& states) {
    const std::size_t n = r.count(kMinCameraStateWireBytes, kMaxVisionCameras);
    states.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) read(r, states.emplace_back());
}

// ---- messages: base fields first, later fields in an optional tail ---------------------------------------------

void write(Writer& w, const Hello& m) {
    w.u32(m.protocolVersion);
    w.u8(static_cast<std::uint8_t>(m.language));
    w.i32(m.wpilibYear);
    w.str(m.wpilibVersion);
    w.str(m.extensionVersion);
    w.i64(m.pid);
    w.str(m.programName);
    w.varint(m.halPortCounts.size());
    for (const auto& [name, count] : m.halPortCounts) {
        w.str(name);
        w.i32(count);
    }
    w.u32(m.proposedWatchdogMs);
    w.boolean(m.supportsArmed);
}

void read(Reader& r, Hello& m) {
    m.protocolVersion = r.u32();
    m.language = static_cast<RobotLanguage>(r.u8());
    m.wpilibYear = r.i32();
    m.wpilibVersion = r.str();
    m.extensionVersion = r.str();
    m.pid = r.i64();
    m.programName = r.str();
    const std::size_t n = r.count(5, kMaxPortCounts);
    m.halPortCounts.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) {
        std::string name = r.str();
        const std::int32_t count = r.i32();
        m.halPortCounts.emplace_back(std::move(name), count);
    }
    if (!r.more()) return;
    m.proposedWatchdogMs = r.u32();
    if (!r.more()) return;
    m.supportsArmed = r.boolean();
}

void write(Writer& w, const HelloAck& m) {
    w.boolean(m.accepted);
    w.str(m.reason);
    w.u32(m.protocolVersion);
    w.u8(static_cast<std::uint8_t>(m.clockMode));
    w.u32(m.stepUs);
    w.u64(m.sessionId);
    w.boolean(m.discoveryOnly);
    w.boolean(m.enableL3Hooks);
    w.u64(m.runEpoch);
    w.u32(m.watchdogTimeoutMs);
    w.u8(static_cast<std::uint8_t>(m.watchdogPolicy));
    write(w, m.vision);
}

void read(Reader& r, HelloAck& m) {
    m.accepted = r.boolean();
    m.reason = r.str();
    m.protocolVersion = r.u32();
    m.clockMode = enumUpTo<ClockMode>(r, 1, "invalid ClockMode");
    m.stepUs = r.u32();
    m.sessionId = r.u64();
    m.discoveryOnly = r.boolean();
    m.enableL3Hooks = r.boolean();
    if (!r.more()) return;
    m.runEpoch = r.u64();
    m.watchdogTimeoutMs = r.u32();
    m.watchdogPolicy = static_cast<WatchdogPolicy>(r.u8());
    if (!r.more()) return;
    read(r, m.vision);
}

void write(Writer& w, const DeviceAnnounce& m) {
    w.u32(m.id.value);
    write(w, m.descriptor);
    w.varint(m.signalSpecs.size());
    for (const SignalSpec& s : m.signalSpecs) write(w, s);
    w.boolean(m.retired);
    w.str(m.observedBus);
}

void read(Reader& r, DeviceAnnounce& m) {
    m.id = DeviceId{r.u32()};
    read(r, m.descriptor);
    const std::size_t n = r.count(2, kMaxSignalSpecs);
    m.signalSpecs.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) read(r, m.signalSpecs.emplace_back());
    m.retired = r.boolean();
    if (!r.more()) return;
    m.observedBus = r.str();
}

void write(Writer& w, const LoopbackSet& m) {
    w.varint(m.loopback.size());
    for (DeviceId id : m.loopback) w.u32(id.value);
}

void read(Reader& r, LoopbackSet& m) {
    const std::size_t n = r.count(4, kMaxLoopbackIds);
    m.loopback.reserve(n);
    for (std::size_t i = 0; i < n && r.ok(); ++i) m.loopback.emplace_back(r.u32());
}

void write(Writer& w, const SensorFrame& m) {
    w.u64(m.tick);
    w.i64(m.simTimeUs);
    w.u32(m.stepUs);
    w.f64(m.batteryVoltageV);
    w.f64(m.batteryCurrentA);
    w.boolean(m.full);
    write(w, m.samples);
    w.u64(m.sessionId);
    w.u64(m.runEpoch);
    write(w, m.vision);
}

void read(Reader& r, SensorFrame& m) {
    m.tick = r.u64();
    m.simTimeUs = r.i64();
    m.stepUs = r.u32();
    m.batteryVoltageV = r.f64();
    m.batteryCurrentA = r.f64();
    m.full = r.boolean();
    read(r, m.samples);
    if (!r.more()) return;
    m.sessionId = r.u64();
    m.runEpoch = r.u64();
    if (!r.more()) return;
    read(r, m.vision);
}

void write(Writer& w, const ActuatorFrame& m) {
    w.u64(m.tick);
    w.i64(m.simTimeUs);
    w.u64(m.robotLoopCount);
    w.boolean(m.full);
    write(w, m.samples);
    w.u64(m.sessionId);
    w.u64(m.runEpoch);
    w.u64(m.ackTick);
    w.u8(static_cast<std::uint8_t>(m.result));
    write(w, m.vision);
}

void read(Reader& r, ActuatorFrame& m) {
    m.tick = r.u64();
    m.simTimeUs = r.i64();
    m.robotLoopCount = r.u64();
    m.full = r.boolean();
    read(r, m.samples);
    if (!r.more()) return;
    m.sessionId = r.u64();
    m.runEpoch = r.u64();
    m.ackTick = r.u64();
    m.result = static_cast<TickResult>(r.u8());
    if (!r.more()) return;
    read(r, m.vision);
}

void write(Writer& w, const DsState& m) {
    w.boolean(m.dsAttached);
    w.boolean(m.fmsAttached);
    w.boolean(m.enabled);
    w.boolean(m.eStop);
    w.u8(static_cast<std::uint8_t>(m.mode));
    w.u8(static_cast<std::uint8_t>(m.station));
    w.f64(m.matchTimeS);
    w.str(m.gameSpecificMessage);
    for (const JoystickState& j : m.joysticks) write(w, j);
    w.u64(m.sessionId);
    w.u64(m.runEpoch);
}

void read(Reader& r, DsState& m) {
    m.dsAttached = r.boolean();
    m.fmsAttached = r.boolean();
    m.enabled = r.boolean();
    m.eStop = r.boolean();
    m.mode = enumUpTo<RobotMode>(r, 2, "invalid RobotMode");
    m.station = enumUpTo<AllianceStation>(r, 5, "invalid AllianceStation");
    m.matchTimeS = r.f64();
    m.gameSpecificMessage = r.str();
    for (JoystickState& j : m.joysticks) read(r, j);
    if (!r.more()) return;
    m.sessionId = r.u64();
    m.runEpoch = r.u64();
}

void write(Writer& w, const SimControl& m) {
    w.u8(static_cast<std::uint8_t>(m.command));
    w.u32(m.stepCount);
    w.f64(m.timeScale);
    w.u64(m.commandId);
    w.u64(m.sessionId);
    w.u64(m.runEpoch);
}

void read(Reader& r, SimControl& m) {
    m.command = static_cast<SimCommand>(r.u8());  // unknown commands are answered with ControlResult::Unsupported
    m.stepCount = r.u32();
    m.timeScale = r.f64();
    if (!r.more()) return;
    m.commandId = r.u64();
    m.sessionId = r.u64();
    m.runEpoch = r.u64();
}

void write(Writer& w, const SimControlAck& m) {
    w.u64(m.commandId);
    w.u8(static_cast<std::uint8_t>(m.command));
    w.u8(static_cast<std::uint8_t>(m.result));
    w.u64(m.sessionId);
    w.u64(m.runEpoch);
    w.u64(m.tick);
    w.str(m.detail);
}

void read(Reader& r, SimControlAck& m) {
    m.commandId = r.u64();
    m.command = static_cast<SimCommand>(r.u8());
    m.result = static_cast<ControlResult>(r.u8());
    m.sessionId = r.u64();
    m.runEpoch = r.u64();
    m.tick = r.u64();
    m.detail = r.str();
}

void write(Writer& w, const Log& m) {
    w.i64(m.simTimeUs);
    w.u8(static_cast<std::uint8_t>(m.level));
    w.str(m.source);
    w.str(m.message);
}

void read(Reader& r, Log& m) {
    m.simTimeUs = r.i64();
    m.level = enumUpTo<LogLevel>(r, 4, "invalid LogLevel");
    m.source = r.str();
    m.message = r.str();
}

void write(Writer& w, const Goodbye& m) { w.str(m.reason); }
void read(Reader& r, Goodbye& m) { m.reason = r.str(); }

template <class T>
Result<Message> decodeBody(std::span<const std::byte> body) {
    ReadState state;
    Reader r(body, &state);
    T value;
    read(r, value);
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
        std::visit([&w](const auto& m) { write(w, m); }, message);
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
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        Result<bool> got = channel.receive(out, left.count() > 0 ? left : std::chrono::milliseconds{0});
        if (got || !isSkippableDecodeError(got.error())) return got;
        if (skipped != nullptr) ++*skipped;
    }
}

}  // namespace wpiemu::ipc
