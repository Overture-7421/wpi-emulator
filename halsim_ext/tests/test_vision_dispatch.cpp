#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <initializer_list>
#include <new>
#include <thread>

#include "fakes.h"
#include "wpiemu/halsim_ext/bridge_set.h"
#include "wpiemu/halsim_ext/sim_session.h"
#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::halsim;
using namespace wpiemu::halsim::testing;

// ---- allocation counter ---------------------------------------------------------------------
// Counts the heap allocations of the threads that opted in. The counters are thread-local, so the
// IPC and test threads never disturb a measurement on the clock thread. Replacing the global
// operator new affects the whole test executable; outside a counting window it costs one
// thread-local read per allocation.
namespace {
thread_local bool t_countAllocations = false;
thread_local std::size_t t_allocations = 0;
void* volatile g_heapSink = nullptr;
}  // namespace

void* operator new(std::size_t size) {
    if (t_countAllocations) ++t_allocations;
    if (void* p = std::malloc(size != 0 ? size : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept {
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}

namespace {

// Heap allocations of the calling thread while the scope is alive.
class AllocationScope {
public:
    AllocationScope() {
        t_allocations = 0;
        t_countAllocations = true;
    }
    ~AllocationScope() { t_countAllocations = false; }
    AllocationScope(const AllocationScope&) = delete;
    AllocationScope& operator=(const AllocationScope&) = delete;
    std::size_t count() const { return t_allocations; }
};

// A bridge whose hooks never allocate. It opens an allocation window in applyInputs (the first hook
// of a tick that has samples for it) and closes it in collectVision (the last), so a window spans
// everything the session does in between: applyVision, the step, poll, collectOutputs and the
// camera-state bookkeeping.
class AllocationMeter final : public sdk::IDeviceBridge {
public:
    static constexpr std::size_t kMaxWindows = 64;

    std::string_view name() const override { return "test.meter"; }
    sdk::Status start(sdk::IBridgeHost& h) override {
        host_ = &h;
        return sdk::okStatus();
    }
    void poll() override {
        if (device_.load() == 0) {
            device_ =
                host_->announce(FakeBridge::makeDescriptor("meter", 1), FakeBridge::makeSpecs())
                    .value;
        }
        if (allocateInPoll.load()) {  // positive control: the counter must see this
            char* p = new char[16];
            g_heapSink = p;
            delete[] p;
        }
    }
    void collectOutputs(std::vector<sdk::SignalSample>&) override {}
    void applyInputs(std::span<const sdk::SignalSample>) override {
        t_allocations = 0;
        t_countAllocations = true;
        open_ = true;
    }
    void applyLoopback(sdk::DeviceId) override {}
    bool owns(sdk::DeviceId id) const override { return id.value == device_.load(); }
    void stop() noexcept override {}
    void collectVision(std::vector<sdk::CameraRobotState>&, bool) override {
        t_countAllocations = false;
        if (!open_) return;
        open_ = false;
        const std::size_t slot = windows_.fetch_add(1);
        if (slot < kMaxWindows) counts_[slot] = t_allocations;
    }

    sdk::DeviceId device() const { return sdk::DeviceId(device_.load()); }
    // Allocations per closed window, oldest first.
    std::vector<std::size_t> windows() const {
        const std::size_t n = std::min<std::size_t>(windows_.load(), kMaxWindows);
        return {counts_.begin(), counts_.begin() + static_cast<std::ptrdiff_t>(n)};
    }

    std::atomic<bool> allocateInPoll{false};

private:
    sdk::IBridgeHost* host_ = nullptr;
    std::atomic<std::uint32_t> device_{0};
    bool open_ = false;  // clock thread only
    std::atomic<std::size_t> windows_{0};
    std::array<std::size_t, kMaxWindows> counts_{};
};

// A bridge that behaves like the contract of the vision hooks says (applyVision gets the frames,
// collectVision appends every camera when `full` and only the changed ones otherwise) and records
// every hook it was called with: name, the host's setup revision and the calling thread.
class VisionProbe final : public sdk::IDeviceBridge {
public:
    struct Event {
        std::string name;
        std::uint64_t revision = 0;  // IBridgeHost::visionSetupRevision() when the hook ran
        std::thread::id thread;
        bool programStarted = false;  // IBridgeHost::programStarted() when the hook ran
    };

    explicit VisionProbe(std::string bridgeName = "test.vision") : name_(std::move(bridgeName)) {}

    std::string_view name() const override { return name_; }

    sdk::Status start(sdk::IBridgeHost& h) override {
        host = &h;
        std::lock_guard lock(mutex_);
        startRevision_ = h.visionSetupRevision();
        startSetup_ = h.visionSetup();
        if (failStart) return sdk::makeError(sdk::ErrorCode::Internal, "scripted start failure");
        return sdk::okStatus();
    }
    void poll() override {
        note("poll");
        std::lock_guard lock(mutex_);
        if (!device_.valid()) {
            device_ =
                host->announce(FakeBridge::makeDescriptor("probe", 1), FakeBridge::makeSpecs());
        }
    }
    void collectOutputs(std::vector<sdk::SignalSample>&) override { note("collectOutputs"); }
    void applyInputs(std::span<const sdk::SignalSample>) override { note("applyInputs"); }
    void applyLoopback(sdk::DeviceId) override { note("applyLoopback"); }
    bool owns(sdk::DeviceId id) const override {
        std::lock_guard lock(mutex_);
        return id == device_;
    }
    void stop() noexcept override { ++stops; }
    void resetInputHistory() override { note("resetInputHistory"); }

    void applyVision(std::span<const sdk::VisionFrame> frames) override {
        {
            std::lock_guard lock(mutex_);
            applied_.emplace_back(frames.begin(), frames.end());
            appliedSetups_.push_back(host->visionSetup());
        }
        note("applyVision");
        if (logInApply) host->logger().warn("vision.test", "camera ignored: scripted");
        if (applyDelay.count() > 0) std::this_thread::sleep_for(applyDelay);
        std::unique_lock lock(mutex_);
        if (blockApply_) {
            blockApply_ = false;
            blocked_ = true;
            cv_.notify_all();
            cv_.wait(lock, [&] { return released_; });
        }
    }
    void collectVision(std::vector<sdk::CameraRobotState>& out, bool full) override {
        // Right after a poll the call is a report outside the tick path (program start, idle
        // polls); in a tick it follows collectOutputs. The report path only hands out cameras when
        // asked to.
        const bool report = lastHook() == "poll";
        note(report ? "reportVision" : "collectVision");
        std::lock_guard lock(mutex_);
        (report ? reportFlags_ : fullFlags_).push_back(full);
        if (report && !reportOutsideTicks) return;
        for (std::size_t i = 0; i < cameras_.size(); ++i)
            if (full || dirty_[i]) out.push_back(cameras_[i]);
        std::fill(dirty_.begin(), dirty_.end(), false);
        for (std::size_t i = 0; i < flood; ++i) {
            sdk::CameraRobotState extra;
            extra.ntName = "extra-" + std::to_string(i);
            out.push_back(std::move(extra));
        }
    }

    // ---- scripting and inspection (the test thread; hooks run on the clock thread) ----
    void note(std::string eventName) {
        const bool started = host != nullptr && host->programStarted();
        std::lock_guard lock(mutex_);
        events_.push_back({std::move(eventName), host != nullptr ? host->visionSetupRevision() : 0,
                           std::this_thread::get_id(), started});
    }
    std::string lastHook() const {
        std::lock_guard lock(mutex_);
        return events_.empty() ? std::string() : events_.back().name;
    }
    // The next applyVision blocks after recording itself until releaseApply(): a clock thread that
    // is stuck inside a bridge call.
    void blockNextApply() {
        std::lock_guard lock(mutex_);
        blockApply_ = true;
        released_ = false;
    }
    bool waitBlocked(std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex_);
        return cv_.wait_for(lock, timeout, [&] { return blocked_; });
    }
    void releaseApply() {
        std::lock_guard lock(mutex_);
        released_ = true;
        cv_.notify_all();
    }
    // Adds or replaces a camera and marks it changed.
    void setCamera(const sdk::CameraRobotState& state) {
        std::lock_guard lock(mutex_);
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            if (cameras_[i].ntName == state.ntName) {
                cameras_[i] = state;
                dirty_[i] = true;
                return;
            }
        }
        cameras_.push_back(state);
        dirty_.push_back(true);
    }
    sdk::DeviceId device() const {
        std::lock_guard lock(mutex_);
        return device_;
    }
    std::vector<Event> events() const {
        std::lock_guard lock(mutex_);
        return events_;
    }
    std::vector<std::string> names() const {
        std::vector<std::string> out;
        for (const Event& e : events()) out.push_back(e.name);
        return out;
    }
    std::ptrdiff_t count(const std::string& eventName) const {
        const std::vector<std::string> all = names();
        return std::count(all.begin(), all.end(), eventName);
    }
    std::vector<std::vector<sdk::VisionFrame>> applied() const {
        std::lock_guard lock(mutex_);
        return applied_;
    }
    std::vector<sdk::VisionSetup> appliedSetups() const {
        std::lock_guard lock(mutex_);
        return appliedSetups_;
    }
    std::vector<bool> fullFlags() const {
        std::lock_guard lock(mutex_);
        return fullFlags_;
    }
    // The `full` flag of every collectVision outside a tick (program start, idle polls).
    std::vector<bool> reportFlags() const {
        std::lock_guard lock(mutex_);
        return reportFlags_;
    }
    std::uint64_t startRevision() const {
        std::lock_guard lock(mutex_);
        return startRevision_;
    }
    sdk::VisionSetup startSetup() const {
        std::lock_guard lock(mutex_);
        return startSetup_;
    }

    sdk::IBridgeHost* host = nullptr;
    bool failStart = false;
    bool logInApply = false;
    std::chrono::milliseconds applyDelay{0};
    std::size_t flood = 0;  // extra, never-changing camera states appended by every collectVision
    // false: a collectVision right after a poll (not in a tick) returns nothing and keeps the
    // pending changes for the tick; true: it reports like the vision bridge does.
    std::atomic<bool> reportOutsideTicks{false};
    std::atomic<int> stops{0};

private:
    std::string name_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    sdk::DeviceId device_;
    std::vector<Event> events_;
    std::vector<std::vector<sdk::VisionFrame>> applied_;
    std::vector<sdk::VisionSetup> appliedSetups_;
    std::vector<bool> fullFlags_;
    std::vector<bool> reportFlags_;
    std::vector<sdk::CameraRobotState> cameras_;
    std::vector<bool> dirty_;
    std::uint64_t startRevision_ = 0;
    sdk::VisionSetup startSetup_;
    bool blockApply_ = false;
    bool blocked_ = false;
    bool released_ = false;
};

VisionProbe& addProbe(Rig& rig, std::string name = "test.vision") {
    auto probe = std::make_unique<VisionProbe>(std::move(name));
    VisionProbe& ref = *probe;
    rig.bridges.push_back(std::move(probe));
    return ref;
}

sdk::VisionFrame makeFrame(const std::string& camera, std::uint64_t sequence,
                           sdk::CameraVendor vendor = sdk::CameraVendor::PhotonVision) {
    sdk::VisionFrame f;
    f.cameraId = camera;
    f.ntName = camera;
    f.vendor = vendor;
    f.sequence = sequence;
    f.captureTimeUs = static_cast<std::int64_t>(sequence) * 5000;
    f.latencyUs = 25000;
    return f;
}

sdk::VisionSetup makeSetup(std::initializer_list<const char*> cameraNames) {
    sdk::VisionSetup setup;
    for (const char* cameraName : cameraNames) {
        sdk::CameraSetup c;
        c.cameraId = cameraName;
        c.ntName = cameraName;
        c.widthPx = 1280;
        c.heightPx = 800;
        c.fxPx = c.fyPx = 640.0;
        c.cxPx = 640.0;
        c.cyPx = 400.0;
        c.latencyUs = 25000;
        setup.cameras.push_back(c);
    }
    return setup;
}

sdk::CameraRobotState makeState(const std::string& camera, std::int32_t pipeline = 0) {
    sdk::CameraRobotState s;
    s.ntName = camera;
    s.cameraId = camera;
    s.pipelineIndex = pipeline;
    s.publishing = true;
    return s;
}

// The hook calls of the LAST tick: from `firstCall` up to its collectVision. Polls that the idle
// clock thread runs between two ticks cannot interleave with a tick, they only come before or
// after it.
std::vector<std::string> lastTickCalls(const std::vector<std::string>& names,
                                       const std::string& firstCall) {
    const auto fromEnd = std::find(names.rbegin(), names.rend(), firstCall);
    REQUIRE(fromEnd != names.rend());
    const auto first = names.begin() + (names.rend() - fromEnd - 1);
    const auto last = std::find(first, names.end(), "collectVision");
    REQUIRE(last != names.end());
    return {first, last + 1};
}

// Rig with one probe bridge and the robot program "started" (the probe's device is announced).
struct Started {
    explicit Started(proto::HelloAck ack = defaultAck(), ExtensionConfig cfg = Rig::defaultConfig())
        : rig(std::move(ack), std::move(cfg)), probe(addProbe(rig)) {
        REQUIRE(rig.start());
        rig.clock.started = true;
        REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    }
    Rig rig;
    VisionProbe& probe;
};

struct Fixture {
    DeviceTable table;
    sdk::NullLogger logger;
    FakeHalClock clock;
    std::atomic<bool> hooks{false};
};

proto::SimControl resetTo(std::uint64_t epoch, std::uint64_t commandId) {
    proto::SimControl reset;
    reset.command = proto::SimCommand::Reset;
    reset.commandId = commandId;
    reset.sessionId = 7;
    reset.runEpoch = epoch;
    return reset;
}

// A discovery-only (Armed) session: devices are announced, nothing is stepped, no ActuatorFrame.
proto::HelloAck armedAck() {
    proto::HelloAck ack = defaultAck();
    ack.discoveryOnly = true;
    return ack;
}

// Position of the first VisionReport / program_started signal in the emulator's message history.
std::ptrdiff_t historyIndex(const std::vector<int>& history, int type) {
    const auto it = std::find(history.begin(), history.end(), type);
    return it == history.end() ? -1 : it - history.begin();
}

}  // namespace

// ---- BridgeSet ----------------------------------------------------------------------------

TEST_CASE("bridge set: camera frames reach every bridge, camera states are appended") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto a = std::make_unique<VisionProbe>("test.a");
    auto b = std::make_unique<VisionProbe>("test.b");
    auto broken = std::make_unique<VisionProbe>("test.broken");
    VisionProbe* pa = a.get();
    VisionProbe* pb = b.get();
    VisionProbe* pbroken = broken.get();
    broken->failStart = true;
    bridges.push_back(std::move(a));
    bridges.push_back(std::move(broken));
    bridges.push_back(std::move(b));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);
    REQUIRE(set.activeCount() == 2);

    const std::vector<sdk::VisionFrame> frames{
        makeFrame("photon-front", 1), makeFrame("limelight-rear", 1, sdk::CameraVendor::Limelight)};
    set.applyVision(frames);
    REQUIRE(pa->applied().size() == 1);
    CHECK(pa->applied()[0] == frames);
    REQUIRE(pb->applied().size() == 1);
    CHECK(pb->applied()[0] == frames);
    CHECK(pbroken->applied().empty());  // a bridge that failed to start is never called

    pa->setCamera(makeState("photon-front"));
    pb->setCamera(makeState("limelight-rear"));
    std::vector<sdk::CameraRobotState> out{makeState("kept")};
    set.collectVision(out, true);
    REQUIRE(out.size() == 3);
    CHECK(out[0].ntName == "kept");  // bridges append, the set never clears
    CHECK(out[1].ntName == "photon-front");
    CHECK(out[2].ntName == "limelight-rear");

    set.collectVision(out, false);  // nothing changed since: nothing is added
    CHECK(out.size() == 3);
}

TEST_CASE("bridge set: without camera frames no bridge is called") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto probe = std::make_unique<VisionProbe>();
    VisionProbe* p = probe.get();
    bridges.push_back(std::move(probe));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);

    set.applyVision(std::span<const sdk::VisionFrame>{});
    CHECK(p->count("applyVision") == 0);

    std::vector<sdk::CameraRobotState> out;
    // Asked every tick: the bridge reports cameras nobody configured.
    set.collectVision(out, false);
    CHECK(p->count("collectVision") == 1);
    CHECK(out.empty());
}

TEST_CASE("bridge set: the camera fan-outs allocate nothing") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    bridges.push_back(std::make_unique<FakeBridge>("fake.a"));  // no vision hooks: the SDK defaults
    bridges.push_back(std::make_unique<FakeBridge>("fake.b"));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);
    std::vector<sdk::CameraRobotState> out;
    const std::vector<sdk::VisionFrame> frames{makeFrame("photon-front", 1)};

    std::size_t allocations = 0;
    {
        AllocationScope scope;
        for (int i = 0; i < 100; ++i) {
            set.applyVision(std::span<const sdk::VisionFrame>{});
            set.applyVision(frames);  // handed on by reference, never copied
            set.collectVision(out, i % 10 == 0);
        }
        allocations = scope.count();
    }
    CHECK(allocations == 0);
    CHECK(out.empty());
}

TEST_CASE("bridge host: visionSetup and its revision follow every adopted HelloAck setup") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto probe = std::make_unique<VisionProbe>();
    VisionProbe* p = probe.get();
    bridges.push_back(std::move(probe));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);
    const sdk::IBridgeHost& host = *p->host;

    CHECK(host.visionSetup().empty());
    CHECK(host.visionSetupRevision() == 0);
    set.adoptVisionSetup(makeSetup({"photon-front"}));
    CHECK(host.visionSetup() == makeSetup({"photon-front"}));
    CHECK(host.visionSetupRevision() == 1);
    // An equal setup still counts as an adoption.
    set.adoptVisionSetup(makeSetup({"photon-front"}));
    CHECK(host.visionSetupRevision() == 2);
    set.adoptVisionSetup(sdk::VisionSetup{});
    CHECK(host.visionSetup().empty());
    CHECK(host.visionSetupRevision() == 3);
}

TEST_CASE("bridge host: programStarted reports the HAL's program-start flag") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto probe = std::make_unique<VisionProbe>();
    VisionProbe* p = probe.get();
    bridges.push_back(std::move(probe));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);

    CHECK_FALSE(p->host->programStarted());
    fx.clock.started = true;
    CHECK(p->host->programStarted());
}

TEST_CASE("bridge set: a closed set calls no hook any more, stop() still reaches the bridges") {
    Fixture fx;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto probe = std::make_unique<VisionProbe>();
    VisionProbe* p = probe.get();
    bridges.push_back(std::move(probe));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    set.start(fx.hooks);
    set.poll();  // announces the probe's device
    const std::vector<sdk::SignalSample> samples{{p->device(), 1, 1.0}};
    const std::vector<sdk::VisionFrame> frames{makeFrame("photon-front", 1)};
    std::vector<sdk::SignalSample> outputs;
    std::vector<sdk::CameraRobotState> states;

    set.close();
    CHECK(set.closed());
    const std::size_t calls = p->events().size();
    set.poll();
    set.resetInputHistory();
    set.applyInputs(samples);
    set.applyVision(frames);
    set.collectOutputs(outputs);
    set.collectVision(states, true);
    set.applyLoopback(std::vector<sdk::DeviceId>{p->device()});
    CHECK(p->events().size() == calls);
    CHECK(p->applied().empty());

    set.stop();
    CHECK(p->stops == 1);
}

namespace {

// Closes its set from inside a hook once armed, like the HAL shutdown hook does while the clock thread is inside this call.
class ClosingBridge final : public sdk::IDeviceBridge {
public:
    explicit ClosingBridge(BridgeSet*& set) : set_(set) {}

    std::string_view name() const override { return "test.closing"; }
    sdk::Status start(sdk::IBridgeHost& h) override {
        host_ = &h;
        return sdk::okStatus();
    }
    void poll() override {
        if (!device_.valid()) device_ = host_->announce(FakeBridge::makeDescriptor("closing", 1), FakeBridge::makeSpecs());
        closeIfArmed();
    }
    void collectOutputs(std::vector<sdk::SignalSample>&) override { closeIfArmed(); }
    void applyInputs(std::span<const sdk::SignalSample>) override { closeIfArmed(); }
    void applyLoopback(sdk::DeviceId) override { closeIfArmed(); }
    bool owns(sdk::DeviceId id) const override { return id == device_; }
    void stop() noexcept override {}
    void resetInputHistory() override { closeIfArmed(); }
    void applyVision(std::span<const sdk::VisionFrame>) override { closeIfArmed(); }
    void collectVision(std::vector<sdk::CameraRobotState>&, bool) override { closeIfArmed(); }

    sdk::DeviceId device() const { return device_; }
    void arm() { armed_ = true; }

private:
    void closeIfArmed() {
        if (armed_ && set_ != nullptr) set_->close();
    }

    BridgeSet*& set_;
    sdk::IBridgeHost* host_ = nullptr;
    sdk::DeviceId device_;
    bool armed_ = false;
};

// One set per hook (closing is permanent): [ClosingBridge, VisionProbe]; the probe must see nothing of the hook that closes the set.
template <class Hook>
void checkLaterBridgeStaysUncalled(const char* what, Hook&& hook) {
    Fixture fx;
    BridgeSet* setPtr = nullptr;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    auto closing = std::make_unique<ClosingBridge>(setPtr);
    auto later = std::make_unique<VisionProbe>("test.later");
    ClosingBridge* c = closing.get();
    VisionProbe* p = later.get();
    bridges.push_back(std::move(closing));
    bridges.push_back(std::move(later));
    BridgeSet set(std::move(bridges), fx.table, fx.logger, fx.clock);
    setPtr = &set;
    set.start(fx.hooks);
    set.poll();  // announces both devices
    REQUIRE(c->device().valid());
    REQUIRE(p->device().valid());
    const std::size_t before = p->events().size();

    c->arm();
    hook(set, c->device(), p->device());
    CHECK_MESSAGE(set.closed(), what);
    CHECK_MESSAGE(p->events().size() == before, what);
}

}  // namespace

TEST_CASE("bridge set: closing the set inside one bridge's hook keeps the bridges after it from being called") {
    const std::vector<sdk::VisionFrame> frames{makeFrame("photon-front", 1)};
    checkLaterBridgeStaysUncalled("poll", [](BridgeSet& s, sdk::DeviceId, sdk::DeviceId) { s.poll(); });
    checkLaterBridgeStaysUncalled("resetInputHistory", [](BridgeSet& s, sdk::DeviceId, sdk::DeviceId) { s.resetInputHistory(); });
    checkLaterBridgeStaysUncalled("applyInputs", [](BridgeSet& s, sdk::DeviceId a, sdk::DeviceId b) {
        const std::vector<sdk::SignalSample> samples{{a, 1, 1.0}, {b, 1, 1.0}};
        s.applyInputs(samples);
    });
    checkLaterBridgeStaysUncalled("collectOutputs", [](BridgeSet& s, sdk::DeviceId, sdk::DeviceId) {
        std::vector<sdk::SignalSample> out;
        s.collectOutputs(out);
    });
    checkLaterBridgeStaysUncalled("applyVision", [&frames](BridgeSet& s, sdk::DeviceId, sdk::DeviceId) { s.applyVision(frames); });
    checkLaterBridgeStaysUncalled("collectVision", [](BridgeSet& s, sdk::DeviceId, sdk::DeviceId) {
        std::vector<sdk::CameraRobotState> out;
        s.collectVision(out, true);
    });
    checkLaterBridgeStaysUncalled("applyLoopback", [](BridgeSet& s, sdk::DeviceId a, sdk::DeviceId b) {
        s.applyLoopback(std::vector<sdk::DeviceId>{a, b});
    });
}

TEST_CASE("session wire limit of camera states matches the binary codec") {
    CHECK(SimSession::kMaxVisionStates == ipc::kMaxVisionCameras);
}

// ---- order and acceptance -----------------------------------------------------------------

TEST_CASE("vision dispatch: applyInputs, applyVision, step, poll, collectOutputs, collectVision") {
    Started t;
    t.rig.clock.onStep = [&](std::uint64_t) { t.probe.note("step"); };
    const std::vector<std::string> expected{"applyInputs", "applyVision",    "step",
                                            "poll",        "collectOutputs", "collectVision"};

    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto ack = t.rig.emulator->tick({{t.probe.device(), 1, 2.5}});
    REQUIRE(ack);
    CHECK(ack->result == proto::TickResult::Completed);
    CHECK(lastTickCalls(t.probe.names(), "applyInputs") == expected);
    REQUIRE(t.probe.applied().size() == 1);
    CHECK(t.probe.applied()[0] == std::vector<sdk::VisionFrame>{makeFrame("photon-front", 1)});

    // A frame without samples for this bridge still applies its camera frames, in the same place.
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2),
                                  makeFrame("limelight-rear", 1, sdk::CameraVendor::Limelight)};
    REQUIRE(t.rig.emulator->tick());
    CHECK(
        lastTickCalls(t.probe.names(), "applyVision") ==
        std::vector<std::string>{"applyVision", "step", "poll", "collectOutputs", "collectVision"});
    REQUIRE(t.probe.applied().size() == 2);
    CHECK(t.probe.applied()[1].size() == 2);  // one call with both frames, in order
    CHECK(t.probe.applied()[1][0].cameraId == "photon-front");
    CHECK(t.probe.applied()[1][1].cameraId == "limelight-rear");
}

TEST_CASE("vision dispatch: only frames the lock-step follower accepts are applied") {
    Started t;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(t.probe.applied().size() == 1);

    // Duplicate tick: a protocol error, nothing is stepped and its vision is ignored.
    proto::SensorFrame dup;
    dup.tick = 1;
    dup.sessionId = 7;
    dup.runEpoch = 1;
    dup.vision = {makeFrame("photon-front", 99)};
    REQUIRE(t.rig.emulator->send(proto::Message(dup)));
    const auto err = t.rig.emulator->take<proto::ActuatorFrame>(1000ms);
    REQUIRE(err);
    CHECK(err->result == proto::TickResult::ProtocolError);
    CHECK(err->vision.empty());

    // Foreign session: dropped silently.
    proto::SensorFrame foreign;
    foreign.tick = 2;
    foreign.sessionId = 99;
    foreign.runEpoch = 1;
    foreign.vision = {makeFrame("photon-front", 98)};
    REQUIRE(t.rig.emulator->send(proto::Message(foreign)));

    // A Reset opens epoch 2; a late frame of epoch 1 is stale and dropped, then a current one is
    // applied.
    REQUIRE(t.rig.emulator->send(proto::Message(resetTo(2, 5))));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    proto::SensorFrame stale;
    stale.tick = 2;
    stale.sessionId = 7;
    stale.runEpoch = 1;
    stale.vision = {makeFrame("photon-front", 97)};
    REQUIRE(t.rig.emulator->send(proto::Message(stale)));
    t.rig.emulator->epoch = 2;
    t.rig.emulator->nextTick = 1;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2)};
    REQUIRE(t.rig.emulator->tick());  // answered after every frame above was handled

    const auto applied = t.probe.applied();
    REQUIRE(applied.size() == 2);
    CHECK(applied[0][0].sequence == 1);
    CHECK(applied[1][0].sequence == 2);
    const RuntimeStats stats = t.rig.runtime->stats();
    CHECK(stats.visionFramesApplied == 2);
    CHECK(stats.visionApplyCalls == 2);
    CHECK(stats.framesDropped == 2);
    CHECK(stats.protocolErrors == 1);
}

TEST_CASE("vision dispatch: a frame of a newer epoch is adopted and its vision applied") {
    Started t;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    REQUIRE(t.rig.emulator->tick());

    t.rig.emulator->epoch = 2;  // no Reset: the follower adopts the epoch from the frame
    t.rig.emulator->nextTick = 1;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2)};
    const auto ack = t.rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->runEpoch == 2);
    CHECK(t.probe.applied().size() == 2);
    CHECK(t.probe.count("resetInputHistory") == 2);  // the connection and the epoch change
}

TEST_CASE("vision dispatch: a frame held before the program starts keeps its vision") {
    Rig rig;
    VisionProbe& probe = addProbe(rig);
    REQUIRE(rig.start());

    proto::SensorFrame early;
    early.tick = 1;
    early.simTimeUs = 5000;
    early.sessionId = 7;
    early.runEpoch = 1;
    early.vision = {makeFrame("photon-front", 7)};
    REQUIRE(rig.emulator->send(proto::Message(early)));
    CHECK_FALSE(rig.emulator->take<proto::ActuatorFrame>(150ms).has_value());
    CHECK(probe.count("applyVision") == 0);  // held, neither stepped nor applied

    // Only one frame can be held: a second one is a protocol error and its vision is ignored.
    proto::SensorFrame second = early;
    second.tick = 2;
    second.vision = {makeFrame("photon-front", 8)};
    REQUIRE(rig.emulator->send(proto::Message(second)));
    const auto refused = rig.emulator->take<proto::ActuatorFrame>(1000ms);
    REQUIRE(refused);
    CHECK(refused->result == proto::TickResult::ProtocolError);
    CHECK(refused->vision.empty());
    CHECK(probe.count("applyVision") == 0);

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto ack = rig.emulator->take<proto::ActuatorFrame>(2000ms);
    REQUIRE(ack);
    CHECK(ack->result == proto::TickResult::Completed);
    REQUIRE(probe.applied().size() == 1);
    REQUIRE(probe.applied()[0].size() == 1);
    CHECK(probe.applied()[0][0].sequence == 7);
}

TEST_CASE("vision dispatch: a free-running session applies camera frames without a step") {
    proto::HelloAck ack = defaultAck();
    ack.clockMode = proto::ClockMode::Realtime;
    Started t(ack);
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto answer = t.rig.emulator->tick();
    REQUIRE(answer);
    CHECK(answer->result == proto::TickResult::Completed);
    CHECK(t.rig.clock.steps == 0);
    REQUIRE(t.probe.applied().size() == 1);
    CHECK(lastTickCalls(t.probe.names(), "applyVision") ==
          std::vector<std::string>{"applyVision", "poll", "collectOutputs", "collectVision"});
}

// ---- camera state: full list or changes ---------------------------------------------------

TEST_CASE("vision dispatch: collectVision full flag (first frame, Reset, epoch, interval)") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.fullFrameInterval = 4;
    Started t(defaultAck(), cfg);
    t.probe.setCamera(makeState("photon-front"));
    t.probe.setCamera(makeState("limelight-rear"));

    const auto first = t.rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->full);
    CHECK(first->vision.size() == 2);

    const auto quiet = t.rig.emulator->tick();
    REQUIRE(quiet);
    CHECK(quiet->vision.empty());  // nothing changed

    t.probe.setCamera(makeState("photon-front", 3));
    const auto changed = t.rig.emulator->tick();
    REQUIRE(changed);
    REQUIRE(changed->vision.size() == 1);
    CHECK(changed->vision[0].ntName == "photon-front");
    CHECK(changed->vision[0].pipelineIndex == 3);

    REQUIRE(t.rig.emulator->tick());
    // The 5th frame is a full one (fullFrameInterval 4).
    const auto periodic = t.rig.emulator->tick();
    REQUIRE(periodic);
    CHECK(periodic->full);
    CHECK(periodic->vision.size() == 2);

    t.rig.emulator->epoch = 2;
    t.rig.emulator->nextTick = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(resetTo(2, 9))));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    const auto afterReset = t.rig.emulator->tick();
    REQUIRE(afterReset);
    CHECK(afterReset->full);
    CHECK(afterReset->vision.size() == 2);
    const auto quietAgain = t.rig.emulator->tick();
    REQUIRE(quietAgain);
    CHECK(quietAgain->vision.empty());

    t.rig.emulator->epoch = 3;  // adopted from the frame, no Reset
    t.rig.emulator->nextTick = 1;
    const auto newEpoch = t.rig.emulator->tick();
    REQUIRE(newEpoch);
    CHECK(newEpoch->vision.size() == 2);

    CHECK(t.probe.fullFlags() ==
          std::vector<bool>{true, false, false, false, true, true, false, true});
}

TEST_CASE("vision dispatch: a reconnect brings its setup, a new revision and a full camera list") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.reconnect = true;
    proto::HelloAck firstAck = defaultAck();
    firstAck.vision = makeSetup({"photon-front"});
    Rig rig(firstAck, cfg);
    VisionProbe& probe = addProbe(rig);
    probe.setCamera(makeState("photon-front"));
    REQUIRE(rig.start());

    // The setup of the first HelloAck is in place before the bridge starts.
    CHECK(probe.startRevision() == 1);
    CHECK(probe.startSetup() == firstAck.vision);

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto before = rig.emulator->tick();
    REQUIRE(before);
    CHECK(before->vision.size() == 1);
    REQUIRE(probe.appliedSetups().size() == 1);
    CHECK(probe.appliedSetups()[0] == firstAck.vision);

    proto::HelloAck secondAck = defaultAck();
    secondAck.sessionId = 8;
    secondAck.runEpoch = 5;
    secondAck.vision = makeSetup({"photon-front", "limelight-rear"});
    std::unique_ptr<FakeEmulator> old = std::move(rig.emulator);
    rig.connectNext(secondAck);
    old->disconnect();
    REQUIRE(waitUntil([&] { return rig.runtime->stats().sessions == 2; }, 3000ms));
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    rig.emulator->nextVision = {makeFrame("limelight-rear", 1, sdk::CameraVendor::Limelight)};
    const auto after = rig.emulator->tick();
    REQUIRE(after);
    CHECK(after->sessionId == 8);
    CHECK(after->full);
    CHECK(after->vision.size() == 1);  // the full list again, although nothing changed

    REQUIRE(probe.appliedSetups().size() == 2);
    CHECK(probe.appliedSetups()[1] == secondAck.vision);
    // The new revision was visible to the bridge before the new session did anything, in particular
    // before it polled and before it applied the first frame.
    const std::vector<VisionProbe::Event> events = probe.events();
    const auto firstOfNewSession = std::find_if(
        events.begin(), events.end(), [&](const VisionProbe::Event& e) { return e.revision == 2; });
    REQUIRE(firstOfNewSession != events.end());
    CHECK(firstOfNewSession->name == "resetInputHistory");
    const auto newApply = std::find_if(
        events.begin(), events.end(),
        [](const VisionProbe::Event& e) { return e.name == "applyVision" && e.revision == 2; });
    REQUIRE(newApply != events.end());
    const auto pollBefore = std::find_if(events.begin(), newApply, [](const VisionProbe::Event& e) {
        return e.name == "poll" && e.revision == 2;
    });
    CHECK(pollBefore != newApply);
    CHECK(probe.fullFlags() == std::vector<bool>{true, true});
}

TEST_CASE("vision dispatch: a HelloAck without cameras is an empty setup and still an adoption") {
    Started t;
    CHECK(t.probe.startSetup().empty());
    CHECK(t.probe.startRevision() == 1);
}

// ---- VisionReport: the camera state outside the tick path ----------------------------------

TEST_CASE("vision report: an armed session reports what the bridge found, before program_started") {
    Rig rig(armedAck());
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    probe.setCamera(makeState("limelight-rear", 2));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    const auto report = rig.emulator->take<proto::VisionReport>(1000ms);
    REQUIRE(report);
    REQUIRE(report->cameras.size() == 2);
    CHECK(report->cameras[0].ntName == "photon-front");
    CHECK(report->cameras[1].ntName == "limelight-rear");
    CHECK(report->cameras[1].pipelineIndex == 2);
    // Reported like the devices: before the program_started signal.
    const std::vector<int> history = rig.emulator->history();
    const std::ptrdiff_t reported =
        historyIndex(history, static_cast<int>(proto::MessageType::VisionReport));
    const std::ptrdiff_t started = historyIndex(history, FakeEmulator::kHistoryLifecycle);
    CHECK(reported >= 0);
    CHECK(reported < started);

    // Once: the bridge is asked again after every poll, but nothing changed since.
    REQUIRE(waitUntil([&] { return probe.reportFlags().size() >= 5; }));
    const std::vector<bool> flags = probe.reportFlags();
    CHECK(flags.front());  // the first call of the connection lists every camera
    CHECK(std::count(flags.begin(), flags.end(), true) == 1);
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);
    CHECK(rig.emulator->count<proto::ActuatorFrame>() == 0);
}

TEST_CASE("vision report: nothing is sent while no camera was found") {
    Rig rig(armedAck());
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    REQUIRE(waitUntil([&] { return probe.reportFlags().size() >= 5; }));  // asked, but empty-handed
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);
}

TEST_CASE("vision report: a camera that appears or changes later is reported on its own") {
    Rig rig(armedAck());
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->take<proto::VisionReport>(1000ms));  // the full one

    probe.setCamera(makeState("photon-front", 3));
    const auto changed = rig.emulator->take<proto::VisionReport>(2000ms);
    REQUIRE(changed);
    REQUIRE(changed->cameras.size() == 1);
    CHECK(changed->cameras[0].ntName == "photon-front");
    CHECK(changed->cameras[0].pipelineIndex == 3);

    probe.setCamera(makeState("limelight-rear"));  // a camera the robot code started to use
    const auto added = rig.emulator->take<proto::VisionReport>(2000ms);
    REQUIRE(added);
    REQUIRE(added->cameras.size() == 1);
    CHECK(added->cameras[0].ntName == "limelight-rear");

    const std::vector<bool> flags = probe.reportFlags();
    CHECK(std::count(flags.begin(), flags.end(), true) == 1);  // only the first call was full
    std::this_thread::sleep_for(100ms);
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);
}

TEST_CASE("vision report: nothing is reported while the robot constructor runs") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.pollWhileConstructing = true;  // WPIEMU_LIVE_DISCOVERY=1
    Rig rig(armedAck(), cfg);
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    REQUIRE(rig.start());
    REQUIRE(waitUntil([&] { return probe.count("poll") >= 3; }));
    CHECK(probe.count("reportVision") == 0);
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto report = rig.emulator->take<proto::VisionReport>(1000ms);
    REQUIRE(report);
    CHECK(report->cameras.size() == 1);
}

TEST_CASE("vision report: lock-step ticks carry the camera state in the ActuatorFrame") {
    Rig rig;
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->take<proto::VisionReport>(1000ms));  // program start, as in any session

    // The first ActuatorFrame lists every camera again, whatever was reported before.
    const auto first = rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->vision.size() == 1);

    // A change made by the robot during the step rides that tick's ActuatorFrame ...
    rig.clock.onStep = [&](std::uint64_t) { probe.setCamera(makeState("photon-front", 3)); };
    const auto changed = rig.emulator->tick();
    rig.clock.onStep = nullptr;
    REQUIRE(changed);
    REQUIRE(changed->vision.size() == 1);
    CHECK(changed->vision[0].pipelineIndex == 3);
    const auto quiet = rig.emulator->tick();
    REQUIRE(quiet);
    CHECK(quiet->vision.empty());

    // ... and is not reported a second time: no VisionReport came out of the ticks.
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);
}

TEST_CASE("vision report: the first ActuatorFrame after BeginLockstep lists every camera again") {
    Rig rig(armedAck());
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    probe.setCamera(makeState("limelight-rear"));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    const auto report = rig.emulator->take<proto::VisionReport>(1000ms);
    REQUIRE(report);
    CHECK(report->cameras.size() == 2);

    rig.emulator->epoch = 2;
    REQUIRE(rig.emulator->sendControl(proto::SimCommand::BeginLockstep, 1));
    const auto begin = rig.emulator->take<proto::SimControlAck>(1000ms);
    REQUIRE(begin);
    CHECK(begin->result == proto::ControlResult::Applied);

    const auto first = rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->full);
    CHECK(first->vision.size() == 2);
    const auto quiet = rig.emulator->tick();
    REQUIRE(quiet);
    CHECK(quiet->vision.empty());
    CHECK(rig.emulator->count<proto::VisionReport>() == 0);
}

TEST_CASE("vision report: a reconnect starts with a full report again") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.reconnect = true;
    Rig rig(armedAck(), cfg);
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.setCamera(makeState("photon-front"));
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->take<proto::VisionReport>(1000ms));

    proto::HelloAck second = armedAck();
    second.sessionId = 8;
    second.runEpoch = 5;
    std::unique_ptr<FakeEmulator> old = std::move(rig.emulator);
    rig.connectNext(second);
    old->disconnect();
    REQUIRE(waitUntil([&] { return rig.runtime->stats().sessions == 2; }, 3000ms));
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    const auto again = rig.emulator->take<proto::VisionReport>(1000ms);
    REQUIRE(again);
    CHECK(again->cameras.size() == 1);  // everything again, although nothing changed
    const std::vector<bool> flags = probe.reportFlags();
    CHECK(std::count(flags.begin(), flags.end(), true) == 2);  // the first call of each connection
}

TEST_CASE("vision report: a stalled lock-step session keeps reporting") {
    Rig rig;
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->tick());

    // 400 ms watchdog: the robot is unfrozen and no ActuatorFrame follows.
    rig.clock.blockNextStep = true;
    const auto stalled = rig.emulator->tick({}, 5000, 12.0, 3000ms);
    REQUIRE(stalled);
    CHECK(stalled->result == proto::TickResult::RobotStalled);

    // The robot, now running free, starts to use a camera.
    probe.setCamera(makeState("photon-front"));
    const auto report = rig.emulator->take<proto::VisionReport>(2000ms);
    REQUIRE(report);
    CHECK(report->cameras.size() == 1);
}

TEST_CASE("vision report: more camera states than the wire allows are cut and reported once") {
    Rig rig(armedAck());
    VisionProbe& probe = addProbe(rig);
    probe.reportOutsideTicks = true;
    probe.flood = SimSession::kMaxVisionStates + 44;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    const auto report = rig.emulator->take<proto::VisionReport>(1000ms);
    REQUIRE(report);
    CHECK(report->cameras.size() == SimSession::kMaxVisionStates);
    const auto isWarning = [](const proto::Log& l) {
        return l.message.find("camera states") != std::string::npos;
    };
    REQUIRE(rig.emulator->take<proto::Log>(1000ms, isWarning));
    CHECK_FALSE(rig.emulator->take<proto::Log>(100ms, isWarning).has_value());  // only once
}

// ---- cost without vision ------------------------------------------------------------------

TEST_CASE("vision dispatch: without frames and cameras nothing is applied, counted or sent") {
    Started t;
    for (int i = 0; i < 5; ++i) {
        const auto ack = t.rig.emulator->tick({{t.probe.device(), 1, 1.0 * i}});
        REQUIRE(ack);
        CHECK(ack->result == proto::TickResult::Completed);
        CHECK(ack->vision.empty());
    }
    CHECK(t.probe.count("applyVision") == 0);
    // Cheap: it is how the bridge reports cameras nobody configured.
    CHECK(t.probe.count("collectVision") == 5);
    const RuntimeStats stats = t.rig.runtime->stats();
    CHECK(stats.ticksCompleted == 5);
    CHECK(stats.visionFramesApplied == 0);
    CHECK(stats.visionApplyCalls == 0);
    CHECK(stats.visionApplyTotalUs == 0);
    CHECK(stats.visionApplyMaxUs == 0);
}

TEST_CASE("vision dispatch: an idle tick allocates nothing, with or without camera frames") {
    Rig rig;
    auto owner = std::make_unique<AllocationMeter>();
    AllocationMeter& meter = *owner;
    rig.bridges.push_back(std::move(owner));
    rig.clock.stepLog.reserve(256);  // the fake clock's bookkeeping must not show up in the window
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    constexpr std::size_t kTicks = 12;
    for (std::size_t i = 0; i < kTicks; ++i) {
        if (i % 2 == 1) rig.emulator->nextVision = {makeFrame("photon-front", i)};
        REQUIRE(rig.emulator->tick({{meter.device(), 1, static_cast<double>(i)}}));
    }
    const std::vector<std::size_t> windows = meter.windows();
    REQUIRE(windows.size() == kTicks);
    for (std::size_t i = 2; i < kTicks; ++i) CHECK(windows[i] == 0);  // the first two ticks warm up

    // The counter works: an allocation made inside a window is seen.
    meter.allocateInPoll = true;
    REQUIRE(rig.emulator->tick({{meter.device(), 1, 0.0}}));
    REQUIRE(meter.windows().size() == kTicks + 1);
    CHECK(meter.windows().back() >= 1);
}

TEST_CASE("vision dispatch: the counters add up frames, fan-outs and the time spent in them") {
    Started t;
    t.probe.applyDelay = 20ms;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1),
                                  makeFrame("limelight-rear", 1, sdk::CameraVendor::Limelight)};
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(t.rig.emulator->tick());  // no frames: not counted
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2)};
    REQUIRE(t.rig.emulator->tick());

    const RuntimeStats stats = t.rig.runtime->stats();
    CHECK(stats.visionFramesApplied == 3);
    CHECK(stats.visionApplyCalls == 2);
    CHECK(stats.visionApplyMaxUs >= 15000);  // each fan-out slept 20 ms
    CHECK(stats.visionApplyTotalUs >= 2 * 15000);
    CHECK(stats.visionApplyTotalUs >= stats.visionApplyMaxUs);
    CHECK(stats.ticksCompleted == 3);
}

TEST_CASE("runtime stats: the vision counters accumulate and keep the slowest fan-out") {
    AtomicRuntimeStats stats;
    CHECK(stats.snapshot().visionApplyCalls == 0);
    stats.recordVisionApply(2, 300);
    stats.recordVisionApply(1, 900);
    stats.recordVisionApply(3, 100);
    const RuntimeStats s = stats.snapshot();
    CHECK(s.visionFramesApplied == 6);
    CHECK(s.visionApplyCalls == 3);
    CHECK(s.visionApplyTotalUs == 1300);
    CHECK(s.visionApplyMaxUs == 900);
}

// ---- robustness ---------------------------------------------------------------------------

TEST_CASE("vision dispatch: a bridge that failed to start never sees vision, the others do") {
    Rig rig;
    VisionProbe& broken = addProbe(rig, "test.broken");
    VisionProbe& good = addProbe(rig, "test.good");
    broken.failStart = true;
    REQUIRE(rig.start());
    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto ack = rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->result == proto::TickResult::Completed);
    CHECK(broken.count("applyVision") == 0);
    CHECK(broken.count("collectVision") == 0);
    CHECK(good.count("applyVision") == 1);
    CHECK(good.count("collectVision") == 1);
}

TEST_CASE("vision dispatch: a bridge that logs a problem does not disturb the tick") {
    Started t;
    t.probe.logInApply = true;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto ack = t.rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->result == proto::TickResult::Completed);
    const auto log = t.rig.emulator->take<proto::Log>(
        1000ms, [](const proto::Log& l) { return l.source == "vision.test"; });
    REQUIRE(log);
    CHECK(log->level == sdk::LogLevel::Warn);
    CHECK(t.rig.logContains("camera ignored"));
}

TEST_CASE("vision dispatch: a slow applyVision within the watchdog budget is accounted") {
    Started t;  // 400 ms watchdog
    t.probe.applyDelay = 60ms;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto ack = t.rig.emulator->tick();
    REQUIRE(ack);
    CHECK(ack->result == proto::TickResult::Completed);
    const RuntimeStats stats = t.rig.runtime->stats();
    CHECK(stats.visionApplyMaxUs >= 50000);
    CHECK(stats.stalls == 0);
    CHECK(stats.emulatorLost == 0);
    CHECK(t.rig.clock.paused);
    t.probe.applyDelay = 0ms;
    CHECK(t.rig.emulator->tick().has_value());
}

TEST_CASE("vision dispatch: an applyVision slower than the watchdog is like any slow bridge") {
    Started t;  // 400 ms watchdog
    t.probe.applyDelay = 700ms;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    CHECK(t.rig.emulator->tick({}, 5000, 12.0, 3000ms).has_value());
    const RuntimeStats stats = t.rig.runtime->stats();
    CHECK(stats.stalls + stats.emulatorLost == 1);
    // The robot was unfrozen by the watchdog, not left waiting for us.
    CHECK_FALSE(t.rig.clock.paused);

    // The emulator that comes back is told the lock-step is over; the vision of that frame is not
    // applied.
    t.probe.applyDelay = 0ms;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2)};
    const auto late = t.rig.emulator->tick();
    REQUIRE(late);
    CHECK(late->result != proto::TickResult::Completed);
    CHECK(late->vision.empty());
    CHECK(t.probe.applied().size() == 1);
}

TEST_CASE("vision dispatch: more camera states than the wire allows are cut and reported once") {
    Started t;
    t.probe.flood = SimSession::kMaxVisionStates + 44;
    const auto first = t.rig.emulator->tick();
    REQUIRE(first);
    CHECK(first->vision.size() == SimSession::kMaxVisionStates);
    const auto second = t.rig.emulator->tick();
    REQUIRE(second);
    CHECK(second->vision.size() == SimSession::kMaxVisionStates);

    const auto isWarning = [](const proto::Log& l) {
        return l.message.find("camera states") != std::string::npos;
    };
    const auto warn = t.rig.emulator->take<proto::Log>(1000ms, isWarning);
    REQUIRE(warn);
    CHECK(warn->level == sdk::LogLevel::Warn);
    CHECK_FALSE(t.rig.emulator->take<proto::Log>(100ms, isWarning).has_value());  // only once
}

// ---- the watchdog path --------------------------------------------------------------------

TEST_CASE("vision dispatch: a stalled step answers later frames without vision") {
    Started t;  // 400 ms watchdog
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    REQUIRE(t.rig.emulator->tick());
    t.rig.clock.blockNextStep = true;

    // The stalled frame was accepted, so its vision went out before the step began.
    t.rig.emulator->nextVision = {makeFrame("photon-front", 2)};
    const auto stalled = t.rig.emulator->tick({}, 5000, 12.0, 3000ms);
    REQUIRE(stalled);
    CHECK(stalled->result == proto::TickResult::RobotStalled);
    CHECK(stalled->vision.empty());  // answered by the watchdog thread, which never asks a bridge
    CHECK(t.probe.applied().size() == 2);
    t.rig.clock.release();

    t.rig.emulator->nextVision = {makeFrame("photon-front", 3)};
    const auto later = t.rig.emulator->tick();
    REQUIRE(later);
    CHECK(later->result == proto::TickResult::RobotStalled);
    CHECK(later->vision.empty());
    CHECK(t.probe.applied().size() == 2);
    // Not for the stalled tick, not for the answered-only ones.
    CHECK(t.probe.count("collectVision") == 1);

    // Only the clock thread ever runs a vision hook.
    std::thread::id clockThread;
    for (const VisionProbe::Event& e : t.probe.events())
        if (e.name == "applyInputs" || e.name == "collectOutputs") clockThread = e.thread;
    for (const VisionProbe::Event& e : t.probe.events())
        if (e.name == "applyVision" || e.name == "collectVision") CHECK(e.thread == clockThread);
    CHECK(t.rig.runtime->stats().stalls == 1);

    // A Reset re-engages lock-step: vision flows again and the camera list is a full one.
    t.probe.setCamera(makeState("photon-front"));
    t.rig.emulator->epoch = 2;
    t.rig.emulator->nextTick = 1;
    REQUIRE(t.rig.emulator->send(proto::Message(resetTo(2, 3))));
    REQUIRE(t.rig.emulator->take<proto::SimControlAck>(1000ms));
    t.rig.emulator->nextVision = {makeFrame("photon-front", 4)};
    const auto recovered = t.rig.emulator->tick();
    REQUIRE(recovered);
    CHECK(recovered->result == proto::TickResult::Completed);
    CHECK(recovered->vision.size() == 1);
    CHECK(t.probe.applied().size() == 3);
}

TEST_CASE("vision dispatch: a lost emulator's late frames are answered without vision") {
    Started t;
    REQUIRE(t.rig.emulator->tick());
    REQUIRE(waitUntil([&] { return !t.rig.clock.paused; }, 2500ms));
    CHECK(t.rig.runtime->stats().emulatorLost == 1);

    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    const auto late = t.rig.emulator->tick();
    REQUIRE(late);
    CHECK(late->result == proto::TickResult::Stopped);
    CHECK(late->vision.empty());
    CHECK(t.probe.count("applyVision") == 0);
    CHECK(t.rig.runtime->stats().visionFramesApplied == 0);
}

// ---- polling and shutdown -----------------------------------------------------------------

TEST_CASE("vision dispatch: the program-start poll precedes the first camera frame") {
    ExtensionConfig cfg = Rig::defaultConfig();
    // The production default: nothing polls while the robot constructor runs.
    cfg.pollWhileConstructing = false;
    Rig rig(defaultAck(), cfg);
    VisionProbe& probe = addProbe(rig);
    REQUIRE(rig.start());

    proto::SensorFrame early;
    early.tick = 1;
    early.simTimeUs = 5000;
    early.sessionId = 7;
    early.runEpoch = 1;
    early.vision = {makeFrame("photon-front", 1)};
    REQUIRE(rig.emulator->send(proto::Message(early)));
    std::this_thread::sleep_for(100ms);
    CHECK(probe.count("poll") == 0);
    CHECK(probe.count("applyVision") == 0);

    rig.clock.started = true;
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));
    REQUIRE(rig.emulator->take<proto::ActuatorFrame>(2000ms));

    const std::vector<std::string> names = probe.names();
    const auto firstPoll = std::find(names.begin(), names.end(), "poll");
    const auto firstApply = std::find(names.begin(), names.end(), "applyVision");
    REQUIRE(firstPoll != names.end());
    REQUIRE(firstApply != names.end());
    CHECK(firstPoll < firstApply);  // the bridge sees the program start before the first frame

    const std::ptrdiff_t polls = probe.count("poll");
    rig.emulator->nextTick = 2;
    REQUIRE(rig.emulator->tick());
    CHECK(probe.count("poll") > polls);  // and every tick polls again after its step
}

TEST_CASE("bridge host: programStarted is false during construction, true from the start poll") {
    ExtensionConfig cfg = Rig::defaultConfig();
    bool livePolling = false;
    SUBCASE("polled only once the program started (the default)") {}
    SUBCASE("polled while the constructor runs (WPIEMU_LIVE_DISCOVERY=1)") {
        livePolling = true;
    }
    cfg.pollWhileConstructing = livePolling;
    Rig rig(defaultAck(), cfg);
    VisionProbe& probe = addProbe(rig);
    REQUIRE(rig.start());

    CHECK_FALSE(probe.host->programStarted());
    if (livePolling) {
        REQUIRE(waitUntil([&] { return probe.count("poll") >= 3; }));
    } else {
        std::this_thread::sleep_for(100ms);
        CHECK(probe.count("poll") == 0);
    }
    for (const VisionProbe::Event& e : probe.events()) CHECK_FALSE(e.programStarted);

    rig.clock.started = true;
    CHECK(probe.host->programStarted());
    // The lifecycle Log is posted right after the poll at program start.
    REQUIRE(rig.emulator->waitForLifecycle(2000ms));

    std::vector<int> seen;  // programStarted() at every poll, as 0 / 1
    for (const VisionProbe::Event& e : probe.events())
        if (e.name == "poll") seen.push_back(e.programStarted ? 1 : 0);
    REQUIRE_FALSE(seen.empty());
    CHECK(std::is_sorted(seen.begin(), seen.end()));  // false while constructing, true from then on
    CHECK(seen.back() == 1);                          // the poll at program start
    CHECK(seen.front() == (livePolling ? 0 : 1));     // only live discovery polls before it
}

TEST_CASE("shutdown: after stop() returns the bridges are stopped and no hook runs again") {
    Started t;
    t.rig.emulator->nextVision = {makeFrame("photon-front", 1)};
    REQUIRE(t.rig.emulator->tick());

    t.rig.runtime->stop();
    CHECK(t.rig.runtime->finished());
    CHECK(t.probe.stops >= 1);
    const std::size_t calls = t.probe.events().size();

    proto::SensorFrame late;  // the emulator keeps talking, nobody is listening any more
    late.tick = 2;
    late.sessionId = 7;
    late.runEpoch = 1;
    late.vision = {makeFrame("photon-front", 2)};
    (void)t.rig.emulator->send(proto::Message(late));
    std::this_thread::sleep_for(100ms);
    CHECK(t.probe.events().size() == calls);
    CHECK(t.probe.applied().size() == 1);
}

TEST_CASE("shutdown: a detached clock thread cannot reach the bridges after stop() returned") {
    ExtensionConfig cfg = Rig::defaultConfig();
    cfg.watchdogEnabled = false;  // this is about the shutdown hook, not about unblocking the robot
    Started t(defaultAck(), cfg);
    struct Release {  // never leave the clock thread stuck when an assertion unwinds the test
        VisionProbe& probe;
        ~Release() { probe.releaseApply(); }
    } release{t.probe};
    REQUIRE(t.rig.emulator->tick());

    t.probe.blockNextApply();
    proto::SensorFrame stuck;
    stuck.tick = 2;
    stuck.simTimeUs = 10000;
    stuck.sessionId = 7;
    stuck.runEpoch = 1;
    stuck.vision = {makeFrame("photon-front", 1)};
    REQUIRE(t.rig.emulator->send(proto::Message(stuck)));
    REQUIRE(t.probe.waitBlocked(2000ms));

    t.rig.runtime->stop();  // waits its bounded 3 s for the clock thread, then detaches it
    CHECK_FALSE(t.rig.runtime->finished());
    const std::size_t calls = t.probe.events().size();

    t.probe.releaseApply();  // the bridge call returns and the thread finishes the tick it was in
    REQUIRE(t.rig.runtime->waitFinished(3000ms));
    // No poll, collectOutputs or collectVision after the hook returned.
    CHECK(t.probe.events().size() == calls);
    CHECK(t.probe.count("collectVision") == 1);
}
