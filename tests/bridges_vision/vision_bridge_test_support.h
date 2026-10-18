#pragma once

// Test doubles for the vision bridge core (VisionBridge): a call-counting INtApi over FakeNtApi, a fake IBridgeHost,
// and a test vendor / publisher pair that record what the bridge asks of them. The real PhotonVision and Limelight
// vendors are other work streams: the core is tested against the contract only. `VisionRig` wires them like the
// extension does.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "vision_frame_support.h"
#include "wpiemu/bridges/vision/camera_publisher.h"
#include "wpiemu/bridges/vision/nt_api.h"
#include "wpiemu/bridges/vision/testing/fake_nt_api.h"
#include "wpiemu/bridges/vision/vision_bridge.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/testing/fake_clock_logger.h"

namespace wpiemu::tests::vision_bridge {

using namespace wpiemu::bridges::vision;

inline constexpr std::int64_t kStepUs = 5000;

// ---- NetworkTables -----------------------------------------------------------------------------------------------

struct NtCalls {
    int available = 0;
    int unavailableReason = 0;
    int nowUs = 0;
    int publish = 0;
    int set = 0;  // every set* and setTopicProperty
    int subscribe = 0;
    int read = 0;
    int topics = 0;
    int hasForeignPublisher = 0;

    int total() const {
        return available + unavailableReason + nowUs + publish + set + subscribe + read + topics + hasForeignPublisher;
    }
    // Everything but the availability probe (and the reason, which is read for the one warning).
    int beyondAvailability() const { return total() - available - unavailableReason; }
};

// Forwards to a FakeNtApi and counts every call, so tests can prove what the bridge did NOT ask of ntcore.
class CountingNtApi final : public INtApi {
public:
    explicit CountingNtApi(testing::FakeNtApi& fake) : fake_(&fake) {}

    mutable NtCalls calls;
    std::string reason = "ntcore.dll is not loaded in this process";

    bool available() const override {
        ++calls.available;
        return fake_->available();
    }
    std::string_view unavailableReason() const override {
        ++calls.unavailableReason;
        return fake_->available() ? std::string_view() : std::string_view(reason);
    }
    std::int64_t nowUs() const override {
        ++calls.nowUs;
        return fake_->nowUs();
    }
    NtPublisher publish(std::string_view topic, NtType type, const NtPublishOptions& options) override {
        ++calls.publish;
        return fake_->publish(topic, type, options);
    }
    bool setBoolean(NtPublisher pub, std::int64_t timeUs, bool value) override {
        ++calls.set;
        return fake_->setBoolean(pub, timeUs, value);
    }
    bool setInteger(NtPublisher pub, std::int64_t timeUs, std::int64_t value) override {
        ++calls.set;
        return fake_->setInteger(pub, timeUs, value);
    }
    bool setDouble(NtPublisher pub, std::int64_t timeUs, double value) override {
        ++calls.set;
        return fake_->setDouble(pub, timeUs, value);
    }
    bool setString(NtPublisher pub, std::int64_t timeUs, std::string_view value) override {
        ++calls.set;
        return fake_->setString(pub, timeUs, value);
    }
    bool setRaw(NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) override {
        ++calls.set;
        return fake_->setRaw(pub, timeUs, value);
    }
    bool setDoubleArray(NtPublisher pub, std::int64_t timeUs, std::span<const double> value) override {
        ++calls.set;
        return fake_->setDoubleArray(pub, timeUs, value);
    }
    bool setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) override {
        ++calls.set;
        return fake_->setTopicProperty(topic, name, jsonValue);
    }
    NtSubscriber subscribe(std::string_view topic, std::size_t queueDepth) override {
        ++calls.subscribe;
        return fake_->subscribe(topic, queueDepth);
    }
    void read(NtSubscriber sub, std::vector<NtValue>& out) override {
        ++calls.read;
        fake_->read(sub, out);
    }
    void topics(std::string_view prefix, std::vector<NtTopicInfo>& out) override {
        ++calls.topics;
        fake_->topics(prefix, out);
    }
    bool hasForeignPublisher(std::string_view topic) override {
        ++calls.hasForeignPublisher;
        return fake_->hasForeignPublisher(topic);
    }

private:
    testing::FakeNtApi* fake_;
};

// ---- bridge host -------------------------------------------------------------------------------------------------

class FakeVisionHost final : public sdk::IBridgeHost {
public:
    sdk::DeviceId announce(const sdk::DeviceDescriptor&, std::vector<sdk::SignalSpec>) override {
        ++announces;
        return sdk::DeviceId{};
    }
    void retire(sdk::DeviceId) override { ++retires; }
    sdk::ILogger& logger() override { return log; }
    bool enableOptionalHooks() const override { return false; }
    const sdk::VisionSetup& visionSetup() const override { return setup; }
    std::uint64_t visionSetupRevision() const override { return revision; }
    bool programStarted() const override { return started; }

    // What the extension does when it adopts a HelloAck: the new setup, then a new revision.
    void adopt(sdk::VisionSetup next) {
        setup = std::move(next);
        ++revision;
    }

    sdk::VisionSetup setup;
    std::uint64_t revision = 1;  // the first connection's HelloAck
    bool started = true;
    int announces = 0;
    int retires = 0;
    sdk::testing::CapturingLogger log;
};

inline std::size_t countLogs(const sdk::testing::CapturingLogger& log, sdk::LogLevel level,
                             std::string_view needle = {}) {
    std::size_t n = 0;
    for (const auto& entry : log.entries())
        if (entry.level == level && entry.message.find(needle) != std::string::npos) ++n;
    return n;
}

inline std::size_t countLogs(const sdk::testing::CapturingLogger& log, std::string_view needle) {
    std::size_t n = 0;
    for (const auto& entry : log.entries())
        if (entry.message.find(needle) != std::string::npos) ++n;
    return n;
}

// ---- vendor and publisher doubles ----------------------------------------------------------------------------------

// What one TestPublisher saw. Shared with the vendor, so tests can look at it after the publisher was destroyed.
struct PublisherProbe {
    sdk::CameraSetup setup;
    int starts = 0;
    int polls = 0;
    int robotStateCalls = 0;
    std::vector<std::uint64_t> sequences;                 // frames published, in order
    std::vector<const sdk::VisionFrame*> frameAddresses;  // where they were (the bridge must not copy frames)
    sdk::CameraRobotState state;                          // what robotState() returns
    std::string throwIn;                                  // "start", "poll", "publish" or "robotState"
    bool throwNonStd = false;                             // throw an int instead of a std::runtime_error
    bool alive = true;                                    // false once the publisher object was destroyed
};

class TestPublisher final : public ICameraPublisher {
public:
    TestPublisher(INtApi& nt, std::shared_ptr<PublisherProbe> probe, std::vector<std::string>* events)
        : nt_(&nt), probe_(std::move(probe)), events_(events) {}
    ~TestPublisher() override { probe_->alive = false; }

    const sdk::CameraSetup& setup() const override { return probe_->setup; }
    void start() override {
        record("start");
        ++probe_->starts;
        maybeThrow("start");
        // A topic of our own, so tests can see that removed cameras are never unpublished.
        nt_->publish("/test/" + probe_->setup.cameraId + "/alive", NtType::Double, {});
    }
    void poll() override {
        record("poll");
        ++probe_->polls;
        maybeThrow("poll");
    }
    void publish(const sdk::VisionFrame& frame) override {
        record("publish:" + std::to_string(frame.sequence));
        maybeThrow("publish");
        probe_->sequences.push_back(frame.sequence);
        probe_->frameAddresses.push_back(&frame);
        probe_->state.publishing = true;
        ++probe_->state.framesPublished;
    }
    sdk::CameraRobotState robotState() const override {
        ++probe_->robotStateCalls;
        maybeThrow("robotState");
        return probe_->state;
    }

private:
    void record(const std::string& what) const {
        if (events_ != nullptr) events_->push_back(what + ":" + probe_->setup.cameraId);
    }
    void maybeThrow(std::string_view where) const {
        if (probe_->throwIn != where) return;
        if (probe_->throwNonStd) throw 42;
        throw std::runtime_error(std::string(where) + " failed");
    }

    INtApi* nt_;
    std::shared_ptr<PublisherProbe> probe_;
    std::vector<std::string>* events_;
};

// A vendor like the real ones as far as the bridge can tell: cameras of its kind get a TestPublisher, and the cameras
// the robot uses are recognised from topics the ROBOT created ("...Request" for PhotonVision-style, "..._set" for
// Limelight-style), never from the publisher's own topics.
class TestVendor final : public IVisionVendor {
public:
    TestVendor(sdk::CameraVendor kind, std::string prefix, std::vector<std::string>* eventLog = nullptr)
        : events(eventLog), kind_(kind), prefix_(std::move(prefix)) {}

    sdk::CameraVendor vendor() const override { return kind_; }
    std::string_view topicPrefix() const override { return prefix_; }

    std::unique_ptr<ICameraPublisher> createPublisher(INtApi& nt, sdk::ILogger&,
                                                      const sdk::CameraSetup& setup) const override {
        ++createCalls;
        if (events != nullptr) events->push_back("create:" + setup.cameraId);
        if (throwOnCreate) throw std::runtime_error("createPublisher failed");
        if (returnNull) return nullptr;
        auto probe = std::make_shared<PublisherProbe>();
        probe->setup = setup;
        probe->state.ntName = setup.ntName;
        probe->state.vendor = setup.vendor;
        probe->state.cameraId = setup.cameraId;
        if (onCreate) onCreate(*probe);
        probes.push_back(probe);
        return std::make_unique<TestPublisher>(nt, std::move(probe), events);
    }

    void discoverRobotCameras(std::span<const NtTopicInfo> topics, std::vector<std::string>& ntNames) const override {
        ++discoverCalls;
        lastTopicCount = topics.size();
        if (throwOnDiscover) throw std::runtime_error("discoverRobotCameras failed");
        const std::string_view leafSuffix = kind_ == sdk::CameraVendor::PhotonVision ? "Request" : "_set";
        for (const NtTopicInfo& topic : topics) {
            std::string_view path = topic.name;
            path.remove_prefix(1);  // the leading '/'
            if (kind_ == sdk::CameraVendor::PhotonVision) {
                if (path.substr(0, 13) != "photonvision/") continue;
                path.remove_prefix(13);
            }
            const std::size_t slash = path.find('/');
            if (slash == std::string_view::npos) continue;  // a root topic such as /photonvision/ledModeRequest
            const std::string_view leaf = path.substr(slash + 1);
            if (leaf.size() < leafSuffix.size() || leaf.substr(leaf.size() - leafSuffix.size()) != leafSuffix) continue;
            const std::string name(path.substr(0, slash));
            if (ignoredNames.count(name) != 0) continue;
            if (std::find(ntNames.begin(), ntNames.end(), name) == ntNames.end()) ntNames.push_back(name);
        }
    }

    // The probe of the most recent publisher made for a camera (null when none).
    std::shared_ptr<PublisherProbe> probeFor(std::string_view cameraId) const {
        for (auto it = probes.rbegin(); it != probes.rend(); ++it)
            if ((*it)->setup.cameraId == cameraId) return *it;
        return nullptr;
    }
    int probeCount(std::string_view cameraId) const {
        int n = 0;
        for (const auto& probe : probes) n += probe->setup.cameraId == cameraId ? 1 : 0;
        return n;
    }

    // Behaviour switches and records (mutable: createPublisher and discoverRobotCameras are const).
    std::vector<std::string>* events;  // where create:/start:/poll:/publish: calls are recorded; null = nowhere
    bool returnNull = false;
    bool throwOnCreate = false;
    bool throwOnDiscover = false;
    std::set<std::string> ignoredNames;  // cameras the "robot" stopped using
    std::function<void(PublisherProbe&)> onCreate;
    mutable std::vector<std::shared_ptr<PublisherProbe>> probes;
    mutable int createCalls = 0;
    mutable int discoverCalls = 0;
    mutable std::size_t lastTopicCount = 0;

private:
    sdk::CameraVendor kind_;
    std::string prefix_;
};

// ---- builders ----------------------------------------------------------------------------------------------------

inline sdk::CameraSetup cameraSetup(std::string id, sdk::CameraVendor vendor, std::string ntName) {
    sdk::CameraSetup setup;
    setup.cameraId = std::move(id);
    setup.vendor = vendor;
    setup.ntName = std::move(ntName);
    setup.widthPx = 1280;
    setup.heightPx = 800;
    setup.fxPx = 640.0;
    setup.fyPx = 640.0;
    setup.cxPx = 640.0;
    setup.cyPx = 400.0;
    setup.frameRateHz = 30.0;
    setup.latencyUs = 25000;
    return setup;
}
inline sdk::CameraSetup photonCamera(std::string id, std::string ntName = {}) {
    return cameraSetup(id, sdk::CameraVendor::PhotonVision, ntName.empty() ? id : std::move(ntName));
}
inline sdk::CameraSetup limelightCamera(std::string id, std::string ntName = "limelight") {
    return cameraSetup(std::move(id), sdk::CameraVendor::Limelight, std::move(ntName));
}
inline sdk::VisionSetup setupOf(std::initializer_list<sdk::CameraSetup> cameras) {
    sdk::VisionSetup setup;
    setup.cameras.assign(cameras.begin(), cameras.end());
    return setup;
}

// A realistic frame (tags 1 and 2 in view of the sample scene) for the camera.
inline sdk::VisionFrame frameFor(const sdk::CameraSetup& camera, std::uint64_t sequence) {
    static const sdk::FiducialLayout layout = vision::sampleLayout();
    vision::FrameSpec spec;
    spec.cameraId = camera.cameraId;
    spec.vendor = camera.vendor;
    spec.ntName = camera.ntName;
    spec.fieldToRobot = vision::sampleRobotPose();
    spec.robotToCamera = vision::sampleFrontMount();
    spec.tagIds = {1, 2};
    spec.sequence = sequence;
    return vision::makeFrame(spec, layout);
}

// ---- rig ---------------------------------------------------------------------------------------------------------

// The bridge wired like the extension wires it: a FakeNtApi behind a call counter, two test vendors (PhotonVision-like
// at "/photonvision/", Limelight-like at "/limelight") and a fake host. Configure `host`, `options`, the vendors and
// the FakeNtApi, then start(); tick() drives the bridge in the extension's per-tick order.
class VisionRig {
public:
    VisionRig() {
        photon = std::make_shared<TestVendor>(sdk::CameraVendor::PhotonVision, "/photonvision/", &events);
        limelight = std::make_shared<TestVendor>(sdk::CameraVendor::Limelight, "/limelight", &events);
        options.nt = nt;
        options.vendors = std::vector<std::shared_ptr<IVisionVendor>>{photon, limelight};
    }

    // Creates and starts the bridge.
    VisionBridge& start() {
        bridge = std::make_unique<VisionBridge>(options);
        const sdk::Status status = bridge->start(host);
        if (!status) throw std::runtime_error("VisionBridge::start failed: " + status.error().message);
        return *bridge;
    }

    // One emulator tick in the extension's order: NT time advances (HALSIM_StepTiming), the frames due are applied, the
    // bridge is polled, and its camera states are collected.
    std::vector<sdk::CameraRobotState> tick(std::span<const sdk::VisionFrame> frames = {}, bool full = false) {
        fake->advanceUs(kStepUs);
        bridge->applyVision(frames);
        bridge->poll();
        std::vector<sdk::CameraRobotState> out;
        bridge->collectVision(out, full);
        return out;
    }
    // `count` polls with time advancing, nothing else.
    void polls(int count) {
        for (int i = 0; i < count; ++i) {
            fake->advanceUs(kStepUs);
            bridge->poll();
        }
    }
    std::vector<sdk::CameraRobotState> collect(bool full) {
        std::vector<sdk::CameraRobotState> out;
        bridge->collectVision(out, full);
        return out;
    }

    // Index of `event` in `events` (-1 when absent).
    int eventIndex(const std::string& event) const {
        for (std::size_t i = 0; i < events.size(); ++i)
            if (events[i] == event) return static_cast<int>(i);
        return -1;
    }
    // Stops the vendors from recording calls (allocation-sensitive tests: recording allocates).
    void recordNothing() {
        photon->events = nullptr;
        limelight->events = nullptr;
    }

    std::shared_ptr<testing::FakeNtApi> fake = std::make_shared<testing::FakeNtApi>();
    std::shared_ptr<CountingNtApi> nt = std::make_shared<CountingNtApi>(*fake);
    std::vector<std::string> events;  // create:/start:/poll:/publish:<seq>: of every publisher, in call order
    std::shared_ptr<TestVendor> photon;
    std::shared_ptr<TestVendor> limelight;
    FakeVisionHost host;
    VisionBridgeOptions options;
    std::unique_ptr<VisionBridge> bridge;  // last: destroyed first, while everything it uses is alive
};

inline const sdk::CameraRobotState* findState(const std::vector<sdk::CameraRobotState>& states,
                                              std::string_view cameraId) {
    for (const auto& state : states)
        if (state.cameraId == cameraId) return &state;
    return nullptr;
}

inline const sdk::CameraRobotState* findDetected(const std::vector<sdk::CameraRobotState>& states,
                                                 std::string_view ntName) {
    for (const auto& state : states)
        if (state.cameraId.empty() && state.ntName == ntName) return &state;
    return nullptr;
}

}  // namespace wpiemu::tests::vision_bridge
