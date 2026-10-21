#pragma once

// Test doubles for the HALSim extension: fake HAL ports, scriptable bridges, and the Rig that runs an ExtensionRuntime
// against them and a FakeEmulator (sdk/testing) at the other end of an in-memory pipe.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/halsim_ext/extension_runtime.h"
#include "wpiemu/halsim_ext/hal_ports.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/vision/vision_bridge.h"
#include "wpiemu/sdk/testing/fake_emulator.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"
#include "wpiemu/sdk/testing/wait.h"

namespace wpiemu::halsim::testing {

namespace proto = sdk::proto;
using namespace std::chrono_literals;

// ---- HAL ports -------------------------------------------------------------------------------------------
class FakeHalClock final : public IHalClock {
public:
    void pauseTiming() override {
        paused = true;
        ++pauseCalls;
    }
    void resumeTiming() override {
        paused = false;
        ++resumeCalls;
        std::lock_guard lock(mutex);
        cv.notify_all();
    }
    bool isTimingPaused() const override { return paused; }
    bool programStarted() const override { return started; }
    bool schedulerArmed() const override { return armed; }
    void stepTiming(std::uint64_t deltaUs) override {
        {
            std::unique_lock lock(mutex);
            const int resumesAtEntry = resumeCalls;
            if (blockNextStep) {
                blockNextStep = false;
                ++blockedSteps;
                // A stuck robot: returns only when released or when someone resumed real-time timing.
                cv.wait(lock, [&] { return releaseStep || resumeCalls != resumesAtEntry; });
                releaseStep = false;
            }
        }
        fpgaUs += deltaUs;
        ++steps;
        stepLog.push_back(deltaUs);
        if (onStep) onStep(deltaUs);
    }
    std::uint64_t robotLoopCount() const override { return loops; }

    void release() {
        std::lock_guard lock(mutex);
        releaseStep = true;
        cv.notify_all();
    }

    std::atomic<bool> paused{false};
    std::atomic<bool> started{false};
    std::atomic<bool> armed{true};
    std::atomic<std::uint64_t> fpgaUs{0};
    std::atomic<std::uint64_t> loops{0};
    std::atomic<int> pauseCalls{0};
    std::atomic<int> resumeCalls{0};
    std::atomic<int> steps{0};
    std::atomic<int> blockedSteps{0};
    std::vector<std::uint64_t> stepLog;  // clock thread only
    std::function<void(std::uint64_t)> onStep;
    std::mutex mutex;
    std::condition_variable cv;
    bool blockNextStep = false;
    bool releaseStep = false;
};

class FakeDriverStation final : public IDriverStationSink {
public:
    void apply(const proto::DsState& state) override {
        std::lock_guard lock(mutex);
        applied.push_back(state);
    }
    std::size_t count() const {
        std::lock_guard lock(mutex);
        return applied.size();
    }
    std::optional<proto::DsState> last() const {
        std::lock_guard lock(mutex);
        if (applied.empty()) return std::nullopt;
        return applied.back();
    }
    mutable std::mutex mutex;
    std::vector<proto::DsState> applied;
};

class FakePower final : public IRobotPowerSink {
public:
    void setBattery(double volts, double amps) override {
        std::lock_guard lock(mutex);
        calls.emplace_back(volts, amps);
    }
    std::optional<std::pair<double, double>> last() const {
        std::lock_guard lock(mutex);
        if (calls.empty()) return std::nullopt;
        return calls.back();
    }
    mutable std::mutex mutex;
    std::vector<std::pair<double, double>> calls;
};

class FakeProcess final : public IProcessControl {
public:
    void terminate(int code) override {
        ++calls;
        lastCode = code;
    }
    std::atomic<int> calls{0};
    std::atomic<int> lastCode{-1};
};

// ---- a device bridge you can script ----------------------------------------------------------------------
class FakeBridge final : public sdk::IDeviceBridge {
public:
    explicit FakeBridge(std::string bridgeName = "fake.bridge") : name_(std::move(bridgeName)) {}

    std::string_view name() const override { return name_; }

    sdk::Status start(sdk::IBridgeHost& h) override {
        host = &h;
        started = true;
        if (failStart) return sdk::makeError(sdk::ErrorCode::Internal, "scripted start failure");
        return sdk::okStatus();
    }
    void poll() override {
        std::vector<Pending> work;
        {
            std::lock_guard lock(mutex_);
            work.swap(pending_);
            ++polls;
        }
        for (Pending& p : work) {
            if (p.retire) {
                const auto it = ids_.find(p.descriptor.key());
                if (it != ids_.end()) {
                    host->retire(it->second);
                    live_.erase(it->second);
                }
            } else {
                const sdk::DeviceId id = host->announce(p.descriptor, p.specs);
                ids_[p.descriptor.key()] = id;
                std::lock_guard lock(mutex_);
                live_[id] = {};
                specs_[id] = p.specs;
            }
        }
    }
    void collectOutputs(std::vector<sdk::SignalSample>& out) override {
        std::lock_guard lock(mutex_);
        for (const auto& [id, values] : outputValues_)
            if (live_.count(id) != 0)
                for (const auto& [signal, value] : values) out.push_back({id, signal, value});
    }
    void applyInputs(std::span<const sdk::SignalSample> samples) override {
        std::lock_guard lock(mutex_);
        inputs_.insert(inputs_.end(), samples.begin(), samples.end());
        ++inputBatches_;
    }
    void applyLoopback(sdk::DeviceId id) override {
        std::lock_guard lock(mutex_);
        loopbacks_.push_back(id);
    }
    bool owns(sdk::DeviceId id) const override {
        std::lock_guard lock(mutex_);
        return live_.count(id) != 0;
    }
    void stop() noexcept override { ++stops; }
    void resetInputHistory() override { ++historyResets; }

    // ---- scripting (any thread) ----
    static sdk::DeviceDescriptor makeDescriptor(std::string type, int id) {
        sdk::DeviceDescriptor d;
        d.family = "fake";
        d.type = std::move(type);
        d.vendor = "test";
        d.id = id;
        return d;
    }
    static std::vector<sdk::SignalSpec> makeSpecs() {
        return {{"out", sdk::SignalDirection::Output, sdk::SignalType::Double, ""},
                {"in", sdk::SignalDirection::Input, sdk::SignalType::Double, ""}};
    }
    void queueDevice(const sdk::DeviceDescriptor& d, std::vector<sdk::SignalSpec> specs = makeSpecs()) {
        std::lock_guard lock(mutex_);
        pending_.push_back({d, std::move(specs), false});
    }
    void queueRetire(const sdk::DeviceDescriptor& d) {
        std::lock_guard lock(mutex_);
        pending_.push_back({d, {}, true});
    }
    void setOutput(sdk::DeviceId id, std::uint16_t signal, double value) {
        std::lock_guard lock(mutex_);
        outputValues_[id][signal] = value;
    }
    sdk::DeviceId idOf(const sdk::DeviceDescriptor& d) const {
        std::lock_guard lock(mutex_);
        const auto it = ids_.find(d.key());
        return it == ids_.end() ? sdk::DeviceId{} : it->second;
    }
    std::vector<sdk::SignalSample> inputs() const {
        std::lock_guard lock(mutex_);
        return inputs_;
    }
    std::vector<sdk::DeviceId> loopbacks() const {
        std::lock_guard lock(mutex_);
        return loopbacks_;
    }
    int inputBatches() const {
        std::lock_guard lock(mutex_);
        return inputBatches_;
    }

    sdk::IBridgeHost* host = nullptr;
    bool failStart = false;
    std::atomic<bool> started{false};
    std::atomic<int> stops{0};
    std::atomic<int> polls{0};
    std::atomic<int> historyResets{0};

private:
    struct Pending {
        sdk::DeviceDescriptor descriptor;
        std::vector<sdk::SignalSpec> specs;
        bool retire = false;
    };
    std::string name_;
    mutable std::mutex mutex_;
    std::vector<Pending> pending_;
    std::map<sdk::DeviceKey, sdk::DeviceId> ids_;   // clock thread
    std::map<sdk::DeviceId, int> live_;
    std::map<sdk::DeviceId, std::vector<sdk::SignalSpec>> specs_;
    std::map<sdk::DeviceId, std::map<std::uint16_t, double>> outputValues_;
    std::vector<sdk::SignalSample> inputs_;
    std::vector<sdk::DeviceId> loopbacks_;
    int inputBatches_ = 0;
};

// ---- a vision bridge that records what it is told -------------------------------------------------------
class FakeVisionBridge final : public sdk::IVisionBridge {
public:
    explicit FakeVisionBridge(std::string name) : name_(std::move(name)) {}

    std::string_view name() const override { return name_; }
    sdk::Status start(sdk::ILogger&) override {
        ++starts;
        if (failStart) return sdk::makeError(sdk::ErrorCode::Internal, "fake vision start failure");
        return sdk::okStatus();
    }
    void configure(const sdk::VisionSetup& setup) override {
        ++configures;
        lastSetup = setup;
    }
    void poll(sdk::SimTimeUs simTimeUs) override {
        ++polls;
        lastPollUs = simTimeUs;
    }
    void apply(std::span<const sdk::CameraFrame> frames, sdk::SimTimeUs simTimeUs) override {
        ++applies;
        lastApplyUs = simTimeUs;
        for (const sdk::CameraFrame& f : frames) frameLog.push_back(f);
    }
    void reset() override { ++resets; }
    void stop() noexcept override { ++stops; }

    bool failStart = false;
    std::atomic<int> starts{0}, configures{0}, polls{0}, applies{0}, resets{0}, stops{0};
    std::atomic<sdk::SimTimeUs> lastPollUs{-1}, lastApplyUs{-1};
    sdk::VisionSetup lastSetup;       // clock thread
    std::vector<sdk::CameraFrame> frameLog;  // clock thread

private:
    std::string name_;
};

// ---- full rig ----------------------------------------------------------------------------------------------
inline proto::HelloAck defaultAck() {
    proto::HelloAck ack;
    ack.sessionId = 7;
    ack.runEpoch = proto::kFirstRunEpoch;
    ack.stepUs = 5000;
    ack.watchdogTimeoutMs = 400;
    return ack;
}

struct Rig {
    explicit Rig(proto::HelloAck ack = defaultAck(), ExtensionConfig cfg = defaultConfig()) : config(std::move(cfg)) {
        codec = std::make_shared<sdk::testing::MemoryMessageCodec>();
        connectNext(ack);
    }
    ~Rig() {
        if (runtime) runtime->stop();
        runtime.reset();
        emulator.reset();
    }

    static ExtensionConfig defaultConfig() {
        ExtensionConfig c;
        c.endpoint = "memory:test";
        c.handshakeTimeout = 2000ms;
        c.connectTimeout = 500ms;
        c.reconnectInterval = 20ms;
        c.reconnect = false;
        c.pollWhileConstructing = true;  // tests that care about the default say so explicitly
        c.logLevel = sdk::LogLevel::Debug;
        return c;
    }

    // Creates the next in-memory pipe; the extension end is queued for connect(), the emulator end replaces `emulator`.
    void connectNext(proto::HelloAck ack) {
        auto pair = sdk::testing::makeInMemoryTransportPair();
        client.push(std::move(pair.b));
        emulator = std::make_unique<sdk::testing::FakeEmulator>(std::move(pair.a), codec);
        emulator->sessionId = ack.sessionId;
        emulator->epoch = ack.runEpoch;
        emulator->autoAck(std::move(ack));
    }

    FakeBridge& addBridge(std::string name = "fake.bridge") {
        auto b = std::make_unique<FakeBridge>(std::move(name));
        FakeBridge& ref = *b;
        bridges.push_back(std::move(b));
        return ref;
    }

    FakeVisionBridge& addVisionBridge(std::string name = "fake.vision") {
        auto b = std::make_unique<FakeVisionBridge>(std::move(name));
        FakeVisionBridge& ref = *b;
        visionBridges.push_back(std::move(b));
        return ref;
    }

    sdk::Status start() {
        RuntimeDeps deps{
            config,
            robot,
            HalPorts{clock, ds, power, process},
            client,
            codec,
            std::move(bridges),
            console ? console : ForwardingLogger::ConsoleSink([this](std::string_view line) {
                std::lock_guard lock(logMutex);
                logLines.emplace_back(line);
            }),
            std::move(visionBridges)};
        runtime = std::make_unique<ExtensionRuntime>(std::move(deps));
        return runtime->start();
    }

    bool logContains(std::string_view needle) {
        std::lock_guard lock(logMutex);
        for (const auto& l : logLines)
            if (l.find(needle) != std::string::npos) return true;
        return false;
    }

    ExtensionConfig config;
    RobotInfo robot{proto::RobotLanguage::Cpp, 2026, "2026.2.1", "test", 4242, "robot", {{"pwm", 20}}};
    std::shared_ptr<sdk::testing::MemoryMessageCodec> codec;
    sdk::testing::QueuedTransportClient client;
    std::unique_ptr<sdk::testing::FakeEmulator> emulator;
    FakeHalClock clock;
    FakeDriverStation ds;
    FakePower power;
    FakeProcess process;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    std::vector<std::unique_ptr<sdk::IVisionBridge>> visionBridges;
    std::unique_ptr<ExtensionRuntime> runtime;
    std::mutex logMutex;
    std::vector<std::string> logLines;
    ForwardingLogger::ConsoleSink console;
};

template <class Pred>
bool waitUntil(Pred pred, std::chrono::milliseconds timeout = 2000ms) {
    return sdk::testing::waitUntil(pred, timeout, 2ms);
}

}  // namespace wpiemu::halsim::testing
