#pragma once

// Test doubles for the HALSim extension: fake HAL ports, a scriptable bridge, an in-memory
// "connector" and a fake emulator that speaks the protocol from the other end of an in-memory pipe.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "wpiemu/halsim_ext/extension_runtime.h"
#include "wpiemu/halsim_ext/hal_ports.h"
#include "wpiemu/sdk/devices/device_bridge.h"
#include "wpiemu/sdk/ipc/lifecycle.h"
#include "wpiemu/sdk/ipc/lockstep.h"
#include "wpiemu/sdk/testing/in_memory_transport.h"

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

// ---- transport plumbing ---------------------------------------------------------------------------------
// ITransportClient that hands out pre-made transports in order (one per connect()).
class QueueClient final : public proto::ITransportClient {
public:
    void push(std::unique_ptr<proto::ITransport> t) {
        std::lock_guard lock(mutex_);
        queue_.push_back(std::move(t));
    }
    sdk::Result<std::unique_ptr<proto::ITransport>> connect(const std::string& endpoint,
                                                            std::chrono::milliseconds timeout) override {
        lastEndpoint = endpoint;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            {
                std::lock_guard lock(mutex_);
                ++attempts;
                if (!queue_.empty()) {
                    auto t = std::move(queue_.front());
                    queue_.pop_front();
                    return t;
                }
            }
            if (std::chrono::steady_clock::now() >= deadline)
                return sdk::makeError(sdk::ErrorCode::Timeout, "no emulator listening");
            std::this_thread::sleep_for(5ms);
        }
    }
    std::string lastEndpoint;
    std::atomic<int> attempts{0};

private:
    std::mutex mutex_;
    std::deque<std::unique_ptr<proto::ITransport>> queue_;
};

// The emulator end of an in-memory pipe. A reader thread collects every inbound message; helpers
// wait for / take messages and send frames with the identity the test chooses.
class FakeEmulator {
public:
    FakeEmulator(std::unique_ptr<proto::ITransport> transport, std::shared_ptr<const proto::IMessageCodec> codec)
        : channel_(std::make_unique<proto::MessageChannel>(std::move(transport), std::move(codec))) {
        reader_ = std::thread([this] { readLoop(); });
    }
    ~FakeEmulator() {
        stop_ = true;
        channel_->close();
        if (reader_.joinable()) reader_.join();
    }

    // Answer the next Hello with this ack (sessionId/epoch are what the tests then stamp on frames).
    void autoAck(proto::HelloAck ack) {
        std::lock_guard lock(mutex_);
        autoAck_ = std::move(ack);
    }
    bool send(proto::Message m) { return static_cast<bool>(channel_->send(m)); }
    void disconnect() { channel_->close(); }

    template <class T, class Pred = std::function<bool(const T&)>>
    std::optional<T> take(std::chrono::milliseconds timeout, Pred pred = [](const T&) { return true; }) {
        std::unique_lock lock(mutex_);
        std::optional<T> found;
        cv_.wait_for(lock, timeout, [&] {
            for (auto it = inbox_.begin(); it != inbox_.end(); ++it) {
                if (const T* v = std::get_if<T>(&*it); v != nullptr && pred(*v)) {
                    found = *v;
                    inbox_.erase(it);
                    return true;
                }
            }
            return false;
        });
        return found;
    }
    template <class T>
    std::size_t count() const {
        std::lock_guard lock(mutex_);
        std::size_t n = 0;
        for (const auto& m : inbox_)
            if (std::holds_alternative<T>(m)) ++n;
        return n;
    }
    bool waitForLifecycle(std::chrono::milliseconds timeout) {
        return take<proto::Log>(timeout, [](const proto::Log& l) {
                   return proto::isLifecycleLog(l, proto::kLifecycleProgramStarted);
               }).has_value();
    }
    bool connectionClosed() const { return closed_; }
    // Message types in arrival order (never consumed by take()).
    static constexpr int kHistoryLifecycle = 1000;  // the program_started Log (other Logs record as MessageType::Log)
    std::vector<int> history() const {
        std::lock_guard lock(mutex_);
        return history_;
    }

    std::uint64_t sessionId = 7;
    std::uint64_t epoch = proto::kFirstRunEpoch;
    std::uint64_t nextTick = 1;
    sdk::SimTimeUs simTimeUs = 0;

    // Sends one SensorFrame and returns the extension's ack (nullopt on timeout).
    std::optional<proto::ActuatorFrame> tick(std::vector<sdk::SignalSample> samples = {},
                                             std::uint32_t stepUs = 5000, double batteryV = 12.0,
                                             std::chrono::milliseconds timeout = 2000ms) {
        proto::SensorFrame f;
        f.tick = nextTick++;
        simTimeUs += stepUs;
        f.simTimeUs = simTimeUs;
        f.stepUs = stepUs;
        f.batteryVoltageV = batteryV;
        f.samples = std::move(samples);
        f.sessionId = sessionId;
        f.runEpoch = epoch;
        const std::uint64_t tickNo = f.tick;
        if (!send(proto::Message(std::move(f)))) return std::nullopt;
        return take<proto::ActuatorFrame>(timeout, [tickNo](const proto::ActuatorFrame& a) {
            return a.ackedTick() == tickNo;
        });
    }
    bool sendControl(proto::SimCommand cmd, std::uint64_t commandId = proto::kUnspecified) {
        proto::SimControl c;
        c.command = cmd;
        c.commandId = commandId;
        c.sessionId = sessionId;
        c.runEpoch = epoch;
        return send(proto::Message(c));
    }
    bool sendDs(proto::DsState ds) {
        ds.sessionId = sessionId;
        ds.runEpoch = epoch;
        return send(proto::Message(std::move(ds)));
    }

private:
    static bool isLifecycle(const proto::Message& m) {
        const auto* log = std::get_if<proto::Log>(&m);
        return log != nullptr && proto::isLifecycleLog(*log, proto::kLifecycleProgramStarted);
    }

    void readLoop() {
        while (!stop_) {
            proto::Message m;
            const sdk::Result<bool> got = channel_->receive(m, 20ms);
            if (!got) {
                closed_ = true;
                cv_.notify_all();
                return;
            }
            if (!got.value()) continue;
            std::optional<proto::HelloAck> reply;
            {
                std::lock_guard lock(mutex_);
                if (std::holds_alternative<proto::Hello>(m) && autoAck_) reply = autoAck_;
                history_.push_back(isLifecycle(m) ? kHistoryLifecycle : static_cast<int>(proto::typeOf(m)));
                inbox_.push_back(std::move(m));
            }
            if (reply) (void)channel_->send(proto::Message(*reply));
            cv_.notify_all();
        }
    }

    std::unique_ptr<proto::MessageChannel> channel_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<proto::Message> inbox_;
    std::vector<int> history_;
    std::optional<proto::HelloAck> autoAck_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> closed_{false};
    std::thread reader_;
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
        emulator = std::make_unique<FakeEmulator>(std::move(pair.a), codec);
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

    sdk::Status start() {
        RuntimeDeps deps{config, robot, HalPorts{clock, ds, power, process}, client, codec, std::move(bridges),
                         [this](std::string_view line) {
                             std::lock_guard lock(logMutex);
                             logLines.emplace_back(line);
                         }};
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
    QueueClient client;
    std::unique_ptr<FakeEmulator> emulator;
    FakeHalClock clock;
    FakeDriverStation ds;
    FakePower power;
    FakeProcess process;
    std::vector<std::unique_ptr<sdk::IDeviceBridge>> bridges;
    std::unique_ptr<ExtensionRuntime> runtime;
    std::mutex logMutex;
    std::vector<std::string> logLines;
};

template <class Pred>
bool waitUntil(Pred pred, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(2ms);
    }
    return pred();
}

}  // namespace wpiemu::halsim::testing
